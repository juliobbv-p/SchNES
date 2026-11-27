// cpu.c specializes this loop for native traces and FDS code in PRG RAM.
// Each variant supplies RUN_FRAME and FRAME_ATTRIBUTES.
// Constant mode flags remove unused paths. CPU_DEBUG adds counts and raster tracing.
static FRAME_ATTRIBUTES NES_NOINLINE void RUN_FRAME(Nes *nes, NesInstructionStats *stats, CpuTraceCache *traces)
{
#if RUN_TRACES
    const uint32_t *arena = traces ? traces->arena : NULL;
#else
    (void)traces;
#endif
#if defined(__GNUC__) || defined(__clang__)
    // Each instruction handler can dispatch its successor inside the quiet window
    static const void *const dispatch[256] = {
#define LABEL_ADDRESS(byte, op, mode) [0x##byte] = __extension__ &&opcode_##byte,
        OPCODES(LABEL_ADDRESS)
#undef LABEL_ADDRESS
    };
#define QUIET_CASE(byte) opcode_##byte:
#if defined(__aarch64__)
// LDRB already zero-extends the opcode to the full dispatch index on ARM64
#define DISPATCH_NEXT() __extension__ ({ \
    uintptr_t opcode; \
    __asm__("ldrb %w[op], [%[code]]" : [op] "=r"(opcode) : [code] "r"(code), "m"(*code)); \
    goto *dispatch[opcode]; \
})
#else
#define DISPATCH_NEXT() __extension__ ({ goto *dispatch[code[0]]; })
#endif
#define CONTINUE_QUIET() do { \
    if (code <= page + 8189 && budget > QUIET_ROM_CYCLES) \
        DISPATCH_NEXT(); \
    goto resume_interpreter; \
} while (0)
#else
#define QUIET_CASE(byte) case 0x##byte:
#define CONTINUE_QUIET() continue
#endif
    NesInstructionStats counts = {0};
    // The helpers count only for a statistics request
    NesInstructionStats *loop_counts = CPU_DEBUG && stats ? &counts : NULL;
#define COUNT(field, n) do { if (CPU_DEBUG) counts.field += (n); } while (0)
    NesCpuRaster *raster = CPU_DEBUG ? nes->cpu_raster : NULL;
#define RECORD_AT(cycle, type) do { if (raster) cpu_record_execution(raster, cycle, type); } while (0)
#define RECORD_QUIET(type) RECORD_AT(deadline - budget, type)
    Cpu *cpu = &nes->cpu;
    // Keep P and the deferred N/Z result local across direct instructions
    uint8_t local_status;
    uint8_t *status = RUN_TRACES ? &cpu->p : &local_status;
    uint16_t local_nz;
    uint16_t *nz_result = RUN_TRACES ? NULL : &local_nz;
    uint64_t frame = nes->ppu.frames;
    const bool fds = RUN_FDS;
    const unsigned code_floor = fds ? 0x6000 : 0x8000;
#if RUN_TRACES
    // FDS predictions execute borrowed traces without warming or compiling them
    const Fds *borrowed = fds ? borrowed_fds_traces(&nes->cart, traces) : NULL;
#define HAVE_TRACES() ((!CPU_DEBUG || traces) && (!borrowed || borrowed->code_map))
#define FIND_TRACE(page, code, bank, warm) find_trace(traces, nes, page, code, bank, (warm) && !borrowed, fds)
#endif
    while (nes->ppu.frames == frame) {
        uint64_t deadline = nes->bus_deadline;

        if (deadline > cpu->cycles + QUIET_RAM_CYCLES && cpu_can_batch(nes)) {
            if (!RUN_TRACES) {
                local_status = cpu->p;
                local_nz = decode_nz(local_status);
            }

            bool ram_read = cart_unbanked_ram(&nes->cart, false);
            bool ram_write = cart_unbanked_ram(&nes->cart, true);
            uint64_t budget = deadline - cpu->cycles;
            uint8_t bus = 0;
            FinalAccess final = {0};
            uint64_t elapsed;
            unsigned back = CPU_SHORT_LOOP_BACK + 1;

            while (cpu->pc >= code_floor && (cpu->pc & 8191) <= 8189 && budget > QUIET_ROM_CYCLES) {
                unsigned bank = cpu->pc & 0xe000;
                const uint8_t *page = bank >= 0x8000 ? cart_prg_page(&nes->cart, cpu->pc) : nes->cart.ram;
                const uint8_t *code = page + (cpu->pc & 8191);
resume_bank:
                // Carry a mapped instruction pointer across straight-line code in this bank
                while (code <= page + 8189 && budget > QUIET_ROM_CYCLES) {
#if RUN_TRACES
                    // The debug loop also runs when traces are unavailable or disabled
                    if (HAVE_TRACES() &&
                        (budget >= 13 || (!nes->ultrafast && at_dmc_deadline(nes, deadline)))) {
                        // Short DMC windows only use previously compiled code
                        BlockTrace *trace = FIND_TRACE(page, code, bank, budget >= 13);

                        if (trace && budget > ((trace->layout >> BLOCK_ENTRY_BITS) & BLOCK_CYCLES_MASK)) {
                            const uint32_t *code_entry = arena + (trace->layout & BLOCK_ENTRY_MASK) * BLOCK_WORDS;
                            RomTraceEntry entry;

                            memcpy(&entry, &code_entry, sizeof entry);
                            // Linked traces return the remaining window and the
                            // number of guest instructions before the final block.
                            unsigned link_budget = (uint16_t)budget;
                            RomTraceResult packet = entry(nes, link_budget << TRACE_LINK_BUDGET_SHIFT);
                            uint64_t result = packet.result;
                            unsigned spent = (result & TRACE_CYCLE_MASK) +
                                link_budget - (uint16_t)(packet.resume >> TRACE_LINK_BUDGET_SHIFT);

                            budget -= spent;
                            RECORD_QUIET(CPU_EXEC_RECOMPILED);
                            COUNT(native_trace, (packet.resume & TRACE_LINK_COUNT_MASK) +
                                                (uint8_t)(result >> TRACE_COUNT_SHIFT));
                            unsigned exit = (uint8_t)(result >> TRACE_EXIT_SHIFT);

                            if (exit) {
                                uint16_t target = result >> TRACE_TARGET_SHIFT;

                                if (exit == TRACE_DIRECT_EXIT) {
                                    bus = result >> TRACE_BUS_SHIFT;

                                    if ((target & 0xe000) != bank) {
                                        cpu->pc = target;
                                        goto next_bank;
                                    }
                                    code = page + (target & 8191);
                                    continue;
                                }
                                if (exit == TRACE_ENTRY_REJECTED) {
                                    code = page + (target & 8191);

                                    if (budget <= QUIET_ROM_CYCLES)
                                        goto resume_interpreter;
                                    goto retry_trace_entry;
                                }
                                bus = result >> TRACE_BUS_SHIFT;
                                cpu->pc = target;
                                code = page + ((result >> TRACE_BRANCH_PC_SHIFT) & 8191);
                                goto control_transfer;
                            }
                            bus = result >> TRACE_BUS_SHIFT;

                            if (!(result & TRACE_WRITE)) {
                                code = page + ((result >> TRACE_TARGET_SHIFT) & 8191);
                                continue;
                            }
                            code = page + ((packet.resume >> TRACE_WRITE_PC_SHIFT) & 8191);

                            if (budget > 4) {
                                uint16_t address = result >> TRACE_TARGET_SHIFT;
                                uint8_t value = result >> TRACE_WRITE_VALUE_SHIFT;

                                if (nes_write_quiet(nes, address, value, deadline - budget + 3)) {
                                    budget -= 4;
                                    RECORD_QUIET(CPU_EXEC_BUS_HANDLER);
                                    bus = value;
                                    COUNT(bus_finish, 1);
                                    code += 3;

                                    // A mapper write can replace the executing PRG page
                                    if (address >= 0x8000 || mmc5_prg_remap(address)) {
                                        cpu->pc = bank + (code - page);
                                        goto next_bank;
                                    }
                                }
                            }
                            continue;
                        }
                        if (trace && (trace->layout & BLOCK_CHECKS_WRITES) && !nes->ultrafast &&
                            at_dmc_deadline(nes, deadline)) {
                            unsigned done = run_dmc_trace(nes, arena, trace->layout,
                                                          deadline, (unsigned)budget);

                            if (done) {
                                COUNT(native_trace, done - 1);
                                deadline = nes->bus_deadline;
                                budget = deadline - cpu->cycles;
                                bus = nes->cpu_bus;

                                if ((cpu->pc & 0xe000) != bank)
                                    goto next_bank;
                                code = page + (cpu->pc & 8191);
                                continue;
                            }
                        }
                    }
retry_trace_entry:
#endif
#if defined(DISPATCH_NEXT)
                    DISPATCH_NEXT();
#else
                    switch (code[0]) {
#endif
#if RUN_TRACES
#define TRY_UNCOMMON(byte, op) do { \
    if (uncommon_rom_opcode(0x##byte, op)) \
        goto uncommon_rom; \
} while (0)
#else
#define TRY_UNCOMMON(byte, op) ((void)0)
#endif
#define QUIET(byte, op, mode) \
                    QUIET_CASE(byte) { \
                        TRY_UNCOMMON(byte, op); \
                        if (mode == REL && !direct_branch_taken(*status, nz_result, 0x##byte)) { \
                            budget -= 2; \
                            RECORD_QUIET(CPU_EXEC_DIRECT); \
                            bus = code[1]; \
                            code += 2; \
                            COUNT(direct, 1); \
                            CONTINUE_QUIET(); \
                        } \
                        const unsigned limit = quiet_rom_cycles(mode, ACCESS(op)); \
                        if (limit > QUIET_ROM_CYCLES && budget <= limit) \
                            goto resume_interpreter; \
                        const bool transfer = mode == REL || op == JMP || op == JSR || op == RTS; \
                        if (transfer) \
                            cpu->pc = bank + (code - page); \
                        if (!run_quiet_instruction(nes, code, status, nz_result, op, mode, ACCESS(op), &budget, \
                                                   &bus, ram_read, ram_write, &final)) \
                            goto resume_interpreter; \
                        RECORD_QUIET(CPU_EXEC_DIRECT); \
                        COUNT(direct, 1); \
                        if (!transfer) { \
                            code += mode_lengths[mode]; \
                            /* Keep the trace lookup at the successor of a status pull */ \
                            if (RUN_TRACES && op == PLP) \
                                continue; \
                            CONTINUE_QUIET(); \
                        } \
                        if (op == JSR || op == RTS || (op == JMP && mode == IND)) \
                            goto transfer_to_pc; \
                        goto control_transfer; \
                    }
                    OPCODES(QUIET)
#undef QUIET
#undef TRY_UNCOMMON
#if RUN_TRACES
uncommon_rom: {
                        const Instruction instruction = instructions[code[0]];
                        const Mode mode = instruction_mode(instruction);
                        const Access access = instruction_access(instruction);

                        if (budget <= quiet_rom_cycles(mode, access) ||
                            !run_quiet_instruction(nes, code, status, nz_result, (Operation)instruction.op,
                                                   mode, access, &budget, &bus, ram_read, ram_write, &final))
                            goto resume_interpreter;
                        code += mode_lengths[mode];
                        RECORD_QUIET(CPU_EXEC_DIRECT);
                        COUNT(direct, 1);
                        CONTINUE_QUIET();
                    }
#endif
#if !defined(DISPATCH_NEXT)
                    }
#endif
control_transfer: {
                        unsigned distance = (uint16_t)((bank + (code - page)) - cpu->pc);

                        if (short_loop_in_bank(cpu->pc, distance) &&
                            (cpu_is_branch(code[0]) || code[0] == 0x4c)) {
                            const uint8_t *target = code - distance;
                            const ShortLoop *loop = &nes->short_loops[short_loop_slot(cpu->pc)];
                            uintptr_t key = mapped_code_key(target, cpu->pc, fds);
                            bool decoded = short_loop_matches(loop, key, distance);
#if RUN_TRACES
                            // A cached loop can spend the current quiet budget and return within this PRG bank
                            if (decoded && (loop->flags & LOOP_NATIVE) && HAVE_TRACES()) {
                                uint64_t result = run_cached_loop(nes, traces, loop, target, budget);

                                if (result) {
                                    budget -= (uint16_t)result;
                                    RECORD_QUIET(CPU_EXEC_LOOP_BATCHED);
                                    bus = result >> LOOP_BUS_SHIFT;
                                    cpu->pc = result >> LOOP_TARGET_SHIFT;
                                    COUNT(skipped, native_loop_instructions(loop, result));
                                    code = page + (cpu->pc & 8191);
                                    continue;
                                }
                            }
#endif
                            if (!decoded || short_loop_enabled(loop, RUN_INTERPRETER)) {
                                back = distance;
                                goto quiet_done;
                            }
                        }
                    }
transfer_to_pc:
                    if ((cpu->pc & 0xe000) != bank)
                        goto next_bank;
                    code = page + (cpu->pc & 8191);
                }
resume_interpreter: {
                    cpu->pc = bank + (code - page);
                    // Give the bus handler a local copy so the trace loop can keep its counters in registers
                    uint64_t access_budget = budget;
                    uint8_t access_bus = bus;
                    bool finished = false;

                    // A bus read can update P before direct dispatch resumes
                    if (RUN_TRACES || final.prefix_cycles) {
                        if (!RUN_TRACES)
                            cpu->p = read_status(local_status, local_nz);
                        finished = run_quiet_final(nes, &final, deadline, &access_budget, &access_bus);
                        if (!RUN_TRACES) {
                            local_status = cpu->p;
                            local_nz = decode_nz(local_status);
                        }
                    }
                    if (finished) {
                        budget = access_budget;
                        RECORD_QUIET(CPU_EXEC_BUS_HANDLER);
                        bus = access_bus;
                        COUNT(bus_finish, 1);
                        if (final.address < 0x8000 && !mmc5_prg_remap(final.address)) {
                            code += final.bytes;
                            goto resume_bank;
                        }
                        cpu->pc += final.bytes;
                        continue;
                    }
                    if (code != page + 8192)
                        break;
                }
next_bank:
                continue;
            }
            if (cpu->pc < 0x2000 && !final.prefix_cycles && budget > QUIET_RAM_CYCLES) {
                uint64_t ram_budget = budget;
                uint8_t ram_bus = bus;

                if (!RUN_TRACES)
                    cpu->p = read_status(local_status, local_nz);
#if CPU_DEBUG
                run_debug_ram(nes, &ram_budget, &ram_bus, ram_read, ram_write, &final, deadline, loop_counts, traces);
#elif RUN_TRACES
                if (traces)
                    run_traced_ram(nes, &ram_budget, &ram_bus, ram_read, ram_write, &final, deadline, traces);
                else
                    run_quiet_ram(nes, &ram_budget, &ram_bus, ram_read, ram_write, &final, deadline);
#else
                run_quiet_ram(nes, &ram_budget, &ram_bus, ram_read, ram_write, &final, deadline);
#endif
                if (!RUN_TRACES) {
                    local_status = cpu->p;
                    local_nz = decode_nz(local_status);
                }
                budget = ram_budget;
                bus = ram_bus;
            }
quiet_done:
            if (!RUN_TRACES)
                cpu->p = read_status(local_status, local_nz);
            elapsed = deadline - budget - cpu->cycles;

            if (elapsed)
                complete_quiet_accesses(nes, bus, (unsigned)elapsed);
            if (back <= CPU_SHORT_LOOP_BACK) {
                try_decoded_loop(nes, back, loop_counts, code_floor);
                RECORD_AT(cpu->cycles, CPU_EXEC_LOOP_BATCHED);
                continue;
            }
            if (final.prefix_cycles) {
                if (raster)
                    raster->current = CPU_EXEC_BUS_HANDLER;
                unsigned finished = finish_bus_access(nes, final);

                COUNT(bus_finish, finished);
                RECORD_AT(cpu->cycles, CPU_EXEC_BUS_HANDLER);
                continue;
            }
        }
#if RUN_TRACES
        // A refill can leave too little room for direct dispatch, even when
        // the next already compiled trace has a provably quiet read.
        deadline = nes->bus_deadline;

        if (traces && HAVE_TRACES() && !nes->ultrafast && at_dmc_deadline(nes, deadline) &&
            deadline > cpu->cycles && deadline - cpu->cycles <= QUIET_ROM_CYCLES &&
            cpu->pc >= 0x8000 && (cpu->pc & 8191) <= 8189 &&
            cpu_can_batch(nes) &&
            (!nes->cart.mmc5 || !nes->cart.mmc5->ram_slots)) {
            unsigned bank = cpu->pc & 0xe000;
            const uint8_t *page = cart_prg_page(&nes->cart, cpu->pc);
            BlockTrace *trace = FIND_TRACE(page, page + (cpu->pc & 8191), bank, false);

            if (trace && (trace->layout & BLOCK_CHECKS_WRITES)) {
                unsigned done = run_dmc_trace(nes, arena, trace->layout,
                                              deadline, (unsigned)(deadline - cpu->cycles));

                if (done) {
                    COUNT(native_trace, done - 1);
                    continue;
                }
            }
        }
#endif
        uint16_t previous = cpu->pc;

        if (raster)
            raster->current = (cpu->jammed || cpu->nmi || cpu->irq) ? CPU_EXEC_INTERRUPT_IDLE : CPU_EXEC_INDIVIDUAL;
        if (cpu_step(nes))
            COUNT(individual, 1);
        RECORD_AT(cpu->cycles, raster->current);
        unsigned back = (uint16_t)(previous - cpu->pc);

        if (back <= CPU_SHORT_LOOP_BACK && nes->ppu.frames == frame) {
            try_decoded_loop(nes, back, loop_counts, code_floor);
            RECORD_AT(cpu->cycles, CPU_EXEC_LOOP_BATCHED);
        }
    }
    if (CPU_DEBUG && stats)
        *stats = counts;
}

#undef RUN_FRAME
#undef FRAME_ATTRIBUTES
#undef RUN_TRACES
#undef RUN_INTERPRETER
#undef RUN_FDS
#undef HAVE_TRACES
#undef FIND_TRACE
#undef DISPATCH_NEXT
#undef CONTINUE_QUIET
#undef QUIET_CASE
#undef COUNT
#undef RECORD_QUIET
#undef RECORD_AT
