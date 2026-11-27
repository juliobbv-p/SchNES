#include <assert.h>

#if defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#include <arm_neon.h>
#endif

// Trace instructions run only within a quiet CPU window. Each entry receives
// the current machine, so cached code never embeds a pointer to mutable state.
enum {
    TRACE_SLOTS = 32768,
    LOOP_TRACE_SLOTS = 256,
    TRACE_REUSE_SLOTS = 1024,
    TRACE_ARENA_SIZE = 2 * 1024 * 1024,
    TRACE_SEGMENTS = 4,
    TRACE_SWEEP_SLOTS = TRACE_SLOTS / 4,
    TRACE_KEY_ROM_LIMIT = 1u << 26,
    TRACE_MIN_INSTRUCTIONS = 2,
    TRACE_MAX_INSTRUCTIONS = 40,
    TRACE_MAX_CYCLES = 96,
    REGULAR_TRACE_MAX_CYCLES = 56,
    RAM_TRACE_MAX_CYCLES = 80,
    TRACE_MAX_EXITS = 7,
    // One bit for each byte of FDS PRG RAM, $6000-$DFFF
    FDS_CODE_MAP_BYTES = 0x8000 / 8,
    TRACE_FORWARD_SPAN = 128,
    TRACE_WARM_HITS = 15,
    TRACE_COMPILES_PER_FRAME = 8,
    TRACE_EMISSION_BUDGET = 1792,
    TRACE_MACHINE_WORDS = 1024, // Includes guarded address calculations and exit stubs
    TRACE_EXIT_WORDS = 384, // Room kept for exit stubs once the trace body stops growing
    BLOCK_ALIGNMENT = 4, BLOCK_WORDS = BLOCK_ALIGNMENT / sizeof(uint32_t),
    DMC_WRITE_MASK_WORDS = 4, DMC_WRITE_MASK_BYTES = DMC_WRITE_MASK_WORDS * sizeof(uint32_t),
    RAM_TRACE_ALIGNMENT = 16,
    TRACE_SEGMENT_SIZE = TRACE_ARENA_SIZE / TRACE_SEGMENTS,
    TRACE_RESERVE = TRACE_MACHINE_WORDS * sizeof(uint32_t) + DMC_WRITE_MASK_BYTES + RAM_TRACE_ALIGNMENT,
    TRACE_CYCLE_MASK = 127,
    TRACE_WRITE = 1 << 7,
    TRACE_COUNT_SHIFT = 8,
    TRACE_BUS_SHIFT = 16,
    TRACE_EXIT_SHIFT = 24,
    // Zero is fallthrough. A loop exit reports both its branch and target
    TRACE_DIRECT_EXIT = 1,
    TRACE_LOOP_EXIT = 2,
    TRACE_ENTRY_REJECTED = 3,
    TRACE_TARGET_SHIFT = 32,
    TRACE_WRITE_VALUE_SHIFT = 48,
    // The second return register packs a 16-bit cycle budget above a 15-bit
    // linked instruction count. Each guest instruction takes at least two cycles.
    // A pending bus write also returns its opcode PC in bits 32–47.
    TRACE_LINK_BUDGET_SHIFT = 15,
    TRACE_LINK_COUNT_MASK = (1u << TRACE_LINK_BUDGET_SHIFT) - 1,
    TRACE_WRITE_PC_SHIFT = 32,
    TRACE_BRANCH_PC_SHIFT = 48,
    LOOP_BUDGET_MAX = UINT16_MAX,
    LOOP_BUS_SHIFT = 16,
    LOOP_TARGET_SHIFT = 32,
    LOOP_COUNT_SHIFT = 48
};

#define BLOCK_CHECKS_WRITES UINT32_C(0x80000000)
#define BLOCK_HAS_WRITE_MASK UINT32_C(0x40000000)
#define BLOCK_DMC_FIXED_LENGTH UINT32_C(0x20000000)

typedef uint64_t (*TraceEntry)(Nes *);
typedef struct {
    uint64_t result, resume;
} RomTraceResult;
typedef RomTraceResult (*RomTraceEntry)(Nes *, uint64_t budget);
typedef uint64_t (*LoopTraceEntry)(Nes *, unsigned);

typedef struct {
    uintptr_t address;
    uint32_t arena_word;
    uint8_t bank, hits, clocks, bytes;
} CpuTrace;

typedef struct {
    uint32_t key;
    // Arena index, maximum cycles, and write-mask flag, or a hit count for cold entries
    uint32_t layout;
} BlockTrace;

enum {
    RAM_BLOCKS = 128, RAM_REUSE_BLOCKS = 16,
    RAM_BYTES = 64, RAM_WORDS = RAM_BYTES / sizeof(uint64_t),
    RAM_MAX_WARM_HITS = 4 * TRACE_WARM_HITS
};

typedef struct {
    uint16_t pc;
    uint8_t opcode, hits, limit;
} RamWarmup;

typedef struct {
    uint64_t bytes[RAM_WORDS];
#if defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint64_t guarded;
#else
    uint64_t mask[RAM_WORDS];
#endif
    uint32_t word;
    uint16_t pc;
    uint8_t clocks, size;
    RamWarmup warmup;
} RamBlock;

typedef struct {
    RamBlock active[RAM_BLOCKS], reusable[RAM_REUSE_BLOCKS];
} RamTraceCache;

enum {
    PENDING_LINK_BUCKETS = 512,
    PENDING_LINK_WORDS = 5, // Use the destination prologue for missing registers
    LINK_WORD_BITS = 19,
    LINK_WORD_MASK = (1u << LINK_WORD_BITS) - 1
};
typedef struct {
    uint32_t key, location; // Arena word below the instruction count
    uint8_t clocks, registers;
} PendingLink;
enum { LINK_HAS_EXTRA_CYCLES = 1u << 7 };
_Static_assert(sizeof(PendingLink) == 12, "Pending native links use 12 bytes");
_Static_assert(TRACE_ARENA_SIZE / sizeof(uint32_t) == 1u << LINK_WORD_BITS,
               "Pending links pack their arena word and instruction count");

// FDS loops run from RAM. A native loop records every code byte it was
// compiled from, and runs only while they are unchanged.
typedef struct {
    uint8_t length, exits;
    uint8_t bytes[CPU_SHORT_LOOP_BACK + 3];
    int16_t exit_offset[TRACE_MAX_EXITS];
    uint8_t exit_bus[TRACE_MAX_EXITS];
} LoopGuard;

struct CpuTraceCache {
    uint32_t *arena;
    unsigned used, remaining_compiles;
    uint32_t segment_generations;
    unsigned segment_end;
    BlockTrace blocks[TRACE_SLOTS];
    CpuTrace loops[LOOP_TRACE_SLOTS];
    BlockTrace reusable_blocks[TRACE_REUSE_SLOTS];
    RamTraceCache *ram_blocks;
    LoopGuard *loop_guards; // Allocated for programs in writable memory
    // FDS: one bit for each byte of PRG RAM that a ROM-style trace depends on
    uint8_t *code_map;
    uint8_t *volatile_map; // FDS: code bytes the program has rewritten
    const uint8_t *code_base;
    uint32_t pending_heads[PENDING_LINK_BUCKETS];
    bool record_bus_writes, arena_wrapped;
    unsigned frame_arena_start, sweep_index;
#ifdef _WIN32
    uint32_t *write_arena;
#endif
};

_Static_assert(sizeof(TraceEntry) == sizeof(void *), "Native function pointer size");
_Static_assert(sizeof(BlockTrace) == 8 && offsetof(BlockTrace, layout) == 4 &&
               offsetof(CpuTraceCache, blocks) % 8 == 0, "Native lookups load each ROM trace slot together");
_Static_assert(LOOP_TRACE_SLOTS <= UINT8_MAX + 1, "Loop slot fits in its decoded cache entry");
_Static_assert(CPU_SHORT_LOOP_BACK * 8 + 3 <= UINT8_MAX, "Loop cycle bound fits in the cache");
_Static_assert(RAM_TRACE_MAX_CYCLES + 8 < TRACE_CYCLE_MASK, "Cycle count fits the trace result");
_Static_assert(RAM_BYTES <= 64 && RAM_BYTES % sizeof(uint64_t) == 0, "RAM trace guards fit the masks");
_Static_assert(TRACE_MACHINE_WORDS <= 8192, "Trace fits a forward TBZ/TBNZ displacement");
_Static_assert(offsetof(Nes, ram) < 4096, "RAM base fits an immediate");
_Static_assert(offsetof(Nes, pad_strobe) % 2 == 0 &&
               offsetof(Nes, pad_strobe_write) == offsetof(Nes, pad_strobe) + 1,
               "Native controller reads load both strobe flags together");
_Static_assert((offsetof(Nes, cart) + offsetof(Cartridge, prg_pages)) % 8 == 0,
               "PRG page table is aligned");

#ifdef _WIN32
static void allocate_trace_arena(CpuTraceCache *cache)
{
    // Two views of the same pages keep writes separate from execution
    HANDLE mapping = CreateFileMappingW(INVALID_HANDLE_VALUE, NULL, PAGE_EXECUTE_READWRITE,
                                         0, TRACE_ARENA_SIZE, NULL);

    if (!mapping)
        return;
    uint32_t *write_arena = MapViewOfFile(mapping, FILE_MAP_WRITE, 0, 0, TRACE_ARENA_SIZE);

    if (write_arena) {
        cache->arena = MapViewOfFile(mapping, FILE_MAP_READ | FILE_MAP_EXECUTE, 0, 0, TRACE_ARENA_SIZE);

        if (cache->arena)
            cache->write_arena = write_arena;
        else
            UnmapViewOfFile(write_arena);
    }
    CloseHandle(mapping);
}

static inline void sync_trace_code(const CpuTraceCache *cache, uint32_t *write_address, size_t bytes)
{
    const uint32_t *entry = cache->arena + (write_address - cache->write_arena);

    FlushInstructionCache(GetCurrentProcess(), entry, bytes);
}

#define trace_write_arena(cache) ((cache)->write_arena)
#define trace_write_protect(enabled) ((void)(enabled))
#else
#define trace_write_arena(cache) ((cache)->arena)
#define trace_write_protect(enabled) pthread_jit_write_protect_np(enabled)
#define sync_trace_code(cache, address, bytes) sys_icache_invalidate(address, bytes)
#endif

#ifndef SCHNES_CPU_DEBUG
void cpu_discard_traces(Cartridge *cart)
{
    CpuTraceCache *cache = cart->trace_cache;

    if (cache) {
#ifdef _WIN32
        UnmapViewOfFile(cache->arena);
        UnmapViewOfFile(cache->write_arena);
#else
        munmap(cache->arena, TRACE_ARENA_SIZE);
#endif
        free(cache->ram_blocks);
        free(cache->loop_guards);
        free(cache->code_map);
        free(cache->volatile_map);
        if (cart_is_fds(cart))
            cart->ext->fds->code_map = NULL;
        free(cache);
        cart->trace_cache = NULL;
    }
}
#endif

enum {
    BLOCK_ENTRY_BITS = 19, BLOCK_ENTRY_MASK = (1u << BLOCK_ENTRY_BITS) - 1,
    BLOCK_CYCLES_MASK = 127
};
_Static_assert((TRACE_SEGMENT_SIZE & (TRACE_SEGMENT_SIZE - 1)) == 0 && TRACE_SEGMENT_SIZE >= TRACE_RESERVE,
               "Trace segments must be powers of two and fit the largest trace");
_Static_assert(TRACE_SLOTS % TRACE_SWEEP_SLOTS == 0, "Trace sweep divides the primary table");
_Static_assert(TRACE_SEGMENTS <= 32, "Each trace segment needs one generation bit");

static inline bool in_trace_segment(unsigned word, unsigned first)
{
    return word - first < TRACE_SEGMENT_SIZE / sizeof(uint32_t);
}

static void retire_rom_slots(BlockTrace *blocks, unsigned count, unsigned first)
{
    for (unsigned i = 0; i < count; ++i) {
        unsigned layout = blocks[i].layout;

        if ((layout >> BLOCK_ENTRY_BITS) && in_trace_segment(layout & BLOCK_ENTRY_MASK, first))
            blocks[i].layout = 0;
    }
}

static bool prepare_trace_segment(CpuTraceCache *cache)
{
    if (cache->used <= cache->segment_end - TRACE_RESERVE)
        return true;

    unsigned start = cache->segment_end & (TRACE_ARENA_SIZE - 1);
    unsigned first = start / sizeof(uint32_t);

    cache->arena_wrapped |= !start;
    if (cache->arena_wrapped) {
        // Spread the lookup scan across frames before overwriting the segment
        retire_rom_slots(cache->blocks + cache->sweep_index, TRACE_SWEEP_SLOTS, first);
        cache->sweep_index += TRACE_SWEEP_SLOTS;
        if (cache->sweep_index < TRACE_SLOTS)
            return false;

        retire_rom_slots(cache->reusable_blocks, TRACE_REUSE_SLOTS, first);
        for (unsigned i = 0; i < LOOP_TRACE_SLOTS; ++i) {
            CpuTrace *trace = &cache->loops[i];

            if (in_trace_segment(trace->arena_word - 1, first)) {
                trace->arena_word = 0;
                trace->hits = 1;
            }
        }
        RamTraceCache *ram = cache->ram_blocks;

        if (ram) {
            for (unsigned i = 0; i < RAM_BLOCKS; ++i) {
                if (in_trace_segment(ram->active[i].word - 1, first)) {
                    ram->active[i].word = 0;
                    ram->active[i].warmup = (RamWarmup){0};
                }
            }
            for (unsigned i = 0; i < RAM_REUSE_BLOCKS; ++i)
                if (in_trace_segment(ram->reusable[i].word - 1, first))
                    ram->reusable[i].word = 0;
        }
        cache->sweep_index = 0;
    }
    // Unresolved stubs are tracked only for the active segment
    memset(cache->pending_heads, 0, sizeof cache->pending_heads);
    cache->used = start;
    cache->segment_end = start + TRACE_SEGMENT_SIZE;
    cache->segment_generations ^= 1u << (start / TRACE_SEGMENT_SIZE);
    return true;
}

static CpuTraceCache *begin_trace_frame(Cartridge *cart, bool ultrafast)
{
    CpuTraceCache *cache = cart->trace_cache;

    if (!cache) {
        if (cart->traces_unavailable)
            return NULL;
        if (cart->prg_size > TRACE_KEY_ROM_LIMIT) {
            cart->traces_unavailable = true;
            return NULL;
        }
        cache = calloc(1, sizeof *cache);
#ifdef _WIN32
        if (cache)
            allocate_trace_arena(cache);
#else
        if (cache && pthread_jit_write_protect_supported_np()) {
            void *memory = mmap(NULL, TRACE_ARENA_SIZE, PROT_READ | PROT_WRITE | PROT_EXEC,
                                MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);

            if (memory != MAP_FAILED)
                cache->arena = memory;
        }
#endif
        if (!cache || !cache->arena) {
            free(cache);
            cart->traces_unavailable = true;
            return NULL;
        }
        cache->segment_end = TRACE_SEGMENT_SIZE;
        cart->trace_cache = cache;
    }
    cache->remaining_compiles = prepare_trace_segment(cache) ? TRACE_COMPILES_PER_FRAME : 0;
    cache->frame_arena_start = cache->used;
    cache->record_bus_writes = !ultrafast;
    return cache;
}

static inline const Fds *borrowed_fds_traces(const Cartridge *cart, const CpuTraceCache *cache)
{
    return cache && cache->code_base && cache->code_base != cart->ram ? cart->ext->fds : NULL;
}

static bool begin_fds_trace_frame(Cartridge *cart, bool ultrafast)
{
    // Runahead borrows published traces and preserves any code-write bailout
    if (borrowed_fds_traces(cart, cart->trace_cache))
        return true;
    CpuTraceCache *cache = begin_trace_frame(cart, ultrafast);

    if (!cache)
        return false;
    if (!cache->loop_guards)
        cache->loop_guards = calloc(LOOP_TRACE_SLOTS, sizeof *cache->loop_guards);
    if (!cache->code_map) {
        cache->code_map = calloc(FDS_CODE_MAP_BYTES, 1);
        cache->volatile_map = calloc(FDS_CODE_MAP_BYTES, 1);
        cache->code_base = cart->ram;
    }
    if (!cache->loop_guards || !cache->code_map || !cache->volatile_map) {
        cpu_discard_traces(cart);
        cart->traces_unavailable = true;
        return false;
    }
    cart->ext->fds->code_map = cache->code_map;
    return true;
}

typedef struct {
    uint32_t words[TRACE_MACHINE_WORDS];
    uint64_t bus_writes[2]; // ROM traces record every possible write cycle
    unsigned size;
    unsigned clocks, extra, dirty, ram_operand, prg_window, cart_ram_window;
    unsigned nz, bus, bus_imm, inputs;
    unsigned carry_end, index_start, index_end, wram_check, wram_end;
    int stack_delta;
    bool stack_replaced;
    uint8_t stack_pending;
    unsigned link_count, pending_count, stack_wrap_count;

    struct { uint16_t check, resume, address; } stack_wraps[(TRACE_MAX_CYCLES + 5) / 6];
    uint32_t publish_key, publish_layout;
    struct { unsigned at, word; } links[TRACE_MAX_EXITS + 1];
    PendingLink pending[TRACE_MAX_EXITS + 1];
    bool uses_stack, loop_body, carry_cached, carry_value, write_overflow;
    bool indirect_wram, indirect_wram_enable;
    bool defer_carry, carry_pending, cart_ram_operand, record_bus_writes;
    bool cart_ram_rom; // $6000 reads use the FME-7 ROM page rather than cartridge RAM
    uint8_t *code_map; // FDS: marks the RAM bytes this trace depends on
    const uint8_t *volatile_map;
    const uint8_t *code_base;
    bool keep_add_sum, wide_accumulator, pad_operand;
    uint32_t self_key; // Key of the block being compiled, which may link to itself
} TraceEmitter;

_Static_assert(sizeof(((TraceEmitter *)0)->bus_writes) == DMC_WRITE_MASK_BYTES,
               "The ROM write mask occupies four arena words");

enum { REG_A=4, REG_X=5, REG_Y=6, REG_P=7, REG_NZ=8, REG_VALUE=9, REG_SUM=10, REG_ADDRESS=11,
       REG_CYCLES=12, REG_TEMP=13, REG_TEMP2=14, REG_RAM=2, REG_ROM=3,
       REG_S=15, REG_LOOP_CARRY=15, REG_TRACE_CARRY=16, REG_CART_RAM=17, REG_NONE=31 };
enum { ARM_EQ=0, ARM_HS=2, ARM_VS=6, ARM_LT=11, ARM_GT=12 };
enum { NZ_CONSTANT = 0x100, NZ_NEGATIVE_CLEAR = 0x200, NZ_ZERO_CLEAR = 0x400 };
enum { ARM_AND=0x0a000000, ARM_EOR=0x4a000000, ARM_ORR=0x2a000000,
       ARM_ADD=0x0b000000, ARM_ADDS=0x2b000000, ARM_SUBS=0x6b000000 };

static inline void init_trace_emitter(TraceEmitter *e)
{
    // Initialize only bookkeeping because emission writes each word before use
    e->size = e->clocks = e->extra = e->dirty = e->bus_imm = 0;
    e->bus_writes[0] = e->bus_writes[1] = 0;
    e->nz = e->bus = REG_NONE;
    e->ram_operand = e->prg_window = e->cart_ram_window = e->inputs = 0;
    e->carry_end = e->index_start = e->index_end = e->wram_check = e->wram_end = 0;
    e->uses_stack = e->stack_replaced = false;
    e->stack_delta = 0;
    e->stack_pending = 0;
    e->link_count = e->pending_count = e->stack_wrap_count = e->publish_layout = 0;
    e->loop_body = e->carry_cached = e->carry_value = false;
    e->write_overflow = true;
    e->indirect_wram = e->indirect_wram_enable = false;
    e->defer_carry = e->carry_pending = e->cart_ram_operand = e->record_bus_writes = false;
    e->cart_ram_rom = false;
    e->code_map = NULL;
    e->volatile_map = NULL;
    e->code_base = NULL;
    e->keep_add_sum = e->wide_accumulator = e->pad_operand = false;
    e->self_key = 0;
}

static inline void emit(TraceEmitter *e, uint32_t word)
{
    e->words[e->size++] = word;
}
static inline void emit_reg(TraceEmitter *e, uint32_t insn, unsigned rd, unsigned rn, unsigned rm)
{
    emit(e, insn | (rm << 16) | (rn << 5) | rd);
}
static inline void emit_imm(TraceEmitter *e, uint32_t insn, unsigned rd, unsigned rn, unsigned imm)
{
    emit(e, insn | (imm << 10) | (rn << 5) | rd);
}
static inline void emit_move(TraceEmitter *e, unsigned rd, unsigned rn)
{
    if (rd != rn)
        emit_reg(e, ARM_ORR, rd, 31, rn);
}
static inline void emit_constant(TraceEmitter *e, unsigned rd, unsigned imm)
{
    emit(e, 0x52800000 | (imm << 5) | rd);
}
static inline void emit_extract(TraceEmitter *e, unsigned rd, unsigned rn, unsigned lsb, unsigned width)
{
    emit(e, 0x53000000 | (lsb << 16) | ((lsb + width - 1) << 10) | (rn << 5) | rd);
}
static inline void emit_insert(TraceEmitter *e, unsigned rd, unsigned rn, unsigned lsb, unsigned width)
{
    emit(e, 0x33000000 | (((32 - lsb) & 31) << 16) | ((width - 1) << 10) | (rn << 5) | rd);
}
static inline void emit_set_condition(TraceEmitter *e, unsigned rd, unsigned cond)
{
    emit(e, 0x1a9f07e0 | ((cond ^ 1) << 12) | rd);
}
static inline unsigned cpu_register_offset(unsigned reg)
{
    if (reg == REG_S)
        return offsetof(Nes, cpu) + offsetof(Cpu, s);
    return offsetof(Nes, cpu) + offsetof(Cpu, a) + (reg - REG_A) + (reg == REG_P);
}

static unsigned emit_status_flags(TraceEmitter *e, unsigned dirty, unsigned nz);
static inline void remember_nz(TraceEmitter *e, unsigned dest)
{
    e->nz = dest;

    if (dest >= REG_A && dest <= REG_Y)
        e->dirty |= 1u << dest;
}

static inline void remember_constant_nz(TraceEmitter *e, unsigned dest, unsigned value)
{
    remember_nz(e, dest);
    e->nz = NZ_CONSTANT | (value & CPU_NEGATIVE) | (value == 0 ? CPU_ZERO : 0);
}

static inline void emit_status_bit(TraceEmitter *e, unsigned bit, bool set)
{
    // ORR with a one-bit mask or AND with its complement
    uint32_t word = set ? 0x32000000 | (((32 - bit) & 31) << 16)
                        : 0x12007800 | ((31 - bit) << 16);

    emit(e, word | (REG_P << 5) | REG_P);
}

static inline void emit_index_change(TraceEmitter *e, Operation op, unsigned count)
{
    unsigned reg = index_is_y(op) ? REG_Y : REG_X;
    int step = index_delta(op, count);
    uint32_t regs = (reg << 5) | reg;

    if (e->size != e->index_end)
        e->index_start = e->size;
    // This span contains only index updates. X and Y commute, and the
    // last guest update still supplies N/Z.
    for (unsigned at = e->index_start; at < e->size; at += 2) {
        uint32_t word = e->words[at];

        if ((word & 31) != reg)
            continue;
        int delta = (word >> 10) & 0xfff;

        if (word & UINT32_C(0x40000000))
            delta = -delta;
        delta += step;
        e->words[at] = (delta < 0 ? 0x51000000 : 0x11000000) |
                       ((unsigned)(delta < 0 ? -delta : delta) << 10) | regs;
        remember_nz(e, reg);
        return;
    }
    emit(e, (step < 0 ? 0x51000000 : 0x11000000) | (count << 10) | regs);
    emit_extract(e, reg, reg, 0, 8);
    e->index_end = e->size;
    remember_nz(e, reg);
}

static inline void emit_carry(TraceEmitter *e, unsigned source)
{
    if (e->carry_cached) {
        emit_move(e, REG_LOOP_CARRY, source);
    } else if (e->defer_carry) {
        emit_move(e, REG_TRACE_CARRY, source);
        e->carry_pending = true;
    } else {
        emit_insert(e, REG_P, source, 0, 1);
    }
    e->dirty |= 1u << REG_P;
}

static inline void note_bus_writes(TraceEmitter *e, unsigned first, unsigned count)
{
    if (!e->record_bus_writes)
        return;
    // Earlier page-crossing reads can move the write to any cycle in this range
    unsigned end = first + count + e->extra;

    for (unsigned cycle = first; cycle < end && cycle < 128; ++cycle)
        e->bus_writes[cycle >> 6] |= UINT64_C(1) << (cycle & 63);
}

// Both sides of a mapped ROM window are plain reads. Select the actual
// 8 KiB page at runtime, including the 16-bit wrap into internal RAM.
static void emit_rom_window_read(TraceEmitter *e, unsigned address, unsigned index, unsigned load)
{
    unsigned slot = (address >> 13) - 4;
    unsigned table = offsetof(Nes, cart) + offsetof(Cartridge, prg_pages) + slot * sizeof(void *);

    emit_constant(e, REG_ADDRESS, address & 8191);
    emit_reg(e, ARM_ADD, REG_ADDRESS, REG_ADDRESS, index);
    unsigned cross = e->extra ? REG_TEMP : REG_CYCLES;

    emit_extract(e, cross, REG_ADDRESS, 13, 1);
    if (e->extra++)
        emit_reg(e, ARM_ADD, REG_CYCLES, REG_CYCLES, cross);
    if (slot < 3) {
        emit_imm(e, 0x91000000, REG_TEMP2, 0, table);
        emit_reg(e, 0xf8605800, REG_ROM, REG_TEMP2, cross);
    } else {
        emit_imm(e, 0xf9400000, REG_ROM, 0, table / 8);
        emit_imm(e, 0x71000000, REG_NONE, cross, 0);
        emit(e, 0x9a800000 | (REG_RAM << 16) | (REG_ROM << 5) | REG_ROM); /* csel xROM, xROM, xRAM, eq */
    }
    emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 13);
    emit_reg(e, 0x38604800, load, REG_ROM, REG_ADDRESS);
    e->prg_window = 0;
    e->clocks += 4;
}

// Record bytes of FDS PRG RAM that the generated code was built from.
// Whether any of these bytes of FDS PRG RAM was rewritten after a trace used it.
static bool volatile_code(const TraceEmitter *e, const uint8_t *bytes, unsigned count)
{
    if (!e->volatile_map)
        return false;
    for (unsigned i = 0; i < count; ++i) {
        uintptr_t offset = (uintptr_t)(bytes + i) - (uintptr_t)e->code_base;

        if (offset < 0x8000 && ((e->volatile_map[offset >> 3] >> (offset & 7)) & 1))
            return true;
    }
    return false;
}

static void note_code(TraceEmitter *e, const uint8_t *bytes, unsigned count)
{
    if (!e->code_map)
        return;
    for (unsigned i = 0; i < count; ++i) {
        uintptr_t offset = (uintptr_t)(bytes + i) - (uintptr_t)e->code_base;

        if (offset < 0x8000)
            e->code_map[offset >> 3] |= (uint8_t)(1u << (offset & 7));
    }
}

// Load the FME-7 ROM page at $6000, offset to the 4 KiB half that `window` selects
static void emit_wram_rom_base(TraceEmitter *e, unsigned window)
{
    emit_imm(e, 0xf9400000, REG_CART_RAM, 0, (offsetof(Nes, cart) + offsetof(Cartridge, ext)) / 8);
    emit_imm(e, 0xf9400000, REG_CART_RAM, REG_CART_RAM, offsetof(MapperExt, wram_rom) / 8);
    if (window & 1)
        emit_imm(e, 0x91400000, REG_CART_RAM, REG_CART_RAM, 1); /* shifted by 12 */
    e->cart_ram_window = window;
}

static bool emit_memory(TraceEmitter *e, const uint8_t *code, Mode mode, Access access, unsigned load)
{
    unsigned address = e->ram_operand ? 0 : code[1];
    unsigned index = mode == ABX || mode == ZPX ? REG_X : REG_Y;
    bool indexed = mode == ZPX || mode == ZPY || mode == ABX || mode == ABY;
    unsigned base_register = REG_RAM;
    unsigned cycles;

    if (mode == ZP || mode == ZPX || mode == ZPY) {
        cycles = indexed ? 4 : 3;

        if (indexed) {
            emit_imm(e, 0x11000000, REG_ADDRESS, index, address);
            emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 8);
        }
    } else if (mode == ABS || mode == ABX || mode == ABY) {
        address |= code[2] << 8;

        if (e->pad_operand) {
            // A quiet controller read returns the shifted bit over the operand's
            // high byte ($40) and refills the shift register with ones.
            unsigned field = offsetof(Nes, pad_shift);
            unsigned source = 0;

            if (indexed) {
                emit_reg(e, 0x8b204000, REG_ADDRESS, 0, index); /* add xAddr, x0, wIndex, uxtw */
                source = REG_ADDRESS;
            } else {
                field += address & 1;
            }
            emit_imm(e, 0x39400000, REG_TEMP, source, field);
            emit(e, 0x12000000 | (REG_TEMP << 5) | load); /* and wLoad, wTemp, #1 */
            emit_imm(e, 0x39000000, load, 0, offsetof(Nes, pad_bit));
            emit(e, 0x32000000 | (26u << 16) | (load << 5) | load); /* orr wLoad, wLoad, #$40 */
            emit_extract(e, REG_TEMP, REG_TEMP, 1, 7);
            emit(e, 0x32000000 | (25u << 16) | (REG_TEMP << 5) | REG_TEMP); /* orr #$80 */
            emit_imm(e, 0x39000000, REG_TEMP, source, field);
            // Leave DMC fetches that clock the port again on the C path
            note_bus_writes(e, 0, e->clocks + 4);
            e->clocks += 4;
            return true;
        }
        bool ram = address < 0x2000 && (!indexed || address <= 0x1f00);
        bool rom = address >= 0x8000 && access == READ;

        if (rom && indexed && (address & 8191) > 7936) {
            emit_rom_window_read(e, address, index, load);
            return true;
        }
        if (!ram && !rom && !e->cart_ram_operand)
            return false;
        cycles = indexed && access != READ ? 5 : 4;

        // The WRAM guard excludes a page crossing at the top of the RAM window
        bool wram_tail = e->cart_ram_operand && address >= 0x7f00;

        if (indexed && access == READ && (address & 255) && !wram_tail) {
            emit_imm(e, 0x71000000, 31, index, 256 - (address & 255));
            // The compare's carry is set exactly when the index crosses a page
            unsigned previous = e->extra++ ? REG_CYCLES : REG_NONE;

            emit_reg(e, 0x1a000000, REG_CYCLES, previous, REG_NONE); /* adc */
        }
        if (rom || e->cart_ram_operand) {
            // Reuse the base for the same 4 KiB half of a cartridge window
            unsigned window = address >> 12;
            unsigned *cached_window = rom ? &e->prg_window : &e->cart_ram_window;

            base_register = rom ? REG_ROM : REG_CART_RAM;

            if (*cached_window != window && !rom && e->cart_ram_rom) {
                emit_wram_rom_base(e, window);
            } else if (*cached_window != window) {
                unsigned field = offsetof(Nes, cart);

                field += rom ? offsetof(Cartridge, prg_pages) + ((window >> 1) - 4) * sizeof(void *)
                             : offsetof(Cartridge, ram);
                emit_imm(e, 0xf9400000, base_register, 0, field / 8);
                if (window & 1)
                    emit_imm(e, 0x91400000, base_register, base_register, 1); /* shifted by 12 */
                *cached_window = window;
            }
            address &= 4095;
        } else {
            address &= 2047;
        }
        if (e->ram_operand) {
            emit_imm(e, 0x39400000, REG_ADDRESS, REG_RAM, e->ram_operand);
            if (address)
                emit_imm(e, 0x11000000, REG_ADDRESS, REG_ADDRESS, address);
            indexed = true;
        } else if (indexed) {
            emit_imm(e, 0x11000000, REG_ADDRESS, index, address);
            if (ram && address >= 1793)
                emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 11);
        }
    } else {
        return false;
    }
    if (access != WRITE) {
        if (indexed)
            emit_reg(e, 0x38604800, load, base_register, REG_ADDRESS);
        else
            emit_imm(e, 0x39400000, load, base_register, address);
    }
    // Keep the store encoding out of the opcode dispatcher. For a store, the
    // caller substitutes the source register into the low five bits.
    if (access != READ) {
        if (indexed)
            emit_reg(e, 0x38204800, REG_VALUE, base_register, REG_ADDRESS);
        else
            emit_imm(e, 0x39000000, REG_VALUE, base_register, address);
    }
    e->clocks += cycles + (access == MODIFY ? 2 : 0);

    if (access != READ) {
        unsigned writes = access == MODIFY ? 2 : 1;
        note_bus_writes(e, e->clocks - writes, writes);
    }
    return true;
}

// These forward guard branches have at least 14 immediate bits. Until the
// exit stub is emitted, keep the previous guard's word index in that field.
enum { GUARD_LINK_MASK = (TRACE_MACHINE_WORDS - 1) << 5 };
_Static_assert(!(TRACE_MACHINE_WORDS & (TRACE_MACHINE_WORDS - 1)), "Guard links use a word-index mask");

static void emit_guard(TraceEmitter *e, unsigned *guard, uint32_t branch)
{
    unsigned previous = *guard;

    *guard = e->size;
    emit(e, branch | (previous << 5));
}

static void patch_guards(TraceEmitter *e, unsigned guard)
{
    while (guard) {
        uint32_t branch = e->words[guard];

        e->words[guard] = (branch & ~GUARD_LINK_MASK) | ((e->size - guard) << 5);
        guard = (branch & GUARD_LINK_MASK) >> 5;
    }
}

static void emit_wram_refill_guard(TraceEmitter *e, unsigned refill, unsigned deadline, unsigned *guard)
{
    unsigned field = offsetof(Nes, apu) + offsetof(Apu, stream_read_cycle);

    emit_imm(e, 0xf9400000, refill, 0, field / 8);
    emit_imm(e, 0xf9400000, deadline, 0, offsetof(Nes, bus_deadline) / 8);
    // The packed link budget is relative to the entry of this block
    emit_reg(e, 0xcb400000 | (TRACE_LINK_BUDGET_SHIFT << 10), deadline, deadline, 1);
    unsigned previous = e->wram_check;

    e->wram_check = e->size;
    // Replace the immediate with the last possible WRAM access on publication
    emit_imm(e, 0x91000000, deadline, deadline, previous);
    emit_reg(e, 0xeb000000, REG_NONE, refill, deadline);
    emit_guard(e, guard, 0x54000003); /* b.lo */
}

// Call a C bus handler from generated code. Every host register the trace can
// hold live (x0-x17, x30) is preserved around the call.
enum { CALLOUT_FRAME = 160 };
static void emit_save_trace_registers(TraceEmitter *e)
{
    emit(e, 0xd1000000 | (CALLOUT_FRAME << 10) | (31 << 5) | 31); /* sub sp, sp, #frame */
    for (unsigned reg = 0; reg < 18; reg += 2)
        emit(e, 0xa9000000 | ((reg & 0x7f) << 15) | ((reg + 1) << 10) | (31 << 5) | reg); /* stp */
    emit(e, 0xf9000000 | (18u << 10) | (31 << 5) | 30); /* str x30, [sp, #144] */
}

static void emit_restore_trace_registers(TraceEmitter *e)
{
    for (unsigned reg = 0; reg < 18; reg += 2)
        emit(e, 0xa9400000 | ((reg & 0x7f) << 15) | ((reg + 1) << 10) | (31 << 5) | reg); /* ldp */
    emit(e, 0xf9400000 | (18u << 10) | (31 << 5) | 30);
    emit(e, 0x91000000 | (CALLOUT_FRAME << 10) | (31 << 5) | 31); /* add sp, sp, #frame */
}

static void emit_call(TraceEmitter *e, uintptr_t function)
{
    emit(e, 0xd2800000 | ((unsigned)(function & 0xffff) << 5) | 16); /* movz x16 */
    for (unsigned hw = 1; hw < 4; ++hw)
        emit(e, 0xf2800000 | (hw << 21) | ((unsigned)((function >> (16 * hw)) & 0xffff) << 5) | 16);
    emit(e, 0xd63f0200); /* blr x16 */
}

// Perform a deadline-safe I/O store at its exact cycle without leaving the trace
static void emit_quiet_store(TraceEmitter *e, unsigned address, unsigned source, unsigned *guard,
                             bool (*handler)(Nes *, uint16_t, uint8_t, uint64_t))
{
    emit_save_trace_registers(e);
    // The packed link budget is relative to the entry of this block
    emit_imm(e, 0xf9400000, 3, 0, offsetof(Nes, bus_deadline) / 8);
    emit_reg(e, 0xcb400000 | (TRACE_LINK_BUDGET_SHIFT << 10), 3, 3, 1);
    emit_imm(e, 0x91000000, 3, 3, e->clocks + 3);
    if (e->extra)
        emit_reg(e, 0x8b000000, 3, 3, REG_CYCLES);
    emit_extract(e, 2, source, 0, 8);
    emit_constant(e, 1, address);
    emit_call(e, (uintptr_t)handler);
    emit_imm(e, 0x71000000, REG_NONE, 0, 0); /* cmp w0, #0 */
    emit_restore_trace_registers(e);
    emit_guard(e, guard, 0x54000000 | ARM_EQ);
}

static void emit_fds_ram_store(TraceEmitter *e, unsigned address, unsigned source, unsigned *guard)
{
    unsigned offset = address - 0x6000;
    unsigned region = offset >> FDS_LOOP_REGION_SHIFT;
    unsigned budget = 1;

    if (e->extra) {
        budget = REG_TEMP2;
        emit_reg(e, 0x4b000000 | (TRACE_LINK_BUDGET_SHIFT << 10), budget, 1, REG_CYCLES);
    }
    // Keep the same spare cycle as the quiet C store, including after a trace link
    emit_imm(e, 0x71400000, REG_NONE, budget, (e->clocks + 5) << (TRACE_LINK_BUDGET_SHIFT - 12));
    emit_guard(e, guard, 0x54000000 | ARM_LT);

    const unsigned cart = offsetof(Nes, cart);

    emit_imm(e, 0xf9400000, REG_TEMP, 0, (cart + offsetof(Cartridge, ext)) / 8);
    emit_imm(e, 0xf9400000, REG_TEMP, REG_TEMP, offsetof(MapperExt, fds) / 8);
    // Both dependency maps can gain entries after this block has been published
    emit_imm(e, 0x39400000, REG_TEMP2, REG_TEMP, offsetof(Fds, loop_map) + (region >> 3));
    emit_guard(e, guard, 0x37000000 | ((region & 7) << 19) | REG_TEMP2); /* tbnz */
    emit_imm(e, 0xf9400000, REG_TEMP2, REG_TEMP, offsetof(Fds, code_map) / 8);
    emit_imm(e, 0x39400000, REG_TEMP2, REG_TEMP2, offset >> 3);
    emit_guard(e, guard, 0x37000000 | ((offset & 7) << 19) | REG_TEMP2); /* tbnz */

    unsigned window = address >> 12;

    if (e->cart_ram_window != window) {
        emit_imm(e, 0xf9400000, REG_CART_RAM, 0, (cart + offsetof(Cartridge, ram)) / 8);
        if (offset >= 4096)
            emit_imm(e, 0x91400000, REG_CART_RAM, REG_CART_RAM, offset >> 12); /* shifted by 12 */
        e->cart_ram_window = window;
    }
    emit_imm(e, 0x39000000, source, REG_CART_RAM, offset & 4095);
}

static void emit_indirect_read(TraceEmitter *e, unsigned zp, unsigned load, unsigned *guard)
{
    // The indirect path uses REG_ROM for its runtime address calculation
    e->prg_window = 0;

    if (zp < 255) {
        emit_imm(e, 0x78400000, REG_TEMP, REG_RAM, zp << 2); /* ldurh w13, [x2, #zp] */
    } else {
        // The high byte wraps within zero page
        emit_imm(e, 0x39400000, REG_TEMP, REG_RAM, 255);
        emit_imm(e, 0x39400000, REG_TEMP2, REG_RAM, 0);
        emit_insert(e, REG_TEMP, REG_TEMP2, 8, 8);
    }
    emit_reg(e, ARM_ADD, REG_ADDRESS, REG_TEMP, REG_Y);
    if (e->indirect_wram) {
        // The sampled pointer suggested cartridge RAM. Verify the base, end
        // address, permissions, and timing again each time the block runs.
        emit_extract(e, REG_ROM, REG_TEMP, 13, 3);
        emit_imm(e, 0x71000000, REG_NONE, REG_ROM, 3);
        emit_guard(e, guard, 0x54000001); /* b.ne */
        emit_guard(e, guard, 0x37780000 | REG_ADDRESS); /* tbnz wAddr, #15 */
        if (e->indirect_wram_enable) {
            unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, mmc3.ram_control);

            emit_imm(e, 0x39400000, REG_TEMP2, 0, field);
            emit_guard(e, guard, 0x36380000 | REG_TEMP2); /* tbz wTemp2, #7 */
        }
        emit_wram_refill_guard(e, REG_TEMP2, REG_ROM, guard);
        emit_reg(e, ARM_EOR, REG_TEMP, REG_TEMP, REG_ADDRESS);
        if (e->extra++) {
            emit_extract(e, REG_TEMP, REG_TEMP, 8, 1);
            emit_reg(e, ARM_ADD, REG_CYCLES, REG_CYCLES, REG_TEMP);
        } else {
            emit_extract(e, REG_CYCLES, REG_TEMP, 8, 1);
        }
        if (e->cart_ram_window != 8) {
            unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, ram);

            emit_imm(e, 0xf9400000, REG_CART_RAM, 0, field / 8);
            e->cart_ram_window = 8;
        }
        emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 13);
        emit_reg(e, 0x38604800, load, REG_CART_RAM, REG_ADDRESS);
        e->clocks += 5;
        e->wram_end = e->clocks + e->extra;
        return;
    }
    // Banks 0 and 4–7 are internal RAM and mapped ROM. Return before
    // reading the addressed byte in other banks.
    emit_extract(e, REG_ROM, REG_ADDRESS, 13, 3);
    emit_imm(e, 0x71000000, 31, REG_ROM, 4);
    emit(e, 0x54000000 | (2 << 5) | ARM_HS);
    emit_guard(e, guard, 0x35000000 | REG_ROM); /* cbnz wROM, exit */

    // Adding an 8-bit index crosses a page exactly when bit 8 changes
    emit_reg(e, ARM_EOR, REG_TEMP, REG_TEMP, REG_ADDRESS);
    if (e->extra++) {
        emit_extract(e, REG_TEMP, REG_TEMP, 8, 1);
        emit_reg(e, ARM_ADD, REG_CYCLES, REG_CYCLES, REG_TEMP);
    } else {
        emit_extract(e, REG_CYCLES, REG_TEMP, 8, 1);
    }

    // The bank comparison still controls the RAM and ROM paths
    unsigned rom = e->size;

    emit(e, 0);
    emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 11);
    emit_reg(e, 0x38604800, load, REG_RAM, REG_ADDRESS);
    unsigned done = e->size;

    emit(e, 0);
    e->words[rom] = 0x54000000 | ((e->size - rom) << 5) | ARM_HS;
    unsigned table = offsetof(Nes, cart) + offsetof(Cartridge, prg_pages) - 4 * sizeof(void *);

    emit_imm(e, 0x91000000, REG_TEMP, 0, table);
    emit_reg(e, 0xf8605800, REG_ROM, REG_TEMP, REG_ROM);
    emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 13);
    emit_reg(e, 0x38604800, load, REG_ROM, REG_ADDRESS);
    e->words[done] = 0x14000000 | (e->size - done);
    e->clocks += 5;
}

// STA (zp),Y into internal RAM, or into cartridge RAM when the sampled pointer
// suggested it. The final store word uses REG_VALUE and is replaced by the caller.
static void emit_indirect_store(TraceEmitter *e, unsigned zp, unsigned *guard)
{
    e->prg_window = 0;

    if (zp < 255) {
        emit_imm(e, 0x78400000, REG_TEMP, REG_RAM, zp << 2); /* ldurh w13, [x2, #zp] */
    } else {
        emit_imm(e, 0x39400000, REG_TEMP, REG_RAM, 255);
        emit_imm(e, 0x39400000, REG_TEMP2, REG_RAM, 0);
        emit_insert(e, REG_TEMP, REG_TEMP2, 8, 8);
    }
    emit_reg(e, ARM_ADD, REG_ADDRESS, REG_TEMP, REG_Y);
    if (e->indirect_wram) {
        // Both the dummy read and the store stay in $6000-$7FFF
        emit_extract(e, REG_ROM, REG_TEMP, 13, 3);
        emit_imm(e, 0x71000000, REG_NONE, REG_ROM, 3);
        emit_guard(e, guard, 0x54000001); /* b.ne */
        emit_guard(e, guard, 0x37780000 | REG_ADDRESS); /* tbnz wAddr, #15 */
        if (e->indirect_wram_enable) {
            unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, mmc3.ram_control);

            emit_imm(e, 0x39400000, REG_TEMP2, 0, field);
            emit_extract(e, REG_TEMP2, REG_TEMP2, 6, 2);
            emit_imm(e, 0x71000000, REG_NONE, REG_TEMP2, 2);
            emit_guard(e, guard, 0x54000001); /* b.ne: enabled and writable */
        }
        emit_wram_refill_guard(e, REG_TEMP2, REG_ROM, guard);
        if (e->cart_ram_window != 8) {
            unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, ram);

            emit_imm(e, 0xf9400000, REG_CART_RAM, 0, field / 8);
            e->cart_ram_window = 8;
        }
        emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 13);
        emit_reg(e, 0x38204800, REG_VALUE, REG_CART_RAM, REG_ADDRESS);
        e->clocks += 6;
        e->wram_end = e->clocks + e->extra;
    } else {
        // The pointer and the final address both lie in internal RAM
        emit_extract(e, REG_ROM, REG_ADDRESS, 13, 4);
        emit_guard(e, guard, 0x35000000 | REG_ROM); /* cbnz */
        emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 11);
        emit_reg(e, 0x38204800, REG_VALUE, REG_RAM, REG_ADDRESS);
        e->clocks += 6;
    }
    note_bus_writes(e, e->clocks - 1, 1);
}

static inline void materialize_stack_pointer(TraceEmitter *e)
{
    if (e->stack_pending) {
        emit_imm(e, 0x11000000, REG_S, REG_S, e->stack_pending);
        emit_extract(e, REG_S, REG_S, 0, 8);
        e->stack_pending = 0;
    }
}

static inline void load_stack_pointer(TraceEmitter *e)
{
    if (!e->uses_stack) {
        emit_imm(e, 0x39400000, REG_S, 0, cpu_register_offset(REG_S));
        e->uses_stack = true;
    }
    materialize_stack_pointer(e);
}

// The guest stack can finish at its entry value even after several writes
static inline void advance_stack_pointer(TraceEmitter *e, int delta)
{
    e->stack_delta += delta;

    if (e->stack_delta || e->stack_replaced)
        e->dirty |= 1u << REG_S;
    else
        e->dirty &= ~(1u << REG_S);
}

static void emit_push(TraceEmitter *e, unsigned reg)
{
    load_stack_pointer(e);
    emit_imm(e, 0x11000000, REG_ADDRESS, REG_S, 256);
    emit_reg(e, 0x38204800, reg, REG_RAM, REG_ADDRESS);
    emit_imm(e, 0x51000000, REG_S, REG_S, 1);
    emit_extract(e, REG_S, REG_S, 0, 8);
    advance_stack_pointer(e, -1);
}

static void emit_push_return(TraceEmitter *e, unsigned address)
{
    load_stack_pointer(e);
    unsigned wrap = e->size;

    emit(e, 0); /* cbz w15, wrap */
    emit_constant(e, REG_VALUE, address);
    emit_imm(e, 0x11000000, REG_ADDRESS, REG_S, 255);
    emit_reg(e, 0x78204800, REG_VALUE, REG_RAM, REG_ADDRESS); /* strh */
    unsigned stub = e->stack_wrap_count++;

    e->stack_wraps[stub].check = wrap;
    e->stack_wraps[stub].resume = e->size;
    e->stack_wraps[stub].address = address;
    e->stack_pending -= 2;
    advance_stack_pointer(e, -2);
}

static void emit_pop_return(TraceEmitter *e)
{
    e->stack_pending += 2;
    advance_stack_pointer(e, 2);
}

// Record guest registers read before the trace has supplied their values
#define TRACE_INPUTS(byte, op, mode) \
    [0x##byte] = \
        (((op) == STA || (op) == ADC || (op) == SBC || (op) == AND || (op) == EOR || \
          (op) == ORA || (op) == BIT || (op) == CMP || (op) == TAX || (op) == TAY || \
          (op) == PHA || (mode) == ACC) << REG_A) | \
        (((op) == STX || (op) == CPX || (op) == INX || (op) == DEX || (op) == TXA || \
          (op) == TXS || (mode) == ABX || (mode) == ZPX) << REG_X) | \
        (((op) == STY || (op) == CPY || (op) == INY || (op) == DEY || (op) == TYA || \
          (mode) == ABY || (mode) == ZPY || (mode) == IZY) << REG_Y),
static const uint8_t trace_inputs[256] = {OPCODES(TRACE_INPUTS)};
#undef TRACE_INPUTS

static bool emit_logic_immediate(TraceEmitter *e, Operation op, unsigned value)
{
    if ((op == AND && !value) || (op == ORA && value == 255)) {
        emit_constant(e, REG_A, value);
        remember_constant_nz(e, REG_A, value);
    } else {
        if (op == AND ? value != 255 : value != 0) {
            unsigned shift = (unsigned)__builtin_ctz(value);
            unsigned mask = value >> shift;

            if (mask & (mask + 1))
                return false;
            unsigned rotation = (32 - shift) & 31;
            unsigned width = 32 - (unsigned)__builtin_clz(mask);
            uint32_t instruction = op == AND ? 0x12000000 : op == ORA ? 0x32000000 : 0x52000000;

            emit(e, instruction | (rotation << 16) | ((width - 1) << 10) | (REG_A << 5) | REG_A);
        }
        e->inputs |= (1u << REG_A) & ~e->dirty;
        remember_nz(e, REG_A);
    }
    e->clocks += 2;
    e->bus = REG_NONE;
    e->bus_imm = value;
    return true;
}

static bool emit_opcode(TraceEmitter *e, const uint8_t *code, Operation op, Mode mode,
                        Access access, unsigned *guard)
{
    // A memory RMW produces both the byte to store and the pending N/Z
    bool modify_memory = access == MODIFY && mode != ACC;
    unsigned load = modify_memory ? REG_NZ : REG_VALUE;

    switch (op) {
    case LDA: load = REG_A; break;
    case LDX: load = REG_X; break;
    case LDY: load = REG_Y; break;
    case CMP: case CPX: case CPY:
    case AND: case EOR: case ORA: case ADC: case SBC:
    case STA: case STX: case STY: case INC: case DEC:
    case ASL: case LSR: case ROL: case ROR:
    case INX: case INY: case DEX: case DEY:
    case TAX: case TAY: case TXA: case TYA:
    case CLC: case SEC: case NOP: case BIT: case CLV: case CLD: case SED:
    case PHA: case PHP: case PLA: case TSX: case TXS:
        break;
    default:
        return false;
    }
    if (mode == IMM && !e->ram_operand && (op == AND || op == ORA || op == EOR) &&
        emit_logic_immediate(e, op, code[1]))
        return true;
    e->inputs |= trace_inputs[code[0]] & ~e->dirty;
    int known_carry = -1;
    uint32_t carry_word = 0;

    // Indirect reads can exit before the arithmetic. Leave their incoming
    // carry in the guest status or the loop's carry register.
    if ((op == ADC || op == SBC) && mode != IZY && e->carry_end && e->size == e->carry_end) {
        known_carry = (int)e->carry_value;
        carry_word = e->words[--e->size];
    }
    uint32_t store = 0;
    unsigned carry = e->carry_cached ? REG_LOOP_CARRY : e->defer_carry ? REG_TRACE_CARRY : REG_TEMP;
    bool compare_immediate = !e->ram_operand && mode == IMM && (op == CMP || op == CPX || op == CPY);

    if (mode == IMM) {
        if (e->ram_operand)
            emit_imm(e, 0x39400000, load, REG_RAM, e->ram_operand);
        else if (!compare_immediate)
            emit_constant(e, load, code[1]);
        e->clocks += 2;
    } else if (mode == ACC || mode == IMP) {
        e->clocks += 2;
    } else if (mode == IZY) {
        if (!guard || (access != READ && op != STA))
            return false;
        if (access == READ) {
            emit_indirect_read(e, code[1], load, guard);
        } else {
            emit_indirect_store(e, code[1], guard);
            store = e->words[--e->size];
        }
    } else {
        if (!emit_memory(e, code, mode, access, load)) {
            if (known_carry >= 0)
                emit(e, carry_word);
            return false;
        }
        if (access != READ)
            store = e->words[--e->size];
    }

    e->bus = mode == ACC || mode == IMP || mode == IMM ? REG_NONE : REG_VALUE;
    e->bus_imm = code[1];

    if (modify_memory)
        e->bus = REG_NZ;
    unsigned dest = mode == ACC ? REG_A : load;

    switch (op) {
    case STA: case STX: case STY:
        dest = op == STA ? REG_A : op == STX ? REG_X : REG_Y;
        e->bus = dest;
        break;
    case LDA: case LDX: case LDY:
        if (e->bus != REG_NONE || e->ram_operand)
            e->bus = load;
        if (mode == IMM && !e->ram_operand)
            remember_constant_nz(e, load, code[1]);
        else
            remember_nz(e, load);
        break;
    case TAX: case TAY: case TXA: case TYA:
        dest = op == TAX ? REG_X : op == TAY ? REG_Y : REG_A;
        emit_move(e, dest, op == TXA ? REG_X : op == TYA ? REG_Y : REG_A);
        remember_nz(e, dest);
        break;
    case INX: case INY: case DEX: case DEY:
        emit_index_change(e, op, 1);
        break;
    case INC: case DEC:
        emit_imm(e, op == INC ? 0x11000000 : 0x51000000, dest, dest, 1);
        emit_extract(e, dest, dest, 0, 8);
        remember_nz(e, dest);
        break;
    case AND: case EOR: case ORA:
        emit_reg(e, op == AND ? ARM_AND : op == EOR ? ARM_EOR : ARM_ORR, REG_A, REG_A, REG_VALUE);
        if (op == AND)
            e->wide_accumulator = false;
        remember_nz(e, REG_A);
        break;
    case CMP: case CPX: case CPY:
        dest = op == CMP ? REG_A : op == CPX ? REG_X : REG_Y;

        if (compare_immediate)
            emit_imm(e, 0x71000000, REG_NZ, dest, code[1]);
        else
            emit_reg(e, ARM_SUBS, REG_NZ, dest, REG_VALUE);
        emit_set_condition(e, carry, ARM_HS);
        emit_carry(e, carry);
        // A byte difference in [-255, 255] has the correct bit 7 and zero test
        remember_nz(e, REG_NZ);
        break;
    case ASL: case LSR: case ROL: case ROR: {
        bool left = op == ASL || op == ROL, rotate = op == ROL || op == ROR;

        if (rotate) {
            if (e->carry_pending)
                emit_move(e, REG_TEMP2, REG_TRACE_CARRY);
            else
                emit_extract(e, REG_TEMP2, REG_P, 0, 1);
        }
        emit_extract(e, carry, dest, left ? 7 : 0, 1);
        emit_carry(e, carry);
        if (left)
            emit(e, UINT32_C(0x531f1800) | (dest << 5) | dest); /* ubfiz #1, #7 */
        else
            emit_extract(e, dest, dest, 1, 7);
        if (rotate)
            emit_insert(e, dest, REG_TEMP2, left ? 0 : 7, 1);
        remember_nz(e, dest);
        break;
    }
    case ADC: case SBC:
        // Align the byte operation with the host sign bit to get native C and V
        if (known_carry == (op == SBC) && e->write_overflow) {
            emit(e, UINT32_C(0x53081c00) | (REG_A << 5) | REG_SUM); /* lsl #24 */
            emit_reg(e, (op == SBC ? ARM_SUBS : ARM_ADDS) | (24u << 10), REG_SUM, REG_SUM, REG_VALUE);
            emit_set_condition(e, REG_TEMP, ARM_VS);
            emit_insert(e, REG_P, REG_TEMP, 6, 1);
            emit_set_condition(e, carry, ARM_HS);
            emit_carry(e, carry);
            emit_extract(e, REG_A, REG_SUM, 24, 8);
            remember_nz(e, REG_A);
            break;
        }
        if (op == SBC)
            emit_imm(e, 0x52000000, REG_VALUE, REG_VALUE, 7); /* eor wValue, wValue, #$ff */
        if (known_carry < 0 && !e->carry_cached && !e->carry_pending)
            emit_extract(e, carry, REG_P, 0, 1);
        unsigned sum = e->keep_add_sum ? REG_A : REG_SUM;

        if (e->wide_accumulator)
            emit_reg(e, 0x0b200000, sum, REG_VALUE, REG_A); /* add wSum, wValue, wA, uxtb */
        else
            emit_reg(e, ARM_ADD, sum, REG_A, REG_VALUE);
        if (known_carry < 0)
            emit_reg(e, ARM_ADD, sum, sum, carry);
        else if (known_carry)
            emit_imm(e, 0x11000000, sum, sum, 1);
        if (e->write_overflow) {
            emit_reg(e, ARM_EOR, REG_TEMP, REG_A, REG_VALUE);
            emit_reg(e, ARM_EOR, REG_TEMP2, REG_A, sum);
            emit_reg(e, 0x0a200000, REG_TEMP, REG_TEMP2, REG_TEMP);
            emit_extract(e, REG_TEMP, REG_TEMP, 7, 1);
            emit_insert(e, REG_P, REG_TEMP, 6, 1);
        }
        emit_extract(e, carry, sum, 8, 1);
        emit_carry(e, carry);
        if (!e->keep_add_sum)
            emit_extract(e, REG_A, sum, 0, 8);
        e->wide_accumulator = e->keep_add_sum;
        remember_nz(e, REG_A);
        if (op == SBC && !e->loop_body && mode != IMM)
            emit_imm(e, 0x52000000, REG_VALUE, REG_VALUE, 7);
        break;
    case BIT:
        // N and V come from the operand, while Z comes from A & operand
        emit_extract(e, REG_TEMP, REG_VALUE, 6, 2);
        emit_insert(e, REG_P, REG_TEMP, 6, 2);
        emit_reg(e, 0x6a000000, REG_NONE, REG_A, REG_VALUE); /* tst */
        emit_set_condition(e, REG_TEMP, ARM_EQ);
        emit_insert(e, REG_P, REG_TEMP, 1, 1);
        e->dirty |= 1u << REG_P;
        e->nz = REG_NONE;
        break;
    case CLV: case CLD: case SED:
        emit_status_bit(e, op == CLV ? 6 : 3, op == SED);
        e->dirty |= 1u << REG_P;
        break;
    case PHA: case PHP:
        note_bus_writes(e, e->clocks, 1);
        if (op == PHP) {
            e->dirty = emit_status_flags(e, e->dirty, e->nz);
            emit(e, 0x321c0400 | (REG_P << 5) | REG_VALUE); /* orr #$30 (B | U) */
        }
        e->bus = op == PHP ? REG_VALUE : REG_A;
        emit_push(e, e->bus);
        ++e->clocks;
        break;
    case PLA:
        load_stack_pointer(e);
        emit_imm(e, 0x11000000, REG_S, REG_S, 1);
        emit_extract(e, REG_S, REG_S, 0, 8);
        emit_imm(e, 0x11000000, REG_ADDRESS, REG_S, 256);
        emit_reg(e, 0x38604800, REG_A, REG_RAM, REG_ADDRESS);
        advance_stack_pointer(e, 1);
        e->bus = REG_A;
        e->clocks += 2;
        remember_nz(e, REG_A);
        break;
    case TXS:
        emit_move(e, REG_S, REG_X);
        e->uses_stack = e->stack_replaced = true;
        e->stack_pending = 0;
        e->dirty |= 1u << REG_S;
        break;
    case TSX:
        load_stack_pointer(e);
        emit_move(e, REG_X, REG_S);
        remember_nz(e, REG_X);
        break;
    case CLC: case SEC:
        if (e->carry_cached) {
            emit_constant(e, REG_LOOP_CARRY, op == SEC);
        } else if (e->defer_carry) {
            emit_constant(e, REG_TRACE_CARRY, op == SEC);
            e->carry_pending = true;
        } else {
            emit_status_bit(e, 0, op == SEC);
        }
        e->carry_end = e->size;
        e->carry_value = op == SEC;
        e->dirty |= 1u << REG_P;
        break;
    case NOP:
        break;
    default:
        return false;
    }
    if (store)
        emit(e, (store & ~31u) | dest);
    return true;
}

typedef struct {
    unsigned check, guards;
    uint32_t branch;
    unsigned offset, target, bus, bus_reg, clocks;
    unsigned dirty, nz, extra, instructions;
    uint8_t stack_pending;
    bool carry_pending, pop_return;
} TraceExit;

static TraceExit trace_resume_exit(const TraceEmitter *e, unsigned pc, unsigned instructions)
{
    return (TraceExit){
        .offset = TRACE_DIRECT_EXIT, .target = pc, .bus = e->bus_imm, .bus_reg = e->bus,
        .clocks = e->clocks, .dirty = e->dirty, .nz = e->nz, .extra = e->extra,
        .instructions = instructions, .stack_pending = e->stack_pending, .carry_pending = e->carry_pending
    };
}

// The taken path and its continuation can materialize one or both guest flags
static unsigned nz_after_branch(unsigned nz, unsigned opcode, bool taken)
{
    if (nz == REG_NONE || (nz & NZ_CONSTANT))
        return nz;
    bool set = ((opcode & 0x20) != 0) == taken;

    if (opcode >= 0xc0)
        return set ? NZ_CONSTANT | CPU_ZERO : nz | NZ_ZERO_CLEAR;
    if (opcode < 0x40)
        return set ? NZ_CONSTANT | CPU_NEGATIVE : nz | NZ_NEGATIVE_CLEAR;
    return nz;
}

static uint32_t trace_branch(const TraceEmitter *e, unsigned opcode)
{
    unsigned bit = (unsigned)__builtin_ctz(cpu_branch_mask(opcode));
    bool positive = (opcode & 0x20) != 0;

    if ((e->nz & NZ_CONSTANT) && (bit == 1 || bit == 7)) {
        bool taken = ((e->nz & (1u << bit)) != 0) == positive;
        return (taken ? 0x34000000 : 0x35000000) | REG_NONE; /* cbz/cbnz wzr */
    }
    if ((bit == 7 && (e->nz & NZ_NEGATIVE_CLEAR)) || (bit == 1 && (e->nz & NZ_ZERO_CLEAR)))
        return (positive ? 0x35000000 : 0x34000000) | REG_NONE;
    // Test a pending zero result directly or test a single bit for other conditions
    if (e->nz != REG_NONE && bit == 1)
        return (positive ? 0x34000000 : 0x35000000) | (e->nz & 31); /* cbz / cbnz */
    unsigned reg = REG_P;

    if (e->carry_cached && bit == 0)
        reg = REG_LOOP_CARRY;
    else if (e->carry_pending && bit == 0)
        reg = REG_TRACE_CARRY;
    else if (e->nz != REG_NONE && bit == 7)
        reg = e->nz & 31;
    return (positive ? 0x37000000 : 0x36000000) | (bit << 19) | reg; /* tbnz / tbz */
}

static unsigned emit_status_flags(TraceEmitter *e, unsigned dirty, unsigned nz)
{
    if (e->carry_pending)
        emit_insert(e, REG_P, REG_TRACE_CARRY, 0, 1);
    if (nz != REG_NONE) {
        if (nz & NZ_CONSTANT) {
            emit_status_bit(e, 7, (nz & CPU_NEGATIVE) != 0);
            emit_status_bit(e, 1, (nz & CPU_ZERO) != 0);
        } else {
            if (nz & NZ_NEGATIVE_CLEAR) {
                emit_status_bit(e, 7, false);
            } else {
                emit_extract(e, REG_TEMP, nz & 31, 7, 1);
                emit_insert(e, REG_P, REG_TEMP, 7, 1);
            }
            if (nz & NZ_ZERO_CLEAR) {
                emit_status_bit(e, 1, false);
            } else {
                emit_imm(e, 0x71000000, 31, nz & 31, 0);
                emit_set_condition(e, REG_TEMP, ARM_EQ);
                emit_insert(e, REG_P, REG_TEMP, 1, 1);
            }
        }
        dirty |= 1u << REG_P;
    }
    return dirty;
}

static void emit_write_registers(TraceEmitter *e, unsigned dirty, unsigned nz)
{
    dirty = emit_status_flags(e, dirty, nz);

    for (unsigned reg = REG_A; reg <= REG_P; ++reg)
        if (dirty & (1u << reg))
            emit_imm(e, 0x39000000, reg, 0, cpu_register_offset(reg));
    if (dirty & (1u << REG_S)) {
        materialize_stack_pointer(e);
        emit_imm(e, 0x39000000, REG_S, 0, cpu_register_offset(REG_S));
    }
}

static inline void emit_return_header(TraceEmitter *e, unsigned low, unsigned bus_reg, unsigned bus_imm,
                                      unsigned extra, unsigned offset, unsigned instructions)
{
    emit_constant(e, 0, low | (instructions << TRACE_COUNT_SHIFT));
    unsigned high = (offset << (TRACE_EXIT_SHIFT - TRACE_BUS_SHIFT)) |
                    (bus_reg == REG_NONE ? bus_imm : 0);

    if (high)
        emit(e, 0x72a00000 | (high << 5)); /* movk w0, #high, lsl #16 */
    if (bus_reg != REG_NONE)
        emit_insert(e, 0, bus_reg, TRACE_BUS_SHIFT, 8);
    if (extra)
        emit_reg(e, ARM_ADD, 0, 0, REG_CYCLES);
}

// The high fields describe either an exit PC or a pending absolute store
static void emit_return(TraceEmitter *e, unsigned clocks, unsigned bus_reg, unsigned bus_imm,
                          unsigned extra, unsigned offset, unsigned target, unsigned instructions,
                          unsigned write_reg)
{
    unsigned low = clocks;

    if (write_reg != REG_NONE)
        low |= TRACE_WRITE;
    emit_return_header(e, low, bus_reg, bus_imm, extra, offset, instructions);
    if (offset || write_reg != REG_NONE)
        emit(e, 0xf2c00000 | (target << 5)); /* movk x0, #target, lsl #32 */
    if (write_reg != REG_NONE)
        emit(e, 0xb3500000 | (7 << 10) | (write_reg << 5)); /* bfi x0, xReg, #48, #8 */
    emit(e, 0xd65f03c0);
}

static void emit_rom_return(TraceEmitter *e, unsigned clocks, unsigned bus_reg, unsigned bus_imm,
                            unsigned extra, unsigned offset, unsigned target, unsigned instructions,
                            unsigned write_reg, unsigned resume_pc, unsigned bank)
{
    // A quiet ROM trace finishes before the next CPU-visible event. A
    // RAM or ROM opcode fetch fully drives the bus before it can be observed.
    if (!offset || target < 0x2000 || target >= 0x8000) {
        bus_reg = REG_NONE;
        bus_imm = 0;
    }
    const bool write = write_reg != REG_NONE;
    const bool loop = offset >= TRACE_LOOP_EXIT;
    const unsigned low = clocks | (write ? TRACE_WRITE : 0);
    unsigned kind = loop ? TRACE_LOOP_EXIT : offset;

    if (offset == TRACE_DIRECT_EXIT) {
        // A known same-bank destination can use the ordinary resume path
        if (!clocks)
            kind = TRACE_ENTRY_REJECTED;
        else if ((target & 0xe000) == bank)
            kind = 0;
    }
    const unsigned next = offset || write ? target : resume_pc;

    if (write)
        emit(e, 0xf2c00001 | (resume_pc << 5)); /* movk x1, #write PC, lsl #32 */
    emit_return_header(e, low, bus_reg, bus_imm, extra, kind, instructions);
    emit(e, 0xf2c00000 | (next << 5)); /* movk x0, #next PC or write address, lsl #32 */
    if (write)
        emit(e, 0xb3500000 | (7 << 10) | (write_reg << 5)); /* bfi x0, xReg, #48, #8 */
    else if (loop)
        emit(e, 0xf2e00000 | (resume_pc << 5)); /* movk x0, #branch PC, lsl #48 */
    emit(e, 0xd65f03c0);
}

static void emit_trace_link(TraceEmitter *e, const CpuTraceCache *cache, const uint8_t *page,
                            unsigned bank, unsigned target, unsigned clocks, unsigned extra,
                            unsigned instruction_count, unsigned valid);

static void emit_rts_result(TraceEmitter *e, unsigned instructions)
{
    // The next opcode comes from ROM, or internal RAM after a wrap at $FFFF.
    // That fetch drives the bus before the dummy read's value can be used.
    emit_return_header(e, e->clocks + 6, REG_NONE, 0, e->extra, TRACE_DIRECT_EXIT, instructions);
    emit_imm(e, 0x11000000, REG_SUM, REG_SUM, 1);
    emit(e, 0xb3400000 | ((64 - TRACE_TARGET_SHIFT) << 16) | (15 << 10) | (REG_SUM << 5));
    emit(e, 0xd65f03c0);
}

static void emit_rom_rts(TraceEmitter *e, unsigned instructions, const CpuTraceCache *cache,
                         const uint8_t *page, unsigned bank, uint16_t expected)
{
    load_stack_pointer(e);
    emit_imm(e, 0x11000000, REG_ADDRESS, REG_S, 1);
    emit_extract(e, REG_ADDRESS, REG_ADDRESS, 0, 8);
    // Fall back before changing the stack or bus value if the read wraps
    emit_imm(e, 0x71000000, 31, REG_ADDRESS, 255);
    unsigned wrap = e->size;

    emit(e, 0);
    emit_imm(e, 0x11000000, REG_TEMP, REG_ADDRESS, 256);
    emit_reg(e, 0x78604800, REG_SUM, REG_RAM, REG_TEMP); /* ldrh w10, [x2, w13, uxtw] */
    unsigned to_fallback = e->size;

    emit(e, 0); /* tbz w10, #15, fallback */
    emit_imm(e, 0x11000000, REG_S, REG_ADDRESS, 1);
    // Only the native return path updates the stack register
    unsigned dirty = e->dirty;

    e->dirty |= 1u << REG_S;
    emit_write_registers(e, e->dirty, e->nz);
    // Validate the observed return address before linking a trace with multiple callers
    unsigned target = (uint16_t)(expected + 1u);

    if ((target & 0xe000) == bank && (target & 8191) <= 8189) {
        unsigned check = e->size, links = e->link_count, pending = e->pending_count;

        emit_constant(e, REG_TEMP, expected);
        emit_reg(e, ARM_SUBS, REG_NONE, REG_SUM, REG_TEMP);
        unsigned other = e->size;

        emit(e, 0); /* b.ne */
        emit_trace_link(e, cache, page, bank, target, e->clocks + 6, e->extra, instructions, e->inputs | e->dirty);
        if (e->link_count == links && e->pending_count == pending)
            e->size = check;
        else
            e->words[other] = 0x54000001 | ((e->size - other) << 5);
    }
    emit_rts_result(e, instructions);
    e->dirty = dirty;
    e->words[to_fallback] = 0x36780000 | ((e->size - to_fallback) << 5) | REG_SUM;
    e->words[wrap] = 0x54000000 | ((e->size - wrap) << 5) | ARM_EQ;
}

static inline uint32_t trace_key_hash(uint32_t key)
{
    return key ^ (key >> 11) ^ (key >> 21);
}

static void record_pending_links(CpuTraceCache *cache, TraceEmitter *e, unsigned start, unsigned first)
{
    for (unsigned i = 0; i < e->pending_count; ++i) {
        const PendingLink *link = &e->pending[i];
        unsigned at = link->location & LINK_WORD_MASK;
        unsigned bucket = trace_key_hash(link->key) & (PENDING_LINK_BUCKETS - 1);

        // An unresolved stub jumps over these words. Once the destination is
        // published, the linker uses the same space for executable code.
        e->words[at + 1] = link->key;
        e->words[at + 2] = cache->pending_heads[bucket];
        e->words[at + 3] = ((link->location >> LINK_WORD_BITS) << 16) |
                           ((unsigned)link->registers << 8) | link->clocks;
        cache->pending_heads[bucket] = start + at - first + 1;
    }
}

static void resolve_pending_links(CpuTraceCache *cache, uint32_t key, unsigned layout, bool writable);

static unsigned publish_trace(CpuTraceCache *cache, TraceEmitter *emitter)
{
    // A side exit in the first loop iteration can still write back any
    // register modified by later iterations. Keep those registers live.
    unsigned inputs = emitter->inputs | (emitter->loop_body ? emitter->dirty : 0);
    unsigned first = REG_P - REG_A;

    for (unsigned reg = REG_Y; reg >= REG_A; --reg)
        if (inputs & (1u << reg))
            emitter->words[--first] = emitter->words[reg - REG_A];
    // Internal jumps are relative to the compacted prologue. Patch jumps
    // between traces against their final addresses in the same arena.
    unsigned start = cache->used / sizeof *cache->arena;

    for (unsigned i = 0; i < emitter->link_count; ++i) {
        unsigned at = emitter->links[i].at;
        unsigned distance = emitter->links[i].word - (start + at - first);

        emitter->words[at] = 0x14000000 | (distance & 0x03ffffff);
    }
    record_pending_links(cache, emitter, start, first);
    size_t bytes = (emitter->size - first) * sizeof emitter->words[0];
    uint32_t *output = trace_write_arena(cache) + cache->used / sizeof *cache->arena;

    trace_write_protect(0);
    memcpy(output, emitter->words + first, bytes);
    if (emitter->publish_layout) {
        if (emitter->publish_layout & BLOCK_HAS_WRITE_MASK) {
            // Traces with writes store the mask immediately before the entry
            memcpy(output - DMC_WRITE_MASK_WORDS, emitter->bus_writes, sizeof emitter->bus_writes);
        }
        resolve_pending_links(cache, emitter->publish_key, emitter->publish_layout, true);
    }
    trace_write_protect(1);
    sync_trace_code(cache, output, bytes);
    unsigned arena_word = cache->used / sizeof *cache->arena + 1;

    cache->used += bytes;

    // A trace that crosses the budget is the last one published this frame
    if (cache->used - cache->frame_arena_start >= TRACE_EMISSION_BUDGET)
        cache->remaining_compiles = 0;
    return arena_word;
}

// Invalidate every reference before the generated code can be overwritten
static void clear_traces(CpuTraceCache *cache)
{
    cache->used = 0;
    cache->frame_arena_start = cache->sweep_index = 0;
    cache->segment_end = TRACE_SEGMENT_SIZE;
    cache->segment_generations = 0;
    cache->arena_wrapped = false;
    memset(cache->pending_heads, 0, sizeof cache->pending_heads);
    memset(cache->blocks, 0, sizeof cache->blocks);
    memset(cache->loops, 0, sizeof cache->loops);
    memset(cache->reusable_blocks, 0, sizeof cache->reusable_blocks);
    if (cache->ram_blocks)
        memset(cache->ram_blocks, 0, sizeof *cache->ram_blocks);
    if (cache->code_map)
        memset(cache->code_map, 0, FDS_CODE_MAP_BYTES);
}

#ifndef SCHNES_CPU_DEBUG
void cpu_forget_fds_traces(Cartridge *cart, uint16_t address)
{
    // Only C code calls this, between traces, so no native code is running
    CpuTraceCache *cache = cart->trace_cache;

    if (!cache)
        return;
    if (borrowed_fds_traces(cart, cache)) {
        // The shared traces still describe the live machine's RAM
        cart->ext->fds->code_map = NULL;
        return;
    }
    clear_traces(cache);
    // Self-modifying code rewrites the same bytes again. Leave their
    // instructions to the interpreter from now on.
    unsigned offset = address - 0x6000u;

    if (cache->volatile_map && offset < 0x8000)
        cache->volatile_map[offset >> 3] |= (uint8_t)(1u << (offset & 7));
}
#endif

static bool trace_arena_has_room(CpuTraceCache *cache)
{
    if (cache->used <= cache->segment_end - TRACE_RESERVE)
        return true;
    cache->remaining_compiles = 0;
    return false;
}

static inline uintptr_t trace_address(const uint8_t *page, const uint8_t *code, unsigned bank, bool fds)
{
    // FDS windows are fixed and runahead can execute from a copy of PRG RAM
    return fds ? bank + (unsigned)(code - page) : (uintptr_t)code;
}

static inline unsigned trace_slot(uintptr_t address)
{
    return (unsigned)(address ^ (address >> 13)) & (TRACE_SLOTS - 1);
}

static uint32_t trace_key(uintptr_t address, unsigned bank)
{
    // The low 26 bits distinguish bytes within the cartridge's ROM allocation.
    // Mix the CPU window into the remaining bits to distinguish its mappings.
    return (uint32_t)address ^ (bank << 13);
}

_Static_assert((TRACE_ARENA_SIZE - TRACE_MACHINE_WORDS * sizeof(uint32_t)) / BLOCK_ALIGNMENT <= BLOCK_ENTRY_MASK,
               "Aligned arena offset fits in the trace layout");
_Static_assert(TRACE_MAX_CYCLES + 8 <= BLOCK_CYCLES_MASK, "Trace cycle bound fits in the layout");
_Static_assert(((2 * (TRACE_MAX_CYCLES + 8) + 1) << (TRACE_LINK_BUDGET_SHIFT - 12)) < 4096,
               "The packed link deadline fits in a shifted immediate");
_Static_assert(TRACE_FORWARD_SPAN + 3 * TRACE_MAX_INSTRUCTIONS + TRACE_LOOP_EXIT <= UINT8_MAX,
               "Trace span and encoded exits fit in a byte");
_Static_assert(TRACE_WARM_HITS + 1 < (1u << BLOCK_ENTRY_BITS), "Cold trace hits fit in the layout");

static inline bool link_in_current_segment(const CpuTraceCache *cache, unsigned layout)
{
    unsigned first = (cache->segment_end - TRACE_SEGMENT_SIZE) / sizeof(uint32_t);

    return in_trace_segment(layout & BLOCK_ENTRY_MASK, first);
}

// A linked destination remains in its segment after its lookup slot is reused
static unsigned encode_rom_link(uint32_t *out, const CpuTraceCache *cache, const PendingLink *link,
                                unsigned layout, unsigned reserved, unsigned *body)
{
    unsigned cycles = (layout >> BLOCK_ENTRY_BITS) & BLOCK_CYCLES_MASK;

    if (cycles < 12)
        cycles = 12;
    unsigned budget = link->registers & LINK_HAS_EXTRA_CYCLES ? REG_TEMP2 : 1;
    // The return budget packs the remaining cycles above the instruction count.
    // Require at least one full spare cycle even when the low count is nonzero.
    const unsigned immediate_shift = TRACE_LINK_BUDGET_SHIFT - 12 + 10;

    out[0] = 0x71400000 | ((link->clocks + cycles + 1) << immediate_shift) | (budget << 5) | REG_NONE;
    out[2] = 0x51400001 | (link->clocks << immediate_shift) | (budget << 5);
    out[3] = 0x11000021 | ((link->location >> LINK_WORD_BITS) << 10);
    unsigned word = (layout & BLOCK_ENTRY_MASK) * BLOCK_WORDS;
    unsigned first_missing = UINT32_MAX, jump = 4;

    for (unsigned reg = REG_A; reg <= REG_Y; ++reg) {
        uint32_t load = 0x39400000 | (cpu_register_offset(reg) << 10) | reg;

        if (trace_write_arena(cache)[word] != load)
            continue;
        if (!(link->registers & (1u << reg))) {
            if (first_missing == UINT32_MAX)
                first_missing = word;
            if (!reserved || jump + 1 < reserved)
                out[jump] = load;
            ++jump;
        }
        ++word;
    }
    if (reserved && jump >= reserved) {
        // Reuse the destination prologue if its register loads overflow the stub
        jump = 4;
        *body = first_missing;
    } else {
        // Both blocks already have the guest status and the internal RAM base
        *body = word + 2;
    }
    out[1] = 0x54000000 | ((reserved ? reserved - 1 : jump) << 5) | ARM_LT;
    return jump;
}

static void resolve_pending_links(CpuTraceCache *cache, uint32_t key, unsigned layout, bool writable)
{
    if (!link_in_current_segment(cache, layout))
        return;
    uint32_t *next = &cache->pending_heads[trace_key_hash(key) & (PENDING_LINK_BUCKETS - 1)];
    bool opened = false;

    while (*next) {
        unsigned word = *next - 1;
        uint32_t *out = trace_write_arena(cache) + word;

        if (out[1] != key) {
            next = &out[2];
            continue;
        }
        if (!writable && !opened) {
            trace_write_protect(0);
            opened = true;
        }
        *next = out[2];
        unsigned details = out[3];
        PendingLink link = {.location = (details >> 16) << LINK_WORD_BITS,
                            .clocks = (uint8_t)details, .registers = (uint8_t)(details >> 8)};
        unsigned target;
        unsigned jump = encode_rom_link(out, cache, &link, layout, PENDING_LINK_WORDS, &target);

        out[jump] = 0x14000000 | ((target - word - jump) & 0x03ffffff);
        sync_trace_code(cache, out, (jump + 1) * sizeof(uint32_t));
    }
    if (opened)
        trace_write_protect(1);
}

// Only the final carry from a run of accumulator shifts can be observed
static void emit_shift_run(TraceEmitter *e, unsigned opcode, unsigned count, uint8_t bus)
{
    bool left = opcode == 0x0a;

    e->inputs |= (1u << REG_A) & ~e->dirty;
    emit_extract(e, REG_TEMP, REG_A, left ? 8 - count : count - 1, 1);
    emit_carry(e, REG_TEMP);
    if (count == 8)
        emit_constant(e, REG_A, 0);
    else if (left)
        emit(e, UINT32_C(0x53000000) | ((32 - count) << 16) | ((7 - count) << 10) | (REG_A << 5) | REG_A);
    else
        emit_extract(e, REG_A, REG_A, count, 8 - count);
    if (count == 8)
        remember_constant_nz(e, REG_A, 0);
    else
        remember_nz(e, REG_A);
    e->clocks += count * 2;
    e->bus = REG_NONE;
    e->bus_imm = bus;
}

static bool emit_rotate_pair(TraceEmitter *e, const uint8_t *code)
{
    // LSR / ROL zp / LSR / ROL zp+1. The final carry comes from the
    // original high byte, and the accumulator provides the new low bits.
    unsigned zp = code[2];

    if (code[0] != 0x4a || code[1] != 0x26 || code[3] != 0x4a ||
        code[4] != 0x26 || zp == 255 || code[5] != zp + 1)
        return false;

    e->inputs |= (1u << REG_A) & ~e->dirty;
    emit_imm(e, 0x78400000, REG_VALUE, REG_RAM, zp << 2);
    emit_extract(e, REG_TRACE_CARRY, REG_VALUE, 15, 1);
    emit_carry(e, REG_TRACE_CARRY);
    emit(e, 0x531f7800 | (REG_VALUE << 5) | REG_VALUE); /* lsl wValue, wValue, #1 */
    emit_insert(e, REG_VALUE, REG_A, 0, 1);
    emit_extract(e, REG_TEMP, REG_A, 1, 1);
    emit_insert(e, REG_VALUE, REG_TEMP, 8, 1);
    emit_imm(e, 0x78000000, REG_VALUE, REG_RAM, zp << 2);
    emit_extract(e, REG_A, REG_A, 2, 6);
    emit_extract(e, REG_NZ, REG_VALUE, 8, 8);
    e->dirty |= 1u << REG_A;
    remember_nz(e, REG_NZ);
    e->wide_accumulator = false;
    e->bus = REG_NZ;
    note_bus_writes(e, e->clocks + 5, 2);
    note_bus_writes(e, e->clocks + 12, 2);
    e->clocks += 14;
    return true;
}

static NES_NOINLINE bool emit_packed_arithmetic(TraceEmitter *e, const uint8_t *code,
                                               unsigned index, unsigned *guard)
{
    // Three consecutive zero-page bytes add or subtract three ROM table
    // bytes. One aligned 24-bit operation gives the original final C and V.
    unsigned zp = code[1];
    bool sub = code[2] == 0x38;
    unsigned op = sub ? 0xfd : 0x7d;

    if (zp > 253)
        return false;
    unsigned addresses[3], bound = 255;
    const uint8_t *part = code;

    for (unsigned i = 0; i < 3; ++i) {
        unsigned skip = i == 0;

        if (part[0] != 0xa5 || part[1] != zp + i || part[2 + skip] != op ||
            part[5 + skip] != 0x85 || part[6 + skip] != zp + i)
            return false;
        addresses[i] = combine(part[3 + skip], part[4 + skip]);

        if (addresses[i] < 0x8000 || addresses[i] >> 13 != addresses[0] >> 13)
            return false;
        unsigned limit = 255 - (addresses[i] & 255);

        if (bound > limit)
            bound = limit;
        part += 7 + skip;
    }
    unsigned page = addresses[0] >> 13;
    // Share the ROM base across the three loads when their displacements
    // fit in the same 4 KiB window.
    unsigned window = addresses[0] >> 12;

    if (addresses[1] >> 12 != window || addresses[2] >> 12 != window || index > bound)
        return false;
    e->inputs |= 1u << REG_X;

    if (bound < 255) {
        emit_imm(e, 0x71000000, REG_NONE, REG_X, bound);
        emit_guard(e, guard, 0x54000008); /* b.hi: the index can cross a page */
    }
    if (e->prg_window != window) {
        unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, prg_pages) + (page - 4) * sizeof(void *);

        emit_imm(e, 0xf9400000, REG_ROM, 0, field / 8);
        if (window & 1)
            emit_imm(e, 0x91400000, REG_ROM, REG_ROM, 1);
        e->prg_window = window;
    }
    emit_reg(e, 0x8b204000, REG_ADDRESS, REG_ROM, REG_X); /* add xAddr, xROM, wX, uxtw */
    emit_imm(e, 0x39400000, REG_VALUE, REG_ADDRESS, addresses[0] & 4095);
    for (unsigned i = 1; i < 3; ++i) {
        emit_imm(e, 0x39400000, REG_TEMP, REG_ADDRESS, addresses[i] & 4095);
        emit_insert(e, REG_VALUE, REG_TEMP, 8 * i, 8);
    }
    emit_imm(e, 0x78400000, REG_NZ, REG_RAM, zp << 2);
    emit_imm(e, 0x39400000, REG_TEMP, REG_RAM, zp + 2);
    emit_insert(e, REG_NZ, REG_TEMP, 16, 8);
    emit(e, 0x53185c00 | (REG_NZ << 5) | REG_NZ); /* lsl wNZ, wNZ, #8 */
    emit_reg(e, (sub ? ARM_SUBS : ARM_ADDS) | (8u << 10), REG_NZ, REG_NZ, REG_VALUE);
    emit_set_condition(e, REG_TRACE_CARRY, ARM_HS);
    emit_carry(e, REG_TRACE_CARRY);
    emit_set_condition(e, REG_TEMP, ARM_VS);
    emit_insert(e, REG_P, REG_TEMP, 6, 1);
    emit_extract(e, REG_TEMP, REG_NZ, 8, 16);
    emit_imm(e, 0x78000000, REG_TEMP, REG_RAM, zp << 2);
    emit_extract(e, REG_A, REG_NZ, 24, 8);
    emit_imm(e, 0x39000000, REG_A, REG_RAM, zp + 2);
    remember_nz(e, REG_A);
    e->wide_accumulator = false;
    e->bus = REG_A;
    note_bus_writes(e, e->clocks + 11, 1);
    note_bus_writes(e, e->clocks + 21, 1);
    note_bus_writes(e, e->clocks + 31, 1);
    e->clocks += 32;
    return true;
}

// Only short backward branches and jumps with a repeatable body need the loop batcher
static unsigned trace_branch_exit(const Nes *nes, const uint8_t *page, unsigned pc,
                                  uint16_t target, int offset)
{
    unsigned distance = (uint16_t)(pc - target);

    if ((unsigned)offset > UINT8_MAX - TRACE_LOOP_EXIT || !short_loop_in_bank(target, distance))
        return TRACE_DIRECT_EXIT;

    const uint8_t *code = page + (target & 8191);
    const ShortLoop *cached = &nes->short_loops[short_loop_slot(target)];
    uintptr_t key = mapped_code_key(code, target, cart_is_fds(&nes->cart));

    if (short_loop_matches(cached, key, distance))
        return cached->cycles ? TRACE_LOOP_EXIT + (unsigned)offset : TRACE_DIRECT_EXIT;
    ShortLoop loop;

    decode_short_loop(&loop, key, code, distance, target);
    return loop.cycles ? TRACE_LOOP_EXIT + (unsigned)offset : TRACE_DIRECT_EXIT;
}

// Look for a short loop immediately after initialization of an index
static unsigned trace_loop_entry_exit(const Nes *nes, const uint8_t *page, unsigned head, unsigned start)
{
    const uint8_t *code = page + (head & 8191);

    for (unsigned back = 0; short_loop_in_bank(head, back);) {
        const Instruction instr = instructions[code[back]];
        const Mode mode = instruction_mode(instr);

        if (mode == REL || instr.op == JMP) {
            if (back >= 8 && closes_short_loop(code, head, back))
                return trace_branch_exit(nes, page, head + back, (uint16_t)head,
                                         (int)(head + back) - (int)start);
            break;
        }
        if (instr.op == JSR || instr.op == RTS || instr.op == RTI)
            break;
        back += mode_lengths[mode];
    }
    return TRACE_DIRECT_EXIT;
}

static bool trace_prg_window_known(const Cartridge *cart, unsigned entry, unsigned bank)
{
    // If these windows move together, the entry page identifies both mappings
    bool same_half = !((entry ^ bank) & 0x4000);

    switch (cart->mapper) {
    case 0: case 3: case 7: case 66: return true;
    // FDS RAM and BIOS never move. Checking the page address would also
    // reject a runahead prediction, which maps its own copy of the RAM.
    case 20: return true;
    case 1: return cart->submapper == 5 || same_half;
    case 2: case 71: return bank >= 0xc000 || same_half;
    // VRC4 can swap $8000 with $C000, so only $E000 stays fixed for these
    case 4: case 21: case 22: case 23: case 25: case 69: return bank == 0xe000;
    // VRC6 switches $8000-$BFFF as one 16 KiB bank
    case 24: case 26: return bank == 0xe000 || (bank < 0xc000 && entry < 0xc000);
    // MMC4 switches $8000-$BFFF as one 16 KiB bank
    case 9: return cart->mmc4 ? bank >= 0xc000 || same_half : bank >= 0xa000;
    case 206: return cart->submapper == 1 || bank >= 0xc000;
    default: return false;
    }
}

static void emit_pending_trace_link(TraceEmitter *e, PendingLink link)
{
    link.location |= e->size;
    // Until the destination has been compiled, jump to the usual return
    emit(e, 0x14000000 | PENDING_LINK_WORDS);
    for (unsigned i = 1; i < PENDING_LINK_WORDS; ++i)
        emit(e, 0xd503201f);
    e->pending[e->pending_count++] = link;
}

static void emit_trace_link(TraceEmitter *e, const CpuTraceCache *cache, const uint8_t *page,
                            unsigned bank, unsigned target, unsigned clocks, unsigned extra,
                            unsigned instruction_count, unsigned valid)
{
    // Same-bank code is identified by the entry's mapping. Both traces remain
    // at their published addresses until the arena resets.
    if ((target & ~8191u) != bank || (target & 8191) > 8189 ||
        e->link_count + e->pending_count >= TRACE_MAX_EXITS + 1)
        return;
    uintptr_t address = trace_address(page, page + (target & 8191), bank, cache->code_map != NULL);
    unsigned slot = trace_slot(address);
    const BlockTrace *next = &cache->blocks[slot];
    unsigned layout = next->layout;
    unsigned cycles = (layout >> BLOCK_ENTRY_BITS) & BLOCK_CYCLES_MASK;
    uint32_t key = trace_key(address, bank);
    bool published = cycles && next->key == key;

    if (!published) {
        // Rejected blocks and unsupported entry opcodes seldom become successors
        Operation op = instructions[page[target & 8191]].op;

        if ((next->key == key && layout > TRACE_WARM_HITS && key != e->self_key) ||
            op == RTI || op == BRK || op == KIL || op == CLI || op == SEI || op == PLP)
            return;
    }

    const unsigned live = valid & ((1u << REG_A) | (1u << REG_X) | (1u << REG_Y));
    PendingLink link = {.key = key, .location = instruction_count << LINK_WORD_BITS,
                        .clocks = clocks, .registers = live};

    if (extra) {
        link.registers |= LINK_HAS_EXTRA_CYCLES;
        emit_reg(e, 0x4b000000 | (TRACE_LINK_BUDGET_SHIFT << 10), REG_TEMP2, 1, REG_CYCLES);
    }
    if (!published) {
        emit_pending_trace_link(e, link);
        return;
    }
    unsigned body, guard = 0;
    unsigned segment = (layout & BLOCK_ENTRY_MASK) / (TRACE_SEGMENT_SIZE / sizeof(uint32_t));

    if (!link_in_current_segment(cache, layout)) {
        unsigned cache_offset = offsetof(Nes, cart) + offsetof(Cartridge, trace_cache);

        emit_imm(e, 0xf9400000, REG_TEMP, 0, cache_offset / 8);
        emit_imm(e, 0xb9400000, REG_TEMP, REG_TEMP, offsetof(CpuTraceCache, segment_generations) / 4);
        guard = e->size;
        emit(e, 0);
    }
    e->size += encode_rom_link(e->words + e->size, cache, &link, layout, 0, &body);
    e->links[e->link_count].at = e->size;
    e->links[e->link_count++].word = body;
    emit(e, 0);
    // FIFO reuse cannot replace a destination twice while the source survives
    if (guard) {
        uint32_t branch = cache->segment_generations & (1u << segment) ? 0x36000000 : 0x37000000;

        e->words[guard] = branch | (segment << 19) | ((e->size - guard) << 5) | REG_TEMP;
    }
}

// An MMC3 $8000 write that keeps both bank modes is quiet (mmc3_write_effects)
static void emit_mmc3_quiet_select(TraceEmitter *e, TraceExit *exit, unsigned reg)
{
    const unsigned select = offsetof(Nes, cart) + offsetof(Cartridge, mmc3.select);

    emit_imm(e, 0x39400000, REG_TEMP, 0, select);
    emit_reg(e, ARM_EOR, REG_TEMP, REG_TEMP, reg);
    emit_extract(e, REG_TEMP, REG_TEMP, 6, 2);
    exit->branch = 0x35000000 | REG_TEMP; /* cbnz wTemp */
    exit->check = e->size;
    emit(e, 0);
    // The low bits choose the next data register without changing either bank mode
    emit_imm(e, 0x39000000, reg, 0, select);
    note_bus_writes(e, e->clocks + 3, 1);
    e->clocks += 4;
    e->bus = reg;
}

static void emit_indirect_jump(TraceEmitter *e, unsigned address, unsigned instructions)
{
    // The 6502 wraps the high-byte read within the pointer's page
    unsigned high = address_on_page(address, address + 1);

    address &= 2047;
    high &= 2047;

    if (!(address & 1)) {
        emit_imm(e, 0x79400000, REG_SUM, REG_RAM, address / 2);
        emit_extract(e, REG_VALUE, REG_SUM, 8, 8);
    } else {
        emit_imm(e, 0x39400000, REG_SUM, REG_RAM, address);
        emit_imm(e, 0x39400000, REG_VALUE, REG_RAM, high);
        emit_insert(e, REG_SUM, REG_VALUE, 8, 8);
    }
    emit_return_header(e, e->clocks, REG_VALUE, 0, e->extra, TRACE_DIRECT_EXIT, instructions);
    emit(e, 0xb3600000 | (15 << 10) | (REG_SUM << 5)); /* bfi x0, x10, #32, #16 */
    emit(e, 0xd65f03c0);
}

// Keep the trace compiler, and the code after it, on its measured offsets
NATIVE_ENTRY_AT(60)
static bool compile_trace(CpuTraceCache *cache, BlockTrace *block, const uint8_t *page,
                           const uint8_t *code, unsigned bank, const Nes *nes)
{
    const Cartridge *cart = &nes->cart;
    // A lookup collision can evict the entry while its code is still in the arena
    uint32_t key = block->key;
    unsigned slot = trace_key_hash(key) & (TRACE_REUSE_SLOTS - 1);
    BlockTrace *reusable = &cache->reusable_blocks[slot];

    if (reusable->key == key && reusable->layout) {
        block->layout = reusable->layout;
        resolve_pending_links(cache, key, block->layout, false);
        return true;
    }

    TraceEmitter e;

    init_trace_emitter(&e);
    e.self_key = key;
    // ROM traces have a spare register for carry even when they use the stack
    e.defer_carry = true;
    e.record_bus_writes = cache->record_bus_writes;
    // Shorter traces fit more often before a regular-mode timing deadline
    unsigned cycle_limit = e.record_bus_writes ? REGULAR_TRACE_MAX_CYCLES : TRACE_MAX_CYCLES;

    // Fit more entry points into the remainder of this frame's budget
    if (cache->used - cache->frame_arena_start >= TRACE_EMISSION_BUDGET / 2)
        cycle_limit /= 2;
    TraceExit exits[TRACE_MAX_EXITS];
    unsigned exit_count = 0, highest = 0, offset = 0, target = 0;

    // Keep the guest registers in host registers through the trace
    for (unsigned reg = REG_A; reg <= REG_P; ++reg)
        emit_imm(&e, 0x39400000, reg, 0, cpu_register_offset(reg));
    emit_imm(&e, 0x91000000, REG_RAM, 0, offsetof(Nes, ram));
    const unsigned start_pc = bank + (unsigned)(code - page);
    unsigned cursor = start_pc, instructions_compiled = 0;
    bool ends_with_rts = false, followed_jump = false;
    unsigned indirect_pointer = UINT32_MAX;
    // Keep one caller's return address and stack depth while tracing a nested call
    unsigned parent_return = 0, parent_pushes = 0;
    unsigned pushed = 0;
    unsigned return_pc = 0, extra_bank = 0, guard_literal = 0;
    bool has_cart_ram = cart->ram_size >= 8192 && cart->mapper != 1 && !cart->mmc5;
    // Guard the FME-7 ROM page used for table reads at $6000
    bool wram_rom = cart->mapper == 69;

    e.cart_ram_rom = wram_rom;
    // Route FDS stores through C to check the RAM code map
    bool fds = cart_is_fds(cart);

    if (fds) {
        e.code_map = cache->code_map;
        e.volatile_map = cache->volatile_map;
        e.code_base = cache->code_base;
    }
    e.indirect_wram_enable = cart->mapper == 4;
    bool checked_read = cart->mapper != 4 && !wram_rom, checked_write = checked_read;
    bool checked_refill = false, checked_pad = false;
    unsigned callouts = 0;
    const uint8_t *extra_page = NULL;

    for (unsigned i = 0; i < TRACE_MAX_INSTRUCTIONS && e.clocks + e.extra < cycle_limit; ++i) {
        // Reserve space for every exit stub as bus handler calls lengthen the body
        if (e.size > TRACE_MACHINE_WORDS - TRACE_EXIT_WORDS)
            break;
        unsigned current_bank = cursor & ~8191u;
        unsigned page_offset = cursor & 8191u;

        if (page_offset > 8189 || (current_bank != bank && current_bank != extra_bank))
            break;
        const uint8_t *current_page = current_bank == bank ? page : extra_page;
        const uint8_t *next = current_page + page_offset;

        if (volatile_code(&e, next, 3))
            break;
        note_code(&e, next, 3);
        int position = current_bank == bank ? (int)cursor - (int)start_pc : -1;
        unsigned opcode = next[0];
        Instruction instr = instructions[opcode];
        Operation op = (Operation)instr.op;
        Mode mode = instruction_mode(instr);
        Access access = instruction_access(instr);

        if (exit_count < TRACE_MAX_EXITS &&
            cart->mapper == 4 && !cart->bus_conflicts && mode == ABS &&
            (op == STA || op == STX || op == STY)) {
            unsigned address = combine(next[1], next[2]);

            if ((address & 0xe001) == 0x8000) {
                TraceExit *bailout = &exits[exit_count++];

                *bailout = trace_resume_exit(&e, cursor, instructions_compiled);
                unsigned reg = op == STA ? REG_A : op == STX ? REG_X : REG_Y;

                e.inputs |= (1u << reg) & ~e.dirty;
                // The existing mapping and raster deadline remain valid on success
                emit_mmc3_quiet_select(&e, bailout, reg);
                cursor += 3;
                ++instructions_compiled;
                continue;
            }
        }
        if (opcode == 0x0a || opcode == 0x4a) {
            unsigned count = 1;
            unsigned room = cycle_limit - e.clocks - e.extra;

            if (opcode == 0x4a && next[1] == 0x26 && room >= 14 &&
                page_offset <= 8192 - 6 && i + 4 <= TRACE_MAX_INSTRUCTIONS && emit_rotate_pair(&e, next)) {
                note_code(&e, next, 6);
                cursor += 6;
                instructions_compiled += 4;
                i += 3;
                continue;
            }
            while (count < 8 && i + count < TRACE_MAX_INSTRUCTIONS &&
                   count * 2 < room && page_offset + count <= 8189 && next[count] == opcode)
                ++count;
            if (count > 1) {
                note_code(&e, next, count + 1);
                emit_shift_run(&e, opcode, count, next[count]);
                cursor += count;
                instructions_compiled += count;
                i += count - 1;
                continue;
            }
        }
        // A bytewise add/subtract can use a single native carry chain when
        // the indexed ROM reads stay within their CPU pages.
        if (opcode == 0xa5 && (next[2] == 0x38 || next[2] == 0x18) &&
            page_offset <= 8192 - 22 && i + 10 <= TRACE_MAX_INSTRUCTIONS &&
            cycle_limit - e.clocks - e.extra >= 32 && exit_count < TRACE_MAX_EXITS &&
            !(e.dirty & (1u << REG_X))) {
            TraceExit bailout = trace_resume_exit(&e, cursor, instructions_compiled);

            if (emit_packed_arithmetic(&e, next, nes->cpu.x, &bailout.guards)) {
                note_code(&e, next, 22);
                if (bailout.guards)
                    exits[exit_count++] = bailout;
                cursor += 22;
                i += 9;
                instructions_compiled += 10;
                continue;
            }
        }
        if (mode == REL) {
            unsigned after = cursor + 2;
            TakenBranch branch_path = taken_branch(after, next[1]);
            uint16_t taken_pc = branch_path.target;
            unsigned taken_bus = current_page[branch_path.bus_address & 8191];

            note_code(&e, current_page + (branch_path.bus_address & 8191), 1);
            note_code(&e, current_page + (taken_pc & 8191), 1);
            uint32_t branch = trace_branch(&e, opcode);

            // A known condition is encoded as CBZ or CBNZ on the zero register
            if ((branch & 31) == REG_NONE) {
                bool taken = (branch & 0x01000000) == 0;

                e.bus = REG_NONE;
                e.bus_imm = taken ? taken_bus : next[1];
                e.clocks += taken ? 3 + branch_path.page_cross : 2;
                ++instructions_compiled;

                if (!taken) {
                    cursor += 2;
                    continue;
                }
                if (taken_pc >= after && taken_pc - start_pc <= TRACE_FORWARD_SPAN &&
                    (taken_pc & 0xe000) == current_bank && (taken_pc & 8191) <= 8189) {
                    cursor = taken_pc;
                    continue;
                }
                offset = trace_branch_exit(nes, current_page, after - 2, taken_pc, position);
                target = taken_pc;
                cursor += 2;
                break;
            }
            if (exit_count == TRACE_MAX_EXITS)
                break;
            unsigned taken_cycles = e.clocks + 3 + branch_path.page_cross;
            // A branch directly to RTS can return from the trace's own call
            bool return_on_branch = return_pc && !pushed && (taken_pc & 0xe000) == current_bank &&
                                   current_page[taken_pc & 8191] == 0x60 &&
                                   taken_cycles + e.extra + 6 <= cycle_limit;

            if (return_on_branch)
                taken_cycles += 6;
            exits[exit_count++] = (TraceExit){
                .check = e.size, .branch = branch,
                .offset = return_on_branch ? TRACE_DIRECT_EXIT
                                           : trace_branch_exit(nes, current_page, after - 2, taken_pc, position),
                .target = return_on_branch ? (uint16_t)return_pc : taken_pc,
                .bus = taken_bus, .bus_reg = REG_NONE,
                .clocks = taken_cycles,
                .dirty = e.dirty, .nz = nz_after_branch(e.nz, opcode, true),
                .extra = e.extra, .instructions = i + 1 + return_on_branch,
                .stack_pending = e.stack_pending,
                .carry_pending = e.carry_pending, .pop_return = return_on_branch
            };
            emit(&e, 0);
            if (taken_cycles + e.extra > highest)
                highest = taken_cycles + e.extra;
            cursor += 2;
            e.clocks += 2;
            e.bus = REG_NONE;
            e.bus_imm = next[1];
            e.nz = nz_after_branch(e.nz, opcode, false);
            ++instructions_compiled;
            continue;
        }
        if (op == RTS && return_pc && !pushed) {
            // This trace pushed the address, and the path to RTS left it intact
            emit_pop_return(&e);
            e.clocks += 6;
            e.bus = REG_NONE;
            const uint8_t *return_page = ((return_pc - 1) & 0xe000) == bank ? page : extra_page;

            e.bus_imm = return_page[(return_pc - 1) & 8191];
            note_code(&e, return_page + ((return_pc - 1) & 8191), 1);
            ++instructions_compiled;
            cursor = return_pc;
            return_pc = parent_return;
            pushed = parent_pushes;
            parent_return = parent_pushes = 0;
            continue;
        }
        if (op == RTS) {
            ends_with_rts = true;
            break;
        }
        if (op == JMP || op == JSR) {
            if (mode == IND) {
                unsigned address = combine(next[1], next[2]);

                if (address < 0x2000) {
                    indirect_pointer = address;
                    e.clocks += 5;
                    ++instructions_compiled;
                }
                break;
            }
            bool descend = op == JSR && !parent_return;

            if (op == JSR) {
                parent_return = return_pc;
                parent_pushes = pushed;
                return_pc = cursor + 3;
                pushed = 0;
                note_bus_writes(&e, e.clocks + 3, 2);
                emit_push_return(&e, return_pc - 1);
                e.clocks += 3;
            }
            target = combine(next[1], next[2]);
            e.bus = REG_NONE;
            e.bus_imm = next[2];
            e.clocks += 3;
            ++instructions_compiled;
            unsigned next_bank = target & 0xe000;
            bool nearby = op == JMP && next_bank == current_bank && target > cursor + 2 &&
                          target - start_pc <= TRACE_FORWARD_SPAN;
            bool call = descend && target >= 0x8000;
            // A leading jump can reuse the entry mapping to compile its destination.
            // Keep short, repeatable back edges available to the loop batcher.
            unsigned jump_exit = op == JMP ?
                trace_branch_exit(nes, current_page, cursor, target, position) : TRACE_DIRECT_EXIT;
            bool entry_jump = !i && (next_bank == bank || trace_prg_window_known(cart, bank, next_bank)) &&
                target >= 0x8000 && jump_exit == TRACE_DIRECT_EXIT;
            bool tail = op == JMP && ((return_pc && !followed_jump) || entry_jump) &&
                        target != cursor && target >= 0x8000;

            if ((target & 8191) <= 8189 && (nearby || call || tail)) {
                bool mapped = next_bank == bank || next_bank == extra_bank;
                bool fixed = mapped || trace_prg_window_known(cart, bank, next_bank);

                // Guard the extra ROM mapping once if the trace uses one
                if (mapped || (!extra_bank && (fixed || exit_count < TRACE_MAX_EXITS))) {
                    if (!mapped) {
                        extra_bank = next_bank;
                        extra_page = cart_prg_page(cart, target);

                        if (!fixed) {
                            unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, prg_pages) +
                                             ((next_bank >> 13) - 4) * sizeof(void *);

                            emit_imm(&e, 0xf9400000, REG_TEMP, 0, field / 8);
                            guard_literal = e.size;
                            emit(&e, 0); /* ldr x14, expected ROM page */
                            emit_reg(&e, 0xeb000000, REG_NONE, REG_TEMP, REG_TEMP2);
                            TraceExit *b = &exits[exit_count++];

                            *b = trace_resume_exit(&e, target, i + 1);
                            b->check = e.size;
                            b->branch = 0x54000001; /* b.ne */
                            emit(&e, 0);
                        }
                    }
                    cursor = target;
                    followed_jump |= tail;
                    continue;
                }
            }
            offset = jump_exit;
            cursor += 3;
            break;
        }
        if ((op == STA || op == STX || op == STY) && mode == ABS && exit_count < TRACE_MAX_EXITS &&
            callouts < 4) {
            unsigned address = combine(next[1], next[2]);
            // Mapper writes can remap code, and a strobe write changes the controller guard
            bool io = address >= 0x2000 && address <= 0x4017 && address != 0x4014 && address != 0x4016;
            bool fds_ram = fds && address >= 0x6000 && address < 0xe000;
            // VRC and FME-7 registers can stay in the trace unless they switch PRG banks
            bool ext = mapper_store_stays_in_trace(cart, address);

            if (io || ext || fds_ram) {
                TraceExit bailout = trace_resume_exit(&e, cursor, i);
                unsigned source = op == STA ? REG_A : op == STX ? REG_X : REG_Y;

                e.inputs |= (1u << source) & ~e.dirty;
                if (fds_ram && !fds_code_byte(cache->code_map, address) &&
                    !fds_loop_region(cart->ext->fds, address)) {
                    emit_fds_ram_store(&e, address, source, &bailout.guards);
                } else {
                    // The register is known here, so call its own quiet path
                    bool (*handler)(Nes *, uint16_t, uint8_t, uint64_t) = nes_write_quiet;

                    if (ext)
                        handler = nes_write_quiet_fixed_prg;
                    else if (fds_ram)
                        handler = fds_write_quiet_data;
                    else if ((address & 0xe007) == 0x2006)
                        handler = nes_write_quiet_ppu_address;
                    else if ((address & 0xe007) == 0x2007)
                        handler = nes_write_quiet_ppu_data;
                    else if (apu_waveform_register(address))
                        handler = nes_write_quiet_waveform;
                    emit_quiet_store(&e, address, source, &bailout.guards, handler);
                    ++callouts;
                }
                // Use the C path when a DMC halt would delay this access
                note_bus_writes(&e, 0, e.clocks + 4);
                e.clocks += 4;
                e.bus = source;
                if (ext && wram_rom) {
                    // The write can map another page at $6000
                    e.cart_ram_window = 0;
                    checked_read = false;
                }
                exits[exit_count++] = bailout;

                // APU control writes can move the DMC refill and stream state
                if (address >= 0x4000 && !apu_waveform_register(address))
                    checked_refill = checked_pad = false;
                cursor += 3;
                ++instructions_compiled;
                continue;
            }
        }
        if (access == READ && (mode == ABS || mode == ABX || mode == ABY) &&
            (op == LDA || op == LDX || op == LDY || op == AND || op == ORA || op == EOR ||
             op == CMP || op == BIT) && exit_count < TRACE_MAX_EXITS) {
            unsigned address = combine(next[1], next[2]);

            if ((address & 0xfffe) == 0x4016 && (mode == ABS || address == 0x4016)) {
                TraceExit bailout = trace_resume_exit(&e, cursor, i);
                unsigned before = e.size, previous_wram_check = e.wram_check;

                if (!checked_pad) {
                    // Strobe held or a pending latch takes the C path
                    emit_imm(&e, 0x79400000, REG_TEMP, 0, offsetof(Nes, pad_strobe) / 2);
                    emit_guard(&e, &bailout.guards, 0x35000000 | REG_TEMP); /* cbnz */
                    emit_imm(&e, 0x39400000, REG_TEMP, 0, offsetof(Nes, apu) + offsetof(Apu, stream_strobe));
                    emit_guard(&e, &bailout.guards, 0x35000000 | REG_TEMP); /* cbnz */
                }
                if (!checked_refill)
                    emit_wram_refill_guard(&e, REG_TEMP, REG_TEMP2, &bailout.guards);
                if (mode != ABS) {
                    unsigned index = mode == ABX ? REG_X : REG_Y;

                    e.inputs |= (1u << index) & ~e.dirty;
                    emit_imm(&e, 0x71000000, REG_NONE, index, 1);
                    emit_guard(&e, &bailout.guards, 0x54000000 | 8); /* b.hi */
                }
                e.pad_operand = true;
                bool emitted = emit_opcode(&e, next, op, mode, access, NULL);

                e.pad_operand = false;

                if (!emitted) {
                    e.size = before;
                    e.wram_check = previous_wram_check;
                    break;
                }
                checked_pad = checked_refill = true;
                e.wram_end = e.clocks + e.extra;

                if (bailout.guards)
                    exits[exit_count++] = bailout;
                cursor += mode_lengths[mode];
                ++instructions_compiled;
                continue;
            }
        }
        bool cart_load = access == READ;
        bool cart_store = access == WRITE && !cart->nvram_size && !fds;

        if (((has_cart_ram && (cart_load || cart_store)) || (wram_rom && cart_load)) &&
            (mode == ABS || mode == ABX || mode == ABY)) {
            unsigned address = combine(next[1], next[2]);

            if (address >= 0x6000 && address < 0x8000) {
                bool check_index = mode != ABS && address > 0x7f00;
                bool check_ram = cart_load ? !checked_read : !checked_write;
                bool check_stream = !checked_refill && (cart_load || mode != ABS);
                bool guarded = check_ram || check_stream || check_index;

                if (guarded && ((!i && !cart_load) || exit_count == TRACE_MAX_EXITS))
                    break;
                TraceExit bailout = trace_resume_exit(&e, cursor, i);
                unsigned before = e.size;

                if (check_ram && wram_rom) {
                    // Load the mapped ROM page and leave RAM and open bus to the C path
                    emit_wram_rom_base(&e, 6);
                    emit_guard(&e, &bailout.guards, 0xb4000000 | REG_CART_RAM); /* cbz */
                    checked_read = true;
                } else if (check_ram) {
                    unsigned field = offsetof(Nes, cart) + offsetof(Cartridge, mmc3.ram_control);

                    emit_imm(&e, 0x39400000, REG_TEMP, 0, field);
                    if (cart_store) {
                        emit_extract(&e, REG_TEMP, REG_TEMP, 6, 2);
                        emit_imm(&e, 0x71000000, REG_NONE, REG_TEMP, 2);
                        emit_guard(&e, &bailout.guards, 0x54000001); /* b.ne */
                    } else {
                        emit_guard(&e, &bailout.guards, 0x36380000 | REG_TEMP); /* tbz wTemp, #7 */
                    }
                    checked_read = true;
                    checked_write |= cart_store;
                }
                unsigned previous_wram_check = e.wram_check;

                if (check_stream) {
                    // A streamed refill can stall a WRAM read or an indexed store's dummy read
                    emit_wram_refill_guard(&e, REG_TEMP, REG_TEMP2, &bailout.guards);
                    checked_refill = true;
                }
                if (check_index) {
                    // An indexed access at the top of WRAM can enter the mapper's ROM window
                    unsigned index = mode == ABX ? REG_X : REG_Y;

                    e.inputs |= (1u << index) & ~e.dirty;
                    emit_imm(&e, 0x71000000, REG_NONE, index, 256 - (address & 255));
                    emit_guard(&e, &bailout.guards, 0x54000000 | ARM_HS);
                }
                e.cart_ram_operand = true;

                if (!emit_opcode(&e, next, op, mode, access, NULL)) {
                    e.size = before;
                    e.wram_check = previous_wram_check;
                    break;
                }
                e.cart_ram_operand = false;
                e.wram_end = e.clocks + e.extra;

                if (bailout.guards)
                    exits[exit_count++] = bailout;
                cursor += mode_lengths[mode];
                ++instructions_compiled;
                continue;
            }
        }
        if (return_pc) {
            // A write can reach the saved return address through a mirror of page $0100
            bool stack_address = access != READ && mode != ACC &&
                                 mode != ZP && mode != ZPX && mode != ZPY;

            if (stack_address && (mode == ABS || mode == ABX || mode == ABY)) {
                unsigned address = combine(next[1], next[2]) & 2047;
                stack_address = address < 0x200 && address + (mode == ABS ? 0 : 255) >= 0x100;
            }
            if (op == TXS || stack_address || (op == PLA && !pushed))
                return_pc = parent_return = parent_pushes = 0;
            else if (op == PHA || op == PHP)
                ++pushed;
            else if (op == PLA)
                --pushed;
        }
        if (mode == IZY && (access == READ || op == STA) && exit_count < TRACE_MAX_EXITS) {
            // Capture the state before the load. The interpreter retries the
            // whole instruction when its effective address fails the guard.
            TraceExit bailout = trace_resume_exit(&e, cursor, i);

            e.indirect_wram = has_cart_ram && (access == READ || (!cart->nvram_size && !fds)) &&
                              (nes->ram[(uint8_t)(next[1] + 1)] & 0xe0) == 0x60;

            if (!emit_opcode(&e, next, op, mode, access, &bailout.guards))
                break;
            exits[exit_count++] = bailout;
        } else if (!emit_opcode(&e, next, op, mode, access, NULL)) {
            break;
        }
        cursor += mode_lengths[mode];
        ++instructions_compiled;

        if ((opcode == 0xa2 || opcode == 0xa0) && next[1] >= 2 && next[1] < 16 &&
            instructions_compiled >= TRACE_MIN_INSTRUCTIONS && current_bank == bank &&
            (cursor & ~8191u) == bank) {
            unsigned loop_exit = trace_loop_entry_exit(nes, page, cursor, start_pc);

            if (loop_exit >= TRACE_LOOP_EXIT) {
                offset = loop_exit;
                target = cursor;
                break;
            }
        }
    }
    if (!instructions_compiled && !ends_with_rts)
        return false;

    if (ends_with_rts) {
        uint8_t sp = nes->cpu.s;
        uint16_t saved = combine(nes->ram[0x100 | (uint8_t)(sp + 1)],
                                 nes->ram[0x100 | (uint8_t)(sp + 2)]);

        if (!instructions_compiled) {
            // A standalone RTS is useful when it can link back to its caller
            if (((uint16_t)(saved + 1) & 0xe000) != bank)
                return false;
            // Retry a wrapped stack through direct dispatch
            offset = TRACE_DIRECT_EXIT;
            target = start_pc;
        }
        emit_rom_rts(&e, instructions_compiled + 1, cache, page, bank, saved);
        unsigned rts_clocks = e.clocks + 6 + e.extra;

        if (highest < rts_clocks)
            highest = rts_clocks;
    }

    if (!offset && (cursor - start_pc >= TRACE_FORWARD_SPAN || (cursor & ~8191u) != bank)) {
        offset = TRACE_DIRECT_EXIT;
        target = (uint16_t)cursor;
    }
    unsigned write_reg = REG_NONE;

    // Tell the runner about a possible store without committing it in generated code
    if (!offset && !ends_with_rts && (cursor & 8191) <= 8189) {
        const uint8_t *next = page + (cursor & 8191);
        unsigned opcode = next[0];

        if ((opcode == 0x8c || opcode == 0x8d || opcode == 0x8e) && !volatile_code(&e, next, 3)) {
            unsigned address = combine(next[1], next[2]);

            if ((address & 0xe000) == 0x8000 || mmc5_prg_remap(address) ||
                (address & 0xfff0) == 0x4000 || (address & 0xe006) == 0x2006 ||
                (cart_has_quiet_registers(cart) && address >= 0x8000)) {
                write_reg = opcode == 0x8d ? REG_A : opcode == 0x8e ? REG_X : REG_Y;
                e.inputs |= (1u << write_reg) & ~e.dirty;
                target = address;
                note_code(&e, next, 3);
            }
        }
    }
    if (instructions_compiled + ends_with_rts < TRACE_MIN_INSTRUCTIONS) {
        // A link or a bus handoff can make one native instruction worthwhile
        bool local_call = *code == 0x20 && (cursor & 0xe000) == bank;

        if (indirect_pointer == UINT32_MAX &&
            (!ends_with_rts && !local_call && write_reg == REG_NONE))
            return false;
    }
    emit_write_registers(&e, e.dirty, e.nz);
    if (indirect_pointer != UINT32_MAX) {
        emit_indirect_jump(&e, indirect_pointer, instructions_compiled);
    } else {
        if (offset == TRACE_DIRECT_EXIT && e.clocks)
            emit_trace_link(&e, cache, page, bank, target, e.clocks, e.extra,
                            instructions_compiled, e.inputs | e.dirty);
        else if (!offset && !ends_with_rts && write_reg == REG_NONE)
            emit_trace_link(&e, cache, page, bank, cursor, e.clocks, e.extra,
                            instructions_compiled, e.inputs | e.dirty);
        unsigned next_pc = offset >= TRACE_LOOP_EXIT ? start_pc + offset - TRACE_LOOP_EXIT : cursor;

        emit_rom_return(&e, e.clocks, e.bus, e.bus_imm, e.extra, offset, target,
                        instructions_compiled, write_reg, (uint16_t)next_pc, bank);
    }
    for (unsigned i = 0; i < exit_count; ++i) {
        TraceExit *b = &exits[i];

        if (b->guards)
            patch_guards(&e, b->guards);
        else
            e.words[b->check] = b->branch | ((e.size - b->check) << 5);
        e.carry_pending = b->carry_pending;
        e.stack_pending = b->stack_pending;

        if (b->pop_return)
            e.stack_pending += 2;
        emit_write_registers(&e, b->dirty | (b->pop_return ? 1u << REG_S : 0), b->nz);
        // A failed entry guard must return to direct dispatch rather than link
        if (b->offset == TRACE_DIRECT_EXIT && b->clocks)
            emit_trace_link(&e, cache, page, bank, b->target, b->clocks,
                            b->extra, b->instructions, e.inputs | b->dirty);
        emit_rom_return(&e, b->clocks, b->bus_reg, b->bus, b->extra, b->offset, b->target,
                        b->instructions, REG_NONE, (uint16_t)(start_pc + b->offset - TRACE_LOOP_EXIT), bank);
    }
    for (unsigned i = 0; i < e.stack_wrap_count; ++i) {
        // Keep stack-page wrapping off the ordinary JSR path
        unsigned check = e.stack_wraps[i].check;
        unsigned address = e.stack_wraps[i].address;

        e.words[check] = 0x34000000 | ((e.size - check) << 5) | REG_S;
        emit_constant(&e, REG_VALUE, address >> 8);
        emit_imm(&e, 0x39000000, REG_VALUE, REG_RAM, 256);
        emit_constant(&e, REG_VALUE, address & 255);
        emit_imm(&e, 0x39000000, REG_VALUE, REG_RAM, 511);
        unsigned back = e.stack_wraps[i].resume - e.size;

        emit(&e, 0x14000000 | (back & 0x03ffffff));
    }
    if (guard_literal) {
        // Each stub returns before this pointer literal
        e.words[guard_literal] = 0x58000000 | ((e.size - guard_literal) << 5) | REG_TEMP2;
        uintptr_t expected = (uintptr_t)extra_page;

        emit(&e, (uint32_t)expected);
        emit(&e, (uint32_t)(expected >> 32));
    }
    // The bound is one cycle beyond the last possible WRAM access, including page crosses
    for (unsigned at = e.wram_check; at;) {
        uint32_t word = e.words[at];

        e.words[at] = (word & ~(UINT32_C(4095) << 10)) | (e.wram_end << 10);
        at = (word >> 10) & 4095;
    }
    unsigned clocks = e.clocks + e.extra > highest ? e.clocks + e.extra : highest;

    // ROM entry offsets and write masks are packed in units of host instructions
    cache->used = (cache->used + BLOCK_ALIGNMENT - 1) & ~(BLOCK_ALIGNMENT - 1);
    bool has_writes = (e.bus_writes[0] | e.bus_writes[1]) != 0;

    if (has_writes)
        cache->used += DMC_WRITE_MASK_BYTES;
    e.publish_key = key;
    e.publish_layout = (cache->used / BLOCK_ALIGNMENT) | (clocks << BLOCK_ENTRY_BITS);

    if (e.record_bus_writes) {
        e.publish_layout |= BLOCK_CHECKS_WRITES;

        // Starting this trace must reach the halt even if linking is unavailable
        if (!exit_count && !e.extra && !ends_with_rts && indirect_pointer == UINT32_MAX &&
            !guard_literal && !e.wram_check)
            e.publish_layout |= BLOCK_DMC_FIXED_LENGTH;
    }
    if (has_writes)
        e.publish_layout |= BLOCK_HAS_WRITE_MASK;
    publish_trace(cache, &e);
    block->layout = e.publish_layout;
    *reusable = *block;
    return true;
}

static inline BlockTrace *find_trace(CpuTraceCache *cache, const Nes *nes, const uint8_t *page,
                                    const uint8_t *code, unsigned bank, bool warm, bool fds)
{
    uintptr_t address = trace_address(page, code, bank, fds);
    unsigned slot = trace_slot(address);
    // The full key distinguishes different CPU windows into the same ROM
    uint32_t key = trace_key(address, bank);
    BlockTrace *trace = &cache->blocks[slot];

    if (trace->key != key) {
        if (warm)
            *trace = (BlockTrace){.key = key, .layout = 1};
        return NULL;
    }
    if (trace->layout >= (1u << BLOCK_ENTRY_BITS))
        return trace;
    if (!warm)
        return NULL;
    if (trace->layout < TRACE_WARM_HITS) {
        ++trace->layout;
    } else if (trace->layout == TRACE_WARM_HITS && cache->remaining_compiles) {
        --cache->remaining_compiles;
        if (!trace_arena_has_room(cache))
            return NULL;
        ++trace->layout;
        if (compile_trace(cache, trace, page, code, bank, nes))
            return trace;
    }
    return NULL;
}

// Keep the DMC and RAM trace runners on their measured cache-line offsets
NATIVE_ENTRY_AT(28)
static NES_NOINLINE unsigned run_dmc_trace(Nes *nes, const uint32_t *arena, uint32_t layout,
                                           uint64_t deadline, unsigned budget)
{
    const uint32_t *code = arena + (layout & BLOCK_ENTRY_MASK) * BLOCK_WORDS;

    if (layout & BLOCK_HAS_WRITE_MASK) {
        unsigned halt_offset = budget - 1;
        uint64_t writes;

        memcpy(&writes, code - DMC_WRITE_MASK_WORDS + (halt_offset >> 6) * (sizeof writes / sizeof *code),
               sizeof writes);
        if (writes & (UINT64_C(1) << (halt_offset & 63)))
            return 0;
    }

    uint64_t start = deadline - budget;
    unsigned limit = (layout >> BLOCK_ENTRY_BITS) & BLOCK_CYCLES_MASK;
    uint64_t raster_limit = nes_dmc_trace_deadline(nes, deadline - 1, start + limit);

    if (!raster_limit)
        return 0;

    // Once the exact refill is queued, the fixed-length trace can link using
    // the new deadline. Reserve the four stalled CPU cycles in that window.
    // Otherwise linked successors must fit entirely before the halt.
    bool queued = (layout & BLOCK_DMC_FIXED_LENGTH) && budget <= limit &&
                  nes_dmc_queue_trace(nes, deadline - 1, raster_limit);
    unsigned link_budget = queued ? (uint16_t)(nes->bus_deadline - start - 4) : budget;
    RomTraceEntry entry;

    memcpy(&entry, &code, sizeof entry);
    RomTraceResult packet = entry(nes, link_budget << TRACE_LINK_BUDGET_SHIFT);
    uint64_t result = packet.result;
    unsigned cycles = (result & TRACE_CYCLE_MASK) +
        link_budget - (uint16_t)(packet.resume >> TRACE_LINK_BUDGET_SHIFT);

    assert(!queued || cycles >= budget);
    if (!cycles)
        return 0;
    nes->cpu.pc = result & TRACE_WRITE ? packet.resume >> TRACE_WRITE_PC_SHIFT : result >> TRACE_TARGET_SHIFT;
    complete_quiet_accesses(nes, result >> TRACE_BUS_SHIFT, (unsigned)(start - nes->cpu.cycles) + cycles);
    if (cycles >= budget) {
        if (CPU_DEBUG && nes->cpu_raster) {
            cpu_record_execution(nes->cpu_raster, deadline - 1, CPU_EXEC_RECOMPILED);
            cpu_record_execution(nes->cpu_raster, deadline + 3, CPU_EXEC_DMA);
        }
        if (queued)
            nes->cpu.cycles += 4;
        else
            nes_dmc_finish_trace(nes, deadline - 1, raster_limit);
    }
    if (CPU_DEBUG && nes->cpu_raster)
        cpu_record_execution(nes->cpu_raster, nes->cpu.cycles, CPU_EXEC_RECOMPILED);
    return (packet.resume & TRACE_LINK_COUNT_MASK) + (uint8_t)(result >> TRACE_COUNT_SHIFT) + 1u;
}


enum { REG_BUDGET=1, REG_REMAINING=16, REG_EXECUTED=17 };

// A loop returns its cycles, final bus value, next PC, and an instruction count
static void emit_loop_return(TraceEmitter *e, unsigned dirty, unsigned nz,
                             unsigned clocks, unsigned extra, unsigned instructions,
                             unsigned bus, unsigned pc, bool variable_cycles)
{
    // Recover the elapsed cycles from the remaining iteration budget
    emit_reg(e, 0x4b000000, REG_REMAINING, REG_BUDGET, REG_REMAINING);
    if (clocks)
        emit_imm(e, 0x11000000, REG_REMAINING, REG_REMAINING, clocks);
    if (extra)
        emit_reg(e, ARM_ADD, REG_REMAINING, REG_REMAINING, REG_CYCLES);
    if (variable_cycles && instructions)
        emit_imm(e, 0x11000000, REG_EXECUTED, REG_EXECUTED, instructions);
    if (e->carry_cached)
        emit_insert(e, REG_P, REG_LOOP_CARRY, 0, 1);
    emit_write_registers(e, dirty, nz);
    emit_move(e, 0, REG_REMAINING);
    emit(e, 0x72a00000 | (bus << 5)); /* movk w0, #bus, lsl #16 */
    emit(e, 0xf2c00000 | (pc << 5)); /* movk x0, #pc, lsl #32 */
    if (variable_cycles)
        emit(e, 0xb3400000 | ((64 - LOOP_COUNT_SHIFT) << 16) | (15 << 10) | (REG_EXECUTED << 5));
    else if (instructions)
        emit(e, 0xf2e00000 | (instructions << 5)); /* movk x0, #instructions, lsl #48 */
    emit(e, 0xd65f03c0);
}

static bool overflow_overwritten(const uint8_t *code, const uint8_t *end)
{
    // A branch can exit the loop with the current flags, even when it
    // tests a flag other than overflow. Stop the search at every branch.
    while (code < end) {
        Instruction next = instructions[*code];
        Mode mode = instruction_mode(next);

        if (mode == REL)
            return false;
        if (next.op == ADC || next.op == SBC || next.op == BIT || next.op == CLV)
            return true;
        code += mode_lengths[mode];
    }
    return false;
}

static bool cache_loop_carry(const uint8_t *code, const uint8_t *end)
{
    unsigned count = 0;

    for (; code < end; code += mode_lengths[instruction_mode(instructions[*code])]) {
        Operation op = (Operation)instructions[*code].op;

        if ((op == ADC || op == SBC) && ++count == 2)
            return true;
    }
    return false;
}

// Byte logical operations can carry the unused ninth accumulator bit until the next add
static bool loop_add_follows(const uint8_t *code, const uint8_t *end)
{
    while (code < end) {
        Instruction next = instructions[*code];

        if (next.op == ADC || next.op == SBC)
            return true;
        if (next.op != AND && next.op != ORA && next.op != EOR)
            return false;
        code += mode_lengths[instruction_mode(next)];
    }
    return false;
}

static bool compile_loop_trace(CpuTraceCache *cache, CpuTrace *trace, const uint8_t *code,
                               unsigned pc, const ShortLoop *loop, unsigned longest)
{
    TraceEmitter e;

    init_trace_emitter(&e);
    e.loop_body = true;
    TraceExit exits[TRACE_MAX_EXITS + 1];
    unsigned exit_count = 0, back = loop->back, bus_reads = 0;
    int16_t bus_offsets[TRACE_MAX_EXITS];
    bool variable_cycles = loop->indexed_reads != 0;

    // Preserve carry across short-loop arithmetic in the unused stack register
    e.carry_cached = cache_loop_carry(code, code + back);
    uint8_t delta_x = 0, delta_y = 0;
    bool writes_x = false, writes_y = false;
    bool writes_nz = false, preserve_nz = false;

    for (unsigned reg = REG_A; reg <= REG_P; ++reg)
        emit_imm(&e, 0x39400000, reg, 0, cpu_register_offset(reg));
    emit_imm(&e, 0x91000000, REG_RAM, 0, offsetof(Nes, ram));
    if (e.carry_cached)
        emit_extract(&e, REG_LOOP_CARRY, REG_P, 0, 1);
    emit_imm(&e, 0x51000000, REG_BUDGET, REG_BUDGET, longest);
    emit_move(&e, REG_REMAINING, REG_BUDGET);
    if (variable_cycles)
        emit_constant(&e, REG_EXECUTED, 0);
    unsigned header = e.size, executed = 0;
    // The short-loop decoder already checked the branch targets and isolated
    // the body from I/O and mapper writes.

    for (unsigned offset = 0; offset < back;) {
        const uint8_t *at = code + offset;
        unsigned opcode = at[0];
        Instruction instr = instructions[opcode];
        Mode mode = instruction_mode(instr);
        Operation op = (Operation)instr.op;

        ++executed;

        if (mode == REL) {
            preserve_nz |= !writes_nz;

            if (exit_count == TRACE_MAX_EXITS)
                return false;
            TakenBranch path = taken_branch(pc + offset + 2, at[1]);

            bus_offsets[bus_reads++] = (int16_t)(path.bus_address - pc);
            exits[exit_count++] = (TraceExit){
                .check = e.size, .branch = trace_branch(&e, opcode), .target = path.target,
                .bus = code[(int)path.bus_address - (int)pc],
                .clocks = e.clocks + 3 + path.page_cross, .dirty = e.dirty, .nz = e.nz,
                .extra = e.extra, .instructions = executed
            };
            emit(&e, 0);
            e.clocks += 2;
        } else if (op == INX || op == DEX || op == INY || op == DEY) {
            unsigned count = 1;

            while (offset + count < back && at[count] == opcode)
                ++count;
            emit_index_change(&e, op, count);
            e.clocks += count * 2;
            offset += count - 1;
            executed += count - 1;
            change_index(op, count, &delta_x, &delta_y);
        } else {
            Access access = instruction_access(instr);

            if (op == ADC || op == SBC) {
                const uint8_t *next = at + mode_lengths[mode];

                e.write_overflow = !overflow_overwritten(next, code + back);
                e.keep_add_sum = !e.write_overflow && loop_add_follows(next, code + back);
            }
            if (!emit_opcode(&e, at, op, mode, access, NULL))
                return false;
            writes_x |= op == LDX || op == TAX;
            writes_y |= op == LDY || op == TAY;
        }
        writes_nz |= op == BIT || e.nz != REG_NONE;
        offset += mode_lengths[mode];
    }

    // A read-only loop with a fixed point is cheaper in the interpreter
    if (!(loop->flags & LOOP_WRITES_RAM) && (!delta_x || writes_x) && (!delta_y || writes_y))
        return false;

    ++executed;

    if (code[back] != 0x4c) {
        // Bit 24 switches CBZ/TBZ and CBNZ/TBNZ to their opposite condition
        uint32_t branch = trace_branch(&e, code[back]) ^ 0x01000000u;

        exits[exit_count++] = (TraceExit){
            .check = e.size, .branch = branch, .target = pc + back + 2,
            .bus = code[back + 1], .clocks = e.clocks + 2, .dirty = e.dirty,
            .nz = e.nz, .extra = e.extra, .instructions = executed
        };
        emit(&e, 0);
    }
    e.clocks += 3;

    if (e.clocks + e.extra != longest)
        return false;
    if (variable_cycles)
        emit_imm(&e, 0x11000000, REG_EXECUTED, REG_EXECUTED, executed);

    // Before an early exit the next iteration may still need the old N and Z
    if (preserve_nz)
        e.dirty = emit_status_flags(&e, e.dirty, e.nz);
    if (e.extra)
        emit_reg(&e, 0x4b000000, REG_REMAINING, REG_REMAINING, REG_CYCLES);
    emit_imm(&e, 0x71000000, REG_REMAINING, REG_REMAINING, e.clocks);
    emit(&e, 0x54000000 | (((header - e.size) & 0x7ffff) << 5) | ARM_GT);
    emit_loop_return(&e, e.dirty, preserve_nz ? REG_NONE : e.nz,
                     0, 0, 0, code[back + 2], pc, variable_cycles);
    for (unsigned i = 0; i < exit_count; ++i) {
        TraceExit *b = &exits[i];

        e.words[b->check] = b->branch | ((e.size - b->check) << 5);
        emit_loop_return(&e, b->dirty | e.dirty, b->nz, b->clocks, b->extra,
                         b->instructions, b->bus, b->target, variable_cycles);
    }

    LoopGuard *guard = cache->loop_guards ? &cache->loop_guards[trace - cache->loops] : NULL;

    if (guard) {
        guard->length = (uint8_t)(back + 3);
        memcpy(guard->bytes, code, back + 3);
        // A taken exit's bus byte can lie past the loop
        guard->exits = (uint8_t)bus_reads;

        for (unsigned i = 0; i < bus_reads; ++i) {
            guard->exit_offset[i] = bus_offsets[i];
            guard->exit_bus[i] = code[bus_offsets[i]];
        }
    }
    trace->clocks = longest;
    trace->arena_word = publish_trace(cache, &e);
    return true;
}

static inline bool native_loop_key_matches(const CpuTrace *trace, uintptr_t address, unsigned pc, unsigned back)
{
    return trace->address == address && trace->bank == pc >> 13 && trace->bytes == back;
}

static NES_NOINLINE bool prepare_native_short_loop(CpuTraceCache *cache, CpuTrace *trace,
                                                    uintptr_t address, const uint8_t *code, unsigned pc,
                                                    const ShortLoop *loop, unsigned longest)
{
    if (!native_loop_key_matches(trace, address, pc, loop->back)) {
        *trace = (CpuTrace){.address = address, .bank = pc >> 13, .bytes = loop->back, .hits = 1};
        return false;
    }
    if (trace->hits != 1 || !cache->remaining_compiles || !trace_arena_has_room(cache))
        return false;
    trace->hits = 2;
    --cache->remaining_compiles;
    return compile_loop_trace(cache, trace, code, pc, loop, longest);
}

static inline CpuTrace *native_loop_slot(CpuTraceCache *cache, uintptr_t address, unsigned pc)
{
    return &cache->loops[(address ^ (address >> 13) ^ (pc >> 6)) & (LOOP_TRACE_SLOTS - 1)];
}

static inline bool native_loop_matches(const CpuTrace *trace, uintptr_t address, unsigned pc, unsigned back)
{
    return trace->arena_word && native_loop_key_matches(trace, address, pc, back);
}

static inline uint64_t execute_native_loop(Nes *nes, const CpuTraceCache *cache, const CpuTrace *trace,
                                           uint64_t available)
{
    const uint32_t *address = cache->arena + trace->arena_word - 1;
    LoopTraceEntry execute_loop;

    memcpy(&execute_loop, &address, sizeof execute_loop);
    unsigned budget = available < LOOP_BUDGET_MAX ? (unsigned)available : LOOP_BUDGET_MAX;

    return execute_loop(nes, budget);
}

static inline unsigned native_loop_instructions(const ShortLoop *loop, uint64_t result)
{
    unsigned instructions = result >> LOOP_COUNT_SHIFT;

    // A partial exit takes fewer cycles than one full iteration
    if (!loop->indexed_reads)
        instructions += ((uint16_t)result / loop->cycles) * loop->instructions;
    return instructions;
}

static NES_NOINLINE bool loop_guard_matches(const LoopGuard *guard, const uint8_t *code)
{
    if (memcmp(guard->bytes, code, guard->length))
        return false;
    for (unsigned i = 0; i < guard->exits; ++i) {
        if (code[guard->exit_offset[i]] != guard->exit_bus[i])
            return false;
    }
    return true;
}

static NES_ALWAYS_INLINE uint64_t run_cached_loop(Nes *nes, CpuTraceCache *cache, const ShortLoop *loop,
                                                  const uint8_t *code, uint64_t available)
{
    // A mapper remap, slot collision, or arena reset can invalidate the hint
    const CpuTrace *trace = &cache->loops[loop->native_slot];

    if (native_loop_matches(trace, loop->code_key, nes->cpu.pc, loop->back) && available > trace->clocks &&
        (!cache->loop_guards || loop_guard_matches(&cache->loop_guards[loop->native_slot], code)))
        return execute_native_loop(nes, cache, trace, available);
    return 0;
}

static NES_ALWAYS_INLINE bool run_native_short_loop(Nes *nes, const uint8_t *code, ShortLoop *loop,
                                 unsigned longest, uint64_t available, uint32_t *batched)
{
    CpuTraceCache *cache = nes->cart.trace_cache;

    if (!cache)
        return false;
    const Fds *borrowed = borrowed_fds_traces(&nes->cart, cache);

    if (borrowed && !borrowed->code_map)
        return false;
    unsigned pc = nes->cpu.pc, back = loop->back;
    uintptr_t address = loop->code_key;
    CpuTrace *trace = native_loop_slot(cache, address, pc);

    if (!native_loop_matches(trace, address, pc, back) &&
        (borrowed || !prepare_native_short_loop(cache, trace, address, code, pc, loop, longest)))
        return false;
    if (cache->loop_guards && !loop_guard_matches(&cache->loop_guards[trace - cache->loops], code)) {
        // Recompile the modified loop on its next run
        if (!borrowed)
            *trace = (CpuTrace){.address = address, .bank = pc >> 13, .bytes = back, .hits = 1};
        return false;
    }
    loop->native_slot = (uint8_t)(trace - cache->loops);
    loop->flags |= LOOP_NATIVE;
    uint64_t result = execute_native_loop(nes, cache, trace, available);

    nes->cpu.pc = result >> LOOP_TARGET_SHIFT;
    complete_quiet_accesses(nes, result >> LOOP_BUS_SHIFT, (uint16_t)result);
    if (batched)
        *batched += native_loop_instructions(loop, result);
    return true;
}

static uint64_t ram_block_writes(unsigned start, const uint8_t *code, Mode mode)
{
    unsigned address = code[1], count = 1;

    if (mode == ZPX || mode == ZPY) {
        address = 0;
        count = 256;
    } else if (mode == ABS || mode == ABX || mode == ABY) {
        address = combine(code[1], code[2]);

        if (mode != ABS)
            count = 256;
    }
    // Project the store range onto this 64-byte trace window, including RAM mirrors
    unsigned offset = (address - start) & 2047;

    if (offset < RAM_BYTES)
        return count == 1 ? UINT64_C(1) << offset : UINT64_MAX << offset;
    if (offset + count <= 2048)
        return 0;
    unsigned wrapped = offset + count - 2048;

    return wrapped >= RAM_BYTES ? UINT64_MAX : (UINT64_C(1) << wrapped) - 1;
}

static NES_NOINLINE void compile_ram_block(CpuTraceCache *cache, RamBlock *block,
                                          const uint8_t *source, unsigned start)
{
    TraceEmitter e;

    init_trace_emitter(&e);
    // Keep carry in a host register between guest arithmetic instructions
    e.defer_carry = true;

    for (unsigned reg = REG_A; reg <= REG_P; ++reg)
        emit_imm(&e, 0x39400000, reg, 0, cpu_register_offset(reg));
    emit_imm(&e, 0x91000000, REG_RAM, 0, offsetof(Nes, ram));
    unsigned length = 0, count = 0, last = 0;
    uint64_t guarded = 0, written = 0;
    uint32_t delay_branch = 0;

    while (length + 3 <= RAM_BYTES && count < TRACE_MAX_INSTRUCTIONS &&
           e.clocks + e.extra < RAM_TRACE_MAX_CYCLES) {
        const uint8_t *code = source + length;
        Instruction instr = instructions[code[0]];
        Operation op = (Operation)instr.op;
        Mode mode = instruction_mode(instr);
        Access access = instruction_access(instr);

        if ((mode == REL && code[1]) || op == PHA || op == PHP || op == PLA || op == TSX || op == TXS)
            break;
        unsigned size = mode_lengths[mode];
        // Read writable operands at runtime. Stop before any earlier store
        // can change a byte that the emitted instructions assume is constant.
        bool live = mode == IMM || (mode == ABS && access == READ);
        uint64_t check = ((UINT64_C(1) << size) - 1) << length;

        if (live)
            check &= ~(UINT64_C(1) << (length + 1));
        if (check & written)
            break;
        e.ram_operand = live ? start + length + 1 : 0;

        if (mode == REL) {
            // A branch to the next instruction only changes the cycle count
            uint32_t branch = trace_branch(&e, code[0]);

            e.clocks += 2;

            if ((branch & 31) == REG_NONE) {
                e.clocks += (branch & 0x01000000) == 0;
            } else {
                if (!e.extra)
                    emit_constant(&e, REG_CYCLES, 0);
                ++e.extra;
                emit(&e, (branch ^ 0x01000000u) | (2 << 5));
                emit_imm(&e, 0x11000000, REG_CYCLES, REG_CYCLES, 1);
            }
            delay_branch = branch;
        } else if (!emit_opcode(&e, code, op, mode, access, NULL)) {
            break;
        } else {
            delay_branch = 0;
        }
        guarded |= check;

        if ((access == WRITE || access == MODIFY) && mode != ACC)
            written |= ram_block_writes(start, code, mode);
        last = length;
        length += size;
        ++count;
    }
    if (count < 4)
        return;
    if (delay_branch) {
        // A taken branch drives the following opcode; otherwise the operand is zero
        e.bus = REG_NONE;
        e.bus_imm = 0;
        bool constant = (delay_branch & 31) == REG_NONE;

        if (!constant || (delay_branch & 0x01000000) == 0) {
            if (!constant) {
                emit_constant(&e, REG_VALUE, 0);
                emit(&e, (delay_branch ^ 0x01000000u) | (2 << 5));
            }
            emit_imm(&e, 0x39400000, REG_VALUE, REG_RAM, start + length);
            e.bus = REG_VALUE;
        }
    } else if (e.bus == REG_NONE) {
        // These instructions do not write RAM. Fetch the operand or dummy-read
        // byte again at exit, after any earlier stores to the same byte.
        emit_imm(&e, 0x39400000, REG_VALUE, REG_RAM, start + last + 1);
        e.bus = REG_VALUE;
    }
    unsigned write_reg = REG_NONE, address = 0;

    if (length + 3 <= RAM_BYTES) {
        unsigned opcode = source[length];
        uint64_t check = UINT64_C(7) << length;

        // Both the opcode and address must survive earlier writes in this block
        if ((opcode == 0x8c || opcode == 0x8d || opcode == 0x8e) && !(check & written)) {
            address = combine(source[length + 1], source[length + 2]);

            if (address >= 0x2000 && address <= 0x4017) {
                write_reg = opcode == 0x8d ? REG_A : opcode == 0x8e ? REG_X : REG_Y;
                e.inputs |= (1u << write_reg) & ~e.dirty;
                guarded |= check;
            }
        }
    }
    emit_write_registers(&e, e.dirty, e.nz);
    emit_return(&e, e.clocks, e.bus, e.bus_imm, e.extra, 0, address, count, write_reg);
    cache->used = (cache->used + RAM_TRACE_ALIGNMENT - 1) & ~(RAM_TRACE_ALIGNMENT - 1);
    block->word = publish_trace(cache, &e);
    block->clocks = e.clocks + e.extra;
    block->size = length;
    memcpy(block->bytes, source, sizeof block->bytes);
#if defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    block->guarded = guarded;
#else
    uint8_t *mask = (uint8_t *)block->mask;

    for (unsigned i = 0; i < RAM_BYTES; ++i)
        mask[i] = guarded & (UINT64_C(1) << i) ? 255 : 0;
#endif
}

#if defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static inline uint8x16_t equal_guard_lanes(const uint8_t *before, const uint8_t *after)
{
    const uint8x16_t bits = {1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128};
    return vandq_u8(vceqq_u8(vld1q_u8(before), vld1q_u8(after)), bits);
}
#endif

static bool ram_trace_matches(const RamBlock *block, const uint8_t *source)
{
#if defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    const uint8_t *before = (const uint8_t *)block->bytes;
    uint8x16_t first = vpaddq_u8(equal_guard_lanes(before, source),
                               equal_guard_lanes(before + 16, source + 16));
    uint8x16_t last = vpaddq_u8(equal_guard_lanes(before + 32, source + 32),
                              equal_guard_lanes(before + 48, source + 48));
    // Each set of eight lanes uses distinct bits, so pairwise sums pack the matches
    uint8x16_t four = vpaddq_u8(first, last);
    uint64_t equal = vgetq_lane_u64(vreinterpretq_u64_u8(vpaddq_u8(four, four)), 0);

    return (block->guarded & ~equal) == 0;
#else
    uint64_t changed = 0;

    // The caller bounds the RAM window, and unused bytes have a zero mask
    for (unsigned i = 0; i < RAM_WORDS; ++i) {
        uint64_t current;

        memcpy(&current, source + i * 8, sizeof current);
        changed |= (current ^ block->bytes[i]) & block->mask[i];
    }
    return changed == 0;
#endif
}

// Inline into the RAM window runner: the call alone cost Slalom about 5% when
// unrelated changes in cpu.c tipped the compiler into outlining it.
static NES_ALWAYS_INLINE const RamBlock *find_ram_block(CpuTraceCache *cache, const uint8_t *source,
                                                        unsigned pc, bool warm)
{
    if (pc > 2048 - RAM_BYTES)
        return NULL;
    if (!cache->ram_blocks) {
        if (!warm)
            return NULL;
        cache->ram_blocks = calloc(1, sizeof *cache->ram_blocks);

        if (!cache->ram_blocks)
            return NULL;
    }
    RamTraceCache *ram = cache->ram_blocks;
    RamBlock *block = &ram->active[pc & (RAM_BLOCKS - 1)];

    if (block->pc == pc && block->word && ram_trace_matches(block, source))
        return block;

    // A collision in the primary table maps to the same secondary slot.
    // Keep both compiled versions while different bytes are warming up.
    RamBlock *reusable = &ram->reusable[pc & (RAM_REUSE_BLOCKS - 1)];

    if (reusable->pc == pc && reusable->word && ram_trace_matches(reusable, source)) {
        if (!warm)
            return reusable;
        RamBlock hit = *reusable;

        *reusable = *block;
        hit.warmup = block->warmup;
        *block = hit;
        return block;
    }
    if (!warm)
        return NULL;

    RamWarmup *warmup = &block->warmup;

    if (warmup->pc != pc || !warmup->limit)
        *warmup = (RamWarmup){.pc = pc, .limit = TRACE_WARM_HITS};
    if (warmup->opcode != source[0]) {
        warmup->opcode = source[0];
        warmup->hits = 0;
    }
    if (warmup->hits < warmup->limit) {
        ++warmup->hits;
        return NULL;
    }
    if (warmup->hits != warmup->limit || !cache->remaining_compiles || !trace_arena_has_room(cache))
        return NULL;

    --cache->remaining_compiles;
    ++warmup->hits;
    RamBlock next = {.pc = pc};

    compile_ram_block(cache, &next, source, pc);
    if (!next.word)
        return NULL;
    if (block->word)
        *reusable = *block;
    next.warmup = *warmup;
    next.warmup.hits = 0;

    // Let stable versions get compiled first, then require more use from each
    // unfamiliar version of the same routine.
    if (next.warmup.limit < RAM_MAX_WARM_HITS)
        next.warmup.limit *= 2;
    *block = next;
    return block;
}
