#if defined(__APPLE__) && defined(__aarch64__) && !defined(__arm64e__)
#ifndef _DARWIN_C_SOURCE
#define _DARWIN_C_SOURCE
#endif
#define CPU_NATIVE_TRACES 1
#include <libkern/OSCacheControl.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/mman.h>
#elif defined(_WIN32) && defined(__aarch64__) && !defined(__arm64ec__) && !defined(_M_ARM64EC)
#define CPU_NATIVE_TRACES 1
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <stdlib.h>
#undef REG_NONE
#else
#define CPU_NATIVE_TRACES 0
#endif

#include "mapper_writes.h"

#include <string.h>

// The debug object (SCHNES_CPU_DEBUG) builds the instrumented frame loop.
// Keep its large dispatch loop out of the gameplay code layout.
#ifdef SCHNES_CPU_DEBUG
#define CPU_DEBUG 1
#else
#define CPU_DEBUG 0
#endif
#if defined(__APPLE__) && (defined(__clang__) || defined(__GNUC__))
#define CPU_DEBUG_CODE __attribute__((section("__TEXT,__cpu_debug,regular,pure_instructions")))
#elif defined(__ELF__) && (defined(__clang__) || defined(__GNUC__))
#define CPU_DEBUG_CODE __attribute__((section(".text.unlikely.cpu_debug")))
#else
#define CPU_DEBUG_CODE
#endif

// FDS programs run from RAM through their own frame loop. Keep it out of the
// cartridge-game layout too.
#if defined(__APPLE__) && (defined(__clang__) || defined(__GNUC__))
#define CPU_FDS_CODE __attribute__((section("__TEXT,__cpu_fds,regular,pure_instructions")))
#elif defined(__ELF__) && (defined(__clang__) || defined(__GNUC__))
#define CPU_FDS_CODE __attribute__((section(".text.cpu_fds")))
#else
#define CPU_FDS_CODE
#endif

enum { C = CPU_CARRY, I = 4, D = 8, B = 16, U = 32, V = CPU_OVERFLOW };

typedef enum {
    IMP, ACC, IMM, ZP, ZPX, ZPY, ABS, ABX, ABY, IND, IZX, IZY, REL
} Mode;

static const uint8_t mode_lengths[] = {
    [IMP] = 1, [ACC] = 1, [IMM] = 2, [ZP] = 2, [ZPX] = 2, [ZPY] = 2,
    [ABS] = 3, [ABX] = 3, [ABY] = 3, [IND] = 3, [IZX] = 2, [IZY] = 2, [REL] = 2
};

typedef enum {
    ADC, AHX, ALR, ANC, AND, ARR, ASL, AXS, BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK,
    BVC, BVS, CLC, CLD, CLI, CLV, CMP, CPX, CPY, DCP, DEC, DEX, DEY, EOR, INC, INX,
    INY, ISC, JMP, JSR, KIL, LAS, LAX, LDA, LDX, LDY, LSR, LXA, NOP, ORA, PHA, PHP, PLA,
    PLP, RLA, ROL, ROR, RRA, RTI, RTS, SAX, SBC, SEC, SED, SEI, SHX, SHY, SLO, SRE,
    STA, STX, STY, TAS, TAX, TAY, TSX, TXA, TXS, TYA, XAA
} Operation;

static inline bool index_is_y(Operation op)
{
    return op == INY || op == DEY;
}

static inline int index_delta(Operation op, unsigned count)
{
    return op == INX || op == INY ? (int)count : -(int)count;
}

static inline uint8_t change_index(Operation op, unsigned count, uint8_t *x, uint8_t *y)
{
    uint8_t *index = index_is_y(op) ? y : x;
    return *index += index_delta(op, count);
}

typedef struct {
    uint8_t op, decode;
} Instruction;

typedef enum { READ, WRITE, UNSTABLE_WRITE, MODIFY } Access;

enum { ACCESS_SHIFT = 4 };

static inline Mode instruction_mode(Instruction instruction)
{
    return (Mode)(instruction.decode & ((1u << ACCESS_SHIFT) - 1));
}

static inline Access instruction_access(Instruction instruction)
{
    return (Access)((instruction.decode >> ACCESS_SHIFT) & 3);
}

#define ACCESS(op) \
    ((op) == STA || (op) == STX || (op) == STY || (op) == SAX ? WRITE : \
     (op) == AHX || (op) == SHX || (op) == SHY || (op) == TAS ? UNSTABLE_WRITE : \
     (op) == ASL || (op) == LSR || (op) == ROL || (op) == ROR || \
     (op) == INC || (op) == DEC || (op) == SLO || (op) == SRE || \
     (op) == RLA || (op) == RRA || (op) == DCP || (op) == ISC ? MODIFY : READ)
// Share the opcode map between the timing interpreter and quiet dispatch
#define OPCODES(M) \
    M(00, BRK, IMP) M(01, ORA, IZX) M(02, KIL, IMP) M(03, SLO, IZX) \
    M(04, NOP, ZP) M(05, ORA, ZP) M(06, ASL, ZP) M(07, SLO, ZP) \
    M(08, PHP, IMP) M(09, ORA, IMM) M(0a, ASL, ACC) M(0b, ANC, IMM) \
    M(0c, NOP, ABS) M(0d, ORA, ABS) M(0e, ASL, ABS) M(0f, SLO, ABS) \
    M(10, BPL, REL) M(11, ORA, IZY) M(12, KIL, IMP) M(13, SLO, IZY) \
    M(14, NOP, ZPX) M(15, ORA, ZPX) M(16, ASL, ZPX) M(17, SLO, ZPX) \
    M(18, CLC, IMP) M(19, ORA, ABY) M(1a, NOP, IMP) M(1b, SLO, ABY) \
    M(1c, NOP, ABX) M(1d, ORA, ABX) M(1e, ASL, ABX) M(1f, SLO, ABX) \
    M(20, JSR, ABS) M(21, AND, IZX) M(22, KIL, IMP) M(23, RLA, IZX) \
    M(24, BIT, ZP) M(25, AND, ZP) M(26, ROL, ZP) M(27, RLA, ZP) \
    M(28, PLP, IMP) M(29, AND, IMM) M(2a, ROL, ACC) M(2b, ANC, IMM) \
    M(2c, BIT, ABS) M(2d, AND, ABS) M(2e, ROL, ABS) M(2f, RLA, ABS) \
    M(30, BMI, REL) M(31, AND, IZY) M(32, KIL, IMP) M(33, RLA, IZY) \
    M(34, NOP, ZPX) M(35, AND, ZPX) M(36, ROL, ZPX) M(37, RLA, ZPX) \
    M(38, SEC, IMP) M(39, AND, ABY) M(3a, NOP, IMP) M(3b, RLA, ABY) \
    M(3c, NOP, ABX) M(3d, AND, ABX) M(3e, ROL, ABX) M(3f, RLA, ABX) \
    M(40, RTI, IMP) M(41, EOR, IZX) M(42, KIL, IMP) M(43, SRE, IZX) \
    M(44, NOP, ZP) M(45, EOR, ZP) M(46, LSR, ZP) M(47, SRE, ZP) \
    M(48, PHA, IMP) M(49, EOR, IMM) M(4a, LSR, ACC) M(4b, ALR, IMM) \
    M(4c, JMP, ABS) M(4d, EOR, ABS) M(4e, LSR, ABS) M(4f, SRE, ABS) \
    M(50, BVC, REL) M(51, EOR, IZY) M(52, KIL, IMP) M(53, SRE, IZY) \
    M(54, NOP, ZPX) M(55, EOR, ZPX) M(56, LSR, ZPX) M(57, SRE, ZPX) \
    M(58, CLI, IMP) M(59, EOR, ABY) M(5a, NOP, IMP) M(5b, SRE, ABY) \
    M(5c, NOP, ABX) M(5d, EOR, ABX) M(5e, LSR, ABX) M(5f, SRE, ABX) \
    M(60, RTS, IMP) M(61, ADC, IZX) M(62, KIL, IMP) M(63, RRA, IZX) \
    M(64, NOP, ZP) M(65, ADC, ZP) M(66, ROR, ZP) M(67, RRA, ZP) \
    M(68, PLA, IMP) M(69, ADC, IMM) M(6a, ROR, ACC) M(6b, ARR, IMM) \
    M(6c, JMP, IND) M(6d, ADC, ABS) M(6e, ROR, ABS) M(6f, RRA, ABS) \
    M(70, BVS, REL) M(71, ADC, IZY) M(72, KIL, IMP) M(73, RRA, IZY) \
    M(74, NOP, ZPX) M(75, ADC, ZPX) M(76, ROR, ZPX) M(77, RRA, ZPX) \
    M(78, SEI, IMP) M(79, ADC, ABY) M(7a, NOP, IMP) M(7b, RRA, ABY) \
    M(7c, NOP, ABX) M(7d, ADC, ABX) M(7e, ROR, ABX) M(7f, RRA, ABX) \
    M(80, NOP, IMM) M(81, STA, IZX) M(82, NOP, IMM) M(83, SAX, IZX) \
    M(84, STY, ZP) M(85, STA, ZP) M(86, STX, ZP) M(87, SAX, ZP) \
    M(88, DEY, IMP) M(89, NOP, IMM) M(8a, TXA, IMP) M(8b, XAA, IMM) \
    M(8c, STY, ABS) M(8d, STA, ABS) M(8e, STX, ABS) M(8f, SAX, ABS) \
    M(90, BCC, REL) M(91, STA, IZY) M(92, KIL, IMP) M(93, AHX, IZY) \
    M(94, STY, ZPX) M(95, STA, ZPX) M(96, STX, ZPY) M(97, SAX, ZPY) \
    M(98, TYA, IMP) M(99, STA, ABY) M(9a, TXS, IMP) M(9b, TAS, ABY) \
    M(9c, SHY, ABX) M(9d, STA, ABX) M(9e, SHX, ABY) M(9f, AHX, ABY) \
    M(a0, LDY, IMM) M(a1, LDA, IZX) M(a2, LDX, IMM) M(a3, LAX, IZX) \
    M(a4, LDY, ZP) M(a5, LDA, ZP) M(a6, LDX, ZP) M(a7, LAX, ZP) \
    M(a8, TAY, IMP) M(a9, LDA, IMM) M(aa, TAX, IMP) M(ab, LXA, IMM) \
    M(ac, LDY, ABS) M(ad, LDA, ABS) M(ae, LDX, ABS) M(af, LAX, ABS) \
    M(b0, BCS, REL) M(b1, LDA, IZY) M(b2, KIL, IMP) M(b3, LAX, IZY) \
    M(b4, LDY, ZPX) M(b5, LDA, ZPX) M(b6, LDX, ZPY) M(b7, LAX, ZPY) \
    M(b8, CLV, IMP) M(b9, LDA, ABY) M(ba, TSX, IMP) M(bb, LAS, ABY) \
    M(bc, LDY, ABX) M(bd, LDA, ABX) M(be, LDX, ABY) M(bf, LAX, ABY) \
    M(c0, CPY, IMM) M(c1, CMP, IZX) M(c2, NOP, IMM) M(c3, DCP, IZX) \
    M(c4, CPY, ZP) M(c5, CMP, ZP) M(c6, DEC, ZP) M(c7, DCP, ZP) \
    M(c8, INY, IMP) M(c9, CMP, IMM) M(ca, DEX, IMP) M(cb, AXS, IMM) \
    M(cc, CPY, ABS) M(cd, CMP, ABS) M(ce, DEC, ABS) M(cf, DCP, ABS) \
    M(d0, BNE, REL) M(d1, CMP, IZY) M(d2, KIL, IMP) M(d3, DCP, IZY) \
    M(d4, NOP, ZPX) M(d5, CMP, ZPX) M(d6, DEC, ZPX) M(d7, DCP, ZPX) \
    M(d8, CLD, IMP) M(d9, CMP, ABY) M(da, NOP, IMP) M(db, DCP, ABY) \
    M(dc, NOP, ABX) M(dd, CMP, ABX) M(de, DEC, ABX) M(df, DCP, ABX) \
    M(e0, CPX, IMM) M(e1, SBC, IZX) M(e2, NOP, IMM) M(e3, ISC, IZX) \
    M(e4, CPX, ZP) M(e5, SBC, ZP) M(e6, INC, ZP) M(e7, ISC, ZP) \
    M(e8, INX, IMP) M(e9, SBC, IMM) M(ea, NOP, IMP) M(eb, SBC, IMM) \
    M(ec, CPX, ABS) M(ed, SBC, ABS) M(ee, INC, ABS) M(ef, ISC, ABS) \
    M(f0, BEQ, REL) M(f1, SBC, IZY) M(f2, KIL, IMP) M(f3, ISC, IZY) \
    M(f4, NOP, ZPX) M(f5, SBC, ZPX) M(f6, INC, ZPX) M(f7, ISC, ZPX) \
    M(f8, SED, IMP) M(f9, SBC, ABY) M(fa, NOP, IMP) M(fb, ISC, ABY) \
    M(fc, NOP, ABX) M(fd, SBC, ABX) M(fe, INC, ABX) M(ff, ISC, ABX)

// Decode the addressing mode and bus access together
#define O(byte, op, mode) \
    [0x##byte] = {op, (mode) | (ACCESS(op) << ACCESS_SHIFT)},
static const Instruction instructions[256] = {OPCODES(O)};
#undef O

// The retained access supplies the bus value for the entire quiet sequence
static inline void complete_quiet_accesses(Nes *nes, uint8_t value, unsigned cycles)
{
    cpu_sample_interrupts(nes, nes->irq_line);
    cpu_drive_bus(nes, value);
    nes->cpu.cycles += cycles - 1;
}

static NES_ALWAYS_INLINE uint8_t fetch(Nes *nes)
{
    return cpu_read(nes, nes->cpu.pc++);
}

static uint16_t combine(uint8_t low, uint8_t high)
{
    return low | (high << 8);
}

static inline int branch_offset(uint8_t byte)
{
    return (byte ^ 0x80) - 0x80;
}

static inline bool page_crossed(uint16_t first, uint16_t second)
{
    return ((first ^ second) & 0xff00) != 0;
}

static inline uint16_t address_on_page(uint16_t page, uint16_t address)
{
    return (page & 0xff00) | (uint8_t)address;
}

typedef struct {
    uint16_t target, bus_address;
    bool page_cross;
} TakenBranch;

static inline TakenBranch taken_branch(uint16_t next, uint8_t displacement)
{
    uint16_t target = next + branch_offset(displacement);
    bool cross = page_crossed(next, target);

    return (TakenBranch){target, cross ? address_on_page(next, target) : next, cross};
}

static NES_ALWAYS_INLINE bool read_word_quiet(Nes *nes, uint16_t address, uint16_t next, uint16_t *word)
{
    if (nes->cpu.cycles + 2 >= nes->bus_deadline)
        return false;

    uint8_t low, high;

    if (address < 0x2000 && next < 0x2000) {
        low = nes->ram[address & 2047];
        high = nes->ram[next & 2047];
    } else if (address >= 0x8000 && (address >> 13) == (next >> 13)) {
        const uint8_t *page = cart_prg_page(&nes->cart, address);

        low = page[address & 8191];
        high = page[next & 8191];
    } else {
        return false;
    }
    // Neither read changes the bus mapping or the interrupt inputs
    complete_quiet_accesses(nes, high, 2);
    *word = combine(low, high);
    return true;
}

static NES_ALWAYS_INLINE uint16_t fetch_word(Nes *nes)
{
    uint16_t address = nes->cpu.pc, word;

    if (read_word_quiet(nes, address, address + 1, &word)) {
        nes->cpu.pc += 2;
        return word;
    }
    uint8_t low = fetch(nes);

    return combine(low, fetch(nes));
}

static uint16_t read_word_wrap(Nes *nes, uint16_t address)
{
    uint16_t high_address = address_on_page(address, address + 1);
    uint16_t word;

    if (read_word_quiet(nes, address, high_address, &word))
        return word;
    uint8_t low = cpu_read(nes, address);

    return combine(low, cpu_read(nes, high_address));
}

static void push(Nes *nes, uint8_t value)
{
    cpu_write(nes, 0x100 | nes->cpu.s--, value);
}

static uint8_t pop(Nes *nes)
{
    return cpu_read(nes, 0x100 | ++nes->cpu.s);
}

static inline void set_status(uint8_t *status, uint8_t flag, bool enabled)
{
    *status = (*status & ~flag) | (enabled ? flag : 0);
}

// The low byte holds the result for Z
// BIT and PLP can set N independently through bit 8
enum { NZ_SIGN = 0x100, NZ_NEGATIVE = NZ_SIGN | CPU_NEGATIVE };

static inline uint16_t decode_nz(uint8_t status)
{
    return ((status & CPU_NEGATIVE) ? NZ_SIGN : 0) | !(status & CPU_ZERO);
}

static inline uint8_t read_status(uint8_t status, uint16_t result)
{
    return (status & ~(CPU_NEGATIVE | CPU_ZERO)) |
        ((result & NZ_NEGATIVE) ? CPU_NEGATIVE : 0) | ((uint8_t)result ? 0 : CPU_ZERO);
}

static inline bool direct_branch_taken(uint8_t status, const uint16_t *result, uint8_t opcode)
{
    unsigned mask = cpu_branch_mask(opcode);
    bool set = (status & mask) != 0;

    if (result && mask == CPU_NEGATIVE)
        set = (*result & NZ_NEGATIVE) != 0;
    else if (result && mask == CPU_ZERO)
        set = (uint8_t)*result == 0;
    return set == ((opcode & 0x20) != 0);
}

static inline uint8_t set_nz(uint8_t *status, uint16_t *result, uint8_t value)
{
    if (result)
        *result = value;
    else
        *status = cpu_nz_flags(*status, value);
    return value;
}

static inline void set_compare(uint8_t *status, uint16_t *result, uint8_t reg, uint8_t value)
{
    if (result) {
        set_status(status, C, reg >= value);
        *result = (uint8_t)(reg - value);
    } else {
        *status = cpu_compare_flags(*status, reg, value);
    }
}

static inline void set_bit(uint8_t *status, uint16_t *result, uint8_t a, uint8_t value)
{
    if (result) {
        *result = ((value & CPU_NEGATIVE) ? NZ_SIGN : 0) | ((a & value) != 0);
        *status = (*status & ~V) | (value & V);
    } else {
        *status = cpu_bit_flags(*status, a, value);
    }
}

static uint8_t nz(Cpu *cpu, uint8_t value)
{
    return set_nz(&cpu->p, NULL, value);
}

static inline void adc_status(Cpu *cpu, uint8_t *status, uint16_t *nz_result, uint8_t value)
{
    unsigned result = cpu->a + value + (*status & C);

    set_status(status, C, result > 255);
    set_status(status, V, (~(cpu->a ^ value) & (cpu->a ^ result) & 128) != 0);
    cpu->a = set_nz(status, nz_result, (uint8_t)result);
}

static void adc(Cpu *cpu, uint8_t value)
{
    adc_status(cpu, &cpu->p, NULL, value);
}

static void compare(Cpu *cpu, uint8_t reg, uint8_t value)
{
    cpu->p = cpu_compare_flags(cpu->p, reg, value);
}

typedef struct {
    uint16_t address, base;
    bool stalled;
} Operand;

static NES_ALWAYS_INLINE Operand operand(Nes *nes, Mode mode, bool write)
{
    Cpu *cpu = &nes->cpu;
    uint16_t base = 0;
    uint16_t address = 0;
    bool stalled = false;

    switch (mode) {
    case ZP: address = fetch(nes); break;
    case ZPX: case ZPY:
        base = fetch(nes);
        cpu_read(nes, base);
        address = (uint8_t)(base + (mode == ZPX ? cpu->x : cpu->y));
        break;
    case ABS: address = fetch_word(nes); break;
    case IZX:
        base = fetch(nes);
        cpu_read(nes, base);
        address = read_word_wrap(nes, (uint8_t)(base + cpu->x));
        break;
    case IZY: case ABX: case ABY: {
        base = mode == IZY ? read_word_wrap(nes, fetch(nes)) : fetch_word(nes);
        address = base + (mode == ABX ? cpu->x : cpu->y);
        uint16_t intermediate = address_on_page(base, address);

        if (write || intermediate != address) {
            uint64_t cycle = cpu->cycles;

            cpu_read(nes, intermediate);
            stalled = cpu->cycles - cycle > 1;
        }
        break;
    }
    default: break;
    }
    return (Operand){address, base, stalled};
}

static uint8_t store_value(Cpu *cpu, Operation op)
{
    switch (op) {
    case STA: return cpu->a;
    case STX: case SHX: return cpu->x;
    case STY: case SHY: return cpu->y;
    case SAX: case AHX: return cpu->a & cpu->x;
    case TAS: return cpu->s = cpu->a & cpu->x;
    default: return 0;
    }
}

static void store(Nes *nes, Operation op, Operand target, bool unstable)
{
    uint8_t value = store_value(&nes->cpu, op);

    if (unstable) {
        if (page_crossed(target.base, target.address))
            target.address &= (uint16_t)(value << 8) | 0xff;
        // A DMA halt during the indexed dummy read bypasses the data mask
        if (!target.stalled)
            value &= (target.base >> 8) + 1;
    }
    cpu_write(nes, target.address, value);
}

static NES_ALWAYS_INLINE uint8_t modify_status(Cpu *cpu, uint8_t *status, uint16_t *nz_result,
                                              Operation op, uint8_t value)
{
    uint8_t carry = *status & C;

    switch (op) {
    case SLO: case ASL:
        set_status(status, C, (value & 128) != 0);
        value = (uint8_t)(value << 1);
        break;
    case SRE: case LSR:
        set_status(status, C, (value & 1) != 0);
        value >>= 1;
        break;
    case RLA: case ROL:
        set_status(status, C, (value & 128) != 0);
        value = (uint8_t)((value << 1) | carry);
        break;
    case RRA: case ROR:
        set_status(status, C, (value & 1) != 0);
        value = (value >> 1) | (carry << 7);
        break;
    case INC: case ISC: ++value; break;
    case DEC: case DCP: --value; break;
    default: break;
    }
    switch (op) {
    case SLO: cpu->a = set_nz(status, nz_result, cpu->a | value); break;
    case SRE: cpu->a = set_nz(status, nz_result, cpu->a ^ value); break;
    case RLA: cpu->a = set_nz(status, nz_result, cpu->a & value); break;
    case RRA: adc_status(cpu, status, nz_result, value); break;
    case DCP: set_compare(status, nz_result, cpu->a, value); break;
    case ISC: adc_status(cpu, status, nz_result, value ^ 255); break;
    default: set_nz(status, nz_result, value); break;
    }
    return value;
}

static uint8_t modify(Cpu *cpu, Operation op, uint8_t value)
{
    return modify_status(cpu, &cpu->p, NULL, op, value);
}

static NES_ALWAYS_INLINE void execute_operation(Nes *nes, uint8_t *status, uint16_t *nz_result,
                                                Operation op, uint8_t value)
{
    Cpu *cpu = &nes->cpu;

    switch (op) {
    case ADC: adc_status(cpu, status, nz_result, value); break;
    case SBC: adc_status(cpu, status, nz_result, value ^ 255); break;
    case CMP: set_compare(status, nz_result, cpu->a, value); break;
    case CPX: set_compare(status, nz_result, cpu->x, value); break;
    case CPY: set_compare(status, nz_result, cpu->y, value); break;
    case BIT: set_bit(status, nz_result, cpu->a, value); break;
    case AND: cpu->a = set_nz(status, nz_result, cpu->a & value); break;
    case EOR: cpu->a = set_nz(status, nz_result, cpu->a ^ value); break;
    case ORA: cpu->a = set_nz(status, nz_result, cpu->a | value); break;
    case LDA: cpu->a = set_nz(status, nz_result, value); break;
    case LDX: cpu->x = set_nz(status, nz_result, value); break;
    case LDY: cpu->y = set_nz(status, nz_result, value); break;
    // LXA ORs A with an unstable constant before the AND. Consoles commonly
    // show $FF, which makes it load the operand like LAX.
    case LXA:
    case LAX: cpu->a = cpu->x = set_nz(status, nz_result, value); break;
    case LAS: cpu->a = cpu->x = cpu->s = set_nz(status, nz_result, value & cpu->s); break;
    case TAX: cpu->x = set_nz(status, nz_result, cpu->a); break;
    case TAY: cpu->y = set_nz(status, nz_result, cpu->a); break;
    case TSX: cpu->x = set_nz(status, nz_result, cpu->s); break;
    case TXA: cpu->a = set_nz(status, nz_result, cpu->x); break;
    case TXS: cpu->s = cpu->x; break;
    case TYA: cpu->a = set_nz(status, nz_result, cpu->y); break;
    case DEX: set_nz(status, nz_result, --cpu->x); break;
    case DEY: set_nz(status, nz_result, --cpu->y); break;
    case INX: set_nz(status, nz_result, ++cpu->x); break;
    case INY: set_nz(status, nz_result, ++cpu->y); break;
    case CLC: *status &= ~C; break;
    case CLD: *status &= ~D; break;
    case CLI: *status &= ~I; break;
    case CLV: *status &= ~V; break;
    case SEC: *status |= C; break;
    case SED: *status |= D; break;
    case SEI: *status |= I; break;
    case PHA: push(nes, cpu->a); break;
    case PHP: push(nes, *status | B | U); break;
    case PLA: case PLP: {
        cpu_read(nes, 0x100 | cpu->s);
        uint8_t pulled = pop(nes);

        if (op == PLA)
            cpu->a = set_nz(status, nz_result, pulled);
        else
            *status = (pulled & ~B) | U;
        break;
    }
    case ANC:
        cpu->a = set_nz(status, nz_result, cpu->a & value);
        set_status(status, C, (cpu->a & 128) != 0);
        break;
    case ALR:
        value &= cpu->a;
        set_status(status, C, (value & 1) != 0);
        cpu->a = set_nz(status, nz_result, value >> 1);
        break;
    case ARR:
        cpu->a = set_nz(status, nz_result, ((cpu->a & value) >> 1) | ((*status & C) << 7));
        set_status(status, C, (cpu->a & 64) != 0);
        set_status(status, V, ((cpu->a ^ (cpu->a << 1)) & 64) != 0);
        break;
    case AXS:
        set_status(status, C, (cpu->a & cpu->x) >= value);
        cpu->x = set_nz(status, nz_result, (uint8_t)((cpu->a & cpu->x) - value));
        break;
    case XAA: cpu->a = set_nz(status, nz_result, (cpu->a | 0xee) & cpu->x & value); break;
    case KIL: cpu->jammed = true; break;
    default: break;
    }
}

static void execute(Nes *nes, Operation op, uint8_t value)
{
    execute_operation(nes, &nes->cpu.p, NULL, op, value);
}

static NES_ALWAYS_INLINE void branch(Nes *nes, uint8_t opcode)
{
    Cpu *cpu = &nes->cpu;
    uint8_t offset = fetch(nes);

    if (!cpu_branch_taken(cpu->p, opcode))
        return;
    bool irq = cpu->irq_sample;
    bool nmi = cpu->nmi_sample;
    TakenBranch taken = taken_branch(cpu->pc, offset);

    cpu_read(nes, cpu->pc);
    if (taken.page_cross) {
        cpu_read(nes, taken.bus_address);
        irq |= cpu->irq_sample;
        nmi |= cpu->nmi_sample;
    }
    // Taken branches poll before the offset fetch and, if needed, before the page fixup
    cpu->irq_sample = irq;
    cpu->nmi_sample = nmi;
    cpu->pc = taken.target;
}

static void interrupt(Nes *nes, bool software, bool nmi)
{
    Cpu *cpu = &nes->cpu;

    if (software)
        fetch(nes);
    else {
        cpu_read(nes, cpu->pc);
        cpu_read(nes, cpu->pc);
    }
    push(nes, cpu->pc >> 8);
    push(nes, (uint8_t)cpu->pc);
    if (cpu->nmi_edge) {
        nmi = true;
        cpu->nmi_edge = false;
    }
    push(nes, cpu->p | U | (software ? B : 0));
    cpu->p |= I;
    cpu->pc = read_word_wrap(nes, nmi ? 0xfffa : 0xfffe);
    cpu->nmi = cpu->irq = false;
}

#ifndef SCHNES_CPU_DEBUG
void cpu_reset(Nes *nes)
{
    cpu_discard_traces(&nes->cart);
    memset(nes->short_loops, 0, sizeof nes->short_loops);
    Cpu *cpu = &nes->cpu;

    cpu->p |= U | I;
    cpu->jammed = cpu->nmi_edge = cpu->nmi = cpu->irq = false;
    cpu_read(nes, cpu->pc);
    cpu_read(nes, cpu->pc);
    for (unsigned i = 0; i < 3; ++i)
        cpu_read(nes, 0x100 | cpu->s--);
    cpu->pc = read_word_wrap(nes, 0xfffc);
}
#endif

enum { LOOP_BRANCH = XAA + 1, LOOP_LOAD_COMPARE, LOOP_INDEX, LOOP_VRAM_FILL, LOOP_RAM_ADD, LOOP_IDLE_SCAN };
enum { LOOP_STATUS = REL + 1 };
enum { LOOP_WRITES_RAM = 1, LOOP_UNIT_COUNTDOWN = 2, LOOP_NATIVE = 4, LOOP_STATUS_FIRST = 8,
       LOOP_CROSS_PAGE_EXIT = 16, LOOP_COUNTER = 32, LOOP_INTERPRETER_ONLY = 64, LOOP_INDEXED_STORE = 128 };
enum { VRAM_FILL_CYCLES = 9, RAM_ADD_CYCLES = 19, RAM_ADD_CARRY_CYCLES = 12 };

static inline uint64_t loop_status_tick(const Nes *nes)
{
    return (nes_allows_deferred_io(nes) && ppu_cached_status_supported(nes)) ||
           nes->ppu.scanline >= NES_HEIGHT
        ? nes->cpu.cycles * NES_PPU_DOTS_PER_CPU : nes->ppu.ticks;
}

static inline uint8_t loop_value(const Nes *nes, const ShortLoopStep *step, unsigned *cycles)
{
    uint8_t value = step->operand;

    switch (step->mode) {
    case ZP: return nes->ram[value];
    case ZPX: return nes->ram[(uint8_t)(value + nes->cpu.x)];
    case ZPY: return nes->ram[(uint8_t)(value + nes->cpu.y)];
    case LOOP_STATUS:
        return ppu_status_value(&nes->ppu, loop_status_tick(nes));
    case ABS: case ABX: case ABY: {
        unsigned base = combine(value, step->target);
        unsigned index = step->mode == ABX ? nes->cpu.x : step->mode == ABY ? nes->cpu.y : 0;
        unsigned address = base + index;

        *cycles += page_crossed(base, address);

        if (address < 0x2000)
            return nes->ram[address & 2047];
        return cart_prg_read(&nes->cart, address);
    }
    default: return value;
    }
}

static inline void loop_store(Nes *nes, const ShortLoopStep *step, uint8_t value)
{
    unsigned address = step->operand;

    switch (step->mode) {
    case ZPX: address = (uint8_t)(address + nes->cpu.x); break;
    case ZPY: address = (uint8_t)(address + nes->cpu.y); break;
    case ABS: address += step->target << 8; break;
    case ABX: address += (step->target << 8) + nes->cpu.x; break;
    case ABY: address += (step->target << 8) + nes->cpu.y; break;
    }
    nes->ram[address & 2047] = value;
}

static inline bool cpu_can_batch(const Nes *nes)
{
    const Cpu *cpu = &nes->cpu;
    return !cpu->jammed && !cpu->nmi && !cpu->irq && !cpu->nmi_edge &&
        (!nes->irq_line || (cpu->p & I));
}

static inline bool at_dmc_deadline(const Nes *nes, uint64_t deadline)
{
    return deadline - 1 == nes->apu.dmc.dma_halt;
}

static inline bool short_loop_in_bank(unsigned pc, unsigned back)
{
    return back <= CPU_SHORT_LOOP_BACK && (pc & 8191) + back + 2 < 8192;
}

static inline bool closes_short_loop(const uint8_t *code, unsigned pc, unsigned back)
{
    const uint8_t *edge = code + back;
    return (edge[1] == 254 - back && cpu_is_branch(edge[0])) ||
           (edge[0] == 0x4c && edge[1] == (uint8_t)pc && edge[2] == pc >> 8);
}

static inline unsigned short_loop_slot(unsigned pc)
{
    return (pc >> 1) & (CPU_SHORT_LOOP_CACHE - 1);
}

static inline uintptr_t mapped_code_key(const uint8_t *code, unsigned pc, bool fds)
{
    // FDS code keeps the same mapping when runahead copies PRG RAM
    return fds ? pc : (uintptr_t)code;
}

static inline bool short_loop_matches(const ShortLoop *loop, uintptr_t key, unsigned back)
{
    return loop->code_key == key && loop->back == back;
}

static inline bool short_loop_enabled(const ShortLoop *loop, bool interpreter)
{
    return loop->cycles && !(loop->flags & (interpreter ? LOOP_CROSS_PAGE_EXIT : LOOP_INTERPRETER_ONLY));
}

// Reset X and Y, then poll a strided zero-page table until an entry is ready
static bool decode_idle_scan(ShortLoop *loop, const uint8_t *code, unsigned back)
{
    if (back < 16 || code[back] != 0x4c || code[0] != 0xa2 || code[2] != 0x86 ||
        code[4] != 0xa0 || code[6] != 0xb5 || code[8] != 0xc9 || code[10] != 0xb0 ||
        12 + branch_offset(code[11]) < (int)back + 3 ||
        code[back - 3] != 0x88 || code[back - 2] != 0xd0 || code[back - 1] != (uint8_t)(6 - back))
        return false;

    unsigned stride = back - 15;

    for (unsigned i = 0; i < stride; ++i)
        if (code[12 + i] != 0xe8)
            return false;

    unsigned count = code[5];
    unsigned cycles = 9 + count * (13 + 2 * stride);

    if (!count || cycles > UINT8_MAX)
        return false;

    // The only store must not overwrite a slot in the table being polled
    for (unsigned i = 0, x = code[1]; i < count; ++i, x += stride)
        if ((uint8_t)(code[7] + x) == code[3])
            return false;

    loop->cycles = (uint8_t)cycles;
    loop->instructions = (uint8_t)(4 + count * (5 + stride));
    loop->repeat_bus_offset = back + 2;
    loop->body[0].op = LOOP_IDLE_SCAN;
    return true;
}

// A body that ends with a unit step of one index register and BNE back,
// and otherwise never touches that register or RAM, repeats exactly once
// its other registers settle. N and Z must be set again before a branch reads them.
static bool counts_down_invariant_body(const ShortLoop *loop, unsigned steps)
{
    if (steps < 2)
        return false;
    const ShortLoopStep *counter = &loop->body[steps - 1], *edge = counter + 1;

    if ((loop->flags & LOOP_WRITES_RAM) || edge->op != LOOP_BRANCH ||
        edge->operand != 0xd0 || (counter->op != INX && counter->op != INY) ||
        (counter->operand != 1 && counter->operand != 255))
        return false;
    bool y = counter->op == INY, nz_set = false;

    for (const ShortLoopStep *step = loop->body; step < counter; ++step) {
        switch (step->op) {
        case NOP: case CLC: case SEC: case CLV:
            break;
        case LOOP_BRANCH:
            if (!nz_set)
                return false;
            break;
        case INX: case INY: case LOOP_INDEX:
        case LDX: case TAX: case TXA: case CPX: case LDY: case TAY: case TYA: case CPY:
            return false;
        default:
            nz_set = true;
            break;
        }
        if (y ? step->mode == ZPY || step->mode == ABY : step->mode == ZPX || step->mode == ABX)
            return false;
    }
    return true;
}

static inline const ShortLoopStep *loop_back_edge(const ShortLoop *loop)
{
    // Forward exits have a nonzero target, and the back edge ends the body
    const ShortLoopStep *edge = loop->body;

    while (edge->op != LOOP_BRANCH || edge->target)
        ++edge;
    return edge;
}

// Decode the straight-line body, any forward exits, and the observed back edge
static void decode_short_loop(ShortLoop *loop, uintptr_t key, const uint8_t *code, unsigned back, uint16_t pc)
{
    loop->code_key = key;
    loop->back = back;
    loop->cycles = 0; // Cache rejected loops too
    loop->first_status_read = 0;
    loop->flags = 0;
    loop->indexed_reads = 0;
    bool crosses = (pc & 255) + back + 2 >= 256;

    // A constant-data PPUDATA store followed by a unit countdown and BNE
    if (!crosses && back == 4 && code[0] == 0x8d && (combine(code[1], code[2]) & 0xe007) == 0x2007 &&
        (code[3] == 0xca || code[3] == 0x88) && code[4] == 0xd0 && code[5] == 0xfa) {
        loop->cycles = VRAM_FILL_CYCLES;
        loop->body[0].op = LOOP_VRAM_FILL;
        return;
    }
    // INC zp; CLC; LDA zp; ADC addend; STA zp; JMP back.
    // The addend must stay constant while the destination changes.
    if (!crosses && back == 9 && code[0] == 0xe6 && code[2] == 0x18 && code[3] == 0xa5 &&
        code[4] == code[1] && code[5] == 0x65 && code[6] != code[1] &&
        code[7] == 0x85 && code[8] == code[1] && code[9] == 0x4c) {
        loop->cycles = RAM_ADD_CYCLES;
        loop->instructions = 6;
        loop->body[0].op = LOOP_RAM_ADD;
        return;
    }
    // LDA zp; ADC addend; STA zp; JMP back, keeping ADC's carry between adds
    if (!crosses && back == 6 && code[0] == 0xa5 && code[2] == 0x65 && code[3] != code[1] &&
        code[4] == 0x85 && code[5] == code[1] && code[6] == 0x4c) {
        loop->cycles = RAM_ADD_CARRY_CYCLES;
        loop->instructions = 4;
        loop->body[0].op = LOOP_RAM_ADD;
        return;
    }
    if (!crosses && decode_idle_scan(loop, code, back))
        return;
    unsigned cycles = 0, steps = 0, instructions_executed = 0;
    bool jump = code[back] == 0x4c;
    unsigned length = back + (jump ? 3 : 2);

    for (unsigned offset = 0; offset < back;) {
        uint8_t opcode = code[offset++];
        Instruction instruction = instructions[opcode];
        Operation op = (Operation)instruction.op;
        Mode mode = instruction_mode(instruction);
        ShortLoopStep *step = &loop->body[steps++];

        ++instructions_executed;
        *step = (ShortLoopStep){.op = op, .mode = mode};

        if (mode == REL) {
            if (offset == back)
                return;
            uint8_t displacement = code[offset++];
            TakenBranch exit = taken_branch(pc + offset, displacement);
            int target = (int)exit.target - pc;

            // Cached native loops resume against the same PRG bank
            if (target < (int)length || ((exit.target ^ pc) & 0xe000))
                return;
            if (exit.page_cross)
                loop->flags |= LOOP_CROSS_PAGE_EXIT;
            cycles += 2;
            step->op = LOOP_BRANCH;
            step->operand = opcode;
            step->target = target;
            step->cycles = cycles + 1 + exit.page_cross;
            step->instructions = instructions_executed;
            step->branch_end = offset;
            continue;
        }

        // Keep the interrupt mask fixed and admit only repeatable bus accesses
        bool store = op == STA || op == STX || op == STY;

        switch (op) {
        case STA: case STX: case STY:
            loop->flags |= LOOP_WRITES_RAM;
            break;
        case ADC: case SBC: case CMP: case CPX: case CPY: case BIT:
        case AND: case EOR: case ORA: case LDA: case LDX: case LDY:
        case TAX: case TAY: case TXA: case TYA: case CLV: case CLC: case SEC: case NOP:
            break;
        case INX: case INY: case DEX: case DEY: {
            unsigned count = 0;
            uint8_t dx = 0, dy = 0;
            Operation index = op, last;

            do {
                last = index;

                switch (index) {
                case INX: ++dx; break;
                case INY: ++dy; break;
                case DEX: --dx; break;
                case DEY: --dy; break;
                default: break;
                }
                ++count;

                if (offset == back)
                    break;
                index = (Operation)instructions[code[offset]].op;

                if (index != INX && index != INY && index != DEX && index != DEY)
                    break;
                ++offset;
            } while (true);
            instructions_executed += count - 1;
            cycles += count * 2;
            // N and Z come from the index changed by the last instruction
            bool flags_from_y = index_is_y(last);

            if (flags_from_y ? dx == 0 : dy == 0) {
                step->op = flags_from_y ? INY : INX;
                step->operand = flags_from_y ? dy : dx;
            } else {
                step->op = LOOP_INDEX;
                step->mode = flags_from_y;
                step->operand = dx;
                step->compare = dy;
            }
            continue;
        }
        default:
            return;
        }
        if (mode == IMP) {
            cycles += 2;
        } else if (mode == IMM || mode == ZP || mode == ZPX || mode == ZPY) {
            if (offset == back)
                return;
            step->operand = code[offset++];
            cycles += mode == IMM ? 2 : mode == ZP ? 3 : 4;

            // CMP supplies all the flags that LDA would have set in this pair
            if (op == LDA && offset + 1 < back && code[offset] == 0xc9) {
                step->op = LOOP_LOAD_COMPARE;
                step->compare = code[offset + 1];
                offset += 2;
                cycles += 2;
                ++instructions_executed;
            }
        } else if ((mode == ABS || mode == ABX || mode == ABY) && offset + 1 < back) {
            unsigned address = combine(code[offset], code[offset + 1]);
            unsigned last = address + (mode == ABS ? 0 : 255);

            if (!store && mode == ABS && (address & 0xe007) == 0x2002) {
                if (!loop->first_status_read) {
                    loop->first_status_read = instructions_executed;

                    if (instructions_executed == 1)
                        loop->flags |= LOOP_STATUS_FIRST;
                } else {
                    loop->flags &= ~LOOP_STATUS_FIRST;
                }
                step->mode = LOOP_STATUS;
            } else if (last < 0x2000 || (!store && address >= 0x8000 && last <= 0xffff)) {
                // Both the effective address and any indexed dummy read must be quiet
                step->operand = (uint8_t)address;
                step->target = address >> 8;

                if (!store && mode != ABS && (address & 255))
                    ++loop->indexed_reads;
            } else {
                return;
            }
            offset += 2;
            cycles += 4 + (store && mode != ABS);
        } else {
            return;
        }
    }
    if (crosses && loop->first_status_read && !(loop->flags & LOOP_STATUS_FIRST))
        return;
    loop->instructions = instructions_executed + 1;
    bool branch_crosses = crosses && !jump;

    // Preserve the recompiler's eligibility for loops that straddle a page
    if (crosses && !(loop->flags & LOOP_STATUS_FIRST))
        loop->flags |= LOOP_INTERPRETER_ONLY;
    loop->cycles = cycles + 3 + branch_crosses;
    // A taken branch reads the target's low byte on the fallthrough page first
    loop->repeat_bus_offset = branch_crosses ? 256 : back + 2;
    loop->body[steps] = (ShortLoopStep){
        .op = jump ? JMP : LOOP_BRANCH, .operand = code[back],
        .cycles = cycles + 2, .instructions = loop->instructions, .branch_end = back + 2
    };
    const ShortLoopStep *first = loop->body, *branch = first + 1;

    if ((first->op == INX || first->op == INY) && (first->operand == 1 || first->operand == 255) &&
        branch->op == LOOP_BRANCH && branch->operand == 0xd0 && !branch->target)
        loop->flags |= LOOP_UNIT_COUNTDOWN;
    else if (counts_down_invariant_body(loop, steps))
        loop->flags |= LOOP_COUNTER;
    else if (steps == 2 && !jump && code[back] == 0xd0 && first->op == STA &&
             ((branch->op == INX && (first->mode == ABX || first->mode == ZPX)) ||
              (branch->op == INY && (first->mode == ABY || first->mode == ZPY)))) {
        loop->flags = (loop->flags & ~LOOP_INTERPRETER_ONLY) | LOOP_INDEXED_STORE;
    }
}

static inline uint32_t loop_registers(const Cpu *cpu)
{
    return (uint32_t)cpu->a | ((uint32_t)cpu->x << 8) |
        ((uint32_t)cpu->y << 16) | ((uint32_t)cpu->p << 24);
}

#if CPU_NATIVE_TRACES
static NES_ALWAYS_INLINE bool run_native_short_loop(Nes *nes, const uint8_t *code, ShortLoop *loop,
                                  unsigned longest, uint64_t budget, uint32_t *batched);
#endif

static void batch_vram_fill(Nes *nes, const uint8_t *code, uint64_t limit, NesInstructionStats *stats)
{
    const Ppu *p = &nes->ppu;
    unsigned address = p->v & 0x3fff;

    if ((p->mask & 0x18) || p->mask_tick || (address & 0x3000) != 0x2000)
        return;

    Cpu *cpu = &nes->cpu;
    uint8_t *counter = code[3] == 0xca ? &cpu->x : &cpu->y;
    unsigned until_zero = *counter ? *counter : 256;
    unsigned increment = p->ctrl & 4 ? 32 : 1;
    // Keep the last increment below the rising A12 edge at $3000
    unsigned until_a12 = (0x2fff - address) / increment;
    unsigned count = (unsigned)((limit - 1 - cpu->cycles) / VRAM_FILL_CYCLES);

    if (count > until_zero)
        count = until_zero;
    if (count > until_a12)
        count = until_a12;
    if (!count)
        return;

    uint64_t tick = (cpu->cycles + 3) * NES_PPU_DOTS_PER_CPU;
    uint64_t last_tick = tick + (count - 1) * VRAM_FILL_CYCLES * NES_PPU_DOTS_PER_CPU;

    ppu_fill_nametable(nes, cpu->a, count, increment, tick, last_tick);
    *counter = nz(cpu, *counter - count);
    bool finished = count == until_zero;

    if (finished)
        cpu->pc += 6;
    complete_quiet_accesses(nes, code[finished ? 5 : 6], count * VRAM_FILL_CYCLES - finished);
    if (stats)
        stats->skipped += count * 3;
}

static inline bool timed_status_loop(const Nes *nes, const ShortLoop *loop)
{
    // A single leading status read has a fixed cycle in every iteration
    return nes_allows_deferred_io(nes) && (loop->flags & LOOP_STATUS_FIRST) && !loop->indexed_reads &&
           !nes->ppu.mask_tick && (ppu_cached_status_supported(nes) || ppu_waiting_sprite_zero(&nes->ppu));
}

#ifndef SCHNES_CPU_DEBUG
static NES_NOINLINE unsigned batch_loop_dmc(Nes *nes, uint64_t cycle, unsigned iteration)
{
    NesCpuRaster *raster = nes->cpu_raster;
    unsigned stall_cycles = 0;

    for (;;) {
        uint64_t halt = nes->apu.dmc.dma_halt;

        if (halt < cycle || nes->bus_deadline != halt + 1)
            break;
        // Allow room for the unfinished iteration after its halted read
        uint64_t raster_limit = nes_dmc_trace_deadline(nes, halt, halt + iteration);

        if (!raster_limit)
            break;
        nes->cpu.cycles = halt;
        nes_dmc_finish_trace(nes, halt, raster_limit);
        cycle = nes->cpu.cycles;
        if (raster) {
            cpu_record_execution(raster, halt, CPU_EXEC_LOOP_BATCHED);
            cpu_record_execution(raster, cycle, CPU_EXEC_DMA);
        }
        stall_cycles += 4;
    }
    return stall_cycles;
}

static NES_NOINLINE void batch_idle_scan(Nes *nes, const ShortLoop *loop, const uint8_t *code, unsigned repeats,
                                        NesInstructionStats *stats)
{
    uint8_t x = code[1], last = 0;
    unsigned stride = loop->back - 15;

    for (unsigned i = 0; i < code[5]; ++i, x += stride) {
        last = nes->ram[(uint8_t)(code[7] + x)];

        if (last >= code[9])
            return;
    }

    // With no ready slots, every pass finishes with the same registers and RAM
    nes->ram[code[3]] = code[1];
    Cpu *cpu = &nes->cpu;

    cpu->a = last;
    cpu->x = x;
    cpu->y = nz(cpu, 0);
    cpu->p &= ~C;
    complete_quiet_accesses(nes, code[loop->repeat_bus_offset], repeats * loop->cycles);
    if (stats)
        stats->skipped += repeats * loop->instructions;
}

static NES_NOINLINE void batch_ram_add_carry(Nes *nes, const ShortLoop *loop, const uint8_t *code,
                                            unsigned count, NesInstructionStats *stats)
{
    Cpu *cpu = &nes->cpu;
    uint8_t *destination = &nes->ram[code[1]];
    unsigned addend = nes->ram[code[3]];

    cpu->a = *destination;

    if (count > 1) {
        unsigned sum = cpu->a + (cpu->p & C);

        // Folding the carry into the stored byte makes each step an add modulo 255
        // Keep zero and 255 distinct, and preserve 256 when adding 255
        if (count > 2 && (sum != 256 || addend != 255)) {
            uint64_t total = sum + (uint64_t)(count - 2) * addend;

            sum = total ? 1 + (unsigned)((total - 1) % 255) : 0;
        }
        // Recover the penultimate byte and carry before computing the final ADC flags
        sum += addend;
        cpu->a = (uint8_t)sum;
        cpu->p = (cpu->p & ~C) | (sum >> 8);
    }
    adc(cpu, (uint8_t)addend);
    *destination = cpu->a;
    complete_quiet_accesses(nes, code[loop->back + 2], count * RAM_ADD_CARRY_CYCLES);
    if (stats)
        stats->skipped += count * loop->instructions;
}

void cpu_batch_short_loop(Nes *nes, const uint8_t *code, unsigned back, NesInstructionStats *stats)
{
    Cpu *cpu = &nes->cpu;
    uint64_t cycles = cpu->cycles, start = cycles, limit = nes->bus_deadline;

    if (cycles + back + 2 >= limit || !cpu_can_batch(nes))
        return;
    ShortLoop *loop = &nes->short_loops[short_loop_slot(cpu->pc)];
    bool fds = cart_is_fds(&nes->cart);
    uintptr_t key = mapped_code_key(code, cpu->pc, fds);

    if (!short_loop_matches(loop, key, back)) {
        decode_short_loop(loop, key, code, back, cpu->pc);
        if (fds && key >= 0x6000 && key < 0xe000) {
            uint8_t *map = nes->cart.ext->fds->loop_map;
            unsigned first = (key - 0x6000) >> FDS_LOOP_REGION_SHIFT;
            unsigned last = (key - 0x6000 + back + 2) >> FDS_LOOP_REGION_SHIFT;

            // A short loop spans at most two regions
            map[first >> 3] |= (uint8_t)(1u << (first & 7));
            map[last >> 3] |= (uint8_t)(1u << (last & 7));
        }
    }
    if (!short_loop_enabled(loop, nes->interpreter))
        return;
    if (loop->body[0].op == LOOP_VRAM_FILL) {
        batch_vram_fill(nes, code, limit, stats);
        return;
    }
    uint64_t status_limit = nes->bus_deadline;
    bool timed_status = false;

    if (loop->first_status_read) {
        timed_status = timed_status_loop(nes, loop);
        uint64_t refill = nes->apu.stream_read_cycle;

        if (refill < status_limit)
            status_limit = refill;
        const Ppu *p = &nes->ppu;

        if ((p->status & 0x80) || cpu->nmi_line ||
            ((p->mask & 0x18) == 8 && !(p->status & 0x20)))
            return;
        bool vblank = p->scanline >= NES_HEIGHT;

        if (vblank) {
            // A read after the vblank edge only updates the I/O latch
            if (!(loop->flags & LOOP_STATUS_FIRST) || loop->indexed_reads ||
                p->scanline < ppu_vblank_line(p) || p->scanline >= ppu_pre_render_line(p) ||
                p->next_dot != NES_DOTS_PER_SCANLINE || p->mask_tick || p->early_draw_tick)
                return;
            timed_status = true;
        }
        if ((p->bus & 31) && loop_status_tick(nes) < p->bus_low_until) {
            uint64_t expiry = (p->bus_low_until + NES_PPU_DOTS_PER_CPU - 1) / NES_PPU_DOTS_PER_CPU;

            if (expiry < status_limit)
                status_limit = expiry;
        }
        if (status_limit < limit)
            limit = status_limit;
        uint64_t event = nes->ppu_deadline;

        if (vblank)
            event = ppu_cycle_at(p, ppu_pre_render_line(p), 0);
        else if (timed_status)
            event = ppu_cached_status_supported(nes) ? ppu_status_read_deadline(nes) : ppu_sprite_zero_cycle(nes);
        if (event < limit)
            limit = event;
    }
    // An indexed read can add one bus cycle on a page crossing
    unsigned longest = loop->cycles + loop->indexed_reads;

    if (cycles + longest >= limit)
        return;

    if (loop->body[0].op == LOOP_IDLE_SCAN) {
        batch_idle_scan(nes, loop, code, (unsigned)((limit - 1 - cycles) / loop->cycles), stats);
        return;
    }

    if (loop->body[0].op == LOOP_RAM_ADD) {
        if (loop->cycles == RAM_ADD_CARRY_CYCLES) {
            batch_ram_add_carry(nes, loop, code, (unsigned)((limit - 1 - cycles) / RAM_ADD_CARRY_CYCLES), stats);
            return;
        }
        unsigned count = (unsigned)((limit - 1 - cycles) / RAM_ADD_CYCLES);
        uint8_t *destination = &nes->ram[code[1]];
        uint8_t addend = nes->ram[code[6]];

        // Batch adds of 1 + addend modulo 256 and run the last ADC for its flags
        cpu->a = (uint8_t)(*destination + (count - 1) * (1u + addend) + 1);
        cpu->p &= ~C;
        adc(cpu, addend);
        *destination = cpu->a;
        complete_quiet_accesses(nes, code[back + 2], count * RAM_ADD_CYCLES);
        if (stats)
            stats->skipped += count * loop->instructions;
        return;
    }

    // Record the final CPU bus value and commit any status-read side effects once
    const ShortLoopStep *exit = NULL;
    unsigned repeats = 0, extra = 0;

    if (!back && (loop->body[0].op == JMP || cpu_branch_taken(cpu->p, code[0]))) {
        // A control transfer back to itself leaves the CPU state unchanged
        goto stable;
    }
    if (loop->flags & LOOP_UNIT_COUNTDOWN) {
        const ShortLoopStep *first = loop->body, *branch_step = first + 1;
        // A unit-step countdown can reach zero without running its body one instruction at a time
        uint8_t *reg = first->op == INX ? &cpu->x : &cpu->y;
        uint8_t distance = first->operand == 1 ? -*reg : *reg;
        unsigned until_zero = distance ? distance : 256;

        repeats = (unsigned)((limit - 1 - cycles) / loop->cycles);

        if (until_zero <= repeats) {
            repeats = until_zero - 1;
            exit = branch_step;
        }
        *reg = nz(cpu, exit ? 0 : (uint8_t)(*reg + repeats * first->operand));
        cycles += repeats * loop->cycles;
        goto finish;
    }
#if CPU_NATIVE_TRACES
    if (!nes->interpreter && !loop->first_status_read && back && limit - cycles > 2 * longest &&
        ((loop->flags & LOOP_WRITES_RAM) || loop->instructions >= 5) &&
        loop->repeat_bus_offset != 256 &&
        run_native_short_loop(nes, code, loop, longest, limit - cycles, stats ? &stats->skipped : NULL))
        return;
#endif
    if ((nes->interpreter || loop->repeat_bus_offset == 256) && (loop->flags & LOOP_INDEXED_STORE)) {
        // Only the index and the flags from its last update survive the loop
        const ShortLoopStep *store = loop->body, *counter = store + 1;
        uint8_t *reg = counter->op == INX ? &cpu->x : &cpu->y;
        unsigned base = combine(store->operand, store->target);
        unsigned mask = store->mode == ZPX || store->mode == ZPY ? 255 : 2047;
        unsigned count = (unsigned)((limit - 1 - cycles) / loop->cycles);
        uint8_t index = *reg;

        do {
            nes->ram[(base + index) & mask] = cpu->a;
            index += counter->operand;

            if (!index) {
                exit = counter + 1;
                break;
            }
            ++repeats;
        } while (repeats < count);
        *reg = nz(cpu, index);
        cycles += repeats * loop->cycles;
        goto finish;
    }
    uint32_t previous_registers = loop_registers(cpu);

    do {
        extra = 0;

        for (const ShortLoopStep *step = loop->body;; ++step) {
            switch (step->op) {
            case JMP: goto repeat;
            case LOOP_LOAD_COMPARE:
                cpu->a = loop_value(nes, step, &extra);
                compare(cpu, cpu->a, step->compare);
                break;
            case LOOP_BRANCH: {
                bool taken = cpu_branch_taken(cpu->p, step->operand);

                if (taken == (step->target != 0)) {
                    exit = step;
                    goto finish;
                }
                if (!step->target)
                    goto repeat;
                break;
            }
            case LOOP_INDEX:
                cpu->x += step->operand;
                cpu->y += step->compare;
                nz(cpu, step->mode ? cpu->y : cpu->x);
                break;
            case STA: loop_store(nes, step, cpu->a); break;
            case STX: loop_store(nes, step, cpu->x); break;
            case STY: loop_store(nes, step, cpu->y); break;
            case INX: nz(cpu, cpu->x += step->operand); break;
            case INY: nz(cpu, cpu->y += step->operand); break;
            case ADC: adc(cpu, loop_value(nes, step, &extra)); break;
            case SBC: adc(cpu, loop_value(nes, step, &extra) ^ 255); break;
            case EOR: cpu->a = nz(cpu, cpu->a ^ loop_value(nes, step, &extra)); break;
            case LDA: cpu->a = nz(cpu, loop_value(nes, step, &extra)); break;
            default:
                execute(nes, (Operation)step->op, loop_value(nes, step, &extra));
                break;
            }
        }
repeat: {
            cycles += loop->cycles + extra;

            // A load can settle a register in the first iteration. From then on, an
            // iteration that leaves every register as it was repeats exactly.
            if (++repeats <= 2 && !(loop->flags & LOOP_WRITES_RAM)) {
                uint32_t registers = loop_registers(cpu);

                if (registers == previous_registers)
                    goto stable;
                // The second iteration changed only the counter and the flags it sets
                if ((loop->flags & LOOP_COUNTER) && repeats == 2) {
                    uint32_t counter_bits = (uint32_t)(CPU_NEGATIVE | CPU_ZERO) << 24 |
                        (loop_back_edge(loop)[-1].op == INY ? 0xff0000 : 0xff00);

                    if (!((registers ^ previous_registers) & ~counter_bits))
                        goto countdown;
                }
                previous_registers = registers;
            }
        }
    } while (cycles + longest < limit);
    goto finish;

countdown: {
    const ShortLoopStep *edge = loop_back_edge(loop), *counter = edge - 1;
    uint8_t *reg = counter->op == INX ? &cpu->x : &cpu->y;
    unsigned iteration = loop->cycles + extra;
    // Match the iterations the loop above would still start before the limit
    unsigned more = cycles + longest < limit ? (unsigned)((limit - cycles - longest + iteration - 1) / iteration) : 0;
    unsigned until_zero = counter->operand == 1 ? 256u - *reg : *reg;

    if (until_zero <= more) {
        more = until_zero - 1;
        exit = edge;
    }
    *reg = nz(cpu, exit ? 0 : (uint8_t)(*reg + more * counter->operand));
    repeats += more;
    cycles += (uint64_t)more * iteration;
    goto finish;
}

stable: {
    if (loop->first_status_read && !timed_status) {
        uint64_t status = ppu_status_cycle(nes);

        if (status > limit)
            limit = status < status_limit ? status : status_limit;
    }
    unsigned iteration = loop->cycles + extra;

    if (!nes->ultrafast && !loop->first_status_read && at_dmc_deadline(nes, limit)) {
        // Each stall advances the CPU clock while the loop stays on the same bus read
        cycles += batch_loop_dmc(nes, cycles, iteration);
        limit = nes->bus_deadline;
    }
    unsigned remaining = (unsigned)((limit - 1 - cycles) / iteration);

    repeats += remaining;
    cycles += remaining * iteration;
}

finish:
    if (loop->first_status_read &&
        (cycles != start || (exit && exit->instructions >= loop->first_status_read))) {
        if (timed_status) {
            uint64_t last = exit ? cycles : cycles - loop->cycles;

            ppu_latch_status_at(&nes->ppu, (last + 3) * NES_PPU_DOTS_PER_CPU);
        } else {
            ppu_read_register(nes, 0x2002);
        }
    }
    if (stats)
        stats->skipped += repeats * loop->instructions;
    uint8_t bus;

    if (exit) {
        if (stats)
            stats->skipped += exit->instructions;
        unsigned after = exit->branch_end;

        // The last bus read can be outside the decoded loop's bytes
        bus = code[after - 1];
        if (exit->target) {
            TakenBranch branch = taken_branch(cpu->pc + after, bus);

            bus = code[(int)branch.bus_address - cpu->pc];
        }
        cpu->pc += exit->target ? exit->target : back + 2;
        cycles += exit->cycles + extra;
    } else {
        bus = code[loop->repeat_bus_offset];
    }
    cpu->cycles = cycles - 1;
    cpu_drive_bus(nes, bus);
    cpu_sample_interrupts(nes, nes->irq_line);
}
#endif

static NES_ALWAYS_INLINE bool zero_page_quiet(Nes *nes, Operation op, Mode mode, Access access)
{
    Cpu *cpu = &nes->cpu;
    unsigned cycles = (mode == ZP ? 2 : 3) + (access == MODIFY ? 2 : 0);
    uint16_t pc = cpu->pc;

    if (pc < 0x8000 || cpu->cycles + cycles >= nes->bus_deadline)
        return false;
    uint8_t addr = cart_prg_read(&nes->cart, pc);

    if (mode != ZP)
        addr += mode == ZPX ? cpu->x : cpu->y;
    uint8_t value;

    if (access == WRITE) {
        value = nes->ram[addr] = store_value(cpu, op);
    } else {
        value = nes->ram[addr];

        if (access == MODIFY)
            value = nes->ram[addr] = modify(cpu, op, value);
        else
            execute(nes, op, value);
    }
    ++cpu->pc;
    complete_quiet_accesses(nes, value, cycles);
    return true;
}

static inline void push_return_quiet(Nes *nes, uint16_t address)
{
    Cpu *cpu = &nes->cpu;

    nes->ram[0x100 | cpu->s--] = address >> 8;
    nes->ram[0x100 | cpu->s--] = (uint8_t)address;
}

static inline bool return_quiet(Nes *nes, uint8_t *bus)
{
    Cpu *cpu = &nes->cpu;
    uint16_t address = combine(nes->ram[0x100 | (uint8_t)(cpu->s + 1)],
                               nes->ram[0x100 | (uint8_t)(cpu->s + 2)]);

    if (address >= 0x8000)
        *bus = cart_prg_read(&nes->cart, address);
    else if (address < 0x2000)
        *bus = nes->ram[address & 2047];
    else
        return false;
    cpu->s += 2;
    cpu->pc = address + 1;
    return true;
}

static NES_ALWAYS_INLINE bool jsr_quiet(Nes *nes)
{
    Cpu *cpu = &nes->cpu;
    unsigned pc = cpu->pc, offset = pc & 8191;

    if (pc < 0x8000 || offset == 8191 || cpu->cycles + 5 >= nes->bus_deadline)
        return false;
    const uint8_t *code = cart_prg_page(&nes->cart, pc) + offset;

    push_return_quiet(nes, pc + 1);
    cpu->pc = combine(code[0], code[1]);
    complete_quiet_accesses(nes, code[1], 5);
    return true;
}

static NES_ALWAYS_INLINE bool rts_quiet(Nes *nes)
{
    Cpu *cpu = &nes->cpu;
    uint8_t value;

    if (cpu->pc < 0x8000 || cpu->cycles + 5 >= nes->bus_deadline ||
        !return_quiet(nes, &value))
        return false;
    complete_quiet_accesses(nes, value, 5);
    return true;
}

static inline void latch_interrupts(Cpu *cpu, bool irq_blocked)
{
    cpu->irq = cpu->irq_sample && !irq_blocked;
    cpu->nmi = cpu->nmi_sample && cpu->nmi_edge;
}

#ifndef SCHNES_CPU_DEBUG
bool cpu_step(Nes *nes)
{
    Cpu *cpu = &nes->cpu;

    if (cpu->jammed) {
        cpu_read(nes, cpu->pc);
        return false;
    }
    if (cpu->nmi || cpu->irq) {
        interrupt(nes, false, cpu->nmi);
        return false;
    }

    bool old_i = (cpu->p & I) != 0;
    uint8_t opcode = fetch(nes);
    Instruction instruction = instructions[opcode];
    Operation op = (Operation)instruction.op;
    Mode mode = instruction_mode(instruction);

    switch (op) {
    case BRK:
        interrupt(nes, true, false);
        return true;
    case JMP: {
        uint16_t address = fetch_word(nes);

        cpu->pc = mode == IND ? read_word_wrap(nes, address) : address;
        break;
    }
    case JSR: {
        if (jsr_quiet(nes))
            break;
        uint8_t low = fetch(nes);

        cpu_read(nes, 0x100 | cpu->s);
        push(nes, cpu->pc >> 8);
        push(nes, (uint8_t)cpu->pc);
        cpu->pc = combine(low, fetch(nes));
        break;
    }
    case RTS: case RTI: {
        if (op == RTS && rts_quiet(nes))
            break;
        cpu_read(nes, cpu->pc);
        cpu_read(nes, 0x100 | cpu->s);
        if (op == RTI)
            cpu->p = (pop(nes) & ~B) | U;
        uint8_t low = pop(nes);

        cpu->pc = combine(low, pop(nes));

        if (op == RTS)
            fetch(nes);
        break;
    }
    default: {
        if (mode == REL) {
            branch(nes, opcode);
            break;
        }
        Access access = instruction_access(instruction);

        if (mode >= ZP && mode <= ZPY && zero_page_quiet(nes, op, mode, access))
            break;
        if (access == READ) {
            uint8_t value;

            switch (mode) {
            case IMP:
                cpu_read(nes, cpu->pc);
                value = 0;
                break;
            case IMM:
                value = fetch(nes);
                break;
            default: {
                Operand target = operand(nes, mode, false);

                value = cpu_read(nes, target.address);
                break;
            }
            }
            execute(nes, op, value);
        } else if (mode == ACC) {
            cpu_read(nes, cpu->pc);
            cpu->a = modify(cpu, op, cpu->a);
        } else {
            Operand target = operand(nes, mode, true);

            if (access == MODIFY) {
                uint8_t value = cpu_read(nes, target.address);

                cpu_write(nes, target.address, value);
                cpu_write(nes, target.address, modify(cpu, op, value));
            } else {
                store(nes, op, target, access == UNSTABLE_WRITE);
            }
        }
        break;
    }
    }
    // Only a sampled input can set an interrupt pending after entry
    if (!cpu->irq_sample && !cpu->nmi_sample)
        return true;
    bool irq_blocked = op == CLI || op == SEI || op == PLP ? old_i : (cpu->p & I) != 0;

    latch_interrupts(cpu, irq_blocked);
    return true;
}
#endif

static NES_ALWAYS_INLINE void try_decoded_loop(Nes *nes, unsigned back, NesInstructionStats *stats, unsigned floor)
{
    unsigned pc = nes->cpu.pc;

    if (pc < floor || !short_loop_in_bank(pc, back))
        return;
    const uint8_t *code = (pc >= 0x8000 ? cart_prg_page(&nes->cart, pc) : nes->cart.ram) + (pc & 8191);

    if (!closes_short_loop(code, pc, back))
        return;
    cpu_batch_short_loop(nes, code, back, stats);
}

enum { QUIET_ROM_CYCLES = 6, QUIET_RAM_CYCLES = 5 };

// Only indexed read-modify-write instructions need more than the common ROM bound
static inline unsigned quiet_rom_cycles(Mode mode, Access access)
{
    if (access == MODIFY) {
        if (mode == IZX || mode == IZY)
            return 8;
        if (mode == ABX || mode == ABY)
            return 7;
    }
    return QUIET_ROM_CYCLES;
}

typedef struct {
    uint16_t address;
    uint8_t op, access, prefix_cycles, bus, bytes;
} FinalAccess;

static inline void take_quiet_branch(Cpu *cpu, const uint8_t *code, uint64_t *budget, uint8_t *bus)
{
    uint16_t pc = cpu->pc;
    TakenBranch taken = taken_branch(pc + 2, code[1]);

    cpu->pc = taken.target;
    *budget -= 3 + taken.page_cross;
    *bus = code[(int)taken.bus_address - pc];
}

// Commit only ordinary register and memory effects here. One remaining bus access
// can be finished separately when earlier reads fit in the quiet window.
static NES_ALWAYS_INLINE bool run_quiet_instruction(Nes *nes, const uint8_t *code, uint8_t *status,
                                                   uint16_t *nz_result,
                                                   Operation op, Mode mode, Access access,
                                                   uint64_t *budget, uint8_t *bus, bool ram_read, bool ram_write,
                                                   FinalAccess *final)
{
    Cpu *cpu = &nes->cpu;
    uint16_t address = 0;
    bool cart_ram = false;
    unsigned clocks = 2;
    uint8_t value;

    if (mode == REL) {
        // Dispatch handles untaken branches before reaching this path
        take_quiet_branch(cpu, code, budget, bus);
        return true;
    }

    switch (op) {
    case BRK: case KIL: case RTI: case CLI:
        return false;
    case JMP: {
        uint16_t target = combine(code[1], code[2]);

        if (mode == IND) {
            uint16_t high = address_on_page(target, target + 1);

            if (target < 0x2000) {
                *bus = nes->ram[high & 2047];
                target = combine(nes->ram[target & 2047], *bus);
            } else if (target >= 0x8000) {
                const uint8_t *page = cart_prg_page(&nes->cart, target);

                *bus = page[high & 8191];
                target = combine(page[target & 8191], *bus);
            } else if (target >= 0x6000 && ram_read &&
                       nes->bus_deadline - *budget + 4 < nes->apu.stream_read_cycle) {
                *bus = nes->cart.ram[high & 8191];
                target = combine(nes->cart.ram[target & 8191], *bus);
            } else {
                return false;
            }
            *budget -= 5;
        } else {
            *bus = code[2];
            *budget -= 3;
        }
        cpu->pc = target;
        return true;
    }
    case JSR: {
        push_return_quiet(nes, cpu->pc + 2);
        cpu->pc = combine(code[1], code[2]);
        *budget -= 6;
        *bus = code[2];
        return true;
    }
    case RTS:
        if (!return_quiet(nes, bus))
            return false;
        *budget -= 6;
        return true;
    case PHA: case PHP: {
        uint8_t flags = nz_result ? read_status(*status, *nz_result) : *status;

        value = op == PHA ? cpu->a : flags | B | U;
        nes->ram[0x100 | cpu->s--] = value;
        *budget -= 3;
        *bus = value;
        return true;
    }
    case PLP:
        // With IRQ asserted, the instruction must sample the old interrupt mask
        if (nes->irq_line)
            return false;
        /* fall through */
    case PLA:
        value = nes->ram[0x100 | ++cpu->s];

        if (op == PLA)
            cpu->a = set_nz(status, nz_result, value);
        else {
            *status = (value & ~B) | U;
            if (nz_result)
                *nz_result = decode_nz(*status);
        }
        *budget -= 4;
        *bus = value;
        return true;
    default:
        break;
    }

    if (access == UNSTABLE_WRITE)
        return false;

    switch (mode) {
    case IMP: case ACC: case IMM:
        value = code[1];
        break;
    case ZP: case ZPX: case ZPY: {
        unsigned index = mode == ZPX ? cpu->x : mode == ZPY ? cpu->y : 0;

        address = (uint8_t)(code[1] + index);
        clocks = mode == ZP ? 3 : 4;
        value = access == WRITE ? 0 : nes->ram[address];
        break;
    }
    default: {
        uint16_t base;
        uint8_t high;
        bool dummy_read = false;
        unsigned index = mode == ABX || mode == IZX ? cpu->x : mode == ABY || mode == IZY ? cpu->y : 0;

        if (mode == IZX || mode == IZY) {
            uint8_t ptr = code[1] + (mode == IZX ? index : 0);

            high = nes->ram[(uint8_t)(ptr + 1)];
            base = combine(nes->ram[ptr], high);
            clocks = mode == IZX ? 6 : 5;
        } else {
            high = code[2];
            base = combine(code[1], high);
            clocks = 4;
        }
        address = base + (mode == IZX ? 0 : index);

        if (mode == ABX || mode == ABY || mode == IZY) {
            bool crossed = page_crossed(base, address);

            dummy_read = access != READ || crossed;

            if (dummy_read) {
                if (base >= 0x2000 && base < 0x8000) {
                    uint64_t read_cycle = nes->bus_deadline - *budget + clocks - 1;
                    uint16_t dummy = address_on_page(base, address);
                    bool open_bus = nes_apu_open_bus(dummy) &&
                        (nes_allows_deferred_io(nes) || read_cycle + 1 < nes->ppu_deadline) &&
                        !nes->apu.stream_strobe;

                    if ((!open_bus && (base < 0x6000 || !ram_read)) ||
                        read_cycle >= nes->apu.stream_read_cycle)
                        return false;
                }
                ++clocks;
            }
        }
        if (address < 0x2000)
            value = access == WRITE ? 0 : nes->ram[address & 2047];
        else if (access == READ && address >= 0x8000)
            value = cart_prg_read(&nes->cart, address);
        else if (address >= 0x6000 && address < 0x8000 && (access == READ ? ram_read : ram_write) &&
                 (access == WRITE ||
                  nes->bus_deadline - *budget + clocks - 1 < nes->apu.stream_read_cycle)) {
            cart_ram = true;
            value = access == WRITE ? 0 : nes->cart.ram[address & 8191];
        } else {
            if (access == READ || access == WRITE) {
                if (dummy_read) {
                    uint16_t intermediate = address_on_page(base, address);

                    if (intermediate < 0x2000)
                        high = nes->ram[intermediate & 2047];
                    else if (intermediate >= 0x8000)
                        high = cart_prg_read(&nes->cart, intermediate);
                    else if (intermediate >= 0x6000)
                        high = nes->cart.ram[intermediate & 8191];
                }
                *final = (FinalAccess){address, op, access, clocks - 1, high, mode_lengths[mode]};
            }
            return false;
        }
        break;
    }
    }
    if (access == READ)
        execute_operation(nes, status, nz_result, op, mode == IMP ? 0 : value);
    else if (mode == ACC)
        cpu->a = modify_status(cpu, status, nz_result, op, cpu->a);
    else {
        if (access == WRITE)
            value = store_value(cpu, op);
        else {
            value = modify_status(cpu, status, nz_result, op, value);
            clocks += 2;
        }
        if (cart_ram)
            cart_store_ram(&nes->cart, address & 8191, value);
        else
            nes->ram[address & 2047] = value;
    }
    *bus = value;
    *budget -= clocks;
    return true;
}

#if CPU_NATIVE_TRACES
// Share the uncommon opcodes without duplicating their addressing-mode handlers
static inline bool uncommon_rom_opcode(unsigned opcode, Operation op)
{
    switch (op) {
    case AHX: case ALR: case ANC: case ARR: case AXS: case DCP: case ISC:
    case LAS: case LAX: case LXA: case RLA: case RRA: case SAX: case SHX:
    case SHY: case SLO: case SRE: case TAS: case XAA:
        return true;
    case NOP:
        return opcode != 0xea;
    default:
        return false;
    }
}
#endif

enum { MMC1_SERIAL_BYTES = 19, MMC1_SERIAL_CYCLES = 28, MMC1_SERIAL_INSTRUCTIONS = 9 };

static NES_NOINLINE bool run_mmc1_serial(Nes *nes, const uint8_t *code, uint16_t address, uint64_t start)
{
    // Five identical STA operands are interleaved with four LSR A instructions
    uint8_t value = nes->cpu.a;

    if ((value & 0x80) || nes->cart.mmc1.shift != 0x10 || nes->cart.bus_conflicts ||
        code[3] != 0x4a || memcmp(code, code + 4, 4) || memcmp(code, code + 8, 4) ||
        memcmp(code, code + 12, 4) || memcmp(code, code + 16, 3))
        return false;
    nes_write_mmc1_serial(nes, address, value, start + MMC1_SERIAL_CYCLES - 1);
    nes->cpu.a = value >> 4;
    nes->cpu.p = cpu_nz_flags((nes->cpu.p & ~C) | ((value >> 3) & C), nes->cpu.a);
    return true;
}

static inline bool run_quiet_final(Nes *nes, FinalAccess *final, uint64_t deadline,
                                   uint64_t *budget, uint8_t *bus)
{
    if (!final->prefix_cycles)
        return false;
    uint64_t cycle = deadline - *budget + final->prefix_cycles;
    uint8_t value;

    if (final->access == WRITE) {
        // Final accesses admit only stable stores. Keep the shared hot path
        // small and leave unstable opcode handling in store_value().
        Cpu *cpu = &nes->cpu;

        switch ((Operation)final->op) {
        case STA: value = cpu->a; break;
        case STX: value = cpu->x; break;
        case STY: value = cpu->y; break;
        default: value = cpu->a & cpu->x; break; // SAX
        }
        if (!nes_write_quiet(nes, final->address, value, cycle))
            return false;
    } else {
        if (!nes_read_quiet(nes, final->address, cycle, final->bus, &value))
            return false;
        execute(nes, (Operation)final->op, value);
    }
    *budget -= final->prefix_cycles + 1;
    *bus = value;
    final->prefix_cycles = 0;
    return true;
}

static NES_NOINLINE unsigned finish_bus_access(Nes *nes, FinalAccess final)
{
    Cpu *cpu = &nes->cpu;

    if (final.op == STA && final.address >= 0x8000 && nes->cart.mapper == 1 &&
        cpu->pc >= 0x8000 && (cpu->pc & 8191) <= 8192 - MMC1_SERIAL_BYTES &&
        cpu->cycles + MMC1_SERIAL_CYCLES < nes->bus_deadline) {
        const uint8_t *code = cart_prg_page(&nes->cart, cpu->pc) + (cpu->pc & 8191);

        if (code[0] == 0x8d && run_mmc1_serial(nes, code, final.address, cpu->cycles)) {
            cpu->pc += MMC1_SERIAL_BYTES;
            complete_quiet_accesses(nes, cpu->a, MMC1_SERIAL_CYCLES);
            return MMC1_SERIAL_INSTRUCTIONS;
        }
    }
    cpu->pc += final.bytes;
    complete_quiet_accesses(nes, final.bus, final.prefix_cycles);
    if (final.access == WRITE)
        cpu_write_slow(nes, final.address, store_value(cpu, (Operation)final.op));
    else
        execute(nes, (Operation)final.op, cpu_read_slow(nes, final.address));
    latch_interrupts(cpu, (cpu->p & I) != 0);
    return 1;
}

#if CPU_NATIVE_TRACES
#if defined(__APPLE__) && defined(__has_attribute) && !defined(SCHNES_CPU_DEBUG)
#if __has_attribute(patchable_function_entry)
// Place a hot entry at a fixed offset within its 64-byte cache line, whatever
// code precedes it. The no-ops sit before the entry and never run.
#define NATIVE_ENTRY_AT(offset) __attribute__((aligned(64), patchable_function_entry((offset) / 4, (offset) / 4)))
#endif
#endif
#ifndef NATIVE_ENTRY_AT
#define NATIVE_ENTRY_AT(offset)
#endif
#include "cpu_aarch64.inc"
#elif !defined(SCHNES_CPU_DEBUG)
void cpu_forget_fds_traces(Cartridge *cart, uint16_t address)
{
    (void)cart;
    (void)address;
}

void cpu_discard_traces(Cartridge *cart)
{
    (void)cart;
}
#endif

// Decode LDY absolute,X and STY to a PPUDATA mirror from live RAM
static inline unsigned chr_upload_pair(const Nes *nes, unsigned pc, uint16_t *address)
{
    if (pc >= 0x2000 || (pc & 2047) > 2042)
        return 0;
    const uint8_t *code = nes->ram + (pc & 2047);

    if (code[0] != 0xbc || code[3] != 0x8c || (combine(code[4], code[5]) & 0xe007) != 0x2007)
        return 0;
    uint16_t base = combine(code[1], code[2]);

    *address = base + nes->cpu.x;

    if (base < 0x8000 || *address < 0x8000)
        return 0;
    return 8 + page_crossed(base, *address);
}

static unsigned batch_chr_upload(Nes *nes, uint64_t *budget, uint8_t *bus, uint64_t deadline)
{
    Cartridge *cart = &nes->cart;
    Ppu *p = &nes->ppu;
    Cpu *cpu = &nes->cpu;

    if (!cart->linear_chr_ram || (p->mask & 0x18) || p->mask_tick || p->v >= 0x2000 || *budget <= 9)
        return 0;

    unsigned increment = p->ctrl & 4 ? 32 : 1;
    unsigned v = p->v, count = 0, pc = cpu->pc;
    uint16_t address, next_address;
    uint8_t value;
    unsigned clocks = chr_upload_pair(nes, pc, &address);

    if (!clocks)
        return 0;

    // Leave lone uploads to the RAM dispatcher
    if (*budget - clocks <= 9 || v + increment >= 0x2000 ||
        !chr_upload_pair(nes, pc + 6, &next_address))
        return 0;

    do {
        value = cart_prg_read(cart, address);
        cart_ppu_write(cart, (uint16_t)v, value);
        v += increment;
        *budget -= clocks;
        pc += 6;
        ++count;
    } while (*budget > 9 && v < 0x2000 && (clocks = chr_upload_pair(nes, pc, &address)));

    ppu_finish_chr_upload(p, (uint16_t)v, value,
                          (deadline - *budget - 1) * NES_PPU_DOTS_PER_CPU);
    cpu->pc = (uint16_t)pc;
    cpu->y = nz(cpu, value);
    *bus = value;
    return count;
}

// Decode RAM instructions as they execute so stores can change the next opcode or operand
static NES_ALWAYS_INLINE void run_ram_window(Nes *nes, uint64_t *budget, uint8_t *bus,
                                             bool ram_read, bool ram_write, FinalAccess *final,
                                             uint64_t deadline, NesInstructionStats *stats,
                                             CpuTraceCache *cache)
{
    Cpu *cpu = &nes->cpu;
    uint64_t remaining = *budget;
    uint8_t last_bus = *bus;
    // The statistics overlay runs the debug object's copy without a raster
    NesCpuRaster *raster = CPU_DEBUG ? nes->cpu_raster : NULL;
#define RECORD_RAM_AT(cycle, type) do { if (raster) cpu_record_execution(raster, cycle, type); } while (0)
#define RECORD_RAM(type) RECORD_RAM_AT(deadline - remaining, type)
#if CPU_NATIVE_TRACES
    bool try_native = cache != NULL;
    const Fds *borrowed = borrowed_fds_traces(&nes->cart, cache);
#else
    (void)cache;
#endif
    // The RAM dispatch handles instructions of up to five cycles
    while (cpu->pc < 0x2000 && (cpu->pc & 2047) <= 2045 && remaining > QUIET_RAM_CYCLES) {
        unsigned offset = cpu->pc & 2047;
        const uint8_t *code = nes->ram + offset;
#if CPU_NATIVE_TRACES
        if (try_native && remaining > 8 && (!borrowed || borrowed->code_map)) {
            try_native = false;
            const RamBlock *block = find_ram_block(cache, code, offset, !borrowed);

            if (block && remaining > block->clocks) {
                const uint32_t *native = cache->arena + block->word - 1;
                TraceEntry entry;

                memcpy(&entry, &native, sizeof entry);
                uint64_t result = entry(nes);

                remaining -= result & TRACE_CYCLE_MASK;
                RECORD_RAM(CPU_EXEC_RECOMPILED);
                last_bus = result >> TRACE_BUS_SHIFT;
                cpu->pc += block->size;

                if (stats)
                    stats->native_trace += (uint8_t)(result >> TRACE_COUNT_SHIFT);
                if ((result & TRACE_WRITE) && remaining > 4) {
                    uint16_t address = result >> TRACE_TARGET_SHIFT;
                    uint8_t value = result >> TRACE_WRITE_VALUE_SHIFT;

                    if (nes_write_quiet(nes, address, value, deadline - remaining + 3)) {
                        remaining -= 4;
                        RECORD_RAM(CPU_EXEC_BUS_HANDLER);
                        last_bus = value;
                        cpu->pc += 3;
                        try_native = true;

                        if (stats)
                            ++stats->bus_finish;
                    }
                }
                continue;
            }
        }
#endif
        uint8_t opcode = code[0];

        if (opcode == 0xbc) {
            unsigned pairs = batch_chr_upload(nes, &remaining, &last_bus, deadline);

            if (pairs) {
                RECORD_RAM(CPU_EXEC_LOOP_BATCHED);
                if (stats)
                    stats->skipped += pairs * 2;
                continue;
            }
        }
        bool ordinary;
        unsigned bytes;
#define RAM_INSTRUCTION(byte, op, mode) \
        case 0x##byte: \
            bytes = mode_lengths[mode]; \
            ordinary = run_quiet_instruction(nes, code, &cpu->p, NULL, op, mode, ACCESS(op), \
                                              &remaining, &last_bus, ram_read, ram_write, final); \
            break;
        switch (opcode) {
            RAM_INSTRUCTION(09, ORA, IMM)
            RAM_INSTRUCTION(18, CLC, IMP)
            RAM_INSTRUCTION(29, AND, IMM)
            RAM_INSTRUCTION(38, SEC, IMP)
            RAM_INSTRUCTION(65, ADC, ZP)
            RAM_INSTRUCTION(69, ADC, IMM)
            RAM_INSTRUCTION(79, ADC, ABY)
            RAM_INSTRUCTION(85, STA, ZP)
            RAM_INSTRUCTION(88, DEY, IMP)
            RAM_INSTRUCTION(8c, STY, ABS)
            RAM_INSTRUCTION(8d, STA, ABS)
            RAM_INSTRUCTION(8e, STX, ABS)
            RAM_INSTRUCTION(98, TYA, IMP)
            RAM_INSTRUCTION(99, STA, ABY)
            RAM_INSTRUCTION(9d, STA, ABX)
            RAM_INSTRUCTION(a0, LDY, IMM)
            RAM_INSTRUCTION(a2, LDX, IMM)
            RAM_INSTRUCTION(a5, LDA, ZP)
        case 0xa9:
            // A RAM immediate load followed by an absolute device store can
            // share one decode and leave the final bus access to the usual handler.
            if (offset <= 2043 && remaining > 6 && code[2] == 0x8d) {
                uint16_t address = combine(code[3], code[4]);

                if (address >= 0x2000 && address <= 0x4017) {
                    uint8_t value = nz(cpu, code[1]);

                    cpu->a = value;

                    if (stats)
                        ++stats->direct;
                    RECORD_RAM_AT(deadline - remaining + 2, CPU_EXEC_DIRECT);
                    last_bus = value;

                    if (!nes_write_quiet(nes, address, value, deadline - remaining + 5)) {
                        cpu->pc += 2;
                        remaining -= 2;
                        *final = (FinalAccess){address, STA, WRITE, 3, code[4], 3};
                        goto done;
                    }
                    cpu->pc += 5;
                    remaining -= 6;
                    RECORD_RAM(CPU_EXEC_BUS_HANDLER);
                    if (stats)
                        ++stats->bus_finish;
#if CPU_NATIVE_TRACES
                    try_native = cache != NULL;
#endif
                    continue;
                }
            }
            bytes = 2;
            ordinary = run_quiet_instruction(nes, code, &cpu->p, NULL, LDA, IMM, READ, &remaining,
                                             &last_bus, ram_read, ram_write, final);
            break;
            RAM_INSTRUCTION(ac, LDY, ABS)
            RAM_INSTRUCTION(ad, LDA, ABS)
            RAM_INSTRUCTION(ae, LDX, ABS)
            RAM_INSTRUCTION(b9, LDA, ABY)
            RAM_INSTRUCTION(bc, LDY, ABX)
            RAM_INSTRUCTION(bd, LDA, ABX)
            RAM_INSTRUCTION(be, LDX, ABY)
            RAM_INSTRUCTION(c8, INY, IMP)
            RAM_INSTRUCTION(ca, DEX, IMP)
            RAM_INSTRUCTION(e6, INC, ZP)
            RAM_INSTRUCTION(e8, INX, IMP)
            RAM_INSTRUCTION(e9, SBC, IMM)
            RAM_INSTRUCTION(ea, NOP, IMP)
        case 0x10: case 0x30: case 0x50: case 0x70:
        case 0x90: case 0xb0: case 0xd0: case 0xf0:
            if (cpu_branch_taken(cpu->p, opcode)) {
                take_quiet_branch(cpu, code, &remaining, &last_bus);
            } else {
                remaining -= 2;
                last_bus = code[1];
                cpu->pc += 2;
            }
            if (stats)
                ++stats->direct;
            RECORD_RAM(CPU_EXEC_DIRECT);
#if CPU_NATIVE_TRACES
            try_native = cache != NULL;
#endif
            continue;
        default:
            goto done;
        }
#undef RAM_INSTRUCTION
        if (ordinary) {
            RECORD_RAM(CPU_EXEC_DIRECT);
            if (stats)
                ++stats->direct;
        } else if (!run_quiet_final(nes, final, deadline, &remaining, &last_bus)) {
            break;
        } else {
            RECORD_RAM(CPU_EXEC_BUS_HANDLER);
            if (stats)
                ++stats->bus_finish;
#if CPU_NATIVE_TRACES
            try_native = cache != NULL;
#endif
        }
        cpu->pc += bytes;
    }
done:
    *budget = remaining;
    *bus = last_bus;
#undef RECORD_RAM
#undef RECORD_RAM_AT
}

#ifndef SCHNES_CPU_DEBUG
static NES_NOINLINE void run_quiet_ram(Nes *nes, uint64_t *budget, uint8_t *bus,
                                      bool ram_read, bool ram_write, FinalAccess *final,
                                      uint64_t deadline)
{
    run_ram_window(nes, budget, bus, ram_read, ram_write, final, deadline, NULL, NULL);
}

#if CPU_NATIVE_TRACES
// Enter the RAM trace runner 28 bytes into a cache line on Apple silicon, as measured best
NATIVE_ENTRY_AT(28)
static NES_NOINLINE void run_traced_ram(Nes *nes, uint64_t *budget, uint8_t *bus,
                                       bool ram_read, bool ram_write, FinalAccess *final,
                                       uint64_t deadline, CpuTraceCache *cache)
{
    run_ram_window(nes, budget, bus, ram_read, ram_write, final, deadline, NULL, cache);
}
#endif

// Interpreter, and Regular and Ultrafast when compiled code is unavailable
#define RUN_FRAME run_interpreter_frame
#define FRAME_ATTRIBUTES
#define RUN_TRACES 0
#define RUN_INTERPRETER 1
#define RUN_FDS cart_is_fds(&nes->cart)
#include "cpu_frame.inc"

#if CPU_NATIVE_TRACES
// Regular and Ultrafast run native traces. Preserve the native dispatch
// alignment across the bus handler and trace exit paths.
#define RUN_FRAME run_frame
#define FRAME_ATTRIBUTES NATIVE_ENTRY_AT(60)
#define RUN_TRACES 1
#define RUN_INTERPRETER 0
#define RUN_FDS 0
#include "cpu_frame.inc"

// FDS programs run from RAM. Their traces are dropped when the program
// rewrites a code byte that one of them was compiled from.
#define RUN_FRAME run_fds_frame
#define FRAME_ATTRIBUTES CPU_FDS_CODE
#define RUN_TRACES 1
#define RUN_INTERPRETER 0
#define RUN_FDS 1
#include "cpu_frame.inc"
#endif

void cpu_forget_fds_loops(Nes *nes, uint16_t address)
{
    unsigned offset = address & 8191;
    unsigned first = offset > CPU_SHORT_LOOP_BACK + 2u ? address - (CPU_SHORT_LOOP_BACK + 2u) : address & 0xe000;

    for (unsigned pc = first & ~1u; pc <= address; pc += 2) {
        ShortLoop *loop = &nes->short_loops[short_loop_slot(pc)];

        if (address - loop->code_key <= loop->back + 2u)
            loop->code_key = 0;
    }
}

bool cpu_recompiler_unavailable(const Nes *nes)
{
#if CPU_NATIVE_TRACES
    // Set when the cache for compiled code cannot be allocated or mapped executable
    return nes->cart.traces_unavailable;
#else
    (void)nes;
    return true;
#endif
}

void cpu_run_frame(Nes *nes)
{
    if (nes->interpreter) {
        run_interpreter_frame(nes, NULL, NULL);
        return;
    }
#if CPU_NATIVE_TRACES
    CpuTraceCache *traces = NULL;

    if (!cart_is_fds(&nes->cart)) {
        traces = begin_trace_frame(&nes->cart, nes->ultrafast);
    } else if (begin_fds_trace_frame(&nes->cart, nes->ultrafast)) {
        // A runahead prediction runs the live machine's traces without compiling
        run_fds_frame(nes, NULL, nes->cart.trace_cache);
        return;
    }
    if (traces) {
        run_frame(nes, NULL, traces);
        return;
    }
#endif
    // The interpreter also serves when compiled code is unavailable
    run_interpreter_frame(nes, NULL, NULL);
}

#else

static CPU_DEBUG_CODE NES_NOINLINE void run_debug_ram(Nes *nes, uint64_t *budget, uint8_t *bus,
                                                     bool ram_read, bool ram_write, FinalAccess *final,
                                                     uint64_t deadline, NesInstructionStats *stats,
                                                     CpuTraceCache *cache)
{
    run_ram_window(nes, budget, bus, ram_read, ram_write, final, deadline, stats, cache);
}

// One loop serves the statistics overlay and the CPU raster debugger, in every
// CPU mode and for cartridge and FDS programs alike.
#define RUN_FRAME run_debug_frame
#define FRAME_ATTRIBUTES CPU_DEBUG_CODE
#define RUN_TRACES CPU_NATIVE_TRACES
#define RUN_INTERPRETER (nes->interpreter)
#define RUN_FDS cart_is_fds(&nes->cart)
#include "cpu_frame.inc"

CPU_DEBUG_CODE void cpu_run_debug_frame(Nes *nes, NesInstructionStats *stats)
{
    CpuTraceCache *traces = NULL;
#if CPU_NATIVE_TRACES
    if (!nes->interpreter) {
        if (!cart_is_fds(&nes->cart))
            traces = begin_trace_frame(&nes->cart, nes->ultrafast);
        else if (begin_fds_trace_frame(&nes->cart, nes->ultrafast))
            traces = nes->cart.trace_cache;
    }
#endif
    run_debug_frame(nes, stats, traces);
}

CPU_DEBUG_CODE void cpu_record_execution(NesCpuRaster *raster, uint64_t cycle, NesCpuExecution execution)
{
    uint64_t end = cycle * NES_PPU_DOTS_PER_CPU;

    if (end <= raster->end_tick)
        return;
    uint64_t count = end - raster->end_tick;
    unsigned offset = raster->end_tick & (NES_CPU_RASTER_TICKS - 1);
    unsigned first = NES_CPU_RASTER_TICKS - offset;
    uint8_t *dst = raster->execution + offset;

    if (count <= first) {
        // Fill short instruction spans with overlapping stores inside the span
        if (count == 6) {
            const uint32_t fill = (uint32_t)execution * UINT32_C(0x01010101);

            memcpy(dst, &fill, sizeof fill);
            memcpy(dst + 2, &fill, sizeof fill);
        } else if (count >= 8 && count <= 16) {
            const uint64_t fill = (uint64_t)execution * UINT64_C(0x0101010101010101);

            memcpy(dst, &fill, sizeof fill);
            memcpy(dst + count - sizeof fill, &fill, sizeof fill);
        } else {
            memset(dst, execution, (size_t)count);
        }
    } else {
        if (count > NES_CPU_RASTER_TICKS) {
            count = NES_CPU_RASTER_TICKS;
            offset = end & (NES_CPU_RASTER_TICKS - 1);
            first = NES_CPU_RASTER_TICKS - offset;
        }
        memset(raster->execution + offset, execution, first);
        memset(raster->execution, execution, (size_t)(count - first));
    }
    raster->end_tick = end;
}
#endif

#undef OPCODES
#undef ACCESS
