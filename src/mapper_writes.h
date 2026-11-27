#ifndef MAPPER_WRITES_H
#define MAPPER_WRITES_H

#include "write_effects.h"

// An enabled MMC3 scanline counter, which PPU address line A12 clocks. The
// VRC4 and FME-7 counters follow the CPU clock instead.
static inline bool a12_clocks_irq(const Cartridge *cart)
{
    return cart->mmc3.irq_enabled && !cart->ext;
}

// An enabled IRQ that PPU activity clocks: the MMC3 counter or the MMC5
// scanline IRQ. PPU accesses cannot move CPU-cycle counters.
static inline bool ppu_clocks_irq(const Cartridge *cart)
{
    return a12_clocks_irq(cart) || (cart->mmc5 && cart->mmc5->irq_enabled);
}

// Once vblank begins, scanline wraps have no effects through pre-render
static inline bool quiet_vblank(const Ppu *p)
{
    return p->scanline >= ppu_vblank_line(p) && p->scanline < ppu_pre_render_line(p) &&
        p->next_dot == NES_DOTS_PER_SCANLINE;
}

static inline Mirroring mapper_mirroring(const Cartridge *cart, Mirroring mirroring)
{
    return cart->mirroring == MIRROR_FOUR ? MIRROR_FOUR : mirroring;
}

// MMC1: the second to fourth serial writes only shift a bit in. Pass the first
// write to the bus handler to look for a serial batch, and the fifth, which
// commits the register.
static NES_ALWAYS_INLINE unsigned mmc1_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    if (address < 0x8000 || (value & 0x80) || c->mmc1.shift == 0x10 || (c->mmc1.shift & 1))
        return WRITE_BUS;
    return WRITE_UNCHANGED;
}

static NES_ALWAYS_INLINE unsigned mmc3_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    switch (address & 0xe001) {
    case 0x8000: {
        // Bit 6 swaps the $8000 and $C000 PRG windows, bit 7 the CHR halves.
        // Native code applies this rule itself: see emit_mmc3_quiet_select.
        unsigned changed = c->mmc3.select ^ value;

        if (!(changed & 0xc0))
            return WRITE_UNCHANGED;
        return ((changed & 0x40) ? WRITE_PRG : 0) | ((changed & 0x80) ? WRITE_RASTER : 0);
    }
    case 0x8001: {
        unsigned bank = c->mmc3.select & 7;

        if (c->mmc3.banks[bank] == value)
            return WRITE_UNCHANGED;
        // CHR changes preserve the IRQ clock, and PRG changes can leave the raster pending
        return bank < 6 ? WRITE_RASTER : WRITE_PRG;
    }
    case 0xa000:
        return mapper_mirroring(c, (value & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL) == c->mirroring ?
            WRITE_TIMED : WRITE_BUS;
    case 0xa001:
        // A CPU batch can keep its cached RAM permissions while they match
        return !((value ^ c->mmc3.ram_control) & 0xc0) ? WRITE_TIMED : WRITE_BUS;
    case 0xc000:
        // A new latch only affects the next reload or a later one
        if (c->mmc3.irq_latch == value)
            return WRITE_TIMED;
        return !c->mmc3.irq_enabled || c->mmc3.irq ||
               (value >= c->mmc3.irq_latch && mmc3_zero_asserts_irq(c)) ||
               (c->mmc3.irq_counter && !c->mmc3.irq_reload) ? WRITE_TIMED | WRITE_RASTER : WRITE_DEADLINE;
    case 0xc001: {
        // The MC-ACC reload also restarts its prescaler at the current dot
        if (c->submapper == 3)
            return WRITE_BUS;
        if (!c->mmc3.irq_enabled || c->mmc3.irq)
            return WRITE_TIMED | WRITE_RASTER;
        unsigned bits = mmc3_blocked_reload_bits(c);
        unsigned clocks = bits ? c->mmc3.irq_counter | bits : c->mmc3.irq_latch + 1u;

        return (mmc3_clocks_to_zero(c) <= clocks && mmc3_zero_asserts_irq(c)) ?
            WRITE_TIMED | WRITE_RASTER : WRITE_DEADLINE;
    }
    case 0xe000:
        // Disabling an unasserted IRQ cannot move the current deadline earlier
        return c->mmc3.irq ? WRITE_DEADLINE : WRITE_TIMED;
    default:
        return c->mmc3.irq_enabled ? WRITE_TIMED : WRITE_DEADLINE;
    }
}

// MMC5 PRG bank selects that map ROM, which leaves the CPU windows readable
// by the fast paths. Their only effect is WRITE_PRG.
static NES_ALWAYS_INLINE bool mmc5_rom_select(uint16_t address, uint8_t value)
{
    return mmc5_prg_remap(address) && ((value & 0x80) || address == 0x5117);
}

static NES_ALWAYS_INLINE unsigned mmc5_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    const Mmc5 *m = c->mmc5;

    if (mmc5_prg_remap(address)) {
        unsigned effects = m->prg[address - 0x5113] != value ? WRITE_PRG : WRITE_UNCHANGED;
        return mmc5_rom_select(address, value) ? effects : effects | WRITE_BUS;
    }
    if (address >= 0x5120 && address <= 0x512b)
        return m->chr[address - 0x5120] == (value | (m->chr_upper << 8)) ? WRITE_TIMED : WRITE_TIMED | WRITE_RASTER;
    switch (address) {
    case 0x5100:
        return m->prg_mode != (value & 3) ? WRITE_PRG | WRITE_BUS : WRITE_BUS;
    case 0x5102: case 0x5103: case 0x5205: case 0x5206:
        return WRITE_TIMED;
    case 0x5130:
        return m->exram_mode == 1 && m->chr_upper != (value & 3) ? WRITE_TIMED | WRITE_RASTER : WRITE_TIMED;
    case 0x5105:
        return m->nametable_map == value ? WRITE_TIMED : WRITE_TIMED | WRITE_RASTER;
    case 0x5203:
        if (m->irq_target == value)
            return WRITE_TIMED;
        return m->irq_enabled ? WRITE_DEADLINE : WRITE_TIMED | WRITE_RASTER;
    case 0x5204:
        return m->irq_enabled == ((value & 0x80) != 0) || (!(value & 0x80) && !m->irq_pending) ?
            WRITE_TIMED : WRITE_DEADLINE;
    default:
        return WRITE_BUS;
    }
}

static NES_ALWAYS_INLINE unsigned axrom_write_effects(const Nes *nes, uint16_t address, uint8_t value)
{
    const Cartridge *c = &nes->cart;

    if (address < 0x8000)
        return WRITE_BUS;
    // The ROM bus conflict can hold the mirroring bit low
    if (c->bus_conflicts)
        value &= cart_prg_read(c, address);
    if (mapper_mirroring(c, (value & 16) ? MIRROR_SINGLE_HIGH : MIRROR_SINGLE_LOW) == c->mirroring)
        return WRITE_PRG;
    // A one-screen change can wait for pre-render only from vblank
    return quiet_vblank(&nes->ppu) ? WRITE_PRG | WRITE_RASTER : WRITE_BUS;
}

// MMC2 and MMC4 switch PRG at $A000 and route CHR writes through the bus handler
static NES_ALWAYS_INLINE unsigned mmc2_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    switch (address >> 12) {
    case 0xa:
        return c->bank != (value & 15) ? WRITE_PRG : WRITE_UNCHANGED;
    case 0xf:
        return c->mirroring == ((value & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL) ? WRITE_TIMED : WRITE_BUS;
    default:
        return WRITE_BUS;
    }
}

// Mapper 71 switches PRG at $C000. Fire Hawk's mirroring writes at $8000-$9FFF
// need the raster up to date. Other boards ignore $8000-$BFFF.
static NES_ALWAYS_INLINE unsigned camerica_write_effects(const Cartridge *c, uint16_t address)
{
    if (address >= 0xc000)
        return WRITE_PRG;
    return address >= 0x8000 && !c->submapper ? WRITE_UNCHANGED : WRITE_BUS;
}

// VRC2/VRC4/VRC6 and FME-7 registers that the CPU can write inside a batch
static inline bool cart_has_quiet_registers(const Cartridge *cart)
{
    unsigned m = cart->mapper;
    return (m >= 21 && m <= 26) || m == 69;
}

static inline bool cart_is_vrc6(const Cartridge *cart)
{
    return cart->mapper == 24 || cart->mapper == 26;
}

// VRC registers that always select a PRG bank: $8000 and $A000, or $8000 and
// $C000 on the VRC6. Other VRC4 and FME-7 writes can switch banks too.
static NES_ALWAYS_INLINE bool vrc_selects_prg(const Cartridge *cart, uint16_t address)
{
    unsigned window = address >> 12;

    if (cart_is_vrc6(cart))
        return window == 0x8 || window == 0xc;
    return window == 0x8 || window == 0xa;
}

static inline unsigned fme7_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    const MapperExt *x = c->ext;

    switch (address >> 13) {
    case 4: case 6: return WRITE_UNCHANGED; // Command and audio register selects
    case 7: return (x->audio.select & 0xf0) ? WRITE_UNCHANGED : WRITE_AUDIO;
    }
    switch (x->command) {
    case 8: return WRITE_UNCHANGED; // $6000 always takes the bus handler's path
    case 9: case 10: case 11: return x->prg[x->command - 8] == value ? WRITE_UNCHANGED : WRITE_PRG;
    case 12: {
        static const Mirroring modes[] = {
            MIRROR_VERTICAL, MIRROR_HORIZONTAL, MIRROR_SINGLE_LOW, MIRROR_SINGLE_HIGH
        };
        return mapper_mirroring(c, modes[value & 3]) == c->mirroring ? WRITE_UNCHANGED : WRITE_BUS;
    }
    case 13: case 14: case 15: return WRITE_IRQ;
    default: return x->chr[x->command] == value ? WRITE_UNCHANGED : WRITE_RASTER;
    }
}

static inline unsigned vrc_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    const MapperExt *x = c->ext;

    if (vrc_selects_prg(c, address)) {
        bool low = (address >> 12) == 0x8;
        unsigned mask = cart_is_vrc6(c) && low ? 15 : 31;

        return x->prg[low ? 0 : 1] == (value & mask) ? WRITE_UNCHANGED : WRITE_PRG;
    }
    unsigned reg = ((address & x->vrc_a0) ? 1u : 0) | ((address & x->vrc_a1) ? 2u : 0);

    if (cart_is_vrc6(c)) {
        switch (address >> 12) {
        case 0x9: case 0xa: return WRITE_AUDIO;
        case 0xb: return reg == 3 ? WRITE_BUS : WRITE_AUDIO; // $B003 sets CHR banking and mirroring
        case 0xf: return reg == 1 || reg == 2 ? WRITE_IRQ : WRITE_UNCHANGED;
        default: return WRITE_BUS; // CHR selects can also change the mirroring
        }
    }
    switch (address >> 12) {
    case 0x9:
        if (x->vrc2 || reg < 2) {
            static const Mirroring modes[] = {
                MIRROR_VERTICAL, MIRROR_HORIZONTAL, MIRROR_SINGLE_LOW, MIRROR_SINGLE_HIGH
            };
            return mapper_mirroring(c, modes[value & (x->vrc2 ? 1 : 3)]) == c->mirroring ? WRITE_UNCHANGED : WRITE_BUS;
        }
        // VRC4 $9002 swaps the $8000 and $C000 windows
        return reg == 2 && x->vrc_swap != ((value & 2) != 0) ? WRITE_PRG : WRITE_UNCHANGED;
    // Latch writes feed the next reload and leave the running counter alone
    case 0xf: return !x->vrc2 && reg >= 2 ? WRITE_IRQ : WRITE_UNCHANGED;
    default: return WRITE_RASTER;
    }
}

// Classify one effect per VRC2, VRC4, VRC6, or FME-7 write
static inline unsigned ext_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    if (address < 0x8000)
        return WRITE_BUS;
    return c->mapper == 69 ? fme7_write_effects(c, address, value) : vrc_write_effects(c, address, value);
}

// The FDS adapter, drive and audio registers sit below $6000 and take the bus handler
static NES_ALWAYS_INLINE bool fds_register(uint16_t address)
{
    return address < 0x6000;
}

// $4023 and $4025 can change the FDS mirroring
static NES_ALWAYS_INLINE bool fds_register_maps_nametables(uint16_t address)
{
    return address == 0x4023 || address == 0x4025;
}

// A write to FDS RAM at $6000-$DFFF that changes a byte native code was compiled from
static NES_ALWAYS_INLINE bool fds_write_changes_code(const Cartridge *c, uint16_t address, uint8_t value)
{
    return fds_code_byte(c->ext->fds->code_map, address) && c->ram[address - 0x6000] != value;
}

// FDS RAM at $6000-$DFFF and the BIOS above it
static NES_ALWAYS_INLINE unsigned fds_write_effects(const Cartridge *c, uint16_t address, uint8_t value)
{
    if (fds_register(address))
        return fds_register_maps_nametables(address) ? WRITE_BUS | WRITE_RASTER : WRITE_BUS;
    if (address >= 0xe000)
        return WRITE_UNCHANGED;
    return fds_write_changes_code(c, address, value) ? WRITE_CODE : WRITE_UNCHANGED;
}

// Cartridge writes from $4020 up
static NES_ALWAYS_INLINE unsigned mapper_write_effects(const Nes *nes, uint16_t address, uint8_t value)
{
    const Cartridge *c = &nes->cart;

    if (cart_has_quiet_registers(c))
        return ext_write_effects(c, address, value);
    if (c->mmc5)
        return mmc5_write_effects(c, address, value);
    switch (c->mapper) {
    case 1: return mmc1_write_effects(c, address, value);
    case 2: return address >= 0x8000 ? WRITE_PRG : WRITE_BUS;
    case 4: return address >= 0x8000 ? mmc3_write_effects(c, address, value) : WRITE_BUS;
    case 7: return axrom_write_effects(nes, address, value);
    case 9: return mmc2_write_effects(c, address, value);
    case 20: return fds_write_effects(c, address, value);
    case 71: return camerica_write_effects(c, address);
    default: return WRITE_BUS;
    }
}

// A DMC sample is at most 4,081 bytes and can cross one 8 KiB boundary
static inline unsigned dmc_sample_pages(unsigned address, unsigned bytes)
{
    unsigned first = (address >> 13) & 3;
    unsigned last = ((address + bytes - 1) >> 13) & 3;

    return (1u << first) | (1u << last);
}

static inline unsigned dmc_prg_pages(const Dmc *d)
{
    unsigned pages = dmc_sample_pages(d->address, d->remaining);

    return d->loop ? pages | dmc_sample_pages(d->start, d->length) : pages;
}

// PRG windows affected when MMC1 commits a register, including a serial reset
static inline unsigned mmc1_dmc_remap_pages(const Cartridge *c, uint16_t address, uint8_t value)
{
    if (c->submapper == 5)
        return 0;
    switch (address >> 13) {
    case 4: return ((c->mmc1.control ^ value) & 0x0c) ? 0x0f : 0;
    case 5: return ((c->mmc1.chr0 ^ value) & 16) && c->prg_size > 262144 ? 0x0f : 0;
    case 7: {
        unsigned mode = c->mmc1.control & 0x0c;
        unsigned changed = (c->mmc1.prg ^ value) & (mode >= 8 ? 15 : 14);

        return !changed ? 0 : mode < 8 ? 0x0f : mode == 8 ? 0x0c : 0x03;
    }
    default: return 0;
    }
}

// These windows may change during a write. Called only when a streamed byte is due.
static inline unsigned mapper_dmc_remap_pages(const Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (c->mmc5)
        return (mmc5_write_effects(c, address, value) & WRITE_PRG) ? 0x0f : 0;
    if (address < 0x8000)
        return 0;
    if (cart_has_quiet_registers(c))
        return (ext_write_effects(c, address, value) & WRITE_PRG) ? 0x07 : 0;
    if (c->bus_conflicts)
        value &= cart_prg_read(c, address);

    switch (c->mapper) {
    case 1:
        if (c->mmc1.last_write != UINT64_MAX && cycle == c->mmc1.last_write + 1)
            return 0;
        if (value & 0x80)
            return mmc1_dmc_remap_pages(c, 0x8000, c->mmc1.control | 0x0c);
        return (c->mmc1.shift & 1) ?
            mmc1_dmc_remap_pages(c, address, (c->mmc1.shift >> 1) | ((value & 1) << 4)) : 0;
    case 2: return c->bank != value ? 0x03 : 0;
    case 4: {
        if ((address & 0xe000) != 0x8000 || !(mmc3_write_effects(c, address, value) & WRITE_PRG))
            return 0;
        // MMC3 swaps only the $8000/$C000 windows when PRG mode changes
        unsigned select = c->mmc3.select;

        return !(address & 1) ? 0x05 : (select & 7) == 7 ? 0x02 : (select & 0x40) ? 0x04 : 0x01;
    }
    case 7: return ((c->bank ^ value) & 7) ? 0x0f : 0;
    case 9: return (mmc2_write_effects(c, address, value) & WRITE_PRG) ? (c->mmc4 ? 0x03 : 0x01) : 0;
    case 66: return ((c->bank ^ value) & 0x30) ? 0x0f : 0;
    case 71: return address >= 0xc000 && c->bank != (value & 15) ? 0x03 : 0;
    case 206: {
        unsigned bank = c->mmc3.select & 7;

        return address < 0xa000 && (address & 1) && !c->submapper && bank >= 6 &&
            ((c->mmc3.banks[bank] ^ value) & 15) ? 1u << (bank - 6) : 0;
    }
    default: return 0;
    }
}

// FDS eligibility is checked per sample because only its BIOS is ROM
static inline bool cart_streams_dmc(const Cartridge *c)
{
    return !c->mmc5 || !c->mmc5->ram_slots;
}

// Compiled code can call nes_write_quiet_fixed_prg for these registers and
// stay in its trace, unless the write switches PRG banks. Registers that
// always select a PRG bank end the trace instead.
static NES_ALWAYS_INLINE bool mapper_store_stays_in_trace(const Cartridge *cart, uint16_t address)
{
    return cart_has_quiet_registers(cart) && address >= 0x8000 &&
        (cart->mapper == 69 || !vrc_selects_prg(cart, address));
}

#endif
