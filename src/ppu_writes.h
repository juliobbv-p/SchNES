#ifndef PPU_WRITES_H
#define PPU_WRITES_H

#include "mapper_writes.h"

// PPU register writes at $2000-$3FFF, landing at a CPU cycle.
//
// The raster runs behind the CPU up to its next event, Nes.ppu_deadline. A
// quiet write applies at its own dot, which Regular's repaints and
// Ultrafast's early draws start from. WRITE_RASTER marks a picture input:
// the raster must have no event pending before the write, unless the quiet
// path's policy for the register accounts for it. Mask writes carry their
// own timestamp (Ppu.mask_tick), scroll writes can be deferred over the
// lines that come before them, and address writes can bring the raster up to
// the write. WRITE_TIMED marks writes that Regular and interpreter apply from
// a raster brought up to the write. Classifications follow the mode: the
// renderers differ in what they can apply in the middle of a line.

static inline bool ppu_palette_address(unsigned address)
{
    return (address & 0x3f00) == 0x3f00;
}

// Whether an A12 change can clock the scanline counter: a rising edge on the
// MMC3, and a falling edge on the MC-ACC.
static NES_ALWAYS_INLINE bool a12_edge_clocks(const Cartridge *cart, bool from, bool to)
{
    return from != to && to != cart_is_mcacc(cart);
}

// Whether a scroll write reaches the visible lines before the next one. The
// vertical scroll waits for pre-render, and the second write sets only that.
static NES_ALWAYS_INLINE bool scroll_write_moves_lines(const Ppu *p, uint8_t value)
{
    return !p->second_write && (p->fine_x != (value & 7) || (p->t & 31) != (value >> 3));
}

static NES_ALWAYS_INLINE unsigned ctrl_write_effects(const Nes *nes, uint8_t value)
{
    const Ppu *p = &nes->ppu;
    unsigned changed = p->ctrl ^ value;

    if (nes->ultrafast) {
        if (!changed && ((p->t >> 10) & 3) == (value & 3))
            return WRITE_UNCHANGED;
        // Turning NMI on while the vblank flag is set raises the NMI line
        if (nes->cpu.nmi_line || (changed & p->status & 0x80))
            return WRITE_DEADLINE;
        // A pattern-table change can advance the first IRQ of the next frame
        if ((changed & 0x38) && a12_clocks_irq(&nes->cart))
            return WRITE_DEADLINE;
        // The latch fields can use the quiet write before the next raster event
        if (!(changed & 0xb8))
            return ((p->t >> 10) & 3) == (value & 3) ? WRITE_UNCHANGED : WRITE_RASTER;
        // Ultrafast takes the other bits only when no line renders before pre-render
        return quiet_vblank(p) ? WRITE_RASTER : WRITE_RASTER | WRITE_BUS;
    }
    // The bus handler wakes the CPU after each NMI enable change, and brings
    // the raster up to pattern table and sprite size changes.
    if (changed & 0x80)
        return WRITE_DEADLINE;
    return (changed & 0x38) ? WRITE_RASTER | WRITE_BUS : WRITE_RASTER;
}

static NES_ALWAYS_INLINE unsigned mask_write_effects(const Nes *nes, uint8_t value)
{
    const Ppu *p = &nes->ppu;
    unsigned changed = p->mask ^ value;

    // One rendering change can be pending at a time. Greyscale, clipping and
    // emphasis changes repaint the line from the raster position.
    if (p->mask_tick || (changed & 0xe7))
        return WRITE_RASTER | WRITE_BUS;
    if (!changed)
        return WRITE_TIMED;
    // Rendering changes the A12 pattern and scanline detection that clock an IRQ
    return ppu_clocks_irq(&nes->cart) ? WRITE_DEADLINE : WRITE_RASTER | WRITE_TIMED;
}

static NES_ALWAYS_INLINE unsigned scroll_write_effects(const Nes *nes, uint8_t value, uint64_t cycle)
{
    const Ppu *p = &nes->ppu;

    // Ultrafast draws a fine-X change early in a visible line from the raster
    // at the write. The position holds once no raster event comes before it.
    if (nes->ultrafast && !p->second_write && p->fine_x != (value & 7) && p->scanline < NES_HEIGHT &&
        p->dot + (cycle * NES_PPU_DOTS_PER_CPU - p->ticks) <= PPU_EARLY_DRAW_DOT)
        return WRITE_RASTER | WRITE_BUS;
    if (scroll_write_moves_lines(p, value))
        return WRITE_RASTER;
    // Pre-render copies the vertical scroll into the address. A CPU batch from
    // the visible lines ends before vblank, so only one from blanking can
    // reach that copy with the raster behind it.
    return p->second_write && p->scanline >= NES_HEIGHT && cycle + 1 >= nes->ppu_deadline ?
        WRITE_RASTER : WRITE_UNCHANGED;
}

// PPUADDR and PPUDATA
static NES_ALWAYS_INLINE unsigned address_write_effects(const Nes *nes, uint16_t address, uint8_t value,
                                                        uint64_t cycle)
{
    const Ppu *p = &nes->ppu;
    bool data = (address & 7) == 7;
    bool first = !data && !p->second_write;
    unsigned next = data ? p->v + ((p->ctrl & 4) ? 32 : 1) : (p->t & 0x7f00) | value;
    bool pending = cycle + 1 >= nes->ppu_deadline;
    // Palette data can change the active backdrop at once. Regular can apply
    // other accesses from the raster position of the last event.
    bool timed = !nes->ultrafast &&
        (nes->interpreter || (data && (ppu_palette_address(p->v) || ppu_palette_address(next))));

    // Only the horizontal nametable bit from the high address byte can
    // affect pending visible lines. The other bits wait for pre-render.
    if (first && !timed && (!pending ||
        (p->scanline < NES_HEIGHT && !((p->t ^ ((unsigned)value << 8)) & 0x400))))
        return WRITE_UNCHANGED;
    // Forced blank shows the backdrop, which only palette addresses change.
    // Quiet vblank leaves every access for pre-render.
    bool picture = (p->mask & 0x18) || p->mask_tick || ppu_palette_address(p->v) ||
                   (!first && ppu_palette_address(next));

    if (picture && !(quiet_vblank(p) && !pending)) {
        // A rendering access can move a scanline IRQ
        if (ppu_clocks_irq(&nes->cart))
            return WRITE_DEADLINE;
        return timed ? WRITE_RASTER | WRITE_TIMED : WRITE_RASTER;
    }
    // A first address byte leaves A12 alone. A data access drives both its
    // starting and incremented address, and either can clock the IRQ counter.
    if (a12_clocks_irq(&nes->cart) && !first) {
        bool a12 = nes->cart.mmc3.a12;

        // A low address held through vblank can qualify the first background fetch
        if ((p->mask & 0x18) && (p->ctrl & 0x38) == 0x10 && !cart_is_mcacc(&nes->cart) &&
            (p->v & 0x1000) && !(next & 0x1000))
            return WRITE_DEADLINE;
        if (data) {
            if (a12_edge_clocks(&nes->cart, a12, (p->v & 0x1000) != 0))
                return WRITE_DEADLINE;
            a12 = (p->v & 0x1000) != 0;
        }
        if (a12_edge_clocks(&nes->cart, a12, (next & 0x1000) != 0))
            return WRITE_DEADLINE;
    }
    return WRITE_UNCHANGED;
}

// Valid for the PPU state before the write, at a cycle the CPU batch can reach
static NES_ALWAYS_INLINE unsigned ppu_write_effects(const Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    switch (address & 7) {
    case 0:
        return ctrl_write_effects(nes, value);
    case 1:
        return mask_write_effects(nes, value);
    case 3:
        // Ultrafast brings the raster up to an OAM address write outside vblank
        return nes->ultrafast && !quiet_vblank(&nes->ppu) ? WRITE_RASTER | WRITE_BUS : WRITE_RASTER | WRITE_TIMED;
    case 5:
        return scroll_write_effects(nes, value, cycle);
    case 6: case 7:
        return address_write_effects(nes, address, value, cycle);
    default:
        // Status and OAM data writes
        return WRITE_BUS;
    }
}

#endif
