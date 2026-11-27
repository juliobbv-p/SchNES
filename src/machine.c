#include "ppu_writes.h"

#include <assert.h>
#include <string.h>

static inline bool stable_visible_raster(const Ppu *p)
{
    return p->scanline < NES_HEIGHT && (p->mask & 0x18) && !p->mask_tick;
}

static inline bool can_defer_io_raster(const Nes *nes)
{
    return nes->ultrafast || (!nes->interpreter && stable_visible_raster(&nes->ppu));
}

static inline bool mapper_irq_enabled(const Cartridge *cart)
{
    return cart->mmc3.irq_enabled || (cart->mmc5 && cart->mmc5->irq_enabled);
}

static void plan_apu_clock(Nes *nes, uint64_t cycle)
{
    const Ppu *p = &nes->ppu;

    if (p->blanking_lines && p->scanline < NES_VBLANK_LINE) {
        uint64_t start = ppu_cycle_at(p, NES_VBLANK_LINE, 0);
        unsigned clocks = ppu_cycle_at(p, ppu_vblank_line(p), 0) - start;

        apu_set_clock_pause(&nes->apu, cycle, start, clocks);
    } else {
        apu_set_clock_pause(&nes->apu, cycle, cycle, 0);
    }
}

static void set_frame_blanking(Nes *nes, unsigned lines);

static void sync_ppu_at(Nes *nes, uint64_t cycle)
{
    if (nes->raster_stats)
        ++nes->raster_stats->ppu_syncs;
    Ppu *p = &nes->ppu;
    uint64_t ticks = cycle * NES_PPU_DOTS_PER_CPU;
    unsigned elapsed = (unsigned)(ticks - p->ticks);
    bool previous_line = (p->ctrl & p->status & 0x80) != 0;
    // Between raster events only the clock and dot change
    unsigned dot = p->dot + elapsed;

    if (dot < p->next_dot && !p->early_draw_tick && (!p->mask_tick || ticks < p->mask_tick)) {
        p->dot = dot;
        p->ticks = ticks;

        if (cycle < nes->ppu_deadline && previous_line == nes->cpu.nmi_line)
            return;
    } else {
        unsigned previous_scanline = p->scanline;

        ppu_advance(nes, elapsed);
        // The pre-render dot skip is settled once the next visible line starts
        if (p->extra_blanking_lines && p->scanline < previous_scanline)
            plan_apu_clock(nes, cycle);
    }
    bool nmi_line = (p->ctrl & p->status & 0x80) != 0;
    // The CPU samples after the first PPU dot of each cycle. A dot 1
    // transition at vblank or pre-render can land after that sample.
    bool late = previous_line != nmi_line && p->dot < 4 &&
        (p->scanline == ppu_vblank_line(p) || p->scanline == ppu_pre_render_line(p));
    bool sample = late ? previous_line : nmi_line;

    if (elapsed && sample != nes->cpu.nmi_line) {
        nes->cpu.nmi_line = sample;
        nes->cpu.nmi_edge |= sample;
    }

    // Dot events happen when their dot runs. A line wrap happens on arrival
    unsigned dots = p->next_dot - p->dot + (p->next_dot < NES_DOTS_PER_SCANLINE - 1);

    if (quiet_vblank(p))
        dots = (ppu_pre_render_line(p) - p->scanline) * NES_DOTS_PER_SCANLINE - p->dot;
    if (p->mask_tick && p->mask_tick - p->ticks < dots)
        dots = (unsigned)(p->mask_tick - p->ticks);
    if (p->early_draw_tick && p->early_draw_tick - p->ticks < dots)
        dots = (unsigned)(p->early_draw_tick - p->ticks);
    if (nmi_line != nes->cpu.nmi_line)
        dots = 1;
    nes->ppu_deadline = cycle + (dots + NES_PPU_DOTS_PER_CPU - 1) / NES_PPU_DOTS_PER_CPU;
}

static inline void sync_ppu(Nes *nes)
{
    sync_ppu_at(nes, nes->cpu.cycles);
}

static inline void clock_devices(Nes *nes, uint64_t until, uint64_t raster_deadline)
{
    nes->cpu.cycles = until;

    if (until >= nes->apu.cpu_deadline)
        apu_advance(&nes->apu, until);
    if (until >= raster_deadline)
        sync_ppu(nes);
}

static inline bool near_ntsc_vblank(const Apu *a, uint64_t cycle)
{
    return a->pause_cycles && cycle - a->pause_start < NES_DOTS_PER_SCANLINE / NES_PPU_DOTS_PER_CPU;
}

static void skip_blanking_wait(Nes *nes)
{
    sync_ppu(nes);
    const Ppu *p = &nes->ppu;

    if ((p->ctrl & 0x80) || (p->mask & 0x18) || p->mask_tick || nes->apu.dmc.remaining ||
        mapper_irq_enabled(&nes->cart))
        return;

    // With rendering and NMI disabled, software can measure the hardware
    // frame length by polling PPUSTATUS. Keep that wait on the NTSC clock.
    NesCpuRaster *raster = nes->cpu_raster;

    if (raster)
        cpu_record_execution(raster, nes->cpu.cycles, raster->current);
    clock_devices(nes, nes->cpu.cycles + nes->apu.pause_cycles, 0);
    if (raster)
        cpu_record_execution(raster, nes->cpu.cycles, CPU_EXEC_INTERRUPT_IDLE);
}

static inline void latch_controllers(Nes *nes)
{
    if (nes->pad_strobe == nes->pad_strobe_write)
        return;
    nes->pad_strobe = nes->pad_strobe_write;

    if (nes->pad_strobe)
        return;
    if (nes->poll_buttons)
        nes->poll_buttons(nes->input_context, nes->buttons);
    nes->pad_shift[0] = nes->buttons[0];
    nes->pad_shift[1] = nes->buttons[1];
}

static NES_NOINLINE void service_cycle(Nes *nes, uint64_t end)
{
    if (end & 1)
        latch_controllers(nes);
    clock_devices(nes, end, nes->ppu_deadline);
}

static void tick(Nes *nes)
{
    uint64_t end = nes->cpu.cycles + 1;

    if (end < nes->ppu_deadline && end < nes->apu.cpu_deadline &&
        nes->pad_strobe == nes->pad_strobe_write)
        nes->cpu.cycles = end;
    else
        service_cycle(nes, end);
}

static NES_NOINLINE uint8_t external_io_read(Nes *nes, uint16_t address, uint8_t open_bus)
{
    if (address < 0x4000) {
        unsigned reg = address & 7;

        if (reg == 2) {
            sync_ppu(nes);
            // In stable vblank there is no extra wakeup if the NMI line stays low
            const Ppu *p = &nes->ppu;
            bool stable_blank = quiet_vblank(p) &&
                !nes->cpu.nmi_line && !(p->ctrl & p->status & 0x80);

            if ((p->scanline >= NES_HEIGHT || (p->status & 0x80)) && !stable_blank)
                nes->ppu_deadline = nes->cpu.cycles + 1;
        } else if (reg == 4 || reg == 7 || nes->cpu.cycles >= nes->ppu_deadline) {
            sync_ppu(nes);
        }
        return ppu_read_register_at(nes, address, nes->ppu.ticks);
    }
    if (address >= 0x4020) {
        if (cart_is_fds(&nes->cart))
            return fds_read(nes, address, open_bus);
        if (address == 0x5204 && nes->cart.mmc5)
            sync_ppu(nes);
        return cart_cpu_read(&nes->cart, address, open_bus);
    }
    return open_bus;
}

static uint8_t external_read(Nes *nes, uint16_t address, uint8_t open_bus)
{
    if (address >= 0x8000)
        return nes->cart.mmc5 ? cart_cpu_read(&nes->cart, address, open_bus)
                              : cart_prg_read(&nes->cart, address);
    if (address < 0x2000)
        return nes->ram[address & 2047];
    return external_io_read(nes, address, open_bus);
}

static uint8_t controller_bit(Nes *nes, uint8_t select)
{
    unsigned port = select & 1;

    // Consecutive reads that hold the same select line clock the pad once
    if (nes->pad_strobe) {
        if (nes->poll_buttons)
            nes->poll_buttons(nes->input_context, nes->buttons);
        nes->pad_bit = nes->buttons[port] & 1;
    } else if (select != nes->pad_select) {
        nes->pad_bit = nes->pad_shift[port] & 1;
        nes->pad_shift[port] = (nes->pad_shift[port] >> 1) | 128;
    }
    nes->pad_select = select;
    return nes->pad_bit;
}

static inline uint8_t read_controller(Nes *nes, uint16_t address)
{
    return (nes->open_bus & 0xe0) | controller_bit(nes, address & 31);
}

static uint8_t bus_read(Nes *nes, uint16_t address)
{
    if (address == 0x4016 || address == 0x4017)
        return nes->open_bus = read_controller(nes, address);
    nes->pad_select = 0;

    if (address == 0x4015)
        return apu_read_status(&nes->apu, nes->cpu_bus, nes->cpu.cycles);
    return nes->open_bus = external_read(nes, address, nes->open_bus);
}

static NES_NOINLINE void sync_dmc_before_remap(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    unsigned changed = mapper_dmc_remap_pages(&nes->cart, address, value, cycle);

    if (changed && (changed & dmc_prg_pages(&nes->apu.dmc)))
        apu_advance_before_refill(&nes->apu, cycle);
}

static NES_NOINLINE uint64_t project_stream_refill(Apu *a, uint64_t cycle)
{
    if (cycle <= a->stream_read_cycle)
        return a->stream_read_cycle;
    Dmc *d = &a->dmc;
    uint64_t first = apu_stream_audio_cycle(a);

    if (d->empty || d->dma_enable > first || d->remaining <= 1)
        return cycle;
    uint64_t elapsed = cycle - first;
    unsigned interval = d->period * 8;

    if (elapsed > UINT32_MAX - interval ||
        (!d->loop && elapsed >= interval * (d->remaining - 1u)))
        return cycle;

    // Once the buffer is full, refills repeat every eight DMC output clocks
    unsigned offset = (unsigned)elapsed;
    unsigned since_refill = offset < interval ? offset : offset % interval;

    if (since_refill)
        offset += interval - since_refill;
    uint64_t projected = first + offset;

    // Across a pause, advance the APU before relying on the next refill time
    if (a->pause_cycles && first < a->pause_start + a->pause_cycles && projected >= a->pause_start)
        return cycle;
    d->stream_read_offset = offset;
    a->stream_read_cycle = projected;
    return projected;
}

// OAM address writes, and mask writes, which carry the time of a rendering
// change themselves (Ppu.mask_tick): raster events before the write still
// see the old mask.
static NES_ALWAYS_INLINE bool apply_timed_ppu_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle,
                                                    unsigned effects)
{
    bool mask = (address & 7) == 1;

    if ((effects & (WRITE_BUS | WRITE_DEADLINE)) || (!mask && cycle + 1 >= nes->ppu_deadline))
        return false;
    ppu_write_register_at(nes, address, value, cycle * NES_PPU_DOTS_PER_CPU);
    if (mask && nes->ppu.mask_tick && cycle + 1 < nes->ppu_deadline)
        nes->ppu_deadline = cycle + 1;
    return true;
}

static NES_NOINLINE bool write_timed_ppu_register(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    unsigned reg = address & 7;
    return (reg == 1 || reg == 3) &&
        apply_timed_ppu_write(nes, address, value, cycle, ppu_write_effects(nes, address, value, cycle));
}

static NES_NOINLINE void write_mmc3_background(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    Cartridge *cart = &nes->cart;
    const Ppu *p = &nes->ppu;
    uint64_t tick = cycle * NES_PPU_DOTS_PER_CPU;
    unsigned dot = p->dot + (unsigned)(tick - p->ticks);
    unsigned window = (p->ctrl >> 4) & 1;
    unsigned select = cart->mmc3.select;

    if ((p->mask & 8) && dot > 1 && dot <= NES_WIDTH &&
        (!(address & 1) || (((select & 7) >= 2) ^ (select >> 7)) == window)) {
        uint64_t old = cart->chr_tags[window];

        cart_cpu_write(cart, address, value, cycle);
        if (old != cart->chr_tags[window])
            ppu_repaint_chr_mapping(nes, tick);
        return;
    }
    cart_cpu_write(cart, address, value, cycle);
}

static NES_NOINLINE bool write_chr_before_first_pixels(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    const Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    unsigned bank = cart->mmc3.select & 7;

    if (bank >= 6 || cart->mmc3.banks[bank] == value || !(p->mask & 8) ||
        p->scanline >= NES_HEIGHT - 1 || p->next_dot != NES_DOTS_PER_SCANLINE ||
        p->mask_tick || p->early_draw_tick || p->deferred_scroll_count)
        return false;

    unsigned line = p->scanline + 1;
    unsigned dot = p->dot + (unsigned)(cycle * NES_PPU_DOTS_PER_CPU - p->ticks);

    if (dot < NES_DOTS_PER_SCANLINE || dot > NES_DOTS_PER_SCANLINE + PPU_BACKGROUND_PREFETCH_DOTS ||
        p->resume_line == line ||
        ((p->mask & 0x10) && (p->sprite_rows[(p->ctrl & 0x20) != 0][line] || p->sprite_exact)))
        return false;

    // The previous row is complete and the next row has no sprite events
    cart_cpu_write(&nes->cart, address, value, cycle);
    sync_ppu_at(nes, cycle);
    return true;
}

// Sunsoft 5B audio and the MC-ACC IRQ reload need the time of the write
static NES_NOINLINE void write_late_mapper_register(Nes *nes, uint16_t address, uint8_t value)
{
    uint64_t cycle = nes->cpu.cycles;

    if (!cart_is_mcacc(&nes->cart) || (address & 0xe001) != 0xc001) {
        nes_expansion_audio_sync(nes, cycle);
        cart_cpu_write(&nes->cart, address, value, cycle);
        return;
    }
    // Count the A12 edges before the write with the old prescaler state
    nes->ppu_deadline = cycle;
    sync_ppu(nes);
    cart_cpu_write(&nes->cart, address, value, cycle);
    ppu_mcacc_reset(nes);
    nes->ppu_deadline = cycle;
    sync_ppu(nes);
}

uint64_t nes_timer_irq_cycle(Nes *nes)
{
    Cartridge *c = &nes->cart;

    if (cart_is_fds(c))
        return fds_irq_cycle(nes);
    uint64_t irq = c->ext->irq_cycle;

    if (nes->cpu.cycles < irq)
        return irq;
    // Assert IRQ at the counter's trip cycle and hold it until acknowledgment
    c->mmc3.irq = true;
    c->mmc3.irq_enabled = false;
    nes->irq_line = true;
    return UINT64_MAX;
}

#ifndef NDEBUG
// The bus handler applies the writes that the quiet path turns down. Debug
// builds check that each write's classification covers the changes it makes.
static uint64_t peripheral_deadline(const Nes *nes, uint64_t until);
static uint64_t ppu_cpu_deadline(const Nes *nes);

typedef struct {
    uint64_t deadline;
    bool irq_line;
    unsigned blanking_lines;
} ApuEvents;

static ApuEvents apu_events(const Nes *nes)
{
    return (ApuEvents){peripheral_deadline(nes, UINT64_MAX), cpu_irq_line(nes), nes->ppu.blanking_lines};
}

// A write that could have stayed in a batch keeps the APU's CPU events and
// the IRQ line, and only a raster effect changes the No-Lag lines.
static bool apu_events_follow(const Nes *nes, const ApuEvents *before, unsigned effects, uint64_t cycle)
{
    ApuEvents after = apu_events(nes);

    if (!(effects & (WRITE_RASTER | WRITE_BUS)) && after.blanking_lines != before->blanking_lines)
        return false;
    return before->deadline <= cycle + 1 || (effects & (WRITE_DEADLINE | WRITE_BUS)) ||
        (after.deadline >= before->deadline && after.irq_line == before->irq_line);
}

typedef struct {
    const uint8_t *prg_pages[4];
    uint64_t chr_tags[2], bg_tags[2];
    unsigned mirroring, nametable_map;
} MapperMapping;

typedef struct {
    uint64_t deadline;
    bool irq_line;
    unsigned nmi_level, counter;
} PpuEvents;

static PpuEvents ppu_events(const Nes *nes)
{
    const Ppu *p = &nes->ppu;
    const Cartridge *c = &nes->cart;
    unsigned counter = a12_clocks_irq(c) ? c->mmc3.irq_counter | c->mmc3.irq_reload << 8 |
        c->mmc3.irq << 9 | c->mmc3_reload_blocked << 10 : 0;

    return (PpuEvents){ppu_cpu_deadline(nes), cpu_irq_line(nes), p->ctrl & p->status & 0x80, counter};
}

// A PPU write that could have stayed in a batch keeps the NMI and IRQ lines,
// the PPU's next CPU event and an enabled scanline counter.
static bool ppu_events_follow(const Nes *nes, const PpuEvents *before, unsigned effects, uint64_t cycle)
{
    PpuEvents after = ppu_events(nes);
    return before->deadline <= cycle + 1 || (effects & (WRITE_DEADLINE | WRITE_BUS)) ||
        (after.deadline >= before->deadline && after.irq_line == before->irq_line &&
         after.nmi_level == before->nmi_level && after.counter == before->counter);
}

static MapperMapping mapper_mapping(const Cartridge *c)
{
    MapperMapping m = {.mirroring = c->mirroring, .nametable_map = cart_nametable_map(c)};

    memcpy(m.prg_pages, c->prg_pages, sizeof m.prg_pages);
    memcpy(m.chr_tags, c->chr_tags, sizeof m.chr_tags);
    if (c->mmc5)
        memcpy(m.bg_tags, c->mmc5->bg_tags, sizeof m.bg_tags);
    return m;
}

static bool mapping_follows(const Cartridge *c, const MapperMapping *before, unsigned effects)
{
    MapperMapping after = mapper_mapping(c);

    if (effects & WRITE_BUS)
        return true;
    if (!(effects & WRITE_PRG) && memcmp(before->prg_pages, after.prg_pages, sizeof after.prg_pages))
        return false;
    return (effects & WRITE_RASTER) ||
        (!memcmp(before->chr_tags, after.chr_tags, sizeof after.chr_tags) &&
         !memcmp(before->bg_tags, after.bg_tags, sizeof after.bg_tags) &&
         before->mirroring == after.mirroring && before->nametable_map == after.nametable_map);
}
#endif

static void bus_write(Nes *nes, uint16_t address, uint8_t value)
{
    nes->open_bus = value;
    nes->pad_select = 0;

    if (address < 0x2000) {
        nes->ram[address & 2047] = value;
    } else if (address < 0x4000) {
        unsigned reg = address & 7;
        const Ppu *p = &nes->ppu;
        uint64_t cycle = nes->cpu.cycles;
#ifndef NDEBUG
        unsigned effects = ppu_write_effects(nes, address, value, cycle);
#endif
        if ((nes->ultrafast || reg == 3) && write_timed_ppu_register(nes, address, value, cycle))
            return;
        // A sprite height change while rendering needs the raster position of
        // the evaluation and fetches. Ultrafast passes the write's own time.
        if (reg >= 6 || reg == 1 || reg == 3 || cycle >= nes->ppu_deadline ||
            (reg == 0 && ((p->ctrl ^ value) & 0x20) && (p->mask & 0x18) && !nes->ultrafast))
            sync_ppu(nes);
        else if (reg == 0 || (reg == 5 && nes->ultrafast)) {
            unsigned limit = nes->ultrafast ? PPU_EARLY_DRAW_DOT : NES_WIDTH;
            unsigned banks = nes->ultrafast ? 0x38 : (p->mask & 8 ? 0x10 : 0);
            bool visual = reg == 0 ? ((p->ctrl ^ value) & banks) != 0
                                  : !p->second_write && p->fine_x != (value & 7);

            if (visual && p->scanline < NES_HEIGHT &&
                p->dot + (cycle * NES_PPU_DOTS_PER_CPU - p->ticks) <= limit)
                sync_ppu(nes);
        }
        // Only a change to NMI enable needs an extra CTRL write deadline
        bool wake = reg == 1 || (reg == 0 && ((p->ctrl ^ value) & 0x80));
        uint64_t tick = (nes->ultrafast || reg == 5) ? cycle * NES_PPU_DOTS_PER_CPU : p->ticks;
#ifndef NDEBUG
        PpuEvents events = ppu_events(nes);
#endif
        ppu_write_register_at(nes, address, value, tick);
        assert(ppu_events_follow(nes, &events, effects, cycle));
        if (wake)
            nes->ppu_deadline = cycle + 1;
    } else if (address == 0x4014) {
        nes->dma_page = value;
        nes->oam_dma = true;
    } else if (address == 0x4016) {
        Apu *a = &nes->apu;
        uint64_t cycle = nes->cpu.cycles;
        bool strobe = (value & 1) != 0;

        // On an odd CPU write cycle the latch change can overlap DMA on the next read
        if (strobe != nes->pad_strobe && (cycle & 1) && a->stream_read_cycle <= cycle + 1 &&
            project_stream_refill(a, cycle) <= cycle + 1) {
            apu_advance_before_refill(a, cycle);
            a->stream_strobe = true;
            apu_set_stream_cart(a, NULL);
        }
        nes->pad_strobe_write = strobe;
    } else if (address <= 0x4017) {
#ifndef NDEBUG
        unsigned effects = apu_write_effects(&nes->apu, address, value, nes->cpu.cycles);
        ApuEvents events = apu_events(nes);
#endif
        apu_write(&nes->apu, address, value, nes->cpu.cycles);
        // A PCM stream that starts in a No-Lag frame ends its added lines
        if (address == 0x4011 && nes->ppu.blanking_lines && nes->apu.dac_writes >= NES_PCM_STREAM_WRITES)
            set_frame_blanking(nes, 0);
        assert(apu_events_follow(nes, &events, effects, nes->cpu.cycles));
    } else if (address >= 0x4020) {
        if (cart_is_fds(&nes->cart)) {
            if (fds_register_maps_nametables(address) && nes->cpu.cycles >= nes->ppu_deadline)
                sync_ppu(nes);
            fds_write(nes, address, value);
            return;
        }
        // Bring the raster up to date before a mapper write can change its inputs
        if ((address >= 0x8000 || nes->cart.mmc5) && nes->cpu.cycles >= nes->ppu_deadline) {
            if ((address & 0xe001) == 0x8001 && nes->ultrafast && nes->cart.mapper == 4 &&
                write_chr_before_first_pixels(nes, address, value, nes->cpu.cycles))
                return;
            sync_ppu(nes);
        }
        if (nes->cpu.cycles > apu_stream_audio_cycle(&nes->apu))
            sync_dmc_before_remap(nes, address, value, nes->cpu.cycles);
#ifndef NDEBUG
        unsigned effects = mapper_write_effects(nes, address, value);
        MapperMapping mapping = mapper_mapping(&nes->cart);
#endif
        if ((address & 0xe000) == 0x8000 && nes->cart.mapper == 4 &&
            nes->ppu.scanline < NES_HEIGHT)
            write_mmc3_background(nes, address, value, nes->cpu.cycles);
        else if ((address >= 0xc000 && (nes->cart.ext || cart_is_mcacc(&nes->cart))) ||
                 (address >= 0x8000 && nes->cart.ext && nes->cart.ext->has_audio))
            write_late_mapper_register(nes, address, value);
        else
            cart_cpu_write(&nes->cart, address, value, nes->cpu.cycles);
        assert(mapping_follows(&nes->cart, &mapping, effects));
        if (nes->apu.stream_cart && nes->cart.mmc5 && nes->cart.mmc5->ram_slots)
            apu_set_stream_cart(&nes->apu, NULL);
    }
}

static inline bool dmc_refill_scheduled(const Apu *a)
{
    // The APU publishes a read cycle for pending fetches it handles itself
    return a->dmc.remaining && a->stream_read_cycle == UINT64_MAX &&
        (a->dmc.empty || apu_schedules_dmc_halt(a) || apu_projects_single_dmc_halt(a));
}

static bool dmc_pending(const Nes *nes)
{
    return dmc_refill_scheduled(&nes->apu) && nes->cpu.cycles >= dmc_fetch_cycle(&nes->apu.dmc);
}

static uint64_t peripheral_deadline(const Nes *nes, uint64_t until)
{
    uint64_t cycle = nes->cpu.cycles;

    if (nes->apu.cpu_deadline < until)
        until = nes->apu.cpu_deadline;
    const Dmc *d = &nes->apu.dmc;

    if (dmc_refill_scheduled(&nes->apu)) {
        uint64_t start = dmc_fetch_cycle(d);

        if (start < until)
            until = start + 1;
    }
    if (d->dma_abort && d->dma_abort >= cycle && d->dma_abort < until)
        until = d->dma_abort + 1;
    return until;
}

static NES_ALWAYS_INLINE uint64_t ppu_cpu_deadline(const Nes *nes)
{
    const Cpu *c = &nes->cpu;
    const Ppu *p = &nes->ppu;

    if (c->nmi_line != ((p->ctrl & p->status & 0x80) != 0))
        return nes->ppu_deadline;

    // Delay raster-only events until an NMI transition, IRQ, or frame boundary.
    // CPU-clocked mapper timers don't need an early pre-render wake in
    // Ultrafast when the APU and CPU share a clock.
    unsigned line, dot, vblank = ppu_vblank_line(p);

    if (p->scanline < vblank || (p->scanline == vblank && p->next_dot == 1)) {
        line = vblank;
        dot = 2;
    } else if ((p->scanline < ppu_pre_render_line(p) || p->next_dot == 1) &&
               (c->nmi_line || (mapper_irq_enabled(&nes->cart) &&
                (!nes->ultrafast || apu_has_independent_clock(&nes->apu) || ppu_clocks_irq(&nes->cart))))) {
        line = ppu_pre_render_line(p);
        dot = 2;
    } else {
        // The flag clear has no CPU deadline if neither NMI nor a raster IRQ needs it
        line = ppu_pre_render_line(p);
        dot = NES_DOTS_PER_SCANLINE - 1;
    }
    uint64_t limit = ppu_cycle_at(p, line, dot);

    if (nes->cart.mmc3.irq_enabled) {
        uint64_t irq = ppu_irq_cycle(nes);

        if (irq < limit)
            limit = irq;
    } else if (nes->cart.mmc5 && nes->cart.mmc5->irq_enabled) {
        uint64_t irq = ppu_mmc5_irq_cycle(nes);

        if (irq < limit)
            limit = irq;
    }
    return limit;
}

static void schedule_bus(Nes *nes)
{
    nes->pad_resume_deadline = 0;

    if (nes->raster_stats)
        ++nes->raster_stats->bus_reschedules;
    // Fast RAM and ROM accesses may run only before the next bus-visible event
    nes->irq_line = cpu_irq_line(nes);
    uint64_t cycle = nes->cpu.cycles;

    // Quiet PRG dispatch assumes immutable ROM in the upper CPU windows
    if (nes->oam_dma || nes->pad_strobe != nes->pad_strobe_write ||
        (nes->cart.mmc5 && nes->cart.mmc5->ram_slots)) {
        nes->bus_deadline = cycle + 1;
        return;
    }
    nes->bus_deadline = peripheral_deadline(nes, ppu_cpu_deadline(nes));
}

static uint8_t dma_read(Nes *nes, uint16_t address, uint16_t halted_address)
{
    // The CPU still drives the upper bits of the APU register decoder while
    // DMA drives its low five bits and the external address bus.
    unsigned reg = (halted_address & 0xffe0) == 0x4000 ? address & 31 : 0;

    if (reg == 0x16 || reg == 0x17) {
        uint8_t value = external_read(nes, address, nes->open_bus | 0x1f);
        uint8_t bit = controller_bit(nes, reg);

        // Model the controller output dominating the PRG ROM conflict, with
        // a driven low taking precedence when the external source is RAM.
        if (address < 0x8000)
            bit &= value;
        nes->open_bus = (value & 0xe0) | bit;
        return nes->open_bus;
    }
    nes->pad_select = 0;
    nes->open_bus = external_read(nes, address, nes->open_bus);

    if (reg == 0x15)
        return apu_read_status(&nes->apu, nes->open_bus, nes->cpu.cycles);
    return nes->open_bus;
}

static inline void dmc_read_rom(Nes *nes, uint64_t get, uint64_t end)
{
    Apu *a = &nes->apu;
    uint8_t value = cart_prg_read(&nes->cart, a->dmc.address);

    apu_dmc_fetch(a, value, get, end);
}

uint64_t nes_dmc_trace_deadline(const Nes *nes, uint64_t halt, uint64_t end)
{
    const Apu *a = &nes->apu;
    const Dmc *d = &a->dmc;
    const Ppu *p = &nes->ppu;

    if (d->dma_halt != halt || !apu_schedules_dmc_halt(a) || d->remaining <= 1 ||
        d->empty || d->stop_after_fetch || d->dma_abort || d->dma_enable > halt ||
        !(halt & 1) || nes->oam_dma || nes->pad_strobe != nes->pad_strobe_write ||
        end + 4 >= a->cpu_deadline ||
        (halt + 4 >= nes->ppu_deadline && !stable_visible_raster(p)))
        return 0;
    uint64_t raster_limit = ppu_cpu_deadline(nes);

    return end + 4 < raster_limit ? raster_limit : 0;
}

bool nes_dmc_queue_trace(Nes *nes, uint64_t halt, uint64_t raster_limit)
{
    Apu *a = &nes->apu;

    // The validated trace leaves the PRG mapping and all I/O alone. The
    // APU can queue this exact-cycle read without advancing its clock.
    if (!apu_queue_dmc_byte(a, cart_prg_read(&nes->cart, a->dmc.address), halt + 3))
        return false;
    nes->bus_deadline = peripheral_deadline(nes, raster_limit);
    return true;
}

void nes_dmc_finish_trace(Nes *nes, uint64_t halt, uint64_t raster_limit)
{
    // Halt, align, and fetch in four cycles because the read falls on an odd cycle
    dmc_read_rom(nes, halt + 3, halt + 4);
    nes->cpu.cycles += 4;
    nes->bus_deadline = peripheral_deadline(nes, raster_limit);
}

static NES_ALWAYS_INLINE void copy_oam_dma(Nes *nes, const uint8_t *source, bool stream_dmc)
{
    unsigned left = 256;

    while (left) {
        uint64_t cycle = nes->cpu.cycles;
        uint64_t raster = nes->ppu_deadline;
        const Ppu *p = &nes->ppu;

        if (!nes->ultrafast && nes->apu.cpu_deadline <= cycle + 2 * left && quiet_vblank(p)) {
            // Preserve the scanline-sized APU sample spans during OAM DMA
            uint64_t line = ppu_cycle_at(p, p->scanline + 1, 0);

            if (cycle >= line) {
                sync_ppu_at(nes, cycle);
                raster = nes->ppu_deadline;
                line = ppu_cycle_at(p, p->scanline + 1, 0);
            }
            if (line < raster)
                raster = line;
        }
        uint64_t deadline = raster;

        if (stream_dmc) {
            // Supply playback samples between blocks without stealing OAM slots
            if (dmc_pending(nes))
                dmc_read_rom(nes, cycle, nes->cpu.cycles);
            deadline = peripheral_deadline(nes, deadline);
        }
        // Each byte writes on its second cycle. A PPU event in the middle
        // of that pair sees the old OAM, so process that first cycle alone.
        unsigned count = !stream_dmc || deadline >= cycle ? (unsigned)((deadline - cycle) / 2) : 0;

        if (!count) {
            clock_devices(nes, cycle + 1, raster);
            count = 1;
        } else if (count > left) {
            count = left;
        }
        ppu_copy_oam(&nes->ppu, source, count);
        source += count;
        clock_devices(nes, cycle + 2 * count, raster);
        left -= count;
    }
    if (stream_dmc && dmc_pending(nes))
        dmc_read_rom(nes, nes->cpu.cycles, nes->cpu.cycles);
    nes->open_bus = source[-1];
}

static bool dma_ram_to_oam(Nes *nes, uint16_t halted_address, const uint8_t *source)
{
    const Dmc *dmc = &nes->apu.dmc;
    bool stream_dmc = dmc->remaining && nes->apu.stream_read_cycle == UINT64_MAX;

    if (stream_dmc && (!nes->ultrafast || dmc->length <= 1))
        return false;

    nes->oam_dma = false;
    bus_read(nes, halted_address);
    tick(nes);
    if (nes->cpu.cycles & 1) {
        bus_read(nes, halted_address);
        tick(nes);
    }

    // Select the loop once because ordinary sprite DMA tracks only PPU events
    if (stream_dmc)
        copy_oam_dma(nes, source, true);
    else
        copy_oam_dma(nes, source, false);
    return true;
}

static NES_NOINLINE void dma(Nes *nes, uint16_t halted_address)
{
    const uint8_t *source = nes->oam_dma && nes->dma_page < 0x20 &&
        nes->pad_strobe == nes->pad_strobe_write &&
        (halted_address < 0x2000 || halted_address >= 0x8000 || (halted_address >= 0x6000 && cart_is_fds(&nes->cart)))
        ? nes->ram + (nes->dma_page & 7) * 256 : NULL;

    if (source && dma_ram_to_oam(nes, halted_address, source))
        return;

    Dmc *dmc = &nes->apu.dmc;
    unsigned offset = nes->oam_dma ? 0 : 256;
    uint16_t page = (uint16_t)nes->dma_page << 8;
    bool oam_latched = false;
    uint8_t oam_value = 0;
    uint64_t dmc_get = dmc_pending(nes) ? nes->cpu.cycles + 2 : 0;

    nes->oam_dma = false;
    bus_read(nes, halted_address);
    tick(nes);

    while (offset < 256 || dmc_get || dmc_pending(nes)) {
        uint64_t cycle = nes->cpu.cycles;
        bool get = !(cycle & 1);

        if (!dmc_get && dmc_pending(nes))
            dmc_get = cycle + 2;

        // During blanking, copy unobstructed RAM-to-OAM pairs up to the next
        // event. A pending DMC get still uses the cycle-by-cycle arbitration.
        const Ppu *p = &nes->ppu;

        if (source && get && !dmc_get && offset < 256 &&
            p->scanline >= NES_HEIGHT && p->scanline < ppu_pre_render_line(p)) {
            uint64_t raster = nes->ppu_deadline;

            // In vblank, a scanline wrap cannot change OAM or request an interrupt
            if (quiet_vblank(p) && !p->mask_tick && !p->early_draw_tick &&
                nes->cpu.nmi_line == ((p->ctrl & p->status & 0x80) != 0))
                raster = ppu_cycle_at(p, ppu_pre_render_line(p), 0);
            uint64_t deadline = peripheral_deadline(nes, raster);

            if (deadline > cycle + 1) {
                unsigned count = (unsigned)((deadline - cycle) / 2);

                if (count > 256 - offset)
                    count = 256 - offset;
                ppu_copy_oam(&nes->ppu, source + offset, count);
                offset += count;
                nes->open_bus = source[offset - 1];
                nes->pad_select = 0;
                clock_devices(nes, cycle + 2 * count, nes->ppu_deadline);
                continue;
            }
        }

        // DMC halt and dummy cycles can overlap sprite transfers. Only the
        // sample read consumes a get cycle that OAM DMA could otherwise use.
        if (!get && oam_latched) {
            bus_write(nes, 0x2004, oam_value);
            oam_latched = false;
            ++offset;
        } else if (get && dmc_get && cycle >= dmc_get) {
            uint8_t sample = dma_read(nes, dmc->address, halted_address);

            // The output clock sees the old buffer if it coincides with a get
            tick(nes);
            apu_advance(&nes->apu, nes->cpu.cycles);
            apu_dmc_supply(&nes->apu, sample, cycle);
            dmc_get = 0;
            continue;
        } else if (get && offset < 256) {
            oam_value = dma_read(nes, page | offset, halted_address);
            oam_latched = true;
        } else {
            bus_read(nes, halted_address);
        }
        tick(nes);
    }
}

static inline void record_dma(Nes *nes, uint64_t start)
{
    NesCpuRaster *raster = nes->cpu_raster;

    if (raster && nes->cpu.cycles > start) {
        cpu_record_execution(raster, start, raster->current);
        cpu_record_execution(raster, nes->cpu.cycles, CPU_EXEC_DMA);
    }
}

// The byte that an FME-7 maps at $6000-$7FFF, if a plain access reaches it.
// Writes reach only RAM without a battery.
static NES_NOINLINE uint8_t *fme7_wram(const Cartridge *c, uint16_t address, bool write)
{
    if (c->mapper != 69)
        return NULL;
    const MapperExt *x = c->ext;
    unsigned reg = x->prg[0];

    if (x->wram_rom) {
        // The bus handler ignores writes to ROM
        return write ? NULL : (uint8_t *)x->wram_rom + (address & 8191);
    }
    if (!(reg & 0x80) || !x->banked_ram_size || (write && c->nvram_size))
        return NULL;
    return c->ram + ((reg & 63u) * 8192u + (address & 8191)) % x->banked_ram_size;
}

static uint8_t *quiet_prg_ram(Nes *nes, uint16_t address, bool write)
{
    if (address < 0x6000 || address >= 0x8000 ||
        nes->cpu.cycles + 1 >= nes->bus_deadline)
        return NULL;
    const Cartridge *c = &nes->cart;

    if ((write && c->nvram_size) || !cart_unbanked_ram(c, write))
        return c->ext ? fme7_wram(c, address, write) : NULL;
    return c->ram + (address & 8191);
}

static void dmc_dma(Nes *nes, uint16_t address)
{
    bool ultrafast = nes->ultrafast;

    if ((address >= 0x2000 && address < 0x8000) ||
        nes->pad_strobe != nes->pad_strobe_write ||
        (nes->cart.mmc5 && nes->cart.mmc5->ram_slots)) {
        dma(nes, address);
        return;
    }

    // RAM and ROM drive all bits on the CPU read, so there is no open-bus
    // effect to reproduce. Multi-byte playback leaves that read unstalled.
    if (ultrafast && nes->apu.dmc.length != 1) {
        dmc_read_rom(nes, nes->cpu.cycles, nes->cpu.cycles);
        return;
    }

    // Preserve the halt, dummy cycle and get-cycle alignment. In the
    // timing modes an intervening device event needs the cycle-by-cycle path.
    uint64_t get = (nes->cpu.cycles + 3) & ~UINT64_C(1);
    uint64_t end = get + 1;
    uint64_t deadline = nes->ppu_deadline;

    // A RAM or ROM access cannot observe raster-only events during the halt
    if (end > deadline && (ultrafast || stable_visible_raster(&nes->ppu)))
        deadline = ppu_cpu_deadline(nes);
    if (end > deadline ||
        (!ultrafast && (end == deadline || end >= nes->apu.cpu_deadline))) {
        dma(nes, address);
        return;
    }
    nes->cpu.cycles = end;
    dmc_read_rom(nes, get, nes->cpu.cycles);
    if (end == deadline)
        sync_ppu(nes);
}

// Keep DMA trace bookkeeping in the out-of-line DMA handlers
static NES_NOINLINE void service_dmc_abort(Nes *nes, uint16_t address)
{
    uint64_t start = nes->cpu.cycles;

    bus_read(nes, address);
    tick(nes);
    record_dma(nes, start);
}

static NES_NOINLINE void service_dma(Nes *nes, uint16_t address, bool oam)
{
    uint64_t start = nes->cpu.cycles;

    if (oam)
        dma(nes, address);
    else
        dmc_dma(nes, address);
    record_dma(nes, start);
}

static NES_ALWAYS_INLINE void finish_cpu_bus(Nes *nes, uint16_t address, bool read)
{
    uint64_t end = nes->cpu.cycles + 1;

    if (nes->ultrafast && (address & 0xe007) == 0x2002 && end < nes->bus_deadline &&
        nes->ppu.scanline < NES_HEIGHT && !(nes->ppu.status & 0x80) &&
        !ppu_clocks_irq(&nes->cart)) {
        nes->cpu.cycles = end;
        return;
    }
    bool latch_pending = nes->pad_strobe != nes->pad_strobe_write;
    // The controller latch still changes on the get-to-put transition.
    // A RAM or ROM access on that cycle only needs CPU-visible PPU events.
    bool memory = address < 0x2000 || address >= 0x8000;
    bool controller_cycle = latch_pending && ((address & 0xfffe) == 0x4016 || memory);
    // Internal RAM writes and APU accesses can leave pixel work pending
    bool defer_raster = address < 0x2000 || (read && memory) ||
                        (address >= 0x4000 && address <= 0x4017 &&
                         address != 0x4014 && address != 0x4016);

    if (end >= nes->ppu_deadline && defer_raster && !latch_pending && !nes->oam_dma &&
        stable_visible_raster(&nes->ppu)) {
        clock_devices(nes, end, ppu_cpu_deadline(nes));
    } else if (end >= nes->ppu_deadline &&
        can_defer_io_raster(nes) &&
        ((address >= 0x4000 && address <= 0x4017) ||
         (nes->ultrafast && (address & 0xe007) == 0x2002) ||
         ((address & 0xe007) == 0x2001 && nes->ppu.mask_tick && !ppu_clocks_irq(&nes->cart)) ||
         controller_cycle || (read && memory)) && !nes->oam_dma && (!latch_pending || controller_cycle)) {
        if (latch_pending && (end & 1))
            latch_controllers(nes);
        clock_devices(nes, end, ppu_cpu_deadline(nes));
    } else {
        tick(nes);
    }
    // An odd-cycle strobe temporarily shortens the CPU window by one
    // access. A plain memory read can restore it after the latch settles.
    if (latch_pending) {
        if (read) {
            if ((address < 0x2000 || address >= 0x8000) && (end & 1) &&
                end < nes->pad_resume_deadline) {
                nes->bus_deadline = nes->pad_resume_deadline;
                nes->pad_resume_deadline = 0;
                return;
            }
        } else if (address == 0x4016 && !(end & 1) && end + 1 < nes->bus_deadline &&
                   (end + 1 < nes->ppu_deadline || can_defer_io_raster(nes)) &&
                   !nes->oam_dma && !nes->apu.stream_strobe && end + 1 < nes->apu.cpu_deadline) {
            nes->pad_resume_deadline = nes->bus_deadline;
            nes->irq_line = cpu_irq_line(nes);
            nes->bus_deadline = end + 1;
            return;
        }
    }
    schedule_bus(nes);
}

static inline bool status_latch_ready(Nes *nes, uint64_t end)
{
    return ppu_cached_status_supported(nes) ? end < ppu_status_read_deadline(nes)
        : ppu_waiting_sprite_zero(&nes->ppu) && end < ppu_sprite_zero_cycle(nes);
}

static inline uint8_t read_quiet_controller(Nes *nes, uint16_t address, uint64_t cycle)
{
    cpu_sample_interrupts(nes, nes->irq_line);
    uint8_t value = read_controller(nes, address);

    nes->open_bus = nes->cpu_bus = value;
    nes->cpu.cycles = cycle + 1;
    return value;
}

static inline bool read_quiet_status(Nes *nes, uint64_t cycle, uint8_t *value)
{
    Ppu *p = &nes->ppu;

    if (!(p->ctrl & 0x80) && !(p->mask & 0x18) && near_ntsc_vblank(&nes->apu, cycle))
        return false;
    bool defer_status = nes->ultrafast ||
        (!p->early_draw_tick && can_defer_io_raster(nes));
    bool visible = p->scanline < NES_HEIGHT && !(p->status & 0x80) &&
        (cycle + 1 < nes->ppu_deadline ||
         (defer_status && status_latch_ready(nes, cycle + 1)));
    // After the vblank edge, the latch can clear without changing the NMI line
    bool blank = !visible && (defer_status || cycle + 1 < nes->ppu_deadline) &&
        quiet_vblank(p) && !p->mask_tick && !p->early_draw_tick &&
        !nes->cpu.nmi_line && !(p->ctrl & p->status & 0x80) &&
        cycle + 1 < ppu_cycle_at(p, ppu_pre_render_line(p), 0);
    bool catch_up = !visible && !blank;

    // Read the current sprite flags without ending the CPU batch. Leave
    // vblank suppression to the full read path.
    if (catch_up && (p->scanline >= NES_HEIGHT || (p->status & 0x80) ||
        cycle + 1 >= ppu_cycle_at(p, NES_HEIGHT, 0)))
        return false;
    cpu_sample_interrupts(nes, nes->irq_line);
    if (catch_up || (!defer_status && visible))
        sync_ppu_at(nes, cycle);
    *value = ppu_latch_status_at(p, cycle * NES_PPU_DOTS_PER_CPU);
    p->status &= 0x7f;
    nes->cpu.cycles = cycle;
    cpu_drive_bus(nes, *value);
    if (catch_up && !defer_status && cycle + 1 >= nes->ppu_deadline)
        sync_ppu(nes);
    return true;
}

static bool quiet_ppu_data_read(const Nes *nes, unsigned reg, uint64_t end)
{
    const Ppu *p = &nes->ppu;

    if (end >= nes->ppu_deadline || p->mask_tick)
        return false;
    bool vblank = quiet_vblank(p);

    if (reg == 4)
        return vblank;
    if (ppu_clocks_irq(&nes->cart))
        return false;
    if (vblank)
        return true;
    // During forced blank the address can select the visible backdrop
    unsigned next = p->v + ((p->ctrl & 4) ? 32 : 1);

    return nes_allows_deferred_io(nes) && !(p->mask & 0x18) &&
           (p->v & 0x3f00) != 0x3f00 && (next & 0x3f00) != 0x3f00;
}

static NES_NOINLINE bool read_quiet_device_status(Nes *nes, uint16_t address, uint64_t cycle,
                                                  uint8_t external_bus, uint8_t internal_bus, uint8_t *value)
{
    if (address == 0x4015 && nes->apu.frame_inhibit && !nes->apu.frame_irq_line &&
        (nes_allows_deferred_io(nes) || cycle + 1 < nes->ppu_deadline)) {
        // The CPU sees the APU status, while the external bus retains its last value
        cpu_sample_interrupts(nes, nes->irq_line);
        *value = apu_read_status(&nes->apu, internal_bus, cycle);
        nes->pad_select = 0;
        nes->open_bus = external_bus;
        nes->cpu_bus = *value;
        nes->cpu.cycles = cycle + 1;
        return true;
    }
    if ((address >> 13) == 3) {
        const uint8_t *byte = fme7_wram(&nes->cart, address, false);

        if (!byte)
            return false;
        *value = *byte;
        cpu_sample_interrupts(nes, nes->irq_line);
        nes->cpu.cycles = cycle;
        cpu_drive_bus(nes, *value);
        return true;
    }
    if (address == 0x5204 && nes->cart.mmc5) {
        Mmc5 *m = nes->cart.mmc5;

        if (m->irq_enabled && m->irq_pending)
            return false;
        const Ppu *p = &nes->ppu;
        // A disabled interrupt can still set the pending bit that this read clears
        bool stable = nes_allows_deferred_io(nes) && m->in_frame && stable_visible_raster(p) &&
            cycle + 1 < ppu_cycle_at(p, NES_HEIGHT, 0) &&
            (m->irq_enabled || cycle + 1 < ppu_mmc5_irq_cycle(nes));

        if (!stable) {
            if (cycle + 1 >= nes->ppu_deadline)
                return false;
            sync_ppu_at(nes, cycle);
        }
        *value = cart_cpu_read(&nes->cart, address, external_bus);
        cpu_sample_interrupts(nes, nes->irq_line);
        nes->cpu.cycles = cycle;
        cpu_drive_bus(nes, *value);
        return true;
    }
    return false;
}

static inline bool read_quiet_io(Nes *nes, uint16_t address, uint64_t cycle,
                                uint8_t external_bus, uint8_t internal_bus, uint8_t *value)
{
    unsigned reg = address & 0xe007;

    if (reg == 0x2002)
        return read_quiet_status(nes, cycle, value);
    if ((reg == 0x2004 || reg == 0x2007) && quiet_ppu_data_read(nes, reg & 7, cycle + 1))
        *value = ppu_read_register_at(nes, address, cycle * NES_PPU_DOTS_PER_CPU);
    else if (nes_apu_open_bus(address) && (nes_allows_deferred_io(nes) || cycle + 1 < nes->ppu_deadline))
        *value = external_bus;
    else
        return (address == 0x4015 || address == 0x5204 || ((address >> 13) == 3 && nes->cart.ext)) &&
            read_quiet_device_status(nes, address, cycle, external_bus, internal_bus, value);
    cpu_sample_interrupts(nes, nes->irq_line);
    nes->cpu.cycles = cycle;
    cpu_drive_bus(nes, *value);
    return true;
}

bool nes_read_quiet(Nes *nes, uint16_t address, uint64_t cycle, uint8_t prior_bus, uint8_t *value)
{
    if (cycle + 1 >= nes->bus_deadline || nes->apu.stream_strobe || cycle >= nes->apu.stream_read_cycle)
        return false;
    if ((address & 0xfffe) == 0x4016) {
        // Opcode and address fetches drove the external bus and released either port
        nes->open_bus = nes->cpu_bus = prior_bus;
        nes->pad_select = 0;
        nes->cpu.cycles = cycle;
        *value = read_quiet_controller(nes, address, cycle);
        return true;
    }
    return read_quiet_io(nes, address, cycle, prior_bus, prior_bus, value);
}

static bool read_memory_dmc(Nes *nes, uint16_t address, uint8_t *value)
{
    if ((address >= 0x2000 && address < 0x8000) || nes->apu.stream_strobe ||
        (nes->cart.mmc5 && nes->cart.mmc5->ram_slots))
        return false;
    uint64_t cycle = nes->cpu.cycles;
    const Dmc *d = &nes->apu.dmc;
    bool irq_fetch = d->irq_enabled;
    bool unstalled = irq_fetch && nes->ultrafast && d->length != 1;
    uint64_t get = unstalled ? cycle : (cycle + 3) & ~UINT64_C(1);
    uint64_t end = get + !unstalled;
    uint64_t raster_limit;

    if (irq_fetch) {
        if (!stable_visible_raster(&nes->ppu) ||
            nes->pad_strobe != nes->pad_strobe_write || (!unstalled && end == nes->ppu_deadline) ||
            end >= nes->apu.cpu_deadline)
            return false;
        raster_limit = ppu_cpu_deadline(nes);

        if (end + 1 >= raster_limit)
            return false;
    } else {
        raster_limit = nes_dmc_trace_deadline(nes, d->dma_halt, cycle + 1);

        if (!raster_limit)
            return false;
    }

    nes->cpu.cycles = end;
    record_dma(nes, cycle);
    dmc_read_rom(nes, get, end);
    *value = address < 0x2000 ? nes->ram[address & 2047] : cart_prg_read(&nes->cart, address);
    bool irq = irq_fetch ? cpu_irq_line(nes) : nes->irq_line;

    cpu_sample_interrupts(nes, irq);
    cpu_drive_bus(nes, *value);
    if (irq_fetch) {
        // Preserve interrupt sampling before the end-of-access APU clock
        if (nes->cpu.cycles >= nes->apu.cpu_deadline) {
            apu_advance(&nes->apu, nes->cpu.cycles);
            irq = cpu_irq_line(nes);
        }
        nes->pad_resume_deadline = 0;
        nes->irq_line = irq;
    }
    nes->bus_deadline = peripheral_deadline(nes, raster_limit);
    return true;
}

uint8_t cpu_read_slow(Nes *nes, uint16_t address)
{
    Apu *a = &nes->apu;
    uint64_t cycle = nes->cpu.cycles;
    bool refill_due = cycle >= a->stream_read_cycle && address >= 0x2000 && address < 0x8000;

    if (refill_due)
        refill_due = cycle >= project_stream_refill(a, cycle);
    // The bus deadline lets raster-only work wait past a controller read
    if (!refill_due && !a->stream_strobe && cycle + 1 < nes->bus_deadline) {
        if ((address & 0xfffe) == 0x4016)
            return read_quiet_controller(nes, address, cycle);
        uint8_t value;

        if (read_quiet_io(nes, address, cycle, nes->open_bus, nes->cpu_bus, &value))
            return value;
    }
    const uint8_t *ram = refill_due ? NULL : quiet_prg_ram(nes, address, false);

    if (ram) {
        uint8_t value = *ram;

        cpu_sample_interrupts(nes, nes->irq_line);
        cpu_drive_bus(nes, value);
        return value;
    }
    const Cartridge *resume_stream = NULL;

    // Keep the bus stall if the refill lands on an I/O or cartridge RAM read
    if (refill_due) {
        apu_advance_before_refill(a, nes->cpu.cycles);
        if (nes->cpu.cycles >= a->stream_read_cycle) {
            resume_stream = a->stream_cart;
            apu_set_stream_cart(a, NULL);
        }
    }
    // Unlike a full DMA, a one-cycle abort expires if the CPU writes here
    if (nes->apu.dmc.dma_abort && nes->apu.dmc.dma_abort == nes->cpu.cycles &&
        !nes->oam_dma && !dmc_pending(nes))
        service_dmc_abort(nes, address);
    if (nes->oam_dma)
        service_dma(nes, address, true);
    else if (dmc_pending(nes)) {
        uint8_t value;

        if (!resume_stream && read_memory_dmc(nes, address, &value))
            return value;
        service_dma(nes, address, false);
    }
    if (a->stream_strobe && nes->pad_strobe == nes->pad_strobe_write) {
        a->stream_strobe = false;
        resume_stream = &nes->cart;
    }
    if (resume_stream)
        apu_set_stream_cart(a, resume_stream);
    if ((address & 0xe007) == 0x2002 && near_ntsc_vblank(a, nes->cpu.cycles))
        skip_blanking_wait(nes);
    cpu_sample_interrupts(nes, cpu_irq_line(nes));
    uint8_t value = bus_read(nes, address);

    nes->cpu_bus = value;
    finish_cpu_bus(nes, address, true);
    return value;
}

static bool scroll_can_cross_deadline(const Cartridge *cart)
{
    switch (cart->mapper) {
    case 0: case 1: case 2: case 3: case 7: case 9: case 71:
        return true;
    case 4:
        return !cart->mmc3.irq_enabled && !cart->mmc3.irq;
    case 5:
        return cart->mmc5 && !cart->mmc5->irq_enabled;
    default:
        return false;
    }
}

static NES_ALWAYS_INLINE bool scroll_window_open(const Nes *nes, uint64_t tick)
{
    const Ppu *p = &nes->ppu;

    // Stay in visible lines. Reads of PPU data and writes that change
    // mapper raster state will synchronize before using pending events.
    return scroll_can_cross_deadline(&nes->cart) && p->scanline < NES_HEIGHT &&
        !p->mask_tick && !p->early_draw_tick &&
        !nes->cpu.nmi_line && !nes->cpu.nmi_edge && !(p->status & 0x80) &&
        p->dot + tick - p->ticks + NES_PPU_DOTS_PER_CPU <
            (NES_HEIGHT - p->scanline) * NES_DOTS_PER_SCANLINE;
}

static NES_ALWAYS_INLINE bool defer_scroll_write(Nes *nes, uint16_t address, uint8_t value, uint64_t tick)
{
    Ppu *p = &nes->ppu;

    if (!scroll_write_moves_lines(p, value)) {
        ppu_write_register_at(nes, address, value, tick);
        return true;
    }
    unsigned dot = p->dot + (unsigned)(tick - p->ticks);
    unsigned line = dot / NES_DOTS_PER_SCANLINE;
    unsigned count = p->deferred_scroll_count;

    // Keep completed lines' scroll across hblank writes. Leave long
    // unchanged spans to the region renderer.
    // These mappers have no per-line IRQ or CHR latch work to replay.
    if (line && line <= PPU_SCROLL_BATCH_LINES && line - count <= 2 &&
        (p->mask & 0x18) && p->scanline + line < NES_HEIGHT &&
        p->next_dot == NES_DOTS_PER_SCANLINE &&
        dot - line * NES_DOTS_PER_SCANLINE >= NES_WIDTH + 2 &&
        nes->cart.mapper != 4 && nes->cart.mapper != 5 && nes->cart.mapper != 9) {
        unsigned scroll = ((p->t & 0x041f) << 3) | p->fine_x;

        while (count < line)
            p->deferred_scroll[count++] = scroll;
        p->deferred_scroll_count = count;
        ppu_write_register_at(nes, address, value, tick);
        return true;
    }
    return false;
}

static NES_NOINLINE bool defer_timed_scroll(Nes *nes, uint16_t address, uint8_t value, uint64_t tick)
{
    return scroll_window_open(nes, tick) && defer_scroll_write(nes, address, value, tick);
}

// An Ultrafast scroll write with a raster event due by its end
static NES_NOINLINE bool quiet_late_scroll_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    uint64_t tick = cycle * NES_PPU_DOTS_PER_CPU;

    if (!scroll_window_open(nes, tick))
        return false;
    if (defer_scroll_write(nes, address, value, tick))
        return true;
    // Apply elapsed horizontal reloads with the old scroll. The CPU
    // can stay in its batch as long as the write fits after this sync.
    if (cycle < nes->ppu_deadline)
        return false;
    sync_ppu_at(nes, cycle);
    if (cycle + 1 >= nes->ppu_deadline)
        return false;
    // With the raster current, a fine-X change can schedule its redraw here
    ppu_write_register_at(nes, address, value, tick);
    return true;
}

static NES_ALWAYS_INLINE bool quiet_scroll_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    uint64_t tick = cycle * NES_PPU_DOTS_PER_CPU;

    if (!nes->ultrafast) {
        // Preserve the raster events on either side of the write
        if (cycle >= nes->ppu_deadline) {
            if (nes->ppu.next_dot == NES_DOTS_PER_SCANLINE && defer_timed_scroll(nes, address, value, tick))
                return true;
            sync_ppu_at(nes, cycle);
        }
        ppu_write_register_at(nes, address, value, tick);
        if (cycle + 1 >= nes->ppu_deadline)
            sync_ppu_at(nes, cycle + 1);
        return true;
    }
    if (cycle + 1 >= nes->ppu_deadline)
        return quiet_late_scroll_write(nes, address, value, cycle);
    if (scroll_write_effects(nes, value, cycle) & WRITE_BUS)
        return false;
    ppu_write_register_at(nes, address, value, tick);
    return true;
}

// PPUADDR and PPUDATA
static NES_ALWAYS_INLINE bool quiet_address_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle,
                                                  unsigned effects)
{
    uint64_t tick = cycle * NES_PPU_DOTS_PER_CPU;

    if (effects & WRITE_RASTER) {
        // A timed access needs the raster at the write. The others can keep
        // the raster position of the last event once due events are done.
        if (cycle >= nes->ppu_deadline) {
            if (effects & WRITE_TIMED)
                return false;
            sync_ppu_at(nes, cycle);
        } else if (effects & WRITE_TIMED) {
            sync_ppu_at(nes, cycle);
        }
        ppu_write_register_at(nes, address, value, tick);
        return true;
    }
    const Ppu *p = &nes->ppu;
    unsigned next = p->v + ((p->ctrl & 4) ? 32 : 1);

    if ((address & 7) == 7 && (p->v & 0x3000) == 0x2000 && !(next & 0x1000)) {
        ppu_write_nametable_at(nes, value, (uint16_t)next, tick);
        return true;
    }
    ppu_write_register_at(nes, address, value, tick);
    return true;
}

static NES_NOINLINE bool quiet_ppu_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (address < 0x2000 || address >= 0x4000)
        return false;
    unsigned reg = address & 7;

    if (reg == 5)
        return quiet_scroll_write(nes, address, value, cycle);
    unsigned effects = ppu_write_effects(nes, address, value, cycle);

    if (effects & (WRITE_BUS | WRITE_DEADLINE))
        return false;
    if (reg >= 6)
        return quiet_address_write(nes, address, value, cycle, effects);
    if ((effects & WRITE_TIMED) && !nes->ultrafast && cycle + 1 >= nes->ppu_deadline)
        return false;
    if (reg != 0)
        return apply_timed_ppu_write(nes, address, value, cycle, effects);
    if ((effects & WRITE_RASTER) && cycle + 1 >= nes->ppu_deadline)
        return false;
    ppu_write_register_at(nes, address, value, cycle * NES_PPU_DOTS_PER_CPU);
    return true;
}

// PPUADDR and PPUDATA stores from compiled code, which knows the register.
// The PPU decodes only the low three address bits.
static NES_ALWAYS_INLINE bool quiet_known_address_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (cycle + 1 >= nes->bus_deadline)
        return false;
    unsigned effects = address_write_effects(nes, address, value, cycle);

    return !(effects & (WRITE_BUS | WRITE_DEADLINE)) && quiet_address_write(nes, address, value, cycle, effects);
}

bool nes_write_quiet_ppu_address(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    (void)address;
    return quiet_known_address_write(nes, 0x2006, value, cycle);
}

bool nes_write_quiet_ppu_data(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    (void)address;
    return quiet_known_address_write(nes, 0x2007, value, cycle);
}

// APU waveform register stores from compiled code
bool nes_write_quiet_waveform(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (cycle + 1 >= nes->bus_deadline)
        return false;
    apu_write_waveform(&nes->apu, address, value, cycle);
    return true;
}

// VRC2/VRC4/VRC6 and FME-7 writes that leave the raster, the CPU deadline and the
// IRQ line as they were. With fixed_prg, PRG bank changes also take the bus handler.
static NES_NOINLINE bool quiet_ext_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle, bool fixed_prg)
{
    Cartridge *cart = &nes->cart;
    unsigned effects = ext_write_effects(cart, address, value);

    if ((effects & WRITE_BUS) || ((effects & WRITE_PRG) && fixed_prg))
        return false;
    // The bus handler renders VRC6 audio before each register write. An
    // ultrasonic saw's sums depend on where rendering stops, so do the same.
    if (cart_is_vrc6(cart) || (effects & WRITE_AUDIO))
        nes_expansion_audio_sync(nes, cycle);
    if (effects & WRITE_PRG) {
        if (cycle > apu_stream_audio_cycle(&nes->apu))
            sync_dmc_before_remap(nes, address, value, cycle);
    } else if (effects & WRITE_RASTER) {
        // These counters follow the CPU clock, so the raster can catch up to
        // the write here. The next raster event must still come after it.
        if (cycle >= nes->ppu_deadline)
            sync_ppu_at(nes, cycle);
        if (cycle + 1 >= nes->ppu_deadline)
            return false;
    } else if (effects & WRITE_IRQ) {
        // Keep the counter state in case the new deadline comes first
        MapperExt *x = cart->ext;
        uint64_t base = x->irq_base, irq_cycle = x->irq_cycle;
        uint16_t counter = x->irq_counter;
        int16_t prescaler = x->irq_prescaler;
        uint8_t latch = x->irq_latch, control = x->irq_control;
        bool irq = cart->mmc3.irq, enabled = cart->mmc3.irq_enabled, line = cpu_irq_line(nes);

        cart_cpu_write(cart, address, value, cycle);
        // The CPU batch can continue if the IRQ line holds and the next bus
        // deadline would come no earlier than the current one.
        if (cpu_irq_line(nes) == line &&
            peripheral_deadline(nes, ppu_cpu_deadline(nes)) >= nes->bus_deadline)
            return true;
        x->irq_base = base;
        x->irq_cycle = irq_cycle;
        x->irq_counter = counter;
        x->irq_prescaler = prescaler;
        x->irq_latch = latch;
        x->irq_control = control;
        cart->mmc3.irq = irq;
        cart->mmc3.irq_enabled = enabled;
        return false;
    }
    cart_cpu_write(cart, address, value, cycle);
    return true;
}

bool nes_write_quiet_fixed_prg(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    return cycle + 1 < nes->bus_deadline && quiet_ext_write(nes, address, value, cycle, true);
}

// MMC3 bank registers, which games write most often
static NES_ALWAYS_INLINE bool quiet_mmc3_bank_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    Cartridge *cart = &nes->cart;
    unsigned effects = mmc3_write_effects(cart, address, value);

    if (!effects) {
        // A register select keeps both bank modes, or a bank stays as it was
        if (!(address & 1))
            cart->mmc3.select = value;
        return true;
    }
    // The raster must be current through a CHR change
    if ((effects & WRITE_RASTER) && cycle + 1 >= nes->ppu_deadline)
        return nes->ultrafast && (address & 1) && write_chr_before_first_pixels(nes, address, value, cycle);
    if ((effects & WRITE_PRG) && cycle > apu_stream_audio_cycle(&nes->apu))
        sync_dmc_before_remap(nes, address, value, cycle);
    if ((effects & WRITE_RASTER) && nes->ppu.scanline < NES_HEIGHT)
        write_mmc3_background(nes, address, value, cycle);
    else
        cart_cpu_write(cart, address, value, cycle);
    return true;
}

static NES_NOINLINE bool quiet_mapper_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    Cartridge *cart = &nes->cart;

    if (cart_is_fds(cart))
        return fds_write_quiet(nes, address, value);
    if ((address & 0xe000) == 0x8000 && cart->mapper == 4)
        return quiet_mmc3_bank_write(nes, address, value, cycle);
    if (cart_has_quiet_registers(cart))
        return quiet_ext_write(nes, address, value, cycle, false);
    // MMC5 ROM bank selects, the most frequent writes here, only remap PRG.
    // sync_dmc_before_remap compares the bank itself.
    if (mmc5_rom_select(address, value) && cart->mmc5) {
        if (cycle > apu_stream_audio_cycle(&nes->apu))
            sync_dmc_before_remap(nes, address, value, cycle);
        cart_cpu_write(cart, address, value, cycle);
        return true;
    }
    unsigned effects = mapper_write_effects(nes, address, value);

    if ((effects & (WRITE_BUS | WRITE_DEADLINE)) ||
        ((effects & WRITE_TIMED) && !nes->ultrafast && cycle + 1 >= nes->ppu_deadline) ||
        ((effects & WRITE_RASTER) && cycle + 1 >= nes->ppu_deadline))
        return false;
    if ((effects & WRITE_PRG) && cycle > apu_stream_audio_cycle(&nes->apu))
        sync_dmc_before_remap(nes, address, value, cycle);
    cart_cpu_write(cart, address, value, cycle);
    return true;
}

void nes_write_mmc1_serial(Nes *nes, uint16_t address, uint8_t bits, uint64_t cycle)
{
    // Settle the prior raster before the fifth write changes CHR or mirroring
    if (address < 0xe000 && cycle >= nes->ppu_deadline)
        sync_ppu_at(nes, cycle);
    if (cycle > apu_stream_audio_cycle(&nes->apu) &&
        (mmc1_dmc_remap_pages(&nes->cart, address, bits) & dmc_prg_pages(&nes->apu.dmc)))
        apu_advance_before_refill(&nes->apu, cycle);
    cart_commit_mmc1_serial(&nes->cart, address, bits, cycle);
}

static NES_NOINLINE bool quiet_apu_write(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    // Waveform registers leave CPU-visible IRQ and DMA deadlines in place
    if (apu_waveform_register(address)) {
        apu_write_waveform(&nes->apu, address, value, cycle);
        return true;
    }
    if (address == 0x4016 && nes->pad_strobe == nes->pad_strobe_write &&
        (nes->pad_strobe == ((value & 1) != 0) ||
         (!(cycle & 1) && (cycle + 1 < nes->ppu_deadline || can_defer_io_raster(nes))))) {
        nes->pad_strobe_write = (value & 1) != 0;
        latch_controllers(nes);
        return true;
    }
    return address != 0x4014 && address != 0x4016 &&
           apu_write_control_quiet(&nes->apu, address, value, cycle);
}

// Each device's quiet path saves only the registers that it needs
bool nes_write_quiet(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (cycle + 1 >= nes->bus_deadline)
        return false;
    if (address < 0x4000)
        return quiet_ppu_write(nes, address, value, cycle);
    if (address <= 0x4017)
        return quiet_apu_write(nes, address, value, cycle);
    return quiet_mapper_write(nes, address, value, cycle);
}

void cpu_write_slow(Nes *nes, uint16_t address, uint8_t value)
{
    uint8_t *ram = quiet_prg_ram(nes, address, true);

    if (ram)
        *ram = value;
    else if (!nes_write_quiet(nes, address, value, nes->cpu.cycles)) {
        cpu_sample_interrupts(nes, cpu_irq_line(nes));
        bus_write(nes, address, value);
        nes->cpu_bus = value;
        Ppu *p = &nes->ppu;
        uint64_t end = nes->cpu.cycles + 1;
        unsigned reg = address & 7;

        if (end < nes->bus_deadline &&
            (((address & 0xe000) == 0x2000 && (reg == 0 || reg == 1 || reg == 6) && !ppu_clocks_irq(&nes->cart)) ||
             (nes->ultrafast && (address & 0xe001) == 0x8001 && nes->cart.mapper == 4 &&
              (nes->cart.mmc3.select & 7) < 6)) &&
            p->scanline < NES_HEIGHT && end < nes->apu.cpu_deadline &&
            !nes->oam_dma && nes->pad_strobe == nes->pad_strobe_write && !nes->pad_resume_deadline &&
            nes->cpu.nmi_line == ((p->ctrl & p->status & 0x80) != 0) &&
            nes->irq_line == cpu_irq_line(nes)) {
            // The earlier write settled its own raster inputs. Only execute
            // the end-of-access clocks that would otherwise have been due.
            if (nes->ultrafast && reg == 1 && p->mask_tick && end >= nes->ppu_deadline)
                nes->cpu.cycles = end;
            else
                tick(nes);
            return;
        }
        finish_cpu_bus(nes, address, false);
        return;
    }
    cpu_sample_interrupts(nes, nes->irq_line);
    cpu_drive_bus(nes, value);
}

void nes_init(Nes *nes)
{
    ppu_init(&nes->ppu);
    nes->ppu.rgb565 = nes->ultrafast;
    nes->apu.ultrafast = nes->ultrafast;
    nes->ppu.blanking_lines = nes->ppu.extra_blanking_lines;
    nes->apu.enhanced_clock = nes->apu.blanking = nes->ppu.extra_blanking_lines != 0;
    apu_init(&nes->apu);
    if (nes->apu.enhanced_clock)
        plan_apu_clock(nes, 0);
    cpu_reset(nes);
}

void nes_reset(Nes *nes)
{
    Ppu *p = &nes->ppu;
    uint64_t cycle;

    // The reset line restarts the CPU and APU. The PPU keeps its raster
    // position, but its control, mask and write toggle are cleared.
    sync_ppu(nes);
    cycle = nes->cpu.cycles;
    ppu_write_register_at(nes, 0x2000, 0, p->ticks);
    ppu_write_register_at(nes, 0x2001, 0, p->ticks);
    p->second_write = false;
    p->read_buffer = 0;
    nes->ppu_deadline = cycle + 1;
    nes->oam_dma = false;
    apu_reset(&nes->apu, cycle);
    schedule_bus(nes);
    cpu_reset(nes);
    schedule_bus(nes);
}

static void set_frame_blanking(Nes *nes, unsigned lines)
{
    // Preserve the phase relative to vblank when moving the extra lines. If
    // the raster is within lines being removed, vblank begins next.
    Ppu *p = &nes->ppu;

    sync_ppu(nes);
    if (p->scanline >= ppu_vblank_line(p))
        p->scanline = p->scanline - p->blanking_lines + lines;
    else if (p->scanline >= NES_VBLANK_LINE + lines) {
        p->scanline = NES_VBLANK_LINE + lines;
        p->dot = 0;
        p->next_dot = 1;
    }
    p->status_deadline = 0;
    p->blanking_lines = (uint8_t)lines;
    nes->apu.blanking = lines != 0;
    plan_apu_clock(nes, nes->cpu.cycles);
    // Recompute the pre-render deadline, including an ultrafast vblank jump
    nes->ppu_deadline = nes->cpu.cycles;
    sync_ppu(nes);
    schedule_bus(nes);
}

void nes_set_enhanced(Nes *nes, bool enabled)
{
    unsigned extra = enabled ? NES_EXTRA_BLANKING_LINES : 0;

    if (nes->ppu.extra_blanking_lines == extra)
        return;
    sync_ppu(nes);
    nes->ppu.extra_blanking_lines = (uint8_t)extra;
    nes->apu.enhanced_clock = enabled;
    set_frame_blanking(nes, extra);
}

// Use the instrumented CPU loop for statistics and raster tracing
static NES_NOINLINE void run_machine_frame(Nes *nes, NesInstructionStats *stats)
{
    Apu *a = &nes->apu;
    const Ppu *p = &nes->ppu;
    const bool expansion_audio = nes->cart.ext && nes->cart.ext->has_audio;

    a->sample_count = 0;
    if (expansion_audio)
        a->wide_pcm = true;

    // Ultrafast draws RGB565. A mode switch redraws the whole next picture
    if (nes->ppu.rgb565 != nes->ultrafast)
        ppu_set_rgb565(nes, nes->ultrafast);
    // Ultrafast can batch DMC reads on eligible PRG mappings
    bool stream = nes->ultrafast && cart_streams_dmc(&nes->cart);

    if (!stream)
        a->stream_strobe = false;
    const Cartridge *cart = stream && !a->stream_strobe ? &nes->cart : NULL;

    // A mode switch can change which APU events the CPU waits for
    if (a->ultrafast != nes->ultrafast || cart != a->stream_cart) {
        apu_advance_before_refill(a, nes->cpu.cycles);
        a->ultrafast = nes->ultrafast;
        apu_set_stream_cart(a, cart);
        schedule_bus(nes);
    }
    if (stats || nes->cpu_raster)
        cpu_run_debug_frame(nes, stats);
    else
        cpu_run_frame(nes);

    uint64_t cycle = nes->cpu.cycles;
    bool synchronized = p->ticks == cycle * NES_PPU_DOTS_PER_CPU;
    bool raster_ready = nes->ultrafast ? synchronized || (quiet_vblank(p) && cycle < nes->ppu_deadline)
        : synchronized && cycle < nes->ppu_deadline && !p->mask_tick && !p->early_draw_tick &&
          nes->cpu.nmi_line == ((p->ctrl & p->status & 0x80) != 0);

    if (!raster_ready)
        sync_ppu(nes);
    apu_advance(a, nes->cpu.cycles);
    if (expansion_audio) {
        nes_expansion_audio_finish_frame(nes);
        a->wide_pcm = false;
    }
    if (!raster_ready || nes->bus_deadline <= nes->cpu.cycles ||
        !a->frame_inhibit || a->dmc.remaining || a->dmc.dma_abort ||
        nes->oam_dma || nes->pad_strobe != nes->pad_strobe_write || nes->irq_line != cpu_irq_line(nes))
        schedule_bus(nes);
}

void nes_frame(Nes *nes)
{
    run_machine_frame(nes, NULL);
}

void nes_frame_with_stats(Nes *nes, NesInstructionStats *stats)
{
    run_machine_frame(nes, stats);
}

void nes_frame_with_raster_stats(Nes *nes, NesRasterStats *stats)
{
    *stats = (NesRasterStats){0};
    nes->raster_stats = stats;
    nes->apu.sync_count = 0;
    run_machine_frame(nes, NULL);
    stats->apu_syncs = nes->apu.sync_count;
    nes->apu.sync_count = UINT32_MAX;
    nes->raster_stats = NULL;
}

void nes_frame_with_cpu_raster(Nes *nes, NesCpuRaster *raster)
{
    uint64_t tick = nes->cpu.cycles * NES_PPU_DOTS_PER_CPU;

    if (raster->end_tick != tick)
        raster->first_tick = raster->end_tick = tick;
    const Ppu *p = &nes->ppu;
    uint64_t previous_line_zero = p->ticks - (p->scanline * NES_DOTS_PER_SCANLINE + p->dot);

    nes->cpu_raster = raster;
    run_machine_frame(nes, NULL);
    nes->cpu_raster = NULL;
    // A frame ends in vblank, after line zero of the image just rendered.
    // Consecutive line-zero ticks also reveal the odd-frame pre-render skip.
    raster->line_zero_tick = p->ticks - (p->scanline * NES_DOTS_PER_SCANLINE + p->dot);
    // Frames with and without No-Lag lines are whole scanlines long
    raster->short_pre_render = (raster->line_zero_tick - previous_line_zero + 1) % NES_DOTS_PER_SCANLINE == 0;
}
