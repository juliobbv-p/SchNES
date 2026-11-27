#include "write_effects.h"
#include "apu_fixed.h"
#include <string.h>

#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

static const uint8_t lengths[32] = {
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};
static const uint16_t noise_periods[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
static const uint16_t dmc_periods[16] = {
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54
};
static const uint8_t duty_bits[4] = {0x02, 0x06, 0x1e, 0xf9};

// Return the first CPU cycle by which the APU has reached a future boundary
static inline uint64_t cpu_clock_cycle(const Apu *a, uint64_t apu)
{
    uint64_t cpu = apu + a->clock_offset;
    return cpu > a->pause_start ? cpu + a->pause_cycles : cpu;
}

static inline uint64_t frame_control_event(const Apu *a, uint64_t event)
{
    if (a->frame_reset < event)
        event = a->frame_reset;
    uint64_t clear = a->cycle | UINT64_C(1);

    if (a->frame_irq_clear && clear < event)
        event = clear;
    return event;
}

static void schedule_cpu_event(Apu *a)
{
    // Output samples and waveform edges are private to the APU. The CPU only
    // needs to wake for IRQ changes and the DMC's next buffer refill request.
    uint64_t event = UINT64_MAX;
    uint64_t cpu_event = UINT64_MAX;

    if (!a->frame_mode && !a->frame_inhibit) {
        event = a->frame_start + 29828;

        if (event < a->cycle)
            event += 29830;
    }
    // With frame IRQs inhibited, the APU can apply frame resets when it next advances
    if (!a->frame_inhibit)
        event = frame_control_event(a, event);
    Dmc *d = &a->dmc;

    a->stream_read_cycle = UINT64_MAX;

    if (d->remaining && apu_streams_dmc(a)) {
        // I/O reads still have to see a refill request at its original cycle
        uint64_t empty = a->cycle + d->timer + (d->bits - 1u) * d->period;

        a->stream_read_cycle = d->empty ? dmc_fetch_cycle(d) : (cpu_clock_cycle(a, empty + 1) | 1u);

        // Wake the CPU when the buffer empties before the IRQ-producing fetch.
        // An empty buffer can start late, so project the tail after its first fill.
        if (d->irq_enabled && d->empty) {
            cpu_event = a->stream_read_cycle + 1;
        } else if (d->irq_enabled) {
            uint64_t boundary = empty + (d->remaining - 1u) * 8u * d->period;

            if (boundary < event)
                event = boundary;
        }
    } else if (d->remaining && !d->empty) {
        uint64_t refill = a->cycle + d->timer + (d->bits - 1u) * d->period;

        // The buffered byte can publish the halt in advance. Render the
        // output edge when the CPU services the DMA or next accesses the APU.
        if (apu_schedules_dmc_halt(a))
            d->dma_halt = (refill + 1) | 1u;
        else if (apu_projects_single_dmc_halt(a))
            d->dma_halt = cpu_clock_cycle(a, refill + 1) | 1u;
        else if (refill < event)
            event = refill;
    }
    a->dmc.stream_read_offset = 0;
    // The bus sees these events on the cycle after they are clocked
    if (event != UINT64_MAX) {
        uint64_t deadline = cpu_clock_cycle(a, event + 1);

        if (deadline < cpu_event)
            cpu_event = deadline;
    }
    a->cpu_deadline = cpu_event;
}

void apu_init(Apu *a)
{
    a->sync_count = UINT32_MAX;
    a->levels_dirty = true;
    a->frame_event = 7456;
    a->frame_reset = UINT64_MAX;
    a->noise.shift = 1;
    a->noise.period = noise_periods[0];
    a->dmc.period = dmc_periods[0];
    a->dmc.start = 0xc000;
    a->dmc.length = 1;
    a->dmc.bits = 8;
    a->dmc.empty = a->dmc.silence = true;
    a->triangle.step = 15;
    schedule_cpu_event(a);
}

static void envelope_write(Envelope *e, uint8_t value)
{
    e->period = value & 15;
    e->constant = (value & 16) != 0;
    e->loop = (value & 32) != 0;
}

static void envelope_tick(Envelope *e)
{
    if (e->start) {
        e->start = false;
        e->decay = 15;
        e->divider = e->period;
    } else if (e->divider) {
        --e->divider;
    } else {
        e->divider = e->period;

        if (e->decay)
            --e->decay;
        else if (e->loop)
            e->decay = 15;
    }
}

static unsigned volume(const Envelope *e)
{
    return e->constant ? e->period : e->decay;
}

static int sweep_target(const Pulse *p, unsigned channel)
{
    int change = p->period >> p->sweep_shift;
    return p->period + (p->sweep_negate ? -change - (channel == 0) : change);
}

static bool pulse_muted(const Pulse *p)
{
    unsigned period = p->period;
    return period < 8 || (!p->sweep_negate && period + (period >> p->sweep_shift) > 0x7ff);
}

static void sweep_tick(Pulse *p, unsigned channel)
{
    if (!p->sweep_timer && p->sweep_enabled && p->sweep_shift && !pulse_muted(p))
        p->period = (uint16_t)sweep_target(p, channel);
    if (!p->sweep_timer || p->sweep_reload) {
        p->sweep_timer = p->sweep_period;
        p->sweep_reload = false;
    } else {
        --p->sweep_timer;
    }
}

static void quarter_frame(Apu *a)
{
    a->levels_dirty = true;
    envelope_tick(&a->pulse[0].envelope);
    envelope_tick(&a->pulse[1].envelope);
    envelope_tick(&a->noise.envelope);
    Triangle *t = &a->triangle;

    if (t->reload)
        t->linear = t->reload_value;
    else if (t->linear)
        --t->linear;
    if (!t->control)
        t->reload = false;
}

static void half_frame(Apu *a)
{
    // Skip counters that a write on this cycle already clocked
    unsigned clocked = a->length_clocked;

    a->length_clocked = 0;

    for (unsigned i = 0; i < 2; ++i) {
        Pulse *p = &a->pulse[i];

        if (p->length && !p->envelope.loop && !(clocked & (1u << i)))
            --p->length;
        sweep_tick(p, i);
    }
    if (a->noise.length && !a->noise.envelope.loop && !(clocked & 8))
        --a->noise.length;
    if (a->triangle.length && !a->triangle.control && !(clocked & 4))
        --a->triangle.length;
}

static NES_ALWAYS_INLINE void sync_tones(Apu *a, uint64_t cycle, bool bulk_noise);

static unsigned sequencer_wait(const Apu *a)
{
    unsigned frame = a->cycle - a->frame_start;
    unsigned wait;

    if (frame < 7457)
        wait = 7456 - frame;
    else if (frame < 14913)
        wait = 14912 - frame;
    else if (frame < 22371)
        wait = 22370 - frame;
    else if (a->frame_mode && frame < 37281)
        wait = 37280 - frame;
    else if (!a->frame_mode && frame < 29828)
        wait = 29827 - frame;
    else
        wait = 0;
    return wait;
}

static void schedule_frame_event(Apu *a)
{
    a->frame_event = frame_control_event(a, a->cycle + sequencer_wait(a));
}

// Apply the $4017 write one cycle before the new sequence starts
static void restart_frame_sequencer(Apu *a, uint64_t cycle)
{
    a->frame_reset = UINT64_MAX;
    a->frame_start = cycle + 1;
    a->frame_mode = a->frame_next_mode;

    if (a->frame_mode) {
        sync_tones(a, cycle, false);
        quarter_frame(a);
        half_frame(a);
    }
}

static void frame_tick(Apu *a, uint64_t cycle)
{
    unsigned frame = cycle - a->frame_start + 1;
    unsigned last_clock = a->frame_mode ? 37281 : 29829;
    bool half_clock = frame == 14913 || frame == last_clock;

    if (half_clock || frame == 7457 || frame == 22371) {
        sync_tones(a, cycle, false);
        quarter_frame(a);
    }
    if (half_clock)
        half_frame(a);
    if (!a->frame_mode && frame >= last_clock - 1) {
        // Status can briefly expose the flag even when the IRQ pin is inhibited
        a->frame_irq = frame <= last_clock || !a->frame_inhibit;

        if (frame == last_clock)
            a->frame_irq_line = !a->frame_inhibit;
    }
    if (frame == last_clock + 1)
        a->frame_start = cycle + 1;
    if (cycle == a->frame_reset)
        restart_frame_sequencer(a, cycle);
    schedule_frame_event(a);
}

static void dmc_restart(Dmc *d)
{
    d->address = d->start;
    d->remaining = d->length;
    d->multi_byte = d->length > 1;
}

static inline void dmc_supply(Dmc *d, uint8_t value, uint64_t cycle)
{
    d->buffer = value;
    d->empty = false;
    d->address = d->address == 0xffff ? 0x8000 : d->address + 1;

    if (d->stop_after_fetch) {
        d->stop_after_fetch = false;
        d->remaining = 0;
    } else if (--d->remaining == 0) {
        if (d->loop)
            dmc_restart(d);
        else {
            d->irq = d->irq_enabled;

            // Playback can end one APU cycle before the output unit asks for
            // a refill. The request then halts the CPU without fetching a byte.
            if (d->bits == 1 && d->timer == 1)
                d->dma_abort = cycle + 3;
        }
    }
}

void apu_dmc_supply(Apu *a, uint8_t value, uint64_t cycle)
{
    dmc_supply(&a->dmc, value, cycle);
    schedule_cpu_event(a);
}

bool apu_queue_dmc_byte(Apu *a, uint8_t value, uint64_t cycle)
{
    Dmc *d = &a->dmc;

    // Leave the fetch before an IRQ-producing final byte on the exact CPU path
    if (!apu_schedules_dmc_halt(a) || d->empty || d->remaining <= 1 ||
        (d->irq_enabled && !d->loop && d->remaining <= 2) ||
        d->stop_after_fetch || d->queued_count == sizeof d->queued_bytes ||
        cycle - d->dma_halt + 2 >= d->period)
        return false;

    // Keep the silent-span reset and the eight-sample held-input mixer at
    // their original boundaries. The active channel lengths hold until the
    // next sequencer event, and each DMC bit spans at most seven PCM samples.
    if (d->period > 7 * NES_CPU_HZ / NES_SAMPLE_RATE || cycle >= a->frame_event ||
        !(a->pulse[0].length || a->pulse[1].length || a->noise.length ||
          (a->triangle.length && a->triangle.linear)))
        return false;

    // The CPU just drove the sample read. This byte cannot affect the
    // output unit until the next byte boundary, so record the value for
    // that boundary and publish the next halt without mixing here.
    d->queued_bytes |= (uint32_t)value << (8 * d->queued_count++);
    d->address = d->address == 0xffff ? 0x8000 : d->address + 1;
    --d->remaining;
    d->dma_halt += 8u * d->period;
    return true;
}

static void dmc_enable(Apu *a, bool enabled, uint64_t cycle)
{
    Dmc *d = &a->dmc;

    if (enabled) {
        d->stop_after_fetch = false;

        if (d->remaining)
            return;
        dmc_restart(d);
        d->dma_enable = (cycle & ~UINT64_C(1)) + 4;

        if (d->empty)
            d->dma_halt = d->dma_enable;
        return;
    }
    if (!d->remaining)
        return;

    // An already-issued load or refill completes even if $4015 clears D4
    bool last_bit = !d->empty && d->bits == 1;

    d->stop_after_fetch = (d->empty && d->dma_halt == cycle) ||
        (last_bit && !d->timer);

    if (d->stop_after_fetch)
        return;
    if (last_bit && d->timer <= 2)
        d->dma_abort = cpu_clock_cycle(a, apu_clock_cycle(a, cycle) + d->timer + 1);
    d->remaining = 0;
}

static inline unsigned step_dmc_level(unsigned level, unsigned shift)
{
    // Keep the old level if this two-unit step leaves the 7-bit range
    unsigned next = level + 4u * (shift & 1) - 2u;

    return next < 128 ? next : level;
}

static inline bool dmc_output_bit(Apu *a, uint64_t cycle)
{
    Dmc *d = &a->dmc;

    if (!d->silence)
        d->level = step_dmc_level(d->level, d->shift);
    d->shift >>= 1;

    if (--d->bits)
        return false;
    d->bits = 8;
    d->silence = d->empty;

    if (!d->empty) {
        d->shift = d->buffer;

        if (d->queued_count) {
            d->buffer = (uint8_t)d->queued_bytes;
            d->queued_bytes >>= 8;
            --d->queued_count;
        } else {
            d->empty = true;
            // A refill attempts to halt the CPU on a put cycle
            d->dma_halt = cpu_clock_cycle(a, cycle + 1) | 1;
        }
    }
    return true;
}

static void dmc_tick(Apu *a, uint64_t cycle)
{
    Dmc *d = &a->dmc;

    if (d->timer) {
        --d->timer;
        return;
    }
    d->timer = d->period - 1;
    dmc_output_bit(a, cycle);
}

static inline void supply_stream(Apu *a, uint64_t cycle)
{
    // advance_apu publishes the final deadline after processing the span
    dmc_supply(&a->dmc, cart_prg_read(a->stream_cart, a->dmc.address), cycle);
}

static unsigned timer_steps(uint16_t *timer, unsigned clocks, unsigned period)
{
    if (clocks <= *timer) {
        *timer -= clocks;
        return 0;
    }
    clocks -= *timer + 1;

    if (clocks <= period) {
        *timer = period - clocks;
        return 1;
    }
    *timer = period - clocks % (period + 1);
    return 1 + clocks / (period + 1);
}

static NES_ALWAYS_INLINE unsigned shift_noise(unsigned shift, unsigned noise_steps, unsigned tap)
{
    if (noise_steps == 1) {
        unsigned feedback = (shift ^ (shift >> tap)) & 1;

        shift = (shift >> 1) | (feedback << 14);
    } else {
        // The new bits can reach either feedback tap only once in 15 clocks
        unsigned feedback_delay = 15 - tap;

        while (noise_steps > 15) {
            unsigned feedback = shift ^ (shift >> tap);

            shift = (feedback ^ (feedback << feedback_delay)) & 0x7fff;
            noise_steps -= 15;
        }
        unsigned feedback = shift ^ (shift >> tap);

        feedback ^= feedback << feedback_delay;
        shift = (shift >> noise_steps) | ((feedback << (15 - noise_steps)) & 0x7fff);
    }
    return shift;
}

static NES_NOINLINE unsigned jump_noise(unsigned shift, unsigned steps, unsigned tap)
{
    // Advancing this 15-bit LFSR by 15 ticks equals XORing the original
    // state with the state advanced by the feedback tap. Squaring that
    // linear transform doubles both distances.
    for (unsigned groups = steps / 15, offset = tap; groups; groups >>= 1, offset <<= 1) {
        if (groups & 1) {
            unsigned remaining = offset, advanced = shift;

            while (tap == 6 && remaining >= 15) {
                // In short mode a new bit reaches the tap after nine clocks
                unsigned feedback = advanced ^ (advanced >> tap);

                advanced = (feedback ^ (feedback << (15 - tap))) & 0x7fff;
                remaining -= 15;
            }
            shift ^= shift_noise(advanced, remaining, tap);
        }
    }
    return shift_noise(shift, steps % 15, tap);
}

static NES_ALWAYS_INLINE void clock_noise(Noise *n, unsigned clocks, bool bulk_noise)
{
    unsigned steps = timer_steps(&n->timer, clocks, n->period - 1);

    if (steps) {
        unsigned tap = n->mode ? 6 : 1;
        n->shift = bulk_noise && steps >= 60 ? jump_noise(n->shift, steps, tap) : shift_noise(n->shift, steps, tap);
    }
}

static NES_ALWAYS_INLINE void clock_pulses(Apu *a, uint64_t start, unsigned clocks)
{
    unsigned half_clocks = (clocks + ((start & 1) == 0)) / 2;

    for (unsigned i = 0; i < 2; ++i) {
        Pulse *p = &a->pulse[i];
        p->step = (p->step + timer_steps(&p->timer, half_clocks, p->period)) & 7;
    }
}

static inline void clock_triangle(Triangle *t, unsigned clocks)
{
    unsigned steps = timer_steps(&t->timer, clocks, t->period);

    if (t->length && t->linear)
        t->step = (t->step + steps) & 31;
}

static NES_ALWAYS_INLINE void sync_tones(Apu *a, uint64_t cycle, bool bulk_noise)
{
    unsigned clocks = cycle - a->tone_cycle;

    clock_pulses(a, a->tone_cycle, clocks);
    a->tone_cycle = cycle;
    clock_triangle(&a->triangle, clocks);
    clock_noise(&a->noise, clocks, bulk_noise);
}

static unsigned pulse_volume(const Pulse *p)
{
    return p->length && !pulse_muted(p) ? volume(&p->envelope) : 0;
}

enum {
    TRIANGLE_AVERAGE = 1, PULSE_FILTER = 2, TRIANGLE_FILTER = 4, SOLO_FILTER_SHIFT = 2,
    TONE_FILTERS = PULSE_FILTER | TRIANGLE_FILTER,
    PULSE_SOLO_FILTER = PULSE_FILTER << SOLO_FILTER_SHIFT,
    TRIANGLE_SOLO_FILTER = TRIANGLE_FILTER << SOLO_FILTER_SHIFT,
    SOLO_TONE_FILTERS = PULSE_SOLO_FILTER | TRIANGLE_SOLO_FILTER,
    PULSE_MIXED_CUTOFF = 96, PULSE_SOLO_CUTOFF = 128,
    TRIANGLE_MIXED_CUTOFF = 5, TRIANGLE_SOLO_CUTOFF = 128
};

static inline MixerLevels mixer_levels(Apu *a)
{
    if (!a->levels_dirty)
        return a->levels;
    a->levels_dirty = false;
    unsigned pulse0 = pulse_volume(&a->pulse[0]), pulse1 = pulse_volume(&a->pulse[1]);
    unsigned noise = a->noise.length ? volume(&a->noise.envelope) : 0;
    bool triangle = a->triangle.length && a->triangle.linear;
    unsigned pulse_mode = 0;
    unsigned triangle_mode = 0;

    if ((pulse0 && a->pulse[0].period <= PULSE_MIXED_CUTOFF) ||
        (pulse1 && a->pulse[1].period <= PULSE_MIXED_CUTOFF)) {
        pulse_mode = PULSE_FILTER;
    } else if (!triangle && !noise && ((pulse0 != 0) != (pulse1 != 0))) {
        unsigned period = pulse0 ? a->pulse[0].period : a->pulse[1].period;

        if (period <= PULSE_SOLO_CUTOFF)
            pulse_mode = PULSE_SOLO_FILTER;
    }
    if (triangle) {
        unsigned period = a->triangle.period;

        if (period < 2)
            triangle_mode = TRIANGLE_AVERAGE;
        else if (period <= TRIANGLE_MIXED_CUTOFF)
            triangle_mode = TRIANGLE_FILTER;
        else if (period <= TRIANGLE_SOLO_CUTOFF && !(pulse0 || pulse1 || noise))
            triangle_mode = TRIANGLE_SOLO_FILTER;
    }

    a->levels = (MixerLevels){
        pulse0, pulse1, noise,
        duty_bits[a->pulse[0].duty],
        duty_bits[a->pulse[1].duty],
        triangle_mode | pulse_mode
    };
    return a->levels;
}

static NES_ALWAYS_INLINE void *sample_output(Apu *a, unsigned offset, bool wide)
{
    return wide ? (void *)(a->wide_samples + offset) : a->samples + offset;
}

static NES_ALWAYS_INLINE void store_sample(void *output, unsigned i, int32_t sample, bool wide)
{
    if (wide)
        ((int32_t *)output)[i] = sample;
    else
        ((int16_t *)output)[i] = mix_pcm16(sample);
}

#if defined(__aarch64__) && defined(__ARM_NEON)
static NES_ALWAYS_INLINE void store_samples_neon(void *output, unsigned i, int32x4_t samples, bool wide)
{
    if (wide)
        vst1q_s32((int32_t *)output + i, samples);
    else
        vst1_s16((int16_t *)output + i, vqrshrn_n_s32(samples, FIXED_PCM_SHIFT));
}

// The bounded filter states keep these rounded Q31 sums within int32_t
static NES_ALWAYS_INLINE int32x4_t filter_mul_add_q31(int32x4_t sum, int32x4_t x, int32x4_t y)
{
#if defined(__ARM_FEATURE_QRDMX)
    return vqrdmlahq_s32(sum, x, y);
#else
    return vaddq_s32(sum, vqrdmulhq_s32(x, y));
#endif
}

// Q31 gains from advancing the two filter stages by one through four samples.
// The low-pass pole is c = 723/2048 and the DC pole is b = 127/128.
// Held input gain at step k (starting at 0) is (1325/16384) * sum(c^i * b^(k-i)).
static const int32x4_t filter_from_held = {173670400, 233624000, 253443060, 259104048};
static const int32x4_t filter_from_low = {758120448, 267637248, 94483267, 33355177};
static const int32_t filter_dc_step = 2081157128; // b^4

_Static_assert(FIXED_HP_SHIFT == 14 && FIXED_HP_STEP == 128 && FIXED_LP_SHIFT == 11 && FIXED_LP_STEP == 1325,
               "Update the NEON filter coefficients when changing the filter");

static NES_NOINLINE void mix_held_neon(FixedAudioFilter *filter, unsigned input, void *output, unsigned count,
                                        bool wide)
{
    // Center the states within the scalar filter's quantization intervals
    const int32_t bias_dc = (int32_t)(input * FIXED_HP_SCALE) + FIXED_HP_SCALE / 2;
    const int32_t bias_low = 1 << (FIXED_LP_SHIFT - 1);
    const int32x4_t high_step = vdupq_n_s32(filter_dc_step);
    int32x4_t high = vdupq_n_s32(bias_dc - filter->dc);
    int32x4_t low = vdupq_n_s32(filter->low - bias_low);
    int32x4_t bias = vdupq_n_s32(bias_low);

    for (unsigned i = 0; i < count; i += 4) {
        int32x4_t held = vqrdmulhq_s32(filter_from_held, high);

        low = filter_mul_add_q31(held, filter_from_low, vdupq_laneq_s32(low, 3));
        store_samples_neon(output, i, vaddq_s32(low, bias), wide);
        high = vqrdmulhq_s32(high, high_step);
    }
    filter->dc = bias_dc - vgetq_lane_s32(high, 0);
    filter->low = vgetq_lane_s32(low, 3) + bias_low;
}
#endif

static inline unsigned triangle_mix_index(unsigned step, bool averaged)
{
    unsigned level = step < 16 ? 15 - step : step - 16;
    return averaged ? FIXED_TRIANGLE_AVERAGE : FIXED_TRIANGLE_WEIGHT * level;
}

// The next noise output bits are already present in the low LFSR bits
static const uint8_t noise_bit_counts[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

// Average a 32-CPU-cycle window of fast noise before the 48 kHz point sample
static inline bool filter_fast_noise(const Apu *a)
{
    return !a->ultrafast && a->noise.length && a->noise.period <= 16;
}

static inline unsigned filtered_noise_weight(unsigned lfsr, unsigned weight, unsigned period)
{
    unsigned shift = period == 4 ? 3 : period == 8 ? 2 : 1;
    unsigned width = 1u << shift;
    unsigned count = noise_bit_counts[~lfsr & ((1u << width) - 1)];

    return (weight * count + width / 2) >> shift;
}

// Number of timer steps until each duty pattern changes its output bit
static const uint8_t pulse_edge_steps[4][8] = {
    {1, 1, 7, 6, 5, 4, 3, 2},
    {1, 2, 1, 6, 5, 4, 3, 2},
    {1, 4, 3, 2, 1, 4, 3, 2},
    {1, 2, 1, 6, 5, 4, 3, 2}
};

static const uint8_t pulse_high_steps[4] = {1, 2, 4, 6};

static inline unsigned regular_tone_filters(const Apu *a, MixerLevels levels)
{
    unsigned flags = levels.tone_flags;

    // Check DMC playback at mix time because it can change between level updates
    if ((flags & SOLO_TONE_FILTERS) && a->dmc.silence)
        flags |= flags >> SOLO_FILTER_SHIFT;
    return flags & TONE_FILTERS;
}

static inline unsigned pulse_deadline(const Pulse *p, uint64_t cycle, unsigned volume)
{
    if (!volume)
        return TONE_WINDOW;
    return 2u * p->timer + 1u + (cycle & 1) +
           (pulse_edge_steps[p->duty][p->step] - 1u) * 2u * (p->period + 1u);
}

static inline unsigned blend_pulse_edge(unsigned before, unsigned after, unsigned clocks)
{
    unsigned total = clocks * fixed_pulse_mix[before] + (TONE_WINDOW - clocks) * fixed_pulse_mix[after];
    return (unsigned)tone_window_average((int32_t)total);
}

static NES_NOINLINE unsigned averaged_pulse_dac(const Pulse *p0, const Pulse *p1, unsigned volumes,
                                               unsigned lo, unsigned hi, unsigned next0, unsigned next1)
{
    // The caller found an edge in the window. A full-width step rules out a second edge from that pulse
    if (next1 >= TONE_WINDOW && 2u * (p0->period + 1u) >= TONE_WINDOW)
        return blend_pulse_edge(lo + hi, (lo ^ (volumes & 15)) + hi, next0);
    if (next0 >= TONE_WINDOW && 2u * (p1->period + 1u) >= TONE_WINDOW)
        return blend_pulse_edge(lo + hi, lo + (hi ^ (volumes >> 4)), next1);

    // Integrate the combined nonlinear DAC across just the edges in the window
    unsigned first = 0, sum = 0;
    unsigned width0 = 2u * (p0->period + 1u), width1 = 2u * (p1->period + 1u);
    unsigned steps0 = pulse_high_steps[p0->duty], steps1 = pulse_high_steps[p1->duty];

    for (;;) {
        unsigned edge = next0 < next1 ? next0 : next1;

        if (edge >= TONE_WINDOW)
            break;
        sum += (edge - first) * fixed_pulse_mix[lo + hi];
        first = edge;

        if (edge == next0) {
            lo ^= volumes & 15;
            next0 += (lo ? steps0 : 8u - steps0) * width0;
        }
        if (edge == next1) {
            hi ^= volumes >> 4;
            next1 += (hi ? steps1 : 8u - steps1) * width1;
        }
    }
    sum += (TONE_WINDOW - first) * fixed_pulse_mix[lo + hi];
    return (unsigned)tone_window_average((int32_t)sum);
}

static NES_NOINLINE unsigned sampled_pulse_dac(const Apu *a, MixerLevels levels, unsigned lo, unsigned hi)
{
    unsigned next0 = pulse_deadline(&a->pulse[0], a->tone_cycle, levels.pulse0);
    unsigned next1 = pulse_deadline(&a->pulse[1], a->tone_cycle, levels.pulse1);

    if (next0 >= TONE_WINDOW && next1 >= TONE_WINDOW)
        return fixed_pulse_mix[lo + hi];
    return averaged_pulse_dac(&a->pulse[0], &a->pulse[1], levels.pulse0 | (levels.pulse1 << 4),
                              lo, hi, next0, next1);
}

// Cache the 32-cycle average for the shortest audible triangle periods
static const uint16_t triangle_window[4][6][32] = {
    {
        {314, 281, 248, 215, 182, 150, 122, 100, 85, 75, 72, 75, 85, 100, 122, 150,
         182, 215, 248, 281, 314, 345, 373, 395, 410, 420, 423, 420, 410, 395, 373, 345},
        {325, 292, 259, 226, 193, 160, 130, 106, 89, 77, 72, 73, 80, 94, 113, 139,
         170, 203, 236, 269, 302, 335, 365, 389, 406, 418, 423, 422, 415, 401, 382, 356},
        {335, 302, 269, 236, 203, 170, 139, 113, 94, 80, 73, 72, 77, 89, 106, 130,
         160, 193, 226, 259, 292, 325, 356, 382, 401, 415, 422, 423, 418, 406, 389, 365},
    },
    {
        {355, 322, 289, 256, 223, 190, 157, 124, 94, 71, 57, 51, 53, 63, 81, 108,
         140, 173, 206, 239, 272, 305, 338, 371, 401, 424, 438, 444, 442, 432, 414, 387},
        {363, 330, 297, 264, 231, 198, 165, 132, 101, 76, 60, 52, 52, 60, 76, 101,
         132, 165, 198, 231, 264, 297, 330, 363, 394, 419, 435, 443, 443, 435, 419, 394},
        {371, 338, 305, 272, 239, 206, 173, 140, 108, 81, 63, 53, 51, 57, 71, 94,
         124, 157, 190, 223, 256, 289, 322, 355, 387, 414, 432, 442, 444, 438, 424, 401},
        {380, 347, 314, 281, 248, 215, 182, 149, 116, 87, 66, 54, 50, 54, 66, 87,
         116, 149, 182, 215, 248, 281, 314, 347, 380, 408, 429, 441, 446, 441, 429, 408},
    },
    {
        {380, 347, 314, 281, 248, 215, 182, 149, 116, 84, 58, 42, 37, 42, 58, 84,
         116, 149, 182, 215, 248, 281, 314, 347, 380, 411, 437, 453, 458, 453, 437, 411},
        {387, 354, 321, 288, 255, 222, 189, 156, 123, 90, 62, 44, 37, 40, 54, 77,
         108, 141, 174, 207, 240, 273, 306, 339, 372, 405, 433, 451, 458, 455, 441, 418},
        {393, 360, 327, 294, 261, 228, 195, 162, 129, 96, 67, 47, 38, 39, 51, 72,
         102, 135, 168, 201, 234, 267, 300, 333, 366, 399, 428, 448, 457, 456, 444, 423},
        {399, 366, 333, 300, 267, 234, 201, 168, 135, 102, 72, 51, 39, 38, 47, 67,
         96, 129, 162, 195, 228, 261, 294, 327, 360, 393, 423, 444, 456, 457, 448, 428},
        {405, 372, 339, 306, 273, 240, 207, 174, 141, 108, 77, 54, 40, 37, 44, 62,
         90, 123, 156, 189, 222, 255, 288, 321, 354, 387, 418, 441, 455, 458, 451, 433},
    },
    {
        {396, 363, 330, 297, 264, 231, 198, 165, 132, 99, 67, 42, 30, 30, 42, 67,
         99, 132, 165, 198, 231, 264, 297, 330, 363, 396, 428, 453, 465, 465, 453, 428},
        {402, 369, 336, 303, 270, 237, 204, 171, 138, 105, 72, 45, 31, 29, 39, 62,
         93, 126, 159, 192, 225, 258, 291, 324, 357, 390, 423, 450, 464, 466, 456, 433},
        {407, 374, 341, 308, 275, 242, 209, 176, 143, 110, 77, 50, 33, 29, 37, 58,
         88, 121, 154, 187, 220, 253, 286, 319, 352, 385, 418, 446, 462, 466, 458, 437},
        {413, 380, 347, 314, 281, 248, 215, 182, 149, 116, 83, 54, 35, 29, 35, 54,
         83, 116, 149, 182, 215, 248, 281, 314, 347, 380, 413, 441, 460, 466, 460, 441},
        {418, 385, 352, 319, 286, 253, 220, 187, 154, 121, 88, 58, 37, 29, 33, 50,
         77, 110, 143, 176, 209, 242, 275, 308, 341, 374, 407, 437, 458, 466, 462, 446},
        {423, 390, 357, 324, 291, 258, 225, 192, 159, 126, 93, 62, 39, 29, 31, 45,
         72, 105, 138, 171, 204, 237, 270, 303, 336, 369, 402, 433, 456, 466, 464, 450},
    },
};

// Exact window average for a single triangle edge, indexed by phase and clocks to the edge
_Static_assert(TONE_WINDOW == 32, "Triangle edge rows cover 32 cycles");
#define TRI_EDGE_SAMPLE(level, direction, clocks) \
    (((level * TONE_WINDOW + (direction) * (TONE_WINDOW - (clocks))) * FIXED_TRIANGLE_WEIGHT + \
      TONE_WINDOW / 2) >> TONE_WINDOW_SHIFT)
#define TRI_EDGE_BLOCK(level, direction, first) \
    TRI_EDGE_SAMPLE(level, direction, first), TRI_EDGE_SAMPLE(level, direction, (first) + 1), \
    TRI_EDGE_SAMPLE(level, direction, (first) + 2), TRI_EDGE_SAMPLE(level, direction, (first) + 3), \
    TRI_EDGE_SAMPLE(level, direction, (first) + 4), TRI_EDGE_SAMPLE(level, direction, (first) + 5), \
    TRI_EDGE_SAMPLE(level, direction, (first) + 6), TRI_EDGE_SAMPLE(level, direction, (first) + 7)
#define TRI_EDGE_ROW(level, direction) \
    {TRI_EDGE_BLOCK(level, direction, 1), TRI_EDGE_BLOCK(level, direction, 9), \
     TRI_EDGE_BLOCK(level, direction, 17), TRI_EDGE_BLOCK(level, direction, 25)}
static const uint16_t triangle_edge[32][TONE_WINDOW] = {
    TRI_EDGE_ROW(15, -1), TRI_EDGE_ROW(14, -1), TRI_EDGE_ROW(13, -1), TRI_EDGE_ROW(12, -1),
    TRI_EDGE_ROW(11, -1), TRI_EDGE_ROW(10, -1), TRI_EDGE_ROW(9, -1), TRI_EDGE_ROW(8, -1),
    TRI_EDGE_ROW(7, -1), TRI_EDGE_ROW(6, -1), TRI_EDGE_ROW(5, -1), TRI_EDGE_ROW(4, -1),
    TRI_EDGE_ROW(3, -1), TRI_EDGE_ROW(2, -1), TRI_EDGE_ROW(1, -1), TRI_EDGE_ROW(0, 0),
    TRI_EDGE_ROW(0, 1), TRI_EDGE_ROW(1, 1), TRI_EDGE_ROW(2, 1), TRI_EDGE_ROW(3, 1),
    TRI_EDGE_ROW(4, 1), TRI_EDGE_ROW(5, 1), TRI_EDGE_ROW(6, 1), TRI_EDGE_ROW(7, 1),
    TRI_EDGE_ROW(8, 1), TRI_EDGE_ROW(9, 1), TRI_EDGE_ROW(10, 1), TRI_EDGE_ROW(11, 1),
    TRI_EDGE_ROW(12, 1), TRI_EDGE_ROW(13, 1), TRI_EDGE_ROW(14, 1), TRI_EDGE_ROW(15, 0),
};
#undef TRI_EDGE_ROW
#undef TRI_EDGE_BLOCK
#undef TRI_EDGE_SAMPLE

static inline unsigned blend_triangle_step(unsigned step, unsigned until)
{
    return triangle_edge[step][(until < TONE_WINDOW ? until : TONE_WINDOW) - 1];
}

static inline unsigned averaged_triangle_index(unsigned period, unsigned step, unsigned until)
{
    if (period <= 6)
        return triangle_window[period - 3][until - 1][step];
    if (period >= TONE_WINDOW)
        return blend_triangle_step(step, until);

    unsigned sum = until * triangle_mix_index(step, false);
    unsigned remaining = TONE_WINDOW - until;

    while (remaining > period) {
        step = (step + 1) & 31;
        sum += period * triangle_mix_index(step, false);
        remaining -= period;
    }
    sum += remaining * triangle_mix_index((step + 1) & 31, false);
    return (unsigned)tone_window_average((int32_t)sum);
}

static NES_ALWAYS_INLINE void mix_sample(Apu *a)
{
    MixerLevels levels = mixer_levels(a);
    unsigned lo = levels.pulse0 * ((levels.duty0 >> a->pulse[0].step) & 1);
    unsigned hi = levels.pulse1 * ((levels.duty1 >> a->pulse[1].step) & 1);
    unsigned filters = a->ultrafast ? 0 : regular_tone_filters(a, levels);
    unsigned p = filters & PULSE_FILTER ? sampled_pulse_dac(a, levels, lo, hi) : fixed_pulse_mix[lo + hi];
    unsigned noise_weight = FIXED_NOISE_WEIGHT * levels.noise;
    unsigned n = a->noise.shift & 1 ? 0 : noise_weight;

    if (filter_fast_noise(a) && a->noise.timer < a->noise.period)
        n = filtered_noise_weight(a->noise.shift, noise_weight, a->noise.period);
    // Average triangle periods beyond the mixer's Nyquist limit
    unsigned t = triangle_mix_index(a->triangle.step, (levels.tone_flags & TRIANGLE_AVERAGE));

    if ((filters & TRIANGLE_FILTER) && a->triangle.timer <= a->triangle.period)
        t = averaged_triangle_index(a->triangle.period + 1u, a->triangle.step, a->triangle.timer + 1u);
    unsigned input = p +
        fixed_tnd_mix[FIXED_DMC_WEIGHT * a->dmc.level + n + t];
    int32_t filtered = filter_fixed_sample(&a->filter, input);

    if (a->sample_count < NES_AUDIO_CAPACITY) {
        unsigned i = a->sample_count++;

        store_sample(sample_output(a, 0, a->wide_pcm), i, filtered, a->wide_pcm);
    }
}

static void apu_tick(Apu *a, uint64_t cycle)
{
    if (cycle == a->frame_event) {
        if ((cycle & 1) && a->frame_irq_clear)
            a->frame_irq = a->frame_irq_line = a->frame_irq_clear = false;
        frame_tick(a, cycle);
    }
    dmc_tick(a, cycle);
    a->sample_phase += NES_SAMPLE_RATE;
    unsigned cpu_hz = NES_CPU_HZ;

    if (a->sample_phase >= cpu_hz) {
        a->sample_phase -= cpu_hz;
        sync_tones(a, cycle + 1, false);
        mix_sample(a);
    }
}

static void advance_silent_dmc(Dmc *d, unsigned clocks)
{
    unsigned steps = timer_steps(&d->timer, clocks, d->period - 1);

    if (steps) {
        d->bits = 8 - ((steps + 8 - d->bits) & 7);
        d->shift >>= steps < 8 ? steps : 8;
    }
}

static bool emit_silence(Apu *a, unsigned phase)
{
    if (a->pulse[0].length || a->pulse[1].length || a->noise.length ||
        (a->triangle.length && a->triangle.linear) || a->dmc.level || a->triangle.step != 15)
        return false;
    FixedAudioFilter f = a->filter;
    // Clear the filter once its state is within a DAC unit of silence
    const int32_t low_unit = 1 << FIXED_LP_SHIFT;

    if ((unsigned)(f.dc + FIXED_HP_SCALE) > 2 * FIXED_HP_SCALE ||
        (unsigned)(f.low + low_unit) > (unsigned)(2 * low_unit))
        return false;
    a->filter = (FixedAudioFilter){0};

    const unsigned cpu_hz = NES_CPU_HZ;
    unsigned samples = phase / cpu_hz;
    // Keep the oscillator state at the same cycle as the last silent sample
    unsigned last = (samples * cpu_hz - a->sample_phase + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;

    sync_tones(a, a->cycle + last, true);
    size_t available = NES_AUDIO_CAPACITY - a->sample_count;
    size_t count = samples < available ? samples : available;

    memset(sample_output(a, a->sample_count, a->wide_pcm), 0,
           count * (a->wide_pcm ? sizeof a->wide_samples[0] : sizeof a->samples[0]));
    a->sample_count += count;
    a->sample_phase = phase - samples * cpu_hz;
    return true;
}

static inline unsigned elapsed_steps(unsigned *deadline, unsigned period, unsigned cursor)
{
    unsigned steps = 1, elapsed = cursor - *deadline;

    if (elapsed >= period)
        steps += elapsed / period;
    *deadline += steps * period;
    return steps;
}

enum { MIX_STEADY_DMC, MIX_CLOCK_DMC, MIX_STREAM_DMC };

static NES_ALWAYS_INLINE void clock_dmc_bit(Apu *a, uint64_t cycle, uint64_t end, bool stream)
{
    Dmc *d = &a->dmc;
    // The mixer keeps the timer in its local deadline between output bits
    bool byte_boundary = dmc_output_bit(a, cycle);

    if (stream && byte_boundary && d->empty && d->remaining &&
        (!d->irq_enabled || d->remaining > 1) && d->dma_halt < cpu_clock_cycle(a, end)) {
        // The new byte cannot affect output until the next bit clock
        d->timer = d->period - (unsigned)(apu_clock_cycle(a, d->dma_halt) - cycle);
        supply_stream(a, d->dma_halt);
    }
}

// Advance to the duty phase at the sampled cycle. In the common case
// only the output edge itself elapsed, so the new level is its inverse.
static inline unsigned pulse_update(unsigned level, unsigned duty, unsigned volume,
                                    unsigned period, unsigned *next, unsigned cursor)
{
    static const uint8_t rise_steps[4] = {1, 1, 1, 3};
    unsigned elapsed = cursor - *next;
    unsigned high = pulse_high_steps[duty];

    if (elapsed < period) {
        *next += (level ? 8 - high : high) * period;
        return level ^ volume;
    }
    unsigned ticks = elapsed / period;
    unsigned step = (rise_steps[duty] + (level ? high : 0) + ticks) & 7;

    *next += (ticks + pulse_edge_steps[duty][step]) * period;
    return volume * ((duty_bits[duty] >> step) & 1);
}

enum {
    MIX_FILTER_NOISE = 1, MIX_MUTED_NOISE = 2, MIX_PCM_FITS = 4, MIX_FILTER_TONES = 8
};

static NES_ALWAYS_INLINE void emit_samples(Apu *a, uint64_t end, unsigned dmc_mix, unsigned options)
{
    const bool filter_noise = options & MIX_FILTER_NOISE;
    const bool fits_buffer = options & MIX_PCM_FITS;
    const unsigned phase = a->sample_phase;
    const unsigned next = (NES_CPU_HZ - phase + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
    const unsigned final_phase = phase + (unsigned)(end - a->cycle) * NES_SAMPLE_RATE;
    const bool wide = a->wide_pcm;

    // The clocked path refreshed the levels while identifying its active voices
    MixerLevels levels = dmc_mix == MIX_CLOCK_DMC ? a->levels : mixer_levels(a);
    const unsigned noise_volume = options & MIX_MUTED_NOISE ? 0 : levels.noise;
    Triangle *triangle = &a->triangle;
    bool tri_clock = triangle->length && triangle->linear && !(levels.tone_flags & TRIANGLE_AVERAGE);

    // The DMC level and held triangle output are constant throughout this span
    if (dmc_mix == MIX_STEADY_DMC && !(levels.pulse0 || levels.pulse1 || noise_volume || tri_clock)) {
        unsigned t = triangle_mix_index(triangle->step, (levels.tone_flags & TRIANGLE_AVERAGE));
        unsigned samples = final_phase / NES_CPU_HZ;
        unsigned last = (samples * NES_CPU_HZ - phase + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
        size_t available = NES_AUDIO_CAPACITY - a->sample_count;
        unsigned written = available < samples ? (unsigned)available : samples;
        void *output = sample_output(a, a->sample_count, wide);
        unsigned input = fixed_tnd_mix[FIXED_DMC_WEIGHT * a->dmc.level + t];
        FixedAudioFilter filter = a->filter;
        unsigned i = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
        if (written >= 8) {
            i = written & ~3u;
            mix_held_neon(&filter, input, output, i, wide);
        }
#endif
        for (; i < written; ++i)
            store_sample(output, i, filter_fixed_sample(&filter, input), wide);
        for (; i < samples; ++i)
            filter_fixed_sample(&filter, input);
        a->filter = filter;
        a->sample_count += written;
        sync_tones(a, a->cycle + last, true);
        a->sample_phase = final_phase % NES_CPU_HZ;
        return;
    }

    // All deadlines are relative to the last synchronized oscillator state
    Pulse *p0 = &a->pulse[0], *p1 = &a->pulse[1];
    Noise *noise = &a->noise;
    uint64_t start = a->tone_cycle;
    unsigned cursor = (unsigned)(a->cycle - start), finish = (unsigned)(end - start);
    unsigned p0_next = levels.pulse0 ? 2u * p0->timer + 1 + (start & 1) : UINT32_MAX;
    unsigned p1_next = levels.pulse1 ? 2u * p1->timer + 1 + (start & 1) : UINT32_MAX;
    unsigned t_next = tri_clock ? triangle->timer + 1u : UINT32_MAX;
    unsigned n_next = noise_volume ? noise->timer + 1u : UINT32_MAX;
    unsigned p0_period = 2u * (p0->period + 1), p1_period = 2u * (p1->period + 1);
    unsigned t_period = triangle->period + 1u, n_period = noise->period;
    // The short noise periods are powers of two
#if defined(__GNUC__) || defined(__clang__)
    unsigned noise_shift = n_period && !(n_period & (n_period - 1)) ?
        (unsigned)__builtin_ctz(n_period) : 0;
#endif
    unsigned tap = noise->mode ? 6 : 1;
    unsigned p0_step = p0->step, p1_step = p1->step, t_step = triangle->step, n_shift = noise->shift;
    unsigned p0_edge = pulse_edge_steps[p0->duty][p0_step];
    unsigned p1_edge = pulse_edge_steps[p1->duty][p1_step];

    if (levels.pulse0)
        p0_next += (p0_edge - 1) * p0_period;
    if (levels.pulse1)
        p1_next += (p1_edge - 1) * p1_period;
    unsigned p0_volume = levels.pulse0 * ((levels.duty0 >> p0_step) & 1);
    unsigned p1_volume = levels.pulse1 * ((levels.duty1 >> p1_step) & 1);
    unsigned t = triangle_mix_index(t_step, (levels.tone_flags & TRIANGLE_AVERAGE));
    unsigned noise_weight = FIXED_NOISE_WEIGHT * noise_volume;
    unsigned n = n_shift & 1 ? 0 : noise_weight;

    if (filter_noise && noise_volume && n_next - cursor <= n_period)
        n = filtered_noise_weight(n_shift, noise_weight, n_period);
    Dmc *dmc = &a->dmc;
    unsigned dmc_next = cursor + dmc->timer + 1u;
    const uint16_t *tnd = fixed_tnd_mix + FIXED_DMC_WEIGHT * dmc->level;
    FixedAudioFilter filter = a->filter;

    const unsigned filters = options & MIX_FILTER_TONES ? regular_tone_filters(a, levels) : 0;
    const bool smooth_pulse = filters & PULSE_FILTER;
    const bool smooth_triangle = filters & TRIANGLE_FILTER;

    // In steady spans the combined pulse DAC value holds until the next edge
    const bool reuse_pulse = smooth_pulse || (dmc_mix == MIX_STEADY_DMC && !filter_noise);
    unsigned pulse = reuse_pulse ? fixed_pulse_mix[p0_volume + p1_volume] : 0;
    const unsigned early = TONE_WINDOW - 1;
    unsigned edge = p0_next < p1_next ? p0_next : p1_next;
    unsigned pulse_due = edge > early ? edge - early : 0;
    unsigned sample_count = a->sample_count;
    void *output = sample_output(a, 0, wide);
    // The reciprocal rounds up by at most 111 Q32 ticks at initialization.
    // Even with rounding accumulated over a sequencer span, that is less
    // than 1/NES_SAMPLE_RATE cycle, so the sample lands on the same cycle.
    const uint64_t cycle_unit = UINT64_C(1) << 32;
    const uint64_t sample_stride = (NES_CPU_HZ * cycle_unit + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
    const unsigned reciprocal_bits = 14;
    const uint32_t reciprocal = ((cycle_unit << reciprocal_bits) + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
    uint64_t sample_fraction = ((uint64_t)(NES_CPU_HZ - phase - 1) * reciprocal >> reciprocal_bits) + 1;
    uint64_t sample_cursor = ((uint64_t)cursor + 1) * cycle_unit + sample_fraction;
    const uint64_t sample_limit = ((uint64_t)finish + 1) * cycle_unit;
    unsigned remaining = final_phase / NES_CPU_HZ;

    cursor += next;

    for (;;) {
        if (!reuse_pulse || (smooth_pulse ? cursor >= pulse_due : cursor >= p0_next || cursor >= p1_next)) {
            if (cursor >= p0_next) {
                p0_volume = pulse_update(p0_volume, p0->duty, levels.pulse0, p0_period,
                                         &p0_next, cursor);
            }
            if (cursor >= p1_next) {
                p1_volume = pulse_update(p1_volume, p1->duty, levels.pulse1, p1_period,
                                         &p1_next, cursor);
            }
            edge = p0_next < p1_next ? p0_next : p1_next;
            pulse_due = edge > early ? edge - early : 0;

            // The next PCM sample is more than 32 cycles away, beyond any
            // edge we blend into this sample. It will recompute the DAC.
            if (smooth_pulse && edge - cursor < TONE_WINDOW)
                pulse = averaged_pulse_dac(p0, p1, levels.pulse0 | (levels.pulse1 << 4),
                                           p0_volume, p1_volume, p0_next - cursor, p1_next - cursor);
            else
                pulse = fixed_pulse_mix[p0_volume + p1_volume];
        }
        if (cursor >= t_next) {
            t_step = (t_step + elapsed_steps(&t_next, t_period, cursor)) & 31;
            if (!smooth_triangle)
                t = triangle_mix_index(t_step, false);
        }
        if (smooth_triangle && t_next - cursor <= t_period)
            t = averaged_triangle_index(t_period, t_step, t_next - cursor);
        if (!(options & MIX_MUTED_NOISE) && cursor >= n_next) {
            unsigned steps;
#if defined(__GNUC__) || defined(__clang__)
            if (noise_shift || filter_noise) {
                steps = 1 + ((cursor - n_next) >> noise_shift);
                n_next += steps << noise_shift;
            } else
#endif
                steps = elapsed_steps(&n_next, n_period, cursor);
            n_shift = shift_noise(n_shift, steps, tap);
            n = filter_noise ? filtered_noise_weight(n_shift, noise_weight, n_period)
                             : n_shift & 1 ? 0 : noise_weight;
        } else if (filter_noise && noise_volume && n_next - cursor <= n_period) {
            // The old period can leave a long countdown across the write
            n = filtered_noise_weight(n_shift, noise_weight, n_period);
        }
        if (dmc_mix != MIX_STEADY_DMC && cursor >= dmc_next) {
            clock_dmc_bit(a, start + dmc_next - 1, end, dmc_mix == MIX_STREAM_DMC);
            dmc_next += dmc->period;
            tnd = fixed_tnd_mix + FIXED_DMC_WEIGHT * dmc->level;
        }
        unsigned mixed = pulse + tnd[n + t];
        int32_t filtered = filter_fixed_sample(&filter, mixed);

        if (fits_buffer || sample_count < NES_AUDIO_CAPACITY)
            store_sample(output, sample_count++, filtered, wide);
        sample_cursor += sample_stride;

        if (fits_buffer ? !--remaining : sample_cursor >= sample_limit)
            break;
        cursor = (unsigned)(sample_cursor >> 32);
    }

    if (dmc_mix != MIX_STEADY_DMC) {
        // The last part of the span can clock a bit after its final PCM sample
        if (finish >= dmc_next) {
            clock_dmc_bit(a, start + dmc_next - 1, end, dmc_mix == MIX_STREAM_DMC);
            dmc_next += dmc->period;
        }
        dmc->timer = dmc_next - finish - 1;
    }

    // Commit the exact oscillator phase even when its output never changed
    clock_pulses(a, start, cursor);
    if (tri_clock) {
        triangle->step = t_step;
        triangle->timer = t_next - cursor - 1;
    } else {
        clock_triangle(triangle, cursor);
    }
    if (noise_volume) {
        noise->shift = n_shift;
        noise->timer = n_next - cursor - 1;
    } else {
        clock_noise(noise, cursor, true);
    }
    a->tone_cycle = start + cursor;
    a->sample_phase = final_phase % NES_CPU_HZ;
    a->sample_count = sample_count;
    a->filter = filter;
}

#if defined(__APPLE__) && (defined(__clang__) || defined(__GNUC__))
__attribute__((section("__TEXT,__tone_filter,regular,pure_instructions")))
#endif
static NES_NOINLINE void emit_tone_samples(Apu *a, uint64_t end, unsigned dmc_mix, unsigned options)
{
    emit_samples(a, end, dmc_mix, options | MIX_FILTER_TONES);
}

static NES_NOINLINE void emit_clocked_samples_with_overflow(Apu *a, uint64_t end)
{
    emit_samples(a, end, MIX_CLOCK_DMC, 0);
}

static inline bool pcm_span_fits(const Apu *a, uint64_t end)
{
    const unsigned final_phase = a->sample_phase + (unsigned)(end - a->cycle) * NES_SAMPLE_RATE;
    const size_t available = NES_AUDIO_CAPACITY - a->sample_count;

    return final_phase < (available + 1) * NES_CPU_HZ;
}

static NES_NOINLINE void emit_fast_samples(Apu *a, uint64_t end, unsigned dmc_mix);
static NES_NOINLINE void emit_regular_samples(Apu *a, uint64_t end, unsigned dmc_mix, unsigned options);

static NES_NOINLINE void emit_clocked_samples(Apu *a, uint64_t end)
{
    if (!a->ultrafast)
        emit_regular_samples(a, end, MIX_CLOCK_DMC, 0);
    else if (pcm_span_fits(a, end))
        emit_fast_samples(a, end, MIX_CLOCK_DMC);
    else
        emit_clocked_samples_with_overflow(a, end);
}

static NES_NOINLINE void emit_streamed_samples(Apu *a, uint64_t end)
{
    if (!a->ultrafast)
        emit_regular_samples(a, end, MIX_STREAM_DMC, 0);
    else if (pcm_span_fits(a, end))
        emit_fast_samples(a, end, MIX_STREAM_DMC);
    else
        emit_samples(a, end, MIX_STREAM_DMC, 0);
}

static NES_NOINLINE void emit_clocked_filtered_samples(Apu *a, uint64_t end)
{
    emit_regular_samples(a, end, MIX_CLOCK_DMC, MIX_FILTER_NOISE);
}

static NES_NOINLINE void emit_steady_samples_with_overflow(Apu *a, uint64_t end)
{
    emit_samples(a, end, MIX_STEADY_DMC, 0);
}

// Ultrafast and Regular mix a span in passes instead of one sample at a
// time. Each voice adds its level changes to a delta buffer at the samples
// where the original loop would see them, a prefix sum rebuilds the DAC
// inputs, and a block filter turns them into PCM. The DAC inputs match the
// sample loop's exactly; only the filter's rounding differs.
//
// The low half of each running sum is the TND table index. The high half is
// the combined pulse index, or the pulse DAC value itself when Regular
// averages fast pulse edges.
static NES_ALWAYS_INLINE int32_t fast_dac_input(uint32_t index, bool pulse_values)
{
    return (pulse_values ? (int32_t)(index >> 16) : fixed_pulse_mix[index >> 16]) + fixed_tnd_mix[index & 0xffff];
}

#if defined(__aarch64__) && defined(__ARM_NEON)
// Four outputs depend linearly on the two filter states and four inputs, so
// independent multiplies replace the scalar recurrence's serial chain. Like
// mix_held_neon, the states are centered within the scalar filter's
// quantization intervals.
static NES_ALWAYS_INLINE void filter_inputs_neon(FixedAudioFilter *filter, const int32_t *delta, void *output,
                                                 unsigned count, bool pulse_values, bool wide)
{
    // Q31 impulse response for input already shifted left by 14 bits
    const int32x4_t col0 = {173670400, 59953600, 19819060, 5660987};
    const int32x4_t col1 = {0, col0[0], col0[1], col0[2]};
    const int32x4_t col2 = {0, 0, col0[0], col0[1]};
    const int32x4_t col3 = {0, 0, 0, col0[0]};
    const int32x4_t from_dc = vnegq_s32(filter_from_held);
    const int32x4_t to_dc = {16387064, 16516096, 16646144, 16777216}; // b^(3-i) / 128
    const int32x4_t dc_step = vdupq_n_s32(filter_dc_step);
    const int32_t bias_dc = FIXED_HP_SCALE / 2, bias_low = 1 << (FIXED_LP_SHIFT - 1);
    const int32x4_t bias = vdupq_n_s32(bias_low);
    int32x4_t dc = vdupq_n_s32(filter->dc - bias_dc);
    int32x4_t low = vdupq_n_s32(filter->low - bias_low);
    unsigned blocks = count & ~3u;
    uint32_t index = 0;

    for (unsigned i = 0; i < blocks; i += 4) {
        int32x4_t x = vdupq_n_s32(0);

        index += (uint32_t)delta[i];
        x = vsetq_lane_s32(fast_dac_input(index, pulse_values), x, 0);
        index += (uint32_t)delta[i + 1];
        x = vsetq_lane_s32(fast_dac_input(index, pulse_values), x, 1);
        index += (uint32_t)delta[i + 2];
        x = vsetq_lane_s32(fast_dac_input(index, pulse_values), x, 2);
        index += (uint32_t)delta[i + 3];
        x = vsetq_lane_s32(fast_dac_input(index, pulse_values), x, 3);
        x = vshlq_n_s32(x, 14);
        int32x4_t y0 = filter_mul_add_q31(vqrdmulhq_laneq_s32(col0, x, 0), col1, vdupq_laneq_s32(x, 1));
        int32x4_t y1 = filter_mul_add_q31(vqrdmulhq_laneq_s32(col2, x, 2), col3, vdupq_laneq_s32(x, 3));
        int32x4_t y = filter_mul_add_q31(vaddq_s32(y0, y1), from_dc, dc);
        int32x4_t to = vqrdmulhq_s32(x, to_dc);

        to = vpaddq_s32(to, to);
        dc = filter_mul_add_q31(vpaddq_s32(to, to), dc, dc_step);
        low = filter_mul_add_q31(y, filter_from_low, vdupq_laneq_s32(low, 3));
        store_samples_neon(output, i, vaddq_s32(low, bias), wide);
    }
    FixedAudioFilter f = {vgetq_lane_s32(dc, 0) + bias_dc, vgetq_lane_s32(low, 3) + bias_low};

    for (unsigned i = blocks; i < count; ++i) {
        index += (uint32_t)delta[i];
        store_sample(output, i, filter_fixed_sample(&f, (unsigned)fast_dac_input(index, pulse_values)), wide);
    }
    *filter = f;
}

static NES_NOINLINE void filter_deltas_neon(FixedAudioFilter *filter, const int32_t *delta, int16_t *output,
                                            unsigned count)
{
    filter_inputs_neon(filter, delta, output, count, false, false);
}

static NES_NOINLINE void filter_values_neon(FixedAudioFilter *filter, const int32_t *delta, int16_t *output,
                                            unsigned count)
{
    filter_inputs_neon(filter, delta, output, count, true, false);
}

static NES_NOINLINE void filter_wide_neon(FixedAudioFilter *filter, const int32_t *delta, int32_t *output,
                                          unsigned count, bool pulse_values)
{
    if (pulse_values)
        filter_inputs_neon(filter, delta, output, count, true, true);
    else
        filter_inputs_neon(filter, delta, output, count, false, true);
}
#endif

// Rebuild the TND table index and the pulse index or value from their
// changes, look up the DAC inputs, and filter them.
static inline void filter_deltas(Apu *a, const int32_t *delta, unsigned count, bool pulse_values)
{
    const bool wide = a->wide_pcm;

#if defined(__aarch64__) && defined(__ARM_NEON)
    if (wide)
        filter_wide_neon(&a->filter, delta, a->wide_samples + a->sample_count, count, pulse_values);
    else if (pulse_values)
        filter_values_neon(&a->filter, delta, a->samples + a->sample_count, count);
    else
        filter_deltas_neon(&a->filter, delta, a->samples + a->sample_count, count);
#else
    uint32_t index = 0;
    void *output = sample_output(a, a->sample_count, wide);

    for (unsigned i = 0; i < count; ++i) {
        index += (uint32_t)delta[i];
        store_sample(output, i, filter_fixed_sample(&a->filter, (unsigned)fast_dac_input(index, pulse_values)), wide);
    }
#endif
}

// Sample positions are Q32 cycles relative to the synchronized oscillator
// state, with the same rounded-up stride as emit_samples.
#define FAST_SAMPLE_STRIDE ((NES_CPU_HZ * (UINT64_C(1) << 32) + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE)

// Voices with shorter periods than these, in CPU cycles, update at every
// sample instead of finding each edge's sample.
enum {
    TRIANGLE_SINGLE_STEP_MIN = 128,
    MAX_PCM_SAMPLE_CLOCKS = (NES_CPU_HZ + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE,
    FAST_TRIANGLE_SAMPLED = 256, FAST_NOISE_SAMPLED = 256, FAST_NOISE_WORDS = 256
};

_Static_assert(MAX_PCM_SAMPLE_CLOCKS >= TONE_WINDOW, "Wide triangle samples must cover at most one edge");
_Static_assert((uint64_t)NES_AUDIO_CAPACITY * NES_CPU_HZ + 2048u * NES_SAMPLE_RATE <= UINT32_MAX,
               "Point-sampled triangle edges fit in a 32-bit sample phase");

// The first sample whose cycle reaches EDGE
static inline unsigned fast_sample_index(uint64_t first, unsigned edge)
{
    uint64_t target = (uint64_t)edge << 32;
    return target <= first ? 0 : (unsigned)((target - first + FAST_SAMPLE_STRIDE - 1) / FAST_SAMPLE_STRIDE);
}

static inline unsigned fast_sample_cursor(uint64_t first, unsigned index)
{
    return (unsigned)((first + index * FAST_SAMPLE_STRIDE) >> 32);
}

static NES_ALWAYS_INLINE int triangle_step_delta(unsigned step)
{
    return step & 15 ? (step & 16 ? FIXED_TRIANGLE_WEIGHT : -FIXED_TRIANGLE_WEIGHT) : 0;
}

static NES_ALWAYS_INLINE void add_single_triangle_edges(int32_t *delta, unsigned last, unsigned cursor,
                                                        unsigned sample_phase, unsigned period,
                                                        unsigned *step, unsigned *next)
{
    unsigned edge = *next, phase = *step;

    // An edge already due at the start belongs to sample zero
    if (edge <= cursor && edge <= last) {
        phase = (phase + 1) & 31;
        delta[0] += triangle_step_delta(phase);
        edge += period;
    }

    // Convert edge cycles to the APU sample phase once and advance by one triangle period per edge
    unsigned position = sample_phase + (edge - cursor - 1) * NES_SAMPLE_RATE;
    unsigned interval = period * NES_SAMPLE_RATE;

    while (edge <= last) {
        phase = (phase + 1) & 31;
        delta[position / NES_CPU_HZ] += triangle_step_delta(phase);
        position += interval;
        edge += period;
    }
    *step = phase;
    *next = edge;
}

static NES_ALWAYS_INLINE void add_triangle_edges(int32_t *delta, unsigned last, uint64_t first, unsigned period,
                                                  unsigned *step, unsigned *next)
{
    unsigned level = triangle_mix_index(*step, false);

    while (*next <= last) {
        unsigned i = fast_sample_index(first, *next);
        unsigned steps = elapsed_steps(next, period, fast_sample_cursor(first, i));
        *step = (*step + steps) & 31;
        unsigned after = triangle_mix_index(*step, false);

        delta[i] += (int32_t)after - (int32_t)level;
        level = after;
    }
}

// The noise register after STEPS steps, from the output bit stream
static NES_ALWAYS_INLINE uint64_t noise_window(const uint64_t *words, unsigned steps)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint64_t window;

    memcpy(&window, (const unsigned char *)words + steps / 8, sizeof window);
    return window >> (steps % 8);
#else
    return words[steps / 64] >> (steps % 64) | words[steps / 64 + 1] << (63 - steps % 64) << 1;
#endif
}

// A pulse adds its volume changes, in the high half of each delta
static NES_ALWAYS_INLINE void fast_pulse_deltas(int32_t *delta, const Pulse *p, unsigned volume, unsigned level,
                                                uint64_t start, uint64_t first, unsigned count)
{
    unsigned period = 2u * (p->period + 1);
    unsigned next = 2u * p->timer + 1 + (start & 1) + (pulse_edge_steps[p->duty][p->step] - 1u) * period;

    for (;;) {
        unsigned i = fast_sample_index(first, next);

        if (i >= count)
            break;
        unsigned after = pulse_update(level, p->duty, volume, period, &next, fast_sample_cursor(first, i));

        delta[i] += ((int32_t)after - (int32_t)level) * 65536;
        level = after;
    }
}

// Regular averages the pulse DAC across edges near a sample, so both pulses
// share one pass. Like emit_samples, it recomputes the DAC value at the
// samples within a window before an edge and at the first one after it, and
// holds the value in between.
static NES_ALWAYS_INLINE void filtered_pulse_values(int32_t *delta, const Apu *a, MixerLevels levels, unsigned value,
                                                    uint64_t start, uint64_t first, unsigned count)
{
    const Pulse *p0 = &a->pulse[0], *p1 = &a->pulse[1];
    unsigned level0 = levels.pulse0 * ((levels.duty0 >> p0->step) & 1);
    unsigned level1 = levels.pulse1 * ((levels.duty1 >> p1->step) & 1);
    const unsigned period0 = 2u * (p0->period + 1), period1 = 2u * (p1->period + 1);
    unsigned next0 = levels.pulse0 ? 2u * p0->timer + 1 + (start & 1) +
        (pulse_edge_steps[p0->duty][p0->step] - 1u) * period0 : UINT32_MAX;
    unsigned next1 = levels.pulse1 ? 2u * p1->timer + 1 + (start & 1) +
        (pulse_edge_steps[p1->duty][p1->step] - 1u) * period1 : UINT32_MAX;
    const unsigned volumes = levels.pulse0 | (levels.pulse1 << 4), early = TONE_WINDOW - 1;
    unsigned edge = next0 < next1 ? next0 : next1;
    unsigned i = fast_sample_index(first, edge > early ? edge - early : 0);

    while (i < count) {
        unsigned cursor = fast_sample_cursor(first, i);

        if (cursor >= next0)
            level0 = pulse_update(level0, p0->duty, levels.pulse0, period0, &next0, cursor);
        if (cursor >= next1)
            level1 = pulse_update(level1, p1->duty, levels.pulse1, period1, &next1, cursor);
        edge = next0 < next1 ? next0 : next1;
        unsigned after = edge - cursor < TONE_WINDOW ?
            averaged_pulse_dac(p0, p1, volumes, level0, level1, next0 - cursor, next1 - cursor) :
            fixed_pulse_mix[level0 + level1];

        delta[i] += ((int32_t)after - (int32_t)value) * 65536;
        value = after;
        ++i;

        if (edge - cursor > early) {
            unsigned due = fast_sample_index(first, edge - early);
            i = due > i ? due : i;
        }
    }
}

// Regular passes its noise and tone filters in OPTIONS and FILTERS. Spans
// whose filtered voices still count down from a longer period fall back to
// the sample loop.
static NES_ALWAYS_INLINE void emit_pass_samples(Apu *a, uint64_t end, unsigned dmc_mix, unsigned options,
                                                bool regular)
{
    // The clocked paths refreshed the levels while identifying their active voices
    MixerLevels levels = mixer_levels(a);
    Triangle *triangle = &a->triangle;
    bool tri_clock = triangle->length && triangle->linear && !(levels.tone_flags & TRIANGLE_AVERAGE);
    const unsigned noise_volume = levels.noise;

    if (dmc_mix == MIX_STEADY_DMC && !(levels.pulse0 || levels.pulse1 || noise_volume || tri_clock)) {
        // The held mixer covers a constant input
        emit_steady_samples_with_overflow(a, end);
        return;
    }

    const unsigned phase = a->sample_phase;
    const unsigned final_phase = phase + (unsigned)(end - a->cycle) * NES_SAMPLE_RATE;
    const unsigned count = final_phase / NES_CPU_HZ;
    Pulse *p0 = &a->pulse[0], *p1 = &a->pulse[1];
    Noise *noise = &a->noise;
    Dmc *dmc = &a->dmc;
    const uint64_t start = a->tone_cycle;
    const unsigned cursor = (unsigned)(a->cycle - start), finish = (unsigned)(end - start);
    // Match emit_samples' first sample position
    const unsigned reciprocal_bits = 14;
    const uint32_t reciprocal = (((UINT64_C(1) << 32) << reciprocal_bits) + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
    const uint64_t first = ((uint64_t)cursor + 1) * (UINT64_C(1) << 32) +
        ((uint64_t)(NES_CPU_HZ - phase - 1) * reciprocal >> reciprocal_bits) + 1;
    const unsigned last = fast_sample_cursor(first, count - 1);

    const unsigned filters = regular ? regular_tone_filters(a, levels) : 0;
    const bool filter_noise = regular && (options & MIX_FILTER_NOISE) && noise_volume;
    const bool pulse_values = filters & PULSE_FILTER;
    int32_t delta[NES_AUDIO_CAPACITY];

    memset(delta, 0, count * sizeof *delta);
    unsigned p0_level = levels.pulse0 * ((levels.duty0 >> p0->step) & 1);
    unsigned p1_level = levels.pulse1 * ((levels.duty1 >> p1->step) & 1);
    unsigned t_step = triangle->step, t = triangle_mix_index(t_step, (levels.tone_flags & TRIANGLE_AVERAGE));
    unsigned noise_weight = FIXED_NOISE_WEIGHT * noise_volume;
    unsigned n_shift = noise->shift, n = n_shift & 1 ? 0 : noise_weight;
    unsigned dmc_level = dmc->level;
    unsigned pulse = pulse_values ? fixed_pulse_mix[p0_level + p1_level] : p0_level + p1_level;

    delta[0] = (int32_t)((pulse << 16) | (FIXED_DMC_WEIGHT * dmc_level + n + t));

    if (pulse_values) {
        filtered_pulse_values(delta, a, levels, pulse, start, first, count);
    } else {
        if (levels.pulse0)
            fast_pulse_deltas(delta, p0, levels.pulse0, p0_level, start, first, count);
        if (levels.pulse1)
            fast_pulse_deltas(delta, p1, levels.pulse1, p1_level, start, first, count);
    }

    unsigned t_next = triangle->timer + 1u, t_period = triangle->period + 1u;

    if (tri_clock && !(filters & TRIANGLE_FILTER)) {
        if (t_period >= TRIANGLE_SINGLE_STEP_MIN && fast_sample_cursor(first, 0) < t_next + t_period) {
            add_single_triangle_edges(delta, last, cursor, phase, t_period, &t_step, &t_next);
        } else if (t_period <= FAST_TRIANGLE_SAMPLED && t_next <= t_period) {
            const uint64_t reciprocal = ((UINT64_C(1) << 32) + t_period - 1) / t_period;
            const unsigned origin = t_next - t_period;
            unsigned steps = 0;
            uint64_t position = first;

#if defined(__aarch64__) && defined(__SIZEOF_INT128__)
            // Scaling by the sample rate makes successive PCM positions exactly NES_CPU_HZ apart
            uint64_t sample = (uint64_t)(cursor + 1u - origin) * NES_SAMPLE_RATE + NES_CPU_HZ - phase - 1u;
            uint64_t divisor = (uint64_t)t_period * NES_SAMPLE_RATE;
            uint64_t inverse = UINT64_MAX / divisor + 1;

            // Below this bound the rounded reciprocal yields the exact integer quotient
            if (sample + (uint64_t)(count - 1) * NES_CPU_HZ < inverse) {
                for (unsigned i = 0; i < count; ++i, sample += NES_CPU_HZ) {
                    steps = (unsigned)(((__uint128_t)sample * inverse) >> 64);
                    unsigned after = triangle_mix_index((t_step + steps) & 31, false);

                    delta[i] += (int32_t)after - (int32_t)t;
                    t = after;
                }
            } else
#endif
            {
                for (unsigned i = 0; i < count; ++i, position += FAST_SAMPLE_STRIDE) {
                    steps = (unsigned)(((uint64_t)((unsigned)(position >> 32) - origin) * reciprocal) >> 32);
                    unsigned after = triangle_mix_index((t_step + steps) & 31, false);

                    delta[i] += (int32_t)after - (int32_t)t;
                    t = after;
                }
            }
            t_step = (t_step + steps) & 31;
            t_next += steps * t_period;
        } else {
            add_triangle_edges(delta, last, first, t_period, &t_step, &t_next);
        }
    } else if (tri_clock) {
        if (t_period > FAST_TRIANGLE_SAMPLED || t_next > t_period) {
            emit_tone_samples(a, end, dmc_mix, options);
            return;
        }
        const uint64_t reciprocal = ((UINT64_C(1) << 32) + t_period - 1) / t_period;
        const unsigned origin = t_next - t_period;
        unsigned steps = 0;
        uint64_t position = first;

        if (t_period >= MAX_PCM_SAMPLE_CLOCKS && fast_sample_cursor(first, 0) < t_next + t_period) {
            // At most one oscillator step elapses between samples
            for (unsigned i = 0; i < count; ++i, position += FAST_SAMPLE_STRIDE) {
                unsigned at = (unsigned)(position >> 32);
                unsigned advance = at >= t_next;

                t_step = (t_step + advance) & 31;
                t_next += advance ? t_period : 0;
                unsigned after = blend_triangle_step(t_step, t_next - at);

                delta[i] += (int32_t)after - (int32_t)t;
                t = after;
            }
        } else {
            // Locate the oscillator phase before averaging the window
            for (unsigned i = 0; i < count; ++i, position += FAST_SAMPLE_STRIDE) {
                unsigned offset = (unsigned)(position >> 32) - origin;

                steps = (unsigned)(((uint64_t)offset * reciprocal) >> 32);
                unsigned after = averaged_triangle_index(t_period, (t_step + steps) & 31,
                                                         (steps + 1) * t_period - offset);

                delta[i] += (int32_t)after - (int32_t)t;
                t = after;
            }
        }
        t_step = (t_step + steps) & 31;
        t_next += steps * t_period;
    }

    unsigned n_next = noise->timer + 1u, n_period = noise->period;

    if (noise_volume) {
        unsigned tap = noise->mode ? 6 : 1;
        // The short noise periods are powers of two
#if defined(__GNUC__) || defined(__clang__)
        unsigned noise_shift = n_period && !(n_period & (n_period - 1)) ? (unsigned)__builtin_ctz(n_period) : 0;
#else
        unsigned noise_shift = 0;
#endif
        // Count every sample's steps from the span start, as for the triangle
        const uint64_t reciprocal = ((UINT64_C(1) << 32) + n_period - 1) / n_period;
        const unsigned origin = n_next - n_period;
        const unsigned last_steps = (unsigned)(((uint64_t)(last - origin) * reciprocal) >> 32);
        // The filtered noise reads one more word
        const unsigned extra_bits = filter_noise ? 64 : 0;

        if (n_period <= FAST_NOISE_SAMPLED && n_next <= n_period &&
            last_steps + 15 + extra_bits <= 64 * FAST_NOISE_WORDS) {
            // After T steps, bit M of the register is the output after T + M
            // steps, so the outputs form one bit stream where bit N + 15 is bit
            // N XOR bit N + TAP. Squaring that recurrence three times puts its
            // terms 120 and 120 - 8 TAP bits back, so each 64-bit word follows
            // from the two before it.
            uint64_t words[FAST_NOISE_WORDS];

            words[0] = words[1] = 0;
            unsigned reg = n_shift;

            for (unsigned bit = 0; bit < 128; bit += 15) {
                words[bit / 64] |= (uint64_t)reg << (bit % 64);

                if (bit % 64 > 64 - 15 && bit / 64 == 0)
                    words[1] |= (uint64_t)reg >> (64 - bit % 64);
                reg = shift_noise(reg, 15, tap);
            }
            const unsigned near = 8 + 8 * tap;
            const unsigned word_count = (last_steps + 14 + extra_bits) / 64 + 1;

            for (unsigned k = 2; k < word_count; ++k)
                words[k] = (words[k - 2] >> 8 | words[k - 1] << 56) ^
                           (words[k - 2] >> near | words[k - 1] << (64 - near));
            uint64_t position = first;

            if (filter_noise) {
                // The filtered periods are powers of two, so the step count
                // is a shift of the sample position from the span's origin.
                // The register's low bits are the next outputs, and the
                // filter weighs how many of them are low.
                const unsigned period_shift = n_period == 4 ? 2 : n_period == 8 ? 3 : 4;
                const unsigned width = 32u >> period_shift, mask = (1u << width) - 1;
                uint16_t weights[9];

                for (unsigned low = 0; low <= width; ++low)
                    weights[width - low] = (uint16_t)filtered_noise_weight(~0u << low, noise_weight, n_period);
                position += (uint64_t)(n_period - n_next) << 32;

                for (unsigned i = 0; i < count; ++i, position += FAST_SAMPLE_STRIDE) {
                    unsigned steps = (unsigned)(position >> (32 + period_shift));
                    unsigned after = weights[noise_bit_counts[noise_window(words, steps) & mask]];

                    delta[i] += (int32_t)after - (int32_t)n;
                    n = after;
                }
            } else {
                for (unsigned i = 0; i < count; ++i, position += FAST_SAMPLE_STRIDE) {
                    unsigned steps = (unsigned)(((uint64_t)((unsigned)(position >> 32) - origin) * reciprocal) >> 32);
                    unsigned after = (words[steps / 64] >> (steps % 64)) & 1 ? 0 : noise_weight;

                    delta[i] += (int32_t)after - (int32_t)n;
                    n = after;
                }
            }
            uint64_t tail = words[last_steps / 64] >> (last_steps % 64);

            if (last_steps % 64 > 64 - 15)
                tail |= words[last_steps / 64 + 1] << (64 - last_steps % 64);
            n_shift = (unsigned)tail & 0x7fff;
            n_next += last_steps * n_period;
        } else if (filter_noise) {
            emit_tone_samples(a, end, dmc_mix, options);
            return;
        } else {
            for (;;) {
                unsigned i = fast_sample_index(first, n_next);

                if (i >= count)
                    break;
                unsigned at = fast_sample_cursor(first, i), steps;

                if (noise_shift) {
                    steps = 1 + ((at - n_next) >> noise_shift);
                    n_next += steps << noise_shift;
                } else
                    steps = elapsed_steps(&n_next, n_period, at);
                n_shift = shift_noise(n_shift, steps, tap);
                unsigned after = n_shift & 1 ? 0 : noise_weight;

                delta[i] += (int32_t)after - (int32_t)n;
                n = after;
            }
        }
    }

    if (dmc_mix != MIX_STEADY_DMC) {
        // Output bits clock in order, at most one per sample, including one
        // after the final sample.
        unsigned dmc_next = cursor + dmc->timer + 1u;

        while (dmc_next <= finish) {
            clock_dmc_bit(a, start + dmc_next - 1, end, dmc_mix == MIX_STREAM_DMC);
            unsigned i = fast_sample_index(first, dmc_next);

            if (i < count)
                delta[i] += FIXED_DMC_WEIGHT * ((int32_t)dmc->level - (int32_t)dmc_level);
            dmc_level = dmc->level;
            dmc_next += dmc->period;
        }
        dmc->timer = dmc_next - finish - 1;
    }

    filter_deltas(a, delta, count, pulse_values);

    // Commit the exact oscillator phase even when its output never changed
    clock_pulses(a, start, last);
    if (tri_clock) {
        triangle->step = t_step;
        triangle->timer = t_next - last - 1;
    } else {
        clock_triangle(triangle, last);
    }
    if (noise_volume) {
        noise->shift = n_shift;
        noise->timer = n_next - last - 1;
    } else {
        clock_noise(noise, last, true);
    }
    a->tone_cycle = start + last;
    a->sample_phase = final_phase % NES_CPU_HZ;
    a->sample_count += count;
}

static NES_NOINLINE void emit_fast_samples(Apu *a, uint64_t end, unsigned dmc_mix)
{
    emit_pass_samples(a, end, dmc_mix, 0, false);
}

// Regular mixes like emit_tone_samples, whose tone filters it always applies
static NES_NOINLINE void emit_regular_samples(Apu *a, uint64_t end, unsigned dmc_mix, unsigned options)
{
    if (pcm_span_fits(a, end))
        emit_pass_samples(a, end, dmc_mix, options, true);
    else
        emit_tone_samples(a, end, dmc_mix, options);
}

static NES_NOINLINE void emit_steady_samples(Apu *a, uint64_t end, unsigned phase)
{
    // A short span between DMC clocks often contributes just one sample
    if (phase < 2 * NES_CPU_HZ) {
        unsigned clocks = (NES_CPU_HZ - a->sample_phase + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;

        sync_tones(a, a->cycle + clocks, false);
        mix_sample(a);
        a->sample_phase = phase - NES_CPU_HZ;
        return;
    }
    if (a->ultrafast) {
        if (!pcm_span_fits(a, end))
            emit_steady_samples_with_overflow(a, end);
        else
            emit_fast_samples(a, end, MIX_STEADY_DMC);
    } else {
        emit_regular_samples(a, end, MIX_STEADY_DMC, filter_fast_noise(a) ? MIX_FILTER_NOISE : 0);
    }
}

static NES_ALWAYS_INLINE void advance_span(Apu *a, uint64_t end, bool silent_dmc)
{
    unsigned clocks = end - a->cycle;

    if (!clocks)
        return;

    unsigned phase = a->sample_phase + clocks * NES_SAMPLE_RATE;

    if (phase < NES_CPU_HZ)
        a->sample_phase = phase;
    else if (!emit_silence(a, phase))
        emit_steady_samples(a, end, phase);
    a->cycle = end;

    if (silent_dmc)
        advance_silent_dmc(&a->dmc, clocks);
    else
        a->dmc.timer -= clocks;
}

static inline uint64_t dmc_stream_fill_cycle(const Apu *a)
{
    const Dmc *d = &a->dmc;

    if (!d->empty || !d->remaining || (d->irq_enabled && d->remaining == 1))
        return UINT64_MAX;
    return dmc_fetch_cycle(d);
}

static inline bool mixing_clocked_voice(Apu *a)
{
    MixerLevels levels = mixer_levels(a);
    return levels.pulse0 || levels.pulse1 || levels.noise ||
           (a->triangle.length && a->triangle.linear && !(levels.tone_flags & TRIANGLE_AVERAGE));
}

static inline unsigned dmc_playback_clocks(const Dmc *d)
{
    unsigned bits = d->bits + (d->empty ? 0u : 8u * (1u + d->queued_count));

    return d->timer + 1 + (bits - 1) * d->period;
}

// Preserve the held mixer's silence fast path and its multi-sample filter
static inline uint64_t dmc_only_end(const Apu *a, uint64_t end)
{
    const Dmc *d = &a->dmc;
    unsigned clocks = dmc_playback_clocks(d);

    if (end - a->cycle > clocks)
        end = a->cycle + clocks; // The held mixer takes over after the last bit
#if defined(__aarch64__) && defined(__ARM_NEON)
    const unsigned scalar_clocks = (7 * NES_CPU_HZ) / NES_SAMPLE_RATE;

    if (d->timer > scalar_clocks || d->period > scalar_clocks)
        return a->cycle;
#endif
    if (a->triangle.step == 15 && !(a->triangle.length && a->triangle.linear) && !(d->level & 1) &&
        !a->noise.length && !a->pulse[0].length && !a->pulse[1].length) {
        unsigned elapsed = end - a->cycle;
        unsigned active_bits = 1 + (elapsed - d->timer - 1) / d->period;

        if (d->level <= 2 * active_bits)
            return a->cycle;
    }
    return end;
}

static NES_ALWAYS_INLINE void advance_apu(Apu *a, uint64_t cycle, bool stream, uint64_t fetch_limit)
{
    // Waveform events use APU cycles. DMA can still fetch while that clock is paused
    while (a->cycle < cycle) {
        uint64_t fetch = stream ? dmc_stream_fill_cycle(a) : UINT64_MAX;
        uint64_t fill = fetch < fetch_limit ? apu_clock_cycle(a, fetch) : UINT64_MAX;

        if (fill <= a->cycle) {
            supply_stream(a, fetch);
            continue;
        }
        bool silent_dmc = a->dmc.silence && a->dmc.empty;
        unsigned wait = a->frame_event - a->cycle;

        if (!silent_dmc && a->dmc.timer < wait) {
            const Dmc *d = &a->dmc;

            // The mixer can clock DMC output bits in either timing mode. The
            // CPU still owns nonstreamed refills and their bus stalls.
            if (!d->silence && (!stream || (!d->empty && d->dma_enable <= cpu_clock_cycle(a, a->cycle)))) {
                uint64_t end = cycle < a->frame_event ? cycle : a->frame_event;
                unsigned clocks = end - a->cycle;

                if (clocks > d->timer + d->period && clocks >= 2 * (NES_CPU_HZ / NES_SAMPLE_RATE + 1) &&
                    (!stream || !filter_fast_noise(a))) {
                    if (!a->ultrafast && (mixer_levels(a).tone_flags & SOLO_TONE_FILTERS)) {
                        // Switch to the solo cutoff when the DMC finishes playing
                        unsigned playing = dmc_playback_clocks(d) - 1;

                        if (clocks > playing) {
                            if (playing <= d->timer + d->period || playing < 2 * (NES_CPU_HZ / NES_SAMPLE_RATE + 1))
                                goto next_bit;
                            end = a->cycle + playing;
                        }
                    }
                    if (stream)
                        emit_streamed_samples(a, end);
                    else if (!mixing_clocked_voice(a)) {
                        end = dmc_only_end(a, end);

                        if (end - a->cycle < 2 * (NES_CPU_HZ / NES_SAMPLE_RATE + 1))
                            goto next_bit;
                        emit_clocked_samples(a, end);
                    } else if (filter_fast_noise(a))
                        emit_clocked_filtered_samples(a, end);
                    else
                        emit_clocked_samples(a, end);
                    a->cycle = end;
                    continue;
                }
            }
next_bit:
            wait = d->timer;
        }
        uint64_t event = a->cycle + wait;

        if (fill <= event && fill < cycle) {
            advance_span(a, fill, silent_dmc);
            supply_stream(a, fetch);
            continue;
        }
        if (event >= cycle) {
            advance_span(a, cycle, silent_dmc);
            break;
        }
        advance_span(a, event, silent_dmc);
        apu_tick(a, a->cycle++);
    }
    if (stream) {
        uint64_t fetch = dmc_stream_fill_cycle(a);

        if (fetch < fetch_limit)
            supply_stream(a, fetch);
    }
}

static NES_NOINLINE void advance_stream(Apu *a, uint64_t cycle, bool allow_edge, bool publish)
{
    advance_apu(a, apu_clock_cycle(a, cycle), true, cycle + allow_edge);
    if (publish)
        schedule_cpu_event(a);
}

static NES_NOINLINE void advance_unstreamed(Apu *a, uint64_t cycle, bool publish)
{
    advance_apu(a, cycle, false, 0);
    if (publish)
        schedule_cpu_event(a);
}

static inline void advance_audio(Apu *a, uint64_t cycle, bool allow_edge, bool publish)
{
    if (a->sync_count != UINT32_MAX)
        ++a->sync_count;
    if (cycle >= apu_stream_audio_cycle(a))
        advance_stream(a, cycle, allow_edge, publish);
    else
        advance_unstreamed(a, apu_clock_cycle(a, cycle), publish);
}

void apu_advance(Apu *a, uint64_t cycle)
{
    advance_audio(a, cycle, true, true);
}

void apu_advance_before_refill(Apu *a, uint64_t cycle)
{
    advance_audio(a, cycle, false, true);
}

void apu_dmc_fetch(Apu *a, uint8_t value, uint64_t get, uint64_t end)
{
    if (apu_queue_dmc_byte(a, value, get))
        return;
    bool advanced = apu_has_independent_clock(a);

    if (advanced) {
        advance_audio(a, end, true, false);
    } else if (a->cycle < end) {
        uint64_t elapsed = end - a->cycle;
        // Fill the buffer at the DMA cycle and defer mixing if no output bit,
        // sequencer event or sample needs the intervening clocks.
        bool defer = a->dmc.empty && end <= a->frame_event && elapsed <= a->dmc.timer &&
                     a->dmc.bits != 1 && end < apu_stream_audio_cycle(a) &&
                     a->sample_phase + elapsed * NES_SAMPLE_RATE < NES_CPU_HZ;

        advanced = !defer;

        if (advanced)
            advance_audio(a, end, true, false);
    }
    Dmc *d = &a->dmc;

    // Supplying the final byte can end playback before the scheduler would
    // publish the halt. Keep the same timestamp an advance would expose.
    if (advanced && !d->empty && d->remaining && apu_schedules_dmc_halt(a) &&
        (d->stop_after_fetch || (d->remaining == 1 && !d->loop)))
        d->dma_halt = (a->cycle + d->timer + (d->bits - 1u) * d->period + 1) | 1u;
    dmc_supply(d, value, get);
    schedule_cpu_event(a);
}

void apu_set_clock_pause(Apu *a, uint64_t cycle, uint64_t start, unsigned clocks)
{
    advance_audio(a, cycle, true, false);
    a->clock_offset = cycle - a->cycle;
    a->pause_start = start;
    a->pause_cycles = clocks;
    schedule_cpu_event(a);
}

void apu_set_stream_cart(Apu *a, const Cartridge *cart)
{
    a->stream_cart = cart;
    schedule_cpu_event(a);
}

uint8_t apu_read_status(Apu *a, uint8_t bus, uint64_t cycle)
{
    advance_audio(a, cycle, true, false);
    uint8_t status = (bus & 32) | (a->dmc.irq ? 128 : 0) | (a->frame_irq ? 64 : 0) |
        (a->dmc.remaining ? 16 : 0) | (a->noise.length ? 8 : 0) |
        (a->triangle.length ? 4 : 0) | (a->pulse[1].length ? 2 : 0) | (a->pulse[0].length ? 1 : 0);

    a->frame_irq_clear = true;
    schedule_frame_event(a);
    schedule_cpu_event(a);
    return status;
}

static void write_pulse(Apu *a, uint16_t address, uint8_t value)
{
    unsigned channel = (address >> 2) & 1;
    Pulse *p = &a->pulse[channel];

    switch (address & 3) {
    case 0:
        p->duty = value >> 6;
        envelope_write(&p->envelope, value);
        break;
    case 1:
        p->sweep_enabled = (value & 128) != 0;
        p->sweep_period = (value >> 4) & 7;
        p->sweep_negate = (value & 8) != 0;
        p->sweep_shift = value & 7;
        p->sweep_reload = true;
        break;
    case 2: p->period = (p->period & 0x700) | value; break;
    case 3:
        p->period = (p->period & 255) | ((value & 7) << 8);

        if (a->length_enabled & (1 << channel))
            p->length = lengths[value >> 3];
        p->step = 0;
        p->envelope.start = true;
        break;
    }
}

static inline void write_frame_counter(Apu *a, uint8_t value, uint64_t cycle)
{
    a->frame_next_mode = (value & 128) != 0;
    a->frame_inhibit = (value & 64) != 0;

    if (a->frame_inhibit)
        a->frame_irq = a->frame_irq_line = false;
    // Count the write cycle so frame reset lands on a put-to-get boundary
    a->frame_reset = (cycle | UINT64_C(1)) + 2;
    schedule_frame_event(a);
}

static inline unsigned dmc_sample_start(uint8_t value)
{
    return 0xc000 + value * 64;
}

static inline unsigned dmc_sample_length(uint8_t value)
{
    return 1 + value * 16;
}

static void apply_register(Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    Dmc *d = &a->dmc;

    if (address <= 0x400f || address == 0x4015)
        a->levels_dirty = true;
    if (address < 0x4008) {
        write_pulse(a, address, value);
        return;
    }

    switch (address) {
    case 0x4008:
        a->triangle.control = (value & 128) != 0;
        a->triangle.reload_value = value & 127;
        break;
    case 0x400a: a->triangle.period = (a->triangle.period & 0x700) | value; break;
    case 0x400b:
        a->triangle.period = (a->triangle.period & 255) | ((value & 7) << 8);

        if (a->length_enabled & 4)
            a->triangle.length = lengths[value >> 3];
        a->triangle.reload = true;
        break;
    case 0x400c: envelope_write(&a->noise.envelope, value); break;
    case 0x400e:
        a->noise.period = noise_periods[value & 15];
        a->noise.mode = (value & 128) != 0;
        break;
    case 0x400f:
        if (a->length_enabled & 8)
            a->noise.length = lengths[value >> 3];
        a->noise.envelope.start = true;
        break;
    case 0x4010:
        d->period = dmc_periods[value & 15];
        d->loop = (value & 64) != 0;
        d->irq_enabled = (value & 128) != 0;

        if (!d->irq_enabled)
            d->irq = false;
        break;
    case 0x4011:
        d->level = value & 127;
        ++a->dac_writes;
        break;
    case 0x4012: d->start = dmc_sample_start(value); break;
    case 0x4013:
        d->length = dmc_sample_length(value);
        break;
    case 0x4015:
        a->length_enabled = value & 15;

        if (!(value & 1)) a->pulse[0].length = 0;
        if (!(value & 2)) a->pulse[1].length = 0;
        if (!(value & 4)) a->triangle.length = 0;
        if (!(value & 8)) a->noise.length = 0;
        dmc_enable(a, (value & 16) != 0, cycle);
        d->irq = false;
        break;
    case 0x4017:
        write_frame_counter(a, value, apu_clock_cycle(a, cycle));
        break;
    }
}

// The channel ($4015 bit) whose length counter a register halts ($4000,
// $4004, $4008, $400C) or reloads (three registers later), or -1.
static int length_channel(uint16_t address)
{
    if (address >= 0x4010 || ((address & 3) && (address & 3) != 3))
        return -1;
    return (address >> 2) & 3;
}

static bool length_clock_at(const Apu *a, uint64_t cycle)
{
    if (cycle != a->frame_event)
        return false;
    unsigned frame = cycle - a->frame_start + 1;

    return frame == 14913 || frame == (a->frame_mode ? 37281u : 29829u) ||
           (cycle == a->frame_reset && a->frame_next_mode);
}

static void write_register(Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    int channel = length_channel(address);

    if (channel < 0 || !length_clock_at(a, apu_clock_cycle(a, cycle))) {
        apply_register(a, address, value, cycle);
        return;
    }
    // A length clock on the write cycle still sees the old halt flag, and a
    // reload is lost if the clock found the counter running. Clock this
    // counter before the write, and leave it out of the sequencer's clock.
    uint8_t *length = channel < 2 ? &a->pulse[channel].length :
                      channel == 2 ? &a->triangle.length : &a->noise.length;
    bool halt = channel < 2 ? a->pulse[channel].envelope.loop :
                channel == 2 ? a->triangle.control : a->noise.envelope.loop;
    bool running = *length != 0;

    if (running && !halt)
        --*length;
    uint8_t clocked = *length;

    a->length_clocked |= (uint8_t)(1u << channel);
    apply_register(a, address, value, cycle);
    if (running && (address & 3) == 3)
        *length = clocked;
}

void apu_write(Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (address == 0x4015 && a->ultrafast && (value & 15) == a->length_enabled) {
        Dmc *d = &a->dmc;
        uint64_t audio_cycle = apu_clock_cycle(a, cycle);
        // Starting a new sample can leave the mixer pending until the next bit.
        // The fetch is still anchored to the cycle of the enable write.
        if (!(value & 16) || d->remaining || d->queued_count || d->stop_after_fetch ||
            audio_cycle > a->frame_event || audio_cycle - a->cycle >= d->timer)
            advance_audio(a, cycle, true, false);
        dmc_enable(a, (value & 16) != 0, cycle);
        d->irq = false;
        schedule_cpu_event(a);
        return;
    }
    advance_audio(a, cycle, true, false);
    // Only a timer, phase, or triangle enable change needs the old
    // oscillators to run all the way to the write.
    if ((address < 0x4008 && (address & 2)) || address == 0x400a || address == 0x400b ||
        address == 0x400e || address == 0x4015)
        sync_tones(a, a->cycle, false);
    write_register(a, address, value, cycle);
    // An idle waveform write leaves the CPU event schedule empty
    if (address >= 0x4010 || a->dmc.remaining || a->cpu_deadline != UINT64_MAX ||
        a->stream_read_cycle != UINT64_MAX)
        schedule_cpu_event(a);
}

void apu_reset(Apu *a, uint64_t cycle)
{
    apu_write(a, 0x4015, 0, cycle);
    // The last $4017 value is written again, with the same sequencer phase
    // relative to the first instruction as at power.
    a->frame_irq = a->frame_irq_line = false;
    uint64_t audio_cycle = apu_clock_cycle(a, cycle);

    restart_frame_sequencer(a, audio_cycle);
    a->frame_start = audio_cycle;
    schedule_frame_event(a);
    schedule_cpu_event(a);
}

// The DMC and frame counter registers. $4014 and $4016 belong to the
// machine: sprite DMA and the controller strobe.
static NES_ALWAYS_INLINE unsigned control_write_effects(const Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    const Dmc *d = &a->dmc;
    // Rendering up to the write would read a due streamed DMC byte, and the
    // APU would have to publish its CPU events again.
    bool stream_due = cycle >= apu_stream_audio_cycle(a);

    switch (address) {
    case 0x4010:
        if (d->period == dmc_periods[value & 15] && d->loop == ((value & 64) != 0) &&
            d->irq_enabled == ((value & 128) != 0) && (d->irq_enabled || !d->irq))
            return WRITE_UNCHANGED;
        // A different rate cannot move a CPU event when no fetches are pending
        return d->remaining || (d->irq && !(value & 128)) ? WRITE_DEADLINE : WRITE_AUDIO;
    case 0x4011:
        if (stream_due)
            return WRITE_DEADLINE;
        // The bus handler ends the added lines of a No-Lag frame that starts streaming PCM
        return a->blanking && a->dac_writes + 1 >= NES_PCM_STREAM_WRITES ? WRITE_RASTER | WRITE_BUS : WRITE_AUDIO;
    case 0x4012:
        // A pending streamed loop can still restart from the old address
        return d->loop && stream_due ? WRITE_DEADLINE : WRITE_AUDIO;
    case 0x4013:
        return d->length != dmc_sample_length(value) && a->stream_cart && d->remaining ? WRITE_DEADLINE : WRITE_AUDIO;
    case 0x4015:
        // Writing the same enables changes no channel or DMC state
        return stream_due || (value & 15) != a->length_enabled || d->irq || d->stop_after_fetch ||
            ((value & 16) != 0) != (d->remaining != 0) ? WRITE_DEADLINE : WRITE_AUDIO;
    case 0x4017:
        // This reset stays private to the APU while frame IRQs remain inhibited
        return (value & 64) && a->frame_inhibit && apu_clock_cycle(a, cycle) <= a->frame_event ?
            WRITE_AUDIO : WRITE_DEADLINE;
    default:
        return WRITE_BUS;
    }
}

unsigned apu_write_effects(const Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    return apu_waveform_register(address) ? WRITE_AUDIO : control_write_effects(a, address, value, cycle);
}

// Regular renders up to each write, as the bus handler does. Ultrafast
// renders only before the writes that change what the mixer reads next.
bool apu_write_control_quiet(Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    Dmc *d = &a->dmc;

    switch (address) {
    case 0x4010: {
        unsigned effects = control_write_effects(a, 0x4010, value, cycle);

        if (effects & (WRITE_DEADLINE | WRITE_BUS))
            return false;
        if (effects)
            apu_write(a, address, value, cycle);
        else if (!a->ultrafast)
            advance_audio(a, cycle, true, false);
        return true;
    }
    case 0x4011:
        if (control_write_effects(a, 0x4011, value, cycle) & (WRITE_DEADLINE | WRITE_BUS))
            return false;
        // Changing the DAC level leaves the CPU-visible event schedule intact
        advance_audio(a, cycle, true, false);
        d->level = value & 127;
        ++a->dac_writes;
        return true;
    case 0x4012:
        if (control_write_effects(a, 0x4012, value, cycle) & (WRITE_DEADLINE | WRITE_BUS))
            return false;
        if (!a->ultrafast)
            advance_audio(a, cycle, true, false);
        d->start = dmc_sample_start(value);
        return true;
    case 0x4013:
        if (control_write_effects(a, 0x4013, value, cycle) & (WRITE_DEADLINE | WRITE_BUS))
            return false;
        if (!a->ultrafast)
            advance_audio(a, cycle, true, false);
        d->length = (uint16_t)dmc_sample_length(value);
        return true;
    case 0x4015:
        if (control_write_effects(a, 0x4015, value, cycle) & (WRITE_DEADLINE | WRITE_BUS))
            return false;
        if (!a->ultrafast)
            apu_write(a, address, value, cycle);
        return true;
    case 0x4017:
        if (control_write_effects(a, 0x4017, value, cycle) & (WRITE_DEADLINE | WRITE_BUS))
            return false;
        if (a->ultrafast)
            write_frame_counter(a, value, apu_clock_cycle(a, cycle));
        else
            apu_write(a, address, value, cycle);
        return true;
    default:
        return false;
    }
}

static bool envelope_matches(const Envelope *e, uint8_t value)
{
    return e->period == (value & 15) && e->constant == ((value & 16) != 0) &&
        e->loop == ((value & 32) != 0);
}

static bool waveform_unchanged(const Apu *a, uint16_t address, uint8_t value)
{
    const Pulse *p = &a->pulse[(address >> 2) & 1];

    switch (address) {
    case 0x4000: case 0x4004:
        return p->duty == value >> 6 && envelope_matches(&p->envelope, value);
    case 0x4002: case 0x4006:
        return (uint8_t)p->period == value;
    case 0x4008:
        return a->triangle.control == ((value & 128) != 0) && a->triangle.reload_value == (value & 127);
    case 0x4009: case 0x400d:
        return true;
    case 0x400a:
        return (uint8_t)a->triangle.period == value;
    case 0x400c:
        return envelope_matches(&a->noise.envelope, value);
    case 0x400e:
        return a->noise.period == noise_periods[value & 15] && a->noise.mode == ((value & 128) != 0);
    default:
        return false;
    }
}

void apu_write_waveform(Apu *a, uint16_t address, uint8_t value, uint64_t cycle)
{
    uint64_t audio_cycle = apu_clock_cycle(a, cycle);

    // Preserve the span boundaries used by the held-input mixer
    if (audio_cycle <= a->frame_event && waveform_unchanged(a, address, value) &&
        mixing_clocked_voice(a))
        return;
    // These controls leave oscillator timing alone. Apply the write now if
    // neither the frame sequencer nor the mixer needs its previous value.
    uint64_t elapsed = audio_cycle - a->cycle;
    unsigned cpu_hz = NES_CPU_HZ;

    if (!(address & 2) && audio_cycle <= a->frame_event && elapsed <= cpu_hz / NES_SAMPLE_RATE &&
        a->sample_phase + (unsigned)elapsed * NES_SAMPLE_RATE < cpu_hz) {
        write_register(a, address, value, cycle);
        return;
    }
    apu_write(a, address, value, cycle);
}
