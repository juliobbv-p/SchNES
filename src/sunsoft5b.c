#include "nes.h"
#include "apu_fixed.h"

#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

// Sunsoft 5B audio. Tone, noise and envelope modulate each PSG channel
// before it reaches the shared expansion DAC. Keep the constant equivalent
// of ultrasonic tones used by volume-register PCM.

enum { PSG_TICK = 16, ULTRASONIC_PERIOD = 3 };

// One channel at full volume is about 3.6 times as loud as a 2A03 pulse
// channel at full volume, matching common hardware-calibrated mixers. The
// DAC rises 1.5 dB per 5-bit level, and levels 0 and 1 are silent.
// Rounded Q8 mixer levels, with level 31 at 0.5354 * FIXED_DAC_GAIN * 256
static const unsigned channel_levels[32] = {
    0, 0, 59565, 70793, 84138, 99998, 118848, 141251,
    167877, 199523, 237133, 281833, 334959, 398100, 473143, 562331,
    668332, 794314, 944044, 1121998, 1333497, 1584864, 1883614, 2238680,
    2660676, 3162220, 3758305, 4466754, 5308747, 6309457, 7498804, 8912345
};

_Static_assert(FIXED_DAC_GAIN == 65024, "Update the 5B volume table when changing the DAC gain");

static unsigned period(unsigned value)
{
    // A period of zero behaves like one
    return value ? value : 1;
}

static unsigned envelope_level(const Sunsoft5b *s)
{
    return s->envelope_step ^ s->envelope_mask;
}

static unsigned tone_period(const Sunsoft5b *s, unsigned ch)
{
    return period(s->regs[ch * 2] | ((s->regs[ch * 2 + 1] & 15) << 8));
}

static void update_output(Sunsoft5b *s, bool seed_channels, unsigned point_channels)
{
    unsigned mixer = s->regs[7];
    unsigned sum = 0;

    for (unsigned ch = 0; ch < 3; ++ch) {
        unsigned amplitude = 0;
        bool noise = s->noise_out || ((mixer >> (ch + 3)) & 1);

        if (noise) {
            unsigned volume = s->regs[8 + ch];
            unsigned level = (volume & 0x10) ? envelope_level(s) : (volume & 15) ? (volume & 15) * 2 + 1 : 0;

            amplitude = channel_levels[level];
            if (!((mixer >> ch) & 1)) {
                if (tone_period(s, ch) <= ULTRASONIC_PERIOD)
                    amplitude = (amplitude + 1) / 2;
                else if (!((s->tone_out >> ch) & 1))
                    amplitude = 0;
            }
        }
        if (s->mix.point_sampling) {
            sum += amplitude;
        } else if (seed_channels) {
            s->mix.channel[ch] = (ExpansionChannel){.target = amplitude};
        } else {
            ExpansionEdge edge = (point_channels & (1u << ch)) ? EXPANSION_POINT : EXPANSION_WINDOW;

            expansion_dac_set(&s->mix, ch, amplitude, edge);
        }
    }
    if (s->mix.point_sampling)
        s->point_output = (sum + 128) & ~255u;
}

static void clock_envelope(Sunsoft5b *s)
{
    if (s->envelope_holding)
        return;
    if (s->envelope_step) {
        --s->envelope_step;
        return;
    }
    // Shapes without the continue bit hold after the first ramp
    unsigned shape = s->regs[13];
    bool hold = (shape & 8) ? (shape & 1) : true;
    bool alternate = (shape & 8) ? (shape & 2) : (shape & 4);

    if (alternate)
        s->envelope_mask ^= 31;
    if (hold)
        s->envelope_holding = true;
    else
        s->envelope_step = 31;
}

static void tick(Sunsoft5b *s)
{
    bool use_filters = !s->mix.point_sampling;
    unsigned point_tones = 0, point_noise = 0;

    for (unsigned ch = 0; ch < 3; ++ch) {
        unsigned value = tone_period(s, ch);

        if (++s->tone_counter[ch] >= value) {
            s->tone_counter[ch] = 0;
            s->tone_out ^= 1u << ch;
            if (use_filters && value > EXPANSION_TONE_MAX_PERIOD)
                point_tones |= 1u << ch;
        }
    }
    s->noise_divider ^= 1;

    if (!s->noise_divider) {
        unsigned value = period(s->regs[6] & 31);

        if (++s->noise_counter >= value) {
            // A 17-bit LFSR with taps at bits 16 and 13
            s->noise_counter = 0;
            unsigned bit = (s->lfsr ^ (s->lfsr >> 3)) & 1;

            s->lfsr = (s->lfsr >> 1) | (bit << 16);
            bool noise_out = s->lfsr & 1;

            if (use_filters && value > EXPANSION_NOISE_MAX_PERIOD && noise_out != s->noise_out)
                point_noise = 7;
            s->noise_out = noise_out;
        }
    }
    if (++s->envelope_counter >= period(s->regs[11] | (s->regs[12] << 8))) {
        s->envelope_counter = 0;
        clock_envelope(s);
    }
    unsigned gates = ~(unsigned)s->regs[7];
    unsigned point_channels = (point_tones & gates) | (point_noise & (gates >> 3));

    update_output(s, false, point_channels);
}

// Ticks until a counter reaches its period and wraps to zero. A counter
// already past a shortened period wraps on the next tick.
static unsigned ticks_to_wrap(unsigned counter, unsigned period)
{
    return counter >= period ? 1 : period - counter;
}

// Advance a wrapping counter by `ticks` and return the number of wraps
static uint64_t advance_counter(uint16_t *counter, unsigned period, uint64_t ticks)
{
    unsigned first = ticks_to_wrap(*counter, period);

    if (ticks < first) {
        *counter = (uint16_t)(*counter + ticks);
        return 0;
    }
    uint64_t rest = ticks - first;

    if (rest < period) {
        *counter = (uint16_t)rest;
        return 1;
    }
    if (rest <= UINT32_MAX) {
        *counter = (uint16_t)((uint32_t)rest % period);
        return 1 + (uint32_t)rest / period;
    }
    *counter = (uint16_t)(rest % period);
    return 1 + rest / period;
}

static void step_lfsr(Sunsoft5b *s, uint64_t steps)
{
    // Up to 14 steps only read bits that were present before the first step
    uint32_t lfsr = s->lfsr;

    while (steps) {
        unsigned n = steps < 14 ? (unsigned)steps : 14;
        uint32_t bits = (lfsr ^ (lfsr >> 3)) & ((1u << n) - 1);

        lfsr = (lfsr >> n) | (bits << (17 - n));
        steps -= n;
    }
    s->lfsr = lfsr;
}

static void clock_envelope_times(Sunsoft5b *s, uint64_t clocks)
{
    while (clocks && !s->envelope_holding) {
        if (s->envelope_step) {
            unsigned n = clocks < s->envelope_step ? (unsigned)clocks : s->envelope_step;

            s->envelope_step = (uint8_t)(s->envelope_step - n);
            clocks -= n;
        } else {
            clock_envelope(s);
            --clocks;
        }
    }
}

// Run `ticks` PSG ticks without recomputing the output. The caller ensures
// that no change in this span is audible.
static void advance_ticks(Sunsoft5b *s, uint64_t ticks)
{
    if (!ticks)
        return;
    for (unsigned ch = 0; ch < 3; ++ch) {
        if (advance_counter(&s->tone_counter[ch], tone_period(s, ch), ticks) & 1)
            s->tone_out ^= 1u << ch;
    }
    // The noise counter advances on every second tick
    uint64_t noise_clocks = s->noise_divider ? (ticks + 1) / 2 : ticks / 2;

    s->noise_divider ^= ticks & 1;
    uint64_t shifts = advance_counter(&s->noise_counter, period(s->regs[6] & 31), noise_clocks);

    if (shifts) {
        step_lfsr(s, shifts);
        s->noise_out = s->lfsr & 1;
    }
    uint64_t clocks = advance_counter(&s->envelope_counter, period(s->regs[11] | (s->regs[12] << 8)), ticks);

    if (clocks)
        clock_envelope_times(s, clocks);
}

static void sync_state(Sunsoft5b *s)
{
    advance_ticks(s, s->ticks - s->state_tick);
    s->state_tick = s->ticks;
}

// Find the next tick that can change the current output
static void schedule_event(Sunsoft5b *s)
{
    unsigned mixer = s->regs[7];
    uint64_t next = UINT64_MAX;
    bool noise = false, envelope = false;

    for (unsigned ch = 0; ch < 3; ++ch) {
        unsigned volume = s->regs[8 + ch];

        // A channel at level zero adds nothing whatever its tone and noise do
        if (!(volume & 0x1f))
            continue;
        envelope |= (volume & 0x10) != 0;
        noise |= !((mixer >> (ch + 3)) & 1);
        unsigned tone = tone_period(s, ch);

        if (!((mixer >> ch) & 1) && tone > ULTRASONIC_PERIOD) {
            unsigned wrap = ticks_to_wrap(s->tone_counter[ch], tone);

            if (wrap < next)
                next = wrap;
        }
    }
    if (noise) {
        unsigned divider = period(s->regs[6] & 31);
        uint64_t wrap = 2 * (uint64_t)ticks_to_wrap(s->noise_counter, divider) - s->noise_divider;

        if (!s->mix.point_sampling && divider > EXPANSION_NOISE_MAX_PERIOD) {
            // Low LFSR bits give the next outputs. Skip clocks with the same level.
            unsigned future = (s->lfsr ^ (s->noise_out ? 0x1ffffu : 0)) >> 1;
            unsigned held = 0;

            while (held < 16 && !(future & 1)) {
                ++held;
                future >>= 1;
            }
            wrap += 2u * held * divider;
        }

        if (wrap < next)
            next = wrap;
    }
    if (envelope && !s->envelope_holding) {
        unsigned wrap = ticks_to_wrap(s->envelope_counter, period(s->regs[11] | (s->regs[12] << 8)));

        if (wrap < next)
            next = wrap;
    }
    s->event_tick = next == UINT64_MAX ? UINT64_MAX : s->ticks + next;
}

// Render up to `end` while the output holds. The PSG clock moves without
// running its counters, which sync_state brings up to date later.
static void render_steady(Sunsoft5b *s, uint64_t end, bool point_sampling)
{
    uint64_t cycles = end - s->mix.cycle;
    unsigned total = s->divider + (unsigned)(cycles % PSG_TICK);

    s->ticks += cycles / PSG_TICK + total / PSG_TICK;
    s->divider = (uint8_t)(total % PSG_TICK);
    if (point_sampling)
        expansion_mix_points(&s->mix, end, s->point_output);
    else
        expansion_dac_render(&s->mix, end, false);
}

static void render(Sunsoft5b *s, uint64_t cycle, bool point_sampling)
{
    if (s->mix.point_sampling != point_sampling) {
        s->mix.point_sampling = point_sampling;
        update_output(s, true, 0);
    }
    if (s->event_tick <= s->ticks)
        schedule_event(s);
    while (s->mix.cycle < cycle) {
        // The output holds until the tick that can change it
        uint64_t left = cycle - s->mix.cycle, ahead = s->event_tick - s->ticks;
        uint64_t to_event = ahead > left / PSG_TICK + 1 ? UINT64_MAX
            : PSG_TICK - s->divider + (ahead - 1) * PSG_TICK;

        if (left < to_event) {
            render_steady(s, cycle, point_sampling);
            break;
        }
        render_steady(s, s->mix.cycle + to_event, point_sampling);
        advance_ticks(s, s->ticks - 1 - s->state_tick);
        tick(s);
        s->state_tick = s->ticks;
        schedule_event(s);
    }
    // Leave every counter current for register writes and for inspection
    sync_state(s);
}

void sunsoft5b_write(Sunsoft5b *s, uint8_t reg, uint8_t value)
{
    reg &= 15;
    sync_state(s);
    s->regs[reg] = value;

    if (reg == 13) {
        // A shape write restarts the envelope
        s->envelope_step = 31;
        s->envelope_mask = (value & 4) ? 31 : 0;
        s->envelope_holding = false;
        s->envelope_counter = 0;
    }
    update_output(s, false, 0);
    schedule_event(s);
}

static NES_ALWAYS_INLINE ExpansionMix *render_expansion_audio(Nes *nes, uint64_t clock)
{
    MapperExt *x = nes->cart.ext;
    bool point_sampling = nes->ultrafast;

    if (cart_is_fds(&nes->cart)) {
        if (point_sampling)
            fds_audio_render_point(&x->fds->audio, clock);
        else
            fds_audio_render(&x->fds->audio, clock);
        return &x->fds->audio.mix;
    }
    if (nes->cart.mapper == 69) {
        render(&x->audio, clock, point_sampling);
        return &x->audio.mix;
    }
    if (point_sampling)
        vrc6_audio_render_point(&x->vrc6, clock);
    else
        vrc6_audio_render(&x->vrc6, clock);
    return &x->vrc6.mix;
}

void nes_expansion_audio_sync(Nes *nes, uint64_t cycle)
{
    const MapperExt *x = nes->cart.ext;

    if (x && x->has_audio)
        render_expansion_audio(nes, apu_clock_cycle(&nes->apu, cycle));
}

void nes_expansion_audio_finish_frame(Nes *nes)
{
    Apu *restrict a = &nes->apu;
    ExpansionMix *restrict m = render_expansion_audio(nes, a->cycle);
    unsigned n = m->count < a->sample_count ? m->count : a->sample_count;
    unsigned i = 0;

    // Sum at full precision and compact forward so unread samples stay intact
#if defined(__aarch64__) && defined(__ARM_NEON)
    for (; i < (n & ~7u); i += 8) {
        int32x4_t first = vaddq_s32(vld1q_s32(a->wide_samples + i), vld1q_s32(m->samples + i));
        int32x4_t second = vaddq_s32(vld1q_s32(a->wide_samples + i + 4), vld1q_s32(m->samples + i + 4));
        int16x4_t low = vqrshrn_n_s32(first, FIXED_PCM_SHIFT);

        vst1q_s16(a->samples + i, vqrshrn_high_n_s32(low, second, FIXED_PCM_SHIFT));
    }
#endif
    for (; i < n; ++i)
        a->samples[i] = mix_pcm16(a->wide_samples[i] + m->samples[i]);
    for (; i < a->sample_count; ++i)
        a->samples[i] = mix_pcm16(a->wide_samples[i]);
    m->count = 0;
}
