#include "nes.h"
#include "apu_fixed.h"

// FDS wavetable audio. Its DAC channel uses the adapter's 2 kHz cutoff.
// Full scale is 1.69 times a 2A03 pulse at volume 15, as in Mesen.

enum { FDS_TICK = 16 };

// The full-volume gain is 1/8000, scaled by 2/divisor for each master volume
#define FDS_LEVEL_Q16(divisor) \
    ((uint32_t)(((uint64_t)FIXED_DAC_GAIN * 65536 + 2000 * (divisor)) / (4000 * (divisor))))
static const uint32_t master_levels[4] = {
    FDS_LEVEL_Q16(2), FDS_LEVEL_Q16(3), FDS_LEVEL_Q16(4), FDS_LEVEL_Q16(5)
};
#undef FDS_LEVEL_Q16

// The adapter's 2 kHz low-pass for 48 kHz point samples, in Q16
enum { FDS_LOWPASS_Q16 = 15096 };

static uint32_t envelope_period(const FdsAudio *a, uint8_t control)
{
    return 8u * ((control & 63u) + 1) * (a->master_speed + 1u);
}

void fds_audio_reset_timers(FdsAudio *a)
{
    a->vol_timer = envelope_period(a, a->vol_control);
    a->mod_timer = envelope_period(a, a->mod_control);
}

static unsigned wave_position(const FdsAudio *a)
{
    return a->wave_acc >> 18;
}

static void update_level(FdsAudio *a)
{
    if (a->wave_write)
        return;
    unsigned gain = a->out_gain < 32 ? a->out_gain : 32;

    a->level = (uint16_t)(a->wave[wave_position(a)] * gain);
}

// Volume changes reach the output only at wave position 0
static void latch_gain(FdsAudio *a)
{
    if (!wave_position(a))
        a->out_gain = a->vol_gain;
}

static void update_pitch(FdsAudio *a)
{
    int temp = a->mod_counter * a->mod_gain;

    if ((temp & 0x0f) && !(temp & 0x800))
        temp += 0x20;
    temp += 0x400;
    unsigned scale = ((unsigned)temp >> 4) & 0xff;

    a->wave_pitch = (a->wave_freq * scale) & 0xfffff;
}

static bool envelopes_run(const FdsAudio *a)
{
    return a->enabled && !a->wave_halt && !a->env_halt && a->master_speed;
}

static bool clock_envelope(const FdsAudio *a, uint32_t *timer, uint8_t control, uint8_t *gain, unsigned cycles)
{
    if ((control & 0x80) || cycles < *timer) {
        if (!(control & 0x80))
            *timer -= cycles;
        return false;
    }
    bool changed = false;
    uint32_t period = envelope_period(a, control);

    while (cycles >= *timer) {
        cycles -= *timer;
        *timer = period;

        if ((control & 0x40) && *gain < 32) {
            ++*gain;
            changed = true;
        } else if (!(control & 0x40) && *gain) {
            --*gain;
            changed = true;
        }
    }
    *timer -= cycles;
    return changed;
}

static void run_envelopes(FdsAudio *a, unsigned cycles)
{
    if (clock_envelope(a, &a->vol_timer, a->vol_control, &a->vol_gain, cycles))
        latch_gain(a);
    if (clock_envelope(a, &a->mod_timer, a->mod_control, &a->mod_gain, cycles))
        update_pitch(a);
}

static bool idle(const FdsAudio *a)
{
    return (a->wave_halt || !a->wave_pitch) && (a->mod_halt || (!a->mod_freq && !a->mod_step)) &&
           (!envelopes_run(a) || ((a->vol_control & a->mod_control) & 0x80));
}

static NES_ALWAYS_INLINE void tick(FdsAudio *a)
{
    if (!a->mod_halt) {
        uint32_t sum = (a->mod_acc & 0xfff) + a->mod_freq;
        bool carry = sum > 0xfff || a->mod_step;

        a->mod_acc = (a->mod_acc & ~0xfffu) | (sum & 0xfff);

        if (carry) {
            static const int8_t steps[8] = {0, 1, 2, 4, 0, -4, -2, -1};
            unsigned entry = a->mod_table[(a->mod_acc >> 13) & 31];
            int counter = entry == 4 ? 0 : a->mod_counter + steps[entry];

            a->mod_counter = (int8_t)(((counter + 64) & 127) - 64);
            a->mod_acc = (a->mod_acc + 0x1000) & 0x3ffff;
            update_pitch(a);
        }
    }
    if (!a->wave_halt) {
        unsigned before = wave_position(a);

        a->wave_acc = (a->wave_acc + a->wave_pitch) & 0xffffff;

        if (wave_position(a) == before)
            return;
        latch_gain(a);
        update_level(a);
    }
}

// Cycles until a running envelope clocks
static uint32_t envelope_due(const FdsAudio *a)
{
    uint32_t due = (a->vol_control & 0x80) ? UINT32_MAX : a->vol_timer;

    if (!(a->mod_control & 0x80) && a->mod_timer < due)
        due = a->mod_timer;
    return due;
}

void fds_audio_render(FdsAudio *a, uint64_t cycle)
{
    ExpansionMix *m = &a->mix;
    uint64_t now = m->cycle;
    uint32_t scale = master_levels[a->master_volume];
    unsigned divider = a->divider;

    uint32_t level = (a->level * scale + 128) >> 8;

    if (m->point_sampling) {
        m->channel[0] = (ExpansionChannel){.edge = now, .start = (uint32_t)a->point_smooth, .target = level};
        m->point_sampling = false;
    }
    expansion_dac_set(m, 0, level, EXPANSION_FDS);
    if (cycle <= now)
        return;
    if (!a->enabled || idle(a)) {
        a->divider = (uint8_t)((divider + cycle - now) % FDS_TICK);
        expansion_dac_render(m, cycle, true);
        return;
    }
    bool envelopes = envelopes_run(a);
    uint32_t pending = 0, due = envelope_due(a);

    while (now < cycle) {
        unsigned step = FDS_TICK - divider;

        if (cycle - now < step)
            step = (unsigned)(cycle - now);
        now += step;
        pending += step;
        divider += step;
        if (divider == FDS_TICK) {
            divider = 0;

            // An envelope clock takes effect before the tick that follows it
            if (envelopes && pending >= due) {
                run_envelopes(a, pending);
                pending = 0;
                due = envelope_due(a);
            }
            unsigned previous = a->level;

            tick(a);
            if (a->level != previous) {
                expansion_dac_render(m, now, true);
                expansion_dac_set(m, 0, (a->level * scale + 128) >> 8, EXPANSION_FDS);
            }
        }
    }
    a->divider = (uint8_t)divider;
    if (envelopes && pending)
        run_envelopes(a, pending);
    expansion_dac_render(m, cycle, true);
}

void fds_audio_render_point(FdsAudio *a, uint64_t cycle)
{
    ExpansionMix *m = &a->mix;

    if (cycle <= m->cycle)
        return;
    if (!m->point_sampling) {
        a->point_smooth = (int32_t)expansion_fds_level(&m->channel[0], m->cycle);
        m->point_sampling = true;
    }
    unsigned divider = a->divider, count = m->count;
    int32_t smooth = a->point_smooth;
    bool resting = !a->enabled || idle(a);

    if (resting && !a->level && !smooth) {
        a->divider = (uint8_t)((divider + cycle - m->cycle) % FDS_TICK);
        expansion_mix_points(m, cycle, 0);
        return;
    }
    uint32_t pending = 0, due = envelope_due(a);
    bool envelopes = envelopes_run(a);
    uint32_t scale = master_levels[a->master_volume];
    ExpansionFilter filter = m->filter;

    while (m->cycle < cycle) {
        unsigned to_sample = (NES_CPU_HZ - m->phase + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
        unsigned span = cycle - m->cycle < to_sample ? (unsigned)(cycle - m->cycle) : to_sample;

        m->cycle += span;
        m->phase += span * NES_SAMPLE_RATE;
        if (resting || idle(a)) {
            divider = (divider + span) % FDS_TICK;
        } else {
            while (span) {
                unsigned step = FDS_TICK - divider;

                if (step > span)
                    step = span;
                pending += step;
                span -= step;
                divider += step;

                if (divider == FDS_TICK) {
                    divider = 0;
                    if (envelopes && pending >= due) {
                        run_envelopes(a, pending);
                        pending = 0;
                        due = envelope_due(a);
                    }
                    tick(a);
                }
            }
        }
        if (m->phase >= NES_CPU_HZ) {
            m->phase -= NES_CPU_HZ;
            int32_t input = (int32_t)((a->level * scale + 128) >> 8);

            // Round down so silence settles at zero
            smooth += (int32_t)(((int64_t)(input - smooth) * FDS_LOWPASS_Q16) >> 16);
            expansion_mix_store(m, &filter, &count, (uint32_t)smooth);
        }
    }
    a->point_smooth = smooth;
    a->divider = (uint8_t)divider;
    m->count = count;
    m->filter = filter;

    if (envelopes && pending)
        run_envelopes(a, pending);
}

void fds_audio_enable(FdsAudio *a, bool enabled)
{
    if (enabled == a->enabled)
        return;
    a->enabled = enabled;

    if (!enabled) {
        a->vol_gain = a->out_gain = 0;
        a->wave_acc = 0;
        a->mod_counter = 0;
        update_pitch(a);
        a->level = 0;
    }
}

static uint8_t read_register(const FdsAudio *a, uint16_t address, uint8_t bus)
{
    if (address < 0x4080)
        return (uint8_t)((bus & 0xc0) | a->wave[a->wave_write ? address & 63 : wave_position(a)]);
    switch (address) {
    case 0x4090: return (uint8_t)((bus & 0xc0) | a->vol_gain);
    case 0x4091: return (uint8_t)(a->wave_acc >> 12);
    case 0x4092: return (uint8_t)((bus & 0xc0) | a->mod_gain);
    case 0x4093: return (uint8_t)((a->mod_acc >> 5) & 0x7f);
    case 0x4097: return (uint8_t)(a->mod_counter & 0x7f);
    default: return bus;
    }
}

uint8_t fds_audio_read(Nes *nes, uint16_t address, uint8_t bus)
{
    nes_expansion_audio_sync(nes, nes->cpu.cycles);
    return read_register(&nes->cart.ext->fds->audio, address, bus);
}

void fds_audio_write(Nes *nes, uint16_t address, uint8_t value)
{
    FdsAudio *a = &nes->cart.ext->fds->audio;

    nes_expansion_audio_sync(nes, nes->cpu.cycles);
    if (address < 0x4080) {
        if (a->wave_write)
            a->wave[address & 63] = value & 63;
        return;
    }
    switch (address) {
    case 0x4080:
        if (!a->enabled)
            break;
        a->vol_control = value;
        a->vol_timer = envelope_period(a, value);

        if (value & 0x80) {
            a->vol_gain = value & 63;
            latch_gain(a);
            if (!a->vol_gain)
                a->out_gain = 0;
        }
        break;
    case 0x4082:
        a->wave_freq = (uint16_t)((a->wave_freq & 0xf00) | value);
        update_pitch(a);
        break;
    case 0x4083:
        a->wave_freq = (uint16_t)((a->wave_freq & 0xff) | (value & 15) << 8);
        a->env_halt = (value & 0x40) != 0;
        a->wave_halt = (value & 0x80) != 0;

        if (a->env_halt)
            fds_audio_reset_timers(a);
        if (a->wave_halt) {
            a->wave_acc = 0;
            latch_gain(a);
        }
        update_pitch(a);
        break;
    case 0x4084:
        a->mod_control = value;
        a->mod_timer = envelope_period(a, value);

        if (value & 0x80)
            a->mod_gain = value & 63;
        update_pitch(a);
        break;
    case 0x4085:
        if (!a->enabled)
            break;
        a->mod_counter = (int8_t)(((value & 0x7f) ^ 0x40) - 0x40);
        update_pitch(a);
        break;
    case 0x4086:
        a->mod_freq = (uint16_t)((a->mod_freq & 0xf00) | value);
        break;
    case 0x4087:
        a->mod_freq = (uint16_t)((a->mod_freq & 0xff) | (value & 15) << 8);
        a->mod_step = (value & 0x40) != 0;
        a->mod_halt = (value & 0x80) != 0;

        if (a->mod_halt)
            a->mod_acc &= ~0x1fffu;
        break;
    case 0x4088:
        if (a->mod_halt) {
            a->mod_table[(a->mod_acc >> 13) & 31] = value & 7;
            a->mod_acc = (a->mod_acc + 0x2000) & 0x3ffff;
        }
        break;
    case 0x4089:
        a->master_volume = value & 3;
        a->wave_write = (value & 0x80) != 0;
        break;
    case 0x408a:
        a->master_speed = value;
        break;
    default:
        return;
    }
    update_level(a);
}
