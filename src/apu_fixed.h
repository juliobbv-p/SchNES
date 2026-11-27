#ifndef SCHNES_APU_FIXED_H
#define SCHNES_APU_FIXED_H

#include "nes.h"

#include <string.h>

enum {
    FIXED_TRIANGLE_WEIGHT = 33, FIXED_NOISE_WEIGHT = 22, FIXED_DMC_WEIGHT = 12,
    // The nonlinear DAC's average over a full ultrasonic triangle period
    FIXED_TRIANGLE_AVERAGE = 241,
    FIXED_HP_SHIFT = 14,
    FIXED_HP_SCALE = 1 << FIXED_HP_SHIFT,
    FIXED_HP_STEP = FIXED_HP_SCALE >> 7, // Approximately 60 Hz at 48 kHz
    FIXED_MIX_SHIFT = 27,
    FIXED_PCM_SHIFT = FIXED_MIX_SHIFT - 15,
    FIXED_LP_SHIFT = FIXED_MIX_SHIFT - 16,
    FIXED_LP_STEP = 1325,
    FIXED_DAC_GAIN = (FIXED_HP_SCALE - FIXED_HP_STEP) << (16 - FIXED_HP_SHIFT)
};

enum {
    TONE_WINDOW_SHIFT = 5, TONE_WINDOW = 1 << TONE_WINDOW_SHIFT,
    EXPANSION_TONE_MAX_PERIOD = 128, EXPANSION_NOISE_MAX_PERIOD = 16
};

static inline int32_t tone_window_average(int32_t area)
{
    return (area + TONE_WINDOW / 2) >> TONE_WINDOW_SHIFT;
}

// Compile-time Q16 DAC curves include the high-pass gain
#define PULSE_DAC(i) ((uint16_t)((UINT64_C(2397) * (i) * FIXED_DAC_GAIN + \
    25 * (8128 + 100 * (i)) / 2) / (25 * (8128 + 100 * (i)))))
static const uint16_t fixed_pulse_mix[31] = {
    PULSE_DAC(0), PULSE_DAC(1), PULSE_DAC(2), PULSE_DAC(3), PULSE_DAC(4), PULSE_DAC(5),
    PULSE_DAC(6), PULSE_DAC(7), PULSE_DAC(8), PULSE_DAC(9), PULSE_DAC(10), PULSE_DAC(11),
    PULSE_DAC(12), PULSE_DAC(13), PULSE_DAC(14), PULSE_DAC(15), PULSE_DAC(16), PULSE_DAC(17),
    PULSE_DAC(18), PULSE_DAC(19), PULSE_DAC(20), PULSE_DAC(21), PULSE_DAC(22), PULSE_DAC(23),
    PULSE_DAC(24), PULSE_DAC(25), PULSE_DAC(26), PULSE_DAC(27), PULSE_DAC(28), PULSE_DAC(29),
    PULSE_DAC(30)
};
#undef PULSE_DAC

#define TND_DENOM(i) (100 * (22638 * FIXED_DMC_WEIGHT + 100 * (i)))
#define TND_DAC(i) ((uint16_t)((UINT64_C(15979) * (i) * FIXED_DAC_GAIN + \
    TND_DENOM(i) / 2) / TND_DENOM(i)))
#define TND_8(i) \
    TND_DAC(i), TND_DAC((i) + 1), TND_DAC((i) + 2), TND_DAC((i) + 3), \
    TND_DAC((i) + 4), TND_DAC((i) + 5), TND_DAC((i) + 6), TND_DAC((i) + 7)
#define TND_64(i) \
    TND_8(i), TND_8((i) + 8), TND_8((i) + 16), TND_8((i) + 24), \
    TND_8((i) + 32), TND_8((i) + 40), TND_8((i) + 48), TND_8((i) + 56)
#define TND_256(i) TND_64(i), TND_64((i) + 64), TND_64((i) + 128), TND_64((i) + 192)
static const uint16_t fixed_tnd_mix[] = {
    TND_256(0), TND_256(256), TND_256(512), TND_256(768), TND_256(1024),
    TND_256(1280), TND_256(1536), TND_256(1792), TND_256(2048),
    TND_8(2304), TND_8(2312), TND_8(2320), TND_8(2328), TND_8(2336), TND_8(2344)
};
#undef TND_256
#undef TND_64
#undef TND_8
#undef TND_DAC
#undef TND_DENOM

_Static_assert(sizeof fixed_tnd_mix / sizeof *fixed_tnd_mix >
               127 * FIXED_DMC_WEIGHT + 15 * (FIXED_NOISE_WEIGHT + FIXED_TRIANGLE_WEIGHT),
               "The fixed-point DAC must cover the full channel range");

static NES_ALWAYS_INLINE int16_t mix_pcm16(int32_t sample)
{
    // Take the rounding bit separately so full-range input cannot overflow
    int32_t rounded = (sample >> FIXED_PCM_SHIFT) + ((sample >> (FIXED_PCM_SHIFT - 1)) & 1);

    return rounded > INT16_MAX ? INT16_MAX : rounded < INT16_MIN ? INT16_MIN : (int16_t)rounded;
}

static NES_ALWAYS_INLINE int32_t filter_fixed_sample(FixedAudioFilter *f, unsigned sample)
{
    int32_t high = (int32_t)sample - (f->dc >> FIXED_HP_SHIFT);

    f->dc += FIXED_HP_STEP * high;
    f->low += FIXED_LP_STEP * (high - (f->low >> FIXED_LP_SHIFT));
    return f->low;
}

enum {
    EXPANSION_DC_SHIFT = 6, // Q30 to Q24
    EXPANSION_LOW_SHIFT = 5, // Q29 to Q24
    EXPANSION_MIX_SHIFT = 29 - FIXED_MIX_SHIFT,
    EXPANSION_LP_STEP_SHIFT = FIXED_LP_SHIFT - EXPANSION_LOW_SHIFT,
    EXPANSION_LP_SCALE = 1 << EXPANSION_LP_STEP_SHIFT
};

_Static_assert(FIXED_HP_SHIFT == 14 && FIXED_HP_STEP == 128, "Expansion DC update assumes a 1/128 coefficient");

// The loudest chip is the 5B at 1.6062 full scale. Use 1.75 for the bounds.
enum { EXPANSION_MAX_INPUT = 7 << 22 };
_Static_assert(((int64_t)EXPANSION_MAX_INPUT + 1) << EXPANSION_DC_SHIFT < INT32_MAX,
               "Expansion DC state must fit in signed 32 bits");
_Static_assert((2 * ((int64_t)EXPANSION_MAX_INPUT + 1) / EXPANSION_LP_SCALE + 1) * FIXED_LP_STEP < INT32_MAX,
               "Expansion LP product must fit in signed 32 bits");

// Filter Q24 DAC input and return a sample on the APU's Q27 mixer scale
static NES_ALWAYS_INLINE int32_t expansion_filter_sample(ExpansionFilter *f, uint32_t input)
{
    int32_t high = (int32_t)input - ((f->dc + (1 << (EXPANSION_DC_SHIFT - 1))) >> EXPANSION_DC_SHIFT);
    int32_t error = high - ((f->low + (1 << (EXPANSION_LOW_SHIFT - 1))) >> EXPANSION_LOW_SHIFT);

    f->dc += (high + 1) >> 1;
    // Round the error to Q18 before multiplying to keep the product in 32 bits
    f->low += FIXED_LP_STEP * ((error + EXPANSION_LP_SCALE / 2) >> EXPANSION_LP_STEP_SHIFT);
    return (f->low + (1 << (EXPANSION_MIX_SHIFT - 1))) >> EXPANSION_MIX_SHIFT;
}

// Renderers keep the filter in a local copy so its state stays in registers
static NES_ALWAYS_INLINE void expansion_mix_store(ExpansionMix *m, ExpansionFilter *f, unsigned *count, uint32_t input)
{
    int32_t sample = expansion_filter_sample(f, input);

    if (*count < NES_AUDIO_CAPACITY)
        m->samples[(*count)++] = sample;
}

// A silent span with no filter tail produces only zeros
static inline void expansion_mix_fill(ExpansionMix *m, uint64_t samples, uint32_t input)
{
    ExpansionFilter filter = m->filter;
    unsigned count = m->count;

    if (!input && !filter.dc && !filter.low) {
        unsigned available = NES_AUDIO_CAPACITY - count;
        unsigned written = samples < available ? (unsigned)samples : available;

        memset(m->samples + count, 0, written * sizeof *m->samples);
        m->count += written;
        return;
    }
    for (uint64_t i = 0; i < samples; ++i)
        expansion_mix_store(m, &filter, &count, input);
    m->count = count;
    m->filter = filter;
}

// Point-sample a span whose unfiltered output is constant
static inline void expansion_mix_points(ExpansionMix *m, uint64_t cycle, uint32_t input)
{
    uint64_t phase = m->phase + (cycle - m->cycle) * NES_SAMPLE_RATE;

    expansion_mix_fill(m, phase / NES_CPU_HZ, input);
    m->cycle = cycle;
    m->phase = (unsigned)(phase % NES_CPU_HZ);
}

// FDS adapter RC decay sampled at sixteenths of a half-life, in Q12
static NES_ALWAYS_INLINE uint32_t expansion_fds_level(const ExpansionChannel *ch, uint64_t cycle)
{
    static const uint16_t decay[16] = {
        4096, 3922, 3756, 3597, 3444, 3298, 3158, 3025,
        2896, 2774, 2656, 2543, 2435, 2332, 2233, 2139
    };
    uint64_t elapsed = cycle - ch->edge;

    if (ch->start == ch->target || elapsed >= 1284)
        return ch->target;
    // The adapter's 2 kHz cutoff gives a half-life of about 99 CPU cycles
    unsigned step = (unsigned)(elapsed * 83) >> 9;

    if (!step)
        return ch->start;
    unsigned factor = decay[step & 15] >> (step >> 4);
    bool falling = ch->start > ch->target;
    unsigned gap = falling ? ch->start - ch->target : ch->target - ch->start;
    unsigned rest = ((gap >> 5) * factor + 64) >> 7;

    return falling ? ch->target + rest : ch->target - rest;
}

static inline unsigned expansion_cycles_to_sample(unsigned phase)
{
    return (NES_CPU_HZ - phase + NES_SAMPLE_RATE - 1) / NES_SAMPLE_RATE;
}

typedef enum { EXPANSION_POINT, EXPANSION_WINDOW, EXPANSION_FDS } ExpansionEdge;

static NES_ALWAYS_INLINE void expansion_dac_set(ExpansionMix *m, unsigned channel, uint32_t level, ExpansionEdge edge)
{
    ExpansionChannel *ch = &m->channel[channel];

    if (edge == EXPANSION_POINT) {
        // A point-sampled gate takes effect immediately, including for any pending sample
        ch->edge_area = 0;
        ch->target = level;
        return;
    }
    if (ch->target == level)
        return;
    if (edge == EXPANSION_FDS) {
        ch->start = expansion_fds_level(ch, m->cycle);
        ch->edge = m->cycle;
    } else {
        unsigned to_sample = expansion_cycles_to_sample(m->phase);

        // Correct the part of the 32-cycle window preceding this edge
        if (to_sample < TONE_WINDOW)
            ch->edge_area += ((int32_t)ch->target - (int32_t)level) * (int32_t)(TONE_WINDOW - to_sample);
    }
    ch->target = level;
}

// The past 32 clocks fit inside one 48 kHz sample interval. Edges already
// seen can only affect the first sample in a span with no further changes.
static NES_ALWAYS_INLINE void expansion_render_edges(ExpansionMix *m, uint64_t samples)
{
    unsigned level = 0;
    int32_t edge_area = 0;

    for (unsigned ch = 0; ch < 3; ++ch) {
        ExpansionChannel *voice = &m->channel[ch];

        level += voice->target;
        edge_area += voice->edge_area;
        voice->edge_area = 0;
    }
    if (edge_area) {
        uint32_t first = (uint32_t)((int32_t)level + tone_window_average(edge_area));

        expansion_mix_store(m, &m->filter, &m->count, first);
        --samples;
    }
    if (samples)
        expansion_mix_fill(m, samples, level);
}

_Static_assert(NES_CPU_HZ / NES_SAMPLE_RATE >= TONE_WINDOW, "Tone windows must fit in a PCM interval");
_Static_assert((int64_t)EXPANSION_MAX_INPUT * TONE_WINDOW < INT32_MAX, "Expansion edge corrections must fit in int32");

// Point-sample the expansion channels after their waveform filters
static NES_ALWAYS_INLINE void expansion_dac_render(ExpansionMix *m, uint64_t cycle, bool fds)
{
    if (cycle <= m->cycle)
        return;
    uint64_t phase = m->phase + (cycle - m->cycle) * NES_SAMPLE_RATE;
    uint64_t samples = phase / NES_CPU_HZ;

    if (samples && !fds) {
        expansion_render_edges(m, samples);
    } else if (samples) {
        const ExpansionChannel *voice = &m->channel[0];
        unsigned step = expansion_cycles_to_sample(m->phase);
        uint64_t point = m->cycle + step;
        unsigned level = expansion_fds_level(voice, point);

        if (level == voice->target) {
            expansion_mix_fill(m, samples, level);
        } else {
            ExpansionFilter filter = m->filter;
            unsigned count = m->count;
            unsigned point_phase = m->phase + step * NES_SAMPLE_RATE - NES_CPU_HZ;

            expansion_mix_store(m, &filter, &count, level);
            for (uint64_t i = 1; i < samples; ++i) {
                step = expansion_cycles_to_sample(point_phase);
                point += step;
                point_phase = point_phase + step * NES_SAMPLE_RATE - NES_CPU_HZ;
                expansion_mix_store(m, &filter, &count, expansion_fds_level(voice, point));
            }
            m->count = count;
            m->filter = filter;
        }
    }
    m->cycle = cycle;
    m->phase = (unsigned)(phase % NES_CPU_HZ);
}

#endif
