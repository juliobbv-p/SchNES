#include "nes.h"
#include "apu_fixed.h"

// VRC6 audio. Regular filters each voice before sampling.
// A pulse at volume 15 is 1.5 times a 2A03 pulse, as in Mesen.

enum { ULTRASONIC_PULSE = 4, ULTRASONIC_SAW = 5 };

// One level step (a 4-bit volume step or a saw step) in the mixer's DAC
// units. Levels count sixteenths of a step, in Q16.
#define VRC6_SIXTEENTH_Q16 ((3 * (uint64_t)FIXED_DAC_GAIN * 65536 + 1600) / 3200)

static bool halted(const Vrc6Audio *v)
{
    return v->control & 1;
}

static unsigned divider(const Vrc6Audio *v, uint16_t period)
{
    unsigned shift = (v->control & 4) ? 8 : (v->control & 2) ? 4 : 0;
    return (period >> shift) + 1u;
}

static void advance_pulse(Vrc6Pulse *p, unsigned divide, unsigned cycles)
{
    if (!p->enabled)
        return;
    if (cycles < p->timer) {
        p->timer = (uint16_t)(p->timer - cycles);
        return;
    }
    cycles -= p->timer;
    unsigned steps = 1 + cycles / divide;

    p->timer = (uint16_t)(divide - cycles % divide);
    p->step = (uint8_t)((p->step - steps) & 15);
}

static void advance_saw(Vrc6Saw *s, unsigned divide, unsigned cycles)
{
    if (!s->enabled)
        return;
    if (cycles < s->timer) {
        s->timer = (uint16_t)(s->timer - cycles);
        return;
    }
    cycles -= s->timer;
    unsigned steps = 1 + cycles / divide;

    s->timer = (uint16_t)(divide - cycles % divide);

    if (steps >= 14u - s->step) {
        s->step = (uint8_t)((s->step + steps) % 14);
        s->accumulator = (uint8_t)((s->step >> 1) * s->rate);
        return;
    }
    unsigned adds = (s->step + steps) / 2 - s->step / 2;

    s->step = (uint8_t)(s->step + steps);
    s->accumulator = (uint8_t)(s->accumulator + adds * s->rate);
}

// Channel levels in sixteenths of a step
static unsigned pulse_level(const Vrc6Pulse *p, unsigned divide, bool running)
{
    if (!p->enabled)
        return 0;
    if (p->ignore_duty)
        return p->volume * 16u;
    if (running && divide <= ULTRASONIC_PULSE)
        return p->volume * (p->duty + 1u);
    return p->step <= p->duty ? p->volume * 16u : 0;
}

static unsigned saw_level(const Vrc6Saw *s, unsigned divide, bool running)
{
    if (!s->enabled)
        return 0;
    if (running && divide <= ULTRASONIC_SAW) {
        unsigned sum = 0;

        for (unsigned k = 0; k < 7; ++k)
            sum += (uint8_t)(k * s->rate) >> 3;
        return (sum * 16 + 3) / 7;
    }
    return (s->accumulator >> 3) * 16u;
}

// Cycles until the next audible change
static inline uint64_t pulse_change(const Vrc6Pulse *p, unsigned divider)
{
    if (!p->enabled || p->ignore_duty || !p->volume || divider <= ULTRASONIC_PULSE)
        return UINT64_MAX;
    unsigned expiries = p->step <= p->duty ? p->step + 1u : (unsigned)(p->step - p->duty);

    return p->timer + (uint64_t)(expiries - 1) * divider;
}

static inline uint64_t saw_change(const Vrc6Saw *s, unsigned divider)
{
    if (!s->enabled || divider <= ULTRASONIC_SAW)
        return UINT64_MAX;
    unsigned step = s->step, accumulator = s->accumulator;

    for (unsigned k = 1; k <= 14; ++k) {
        if (step >= 13) {
            step = 0;
            accumulator = 0;
        } else {
            accumulator = (uint8_t)(accumulator + (step & 1) * s->rate);
            ++step;
        }
        if ((accumulator >> 3) != (unsigned)(s->accumulator >> 3))
            return s->timer + (uint64_t)(k - 1) * divider;
    }
    return UINT64_MAX;
}

static inline uint64_t soonest_edge(uint64_t a, uint64_t b, uint64_t c)
{
    uint64_t first = a < b ? a : b;

    return first < c ? first : c;
}

static inline uint64_t next_level_change(const Vrc6Audio *v, const unsigned divides[3])
{
    return soonest_edge(pulse_change(&v->pulse[0], divides[0]), pulse_change(&v->pulse[1], divides[1]),
                        saw_change(&v->saw, divides[2]));
}

static inline uint32_t dac_level(unsigned steps)
{
    return (uint32_t)((steps * VRC6_SIXTEENTH_Q16 + 128) >> 8);
}

static inline bool needs_window(const ExpansionChannel *ch, unsigned divider, bool active)
{
    return divider <= EXPANSION_TONE_MAX_PERIOD + 1 && (active || ch->target || ch->edge_area);
}

static NES_NOINLINE void render_held(Vrc6Audio *v, uint64_t end, const unsigned divides[3])
{
    ExpansionMix *m = &v->mix;
    uint64_t start = m->cycle, length = end - start;
    bool running = !halted(v);
    uint32_t levels[3] = {
        dac_level(pulse_level(&v->pulse[0], divides[0], running)),
        dac_level(pulse_level(&v->pulse[1], divides[1], running)),
        dac_level(saw_level(&v->saw, divides[2], running))
    };
    uint64_t clocked[3] = {0}, edges[3] = {UINT64_MAX, UINT64_MAX, UINT64_MAX};

    m->point_sampling = false;
    if (running) {
        edges[0] = pulse_change(&v->pulse[0], divides[0]);
        edges[1] = pulse_change(&v->pulse[1], divides[1]);
        edges[2] = saw_change(&v->saw, divides[2]);
    }
    for (;;) {
        uint64_t edge = soonest_edge(edges[0], edges[1], edges[2]);

        if (edge >= length) {
            expansion_mix_points(m, end, levels[0] + levels[1] + levels[2]);
            break;
        }
        expansion_mix_points(m, start + edge, levels[0] + levels[1] + levels[2]);

        // The point sampler clocks only voices changing at this edge
        for (unsigned ch = 0; ch < 2; ++ch) {
            if (edges[ch] != edge)
                continue;
            Vrc6Pulse *pulse = &v->pulse[ch];
            bool high = pulse->step > pulse->duty;
            unsigned ticks = high ? pulse->duty + 1u : 15u - pulse->duty;

            pulse->step = high ? pulse->duty : 15;
            pulse->timer = (uint16_t)divides[ch];
            clocked[ch] = edge;
            levels[ch] = high ? dac_level(pulse->volume * 16u) : 0;
            edges[ch] = edge + ticks * divides[ch];
        }
        if (edges[2] == edge) {
            advance_saw(&v->saw, divides[2], (unsigned)(edge - clocked[2]));
            clocked[2] = edge;
            levels[2] = dac_level((v->saw.accumulator >> 3) * 16u);
            uint64_t following = saw_change(&v->saw, divides[2]);

            edges[2] = following == UINT64_MAX ? following : edge + following;
        }
    }
    if (running) {
        advance_pulse(&v->pulse[0], divides[0], (unsigned)(length - clocked[0]));
        advance_pulse(&v->pulse[1], divides[1], (unsigned)(length - clocked[1]));
        advance_saw(&v->saw, divides[2], (unsigned)(length - clocked[2]));
    }
    for (unsigned ch = 0; ch < 3; ++ch)
        m->channel[ch] = (ExpansionChannel){.target = levels[ch]};
}

static NES_ALWAYS_INLINE void render(Vrc6Audio *v, uint64_t cycle, bool point_sampling)
{
    unsigned divides[3] = {divider(v, v->pulse[0].period), divider(v, v->pulse[1].period),
                           divider(v, v->saw.period)};
    bool running = !halted(v);
    ExpansionMix *m = &v->mix;

    if (cycle <= m->cycle)
        return;
    if (!point_sampling && cycle - m->cycle <= UINT32_MAX &&
        !needs_window(&m->channel[0], divides[0], v->pulse[0].enabled && v->pulse[0].volume) &&
        !needs_window(&m->channel[1], divides[1], v->pulse[1].enabled && v->pulse[1].volume) &&
        !needs_window(&m->channel[2], divides[2], v->saw.enabled)) {
        render_held(v, cycle, divides);
        return;
    }
    bool seed_channels = !point_sampling && m->point_sampling;

    m->point_sampling = point_sampling;
    while (m->cycle < cycle) {
        unsigned levels[3] = {
            pulse_level(&v->pulse[0], divides[0], running),
            pulse_level(&v->pulse[1], divides[1], running),
            saw_level(&v->saw, divides[2], running)
        };
        uint64_t event = running ? next_level_change(v, divides) : UINT64_MAX;
        uint64_t left = cycle - m->cycle;
        uint64_t span = left < event ? left : event;

        if (span > UINT32_MAX)
            span = UINT32_MAX;
        if (point_sampling) {
            unsigned total = levels[0] + levels[1] + levels[2];

            expansion_mix_points(m, m->cycle + span, dac_level(total));
        } else {
            for (unsigned ch = 0; ch < 3; ++ch) {
                uint32_t level = dac_level(levels[ch]);
                // The effective timer value is one less than the divider
                ExpansionEdge edge = divides[ch] <= EXPANSION_TONE_MAX_PERIOD + 1 ? EXPANSION_WINDOW : EXPANSION_POINT;

                if (seed_channels)
                    m->channel[ch] = (ExpansionChannel){.target = level};
                else
                    expansion_dac_set(m, ch, level, edge);
            }
            seed_channels = false;
            expansion_dac_render(m, m->cycle + span, false);
        }
        if (running) {
            advance_pulse(&v->pulse[0], divides[0], (unsigned)span);
            advance_pulse(&v->pulse[1], divides[1], (unsigned)span);
            advance_saw(&v->saw, divides[2], (unsigned)span);
        }
    }
}

void vrc6_audio_render(Vrc6Audio *v, uint64_t cycle)
{
    render(v, cycle, false);
}

void vrc6_audio_render_point(Vrc6Audio *v, uint64_t cycle)
{
    render(v, cycle, true);
}

void vrc6_audio_write(Vrc6Audio *v, uint16_t address, uint8_t value)
{
    unsigned reg = address & 3;

    if (address < 0xb000) {
        if (address < 0x9000)
            return;
        if (address < 0xa000 && reg == 3) {
            v->control = value & 7;
            return;
        }
        Vrc6Pulse *p = &v->pulse[address >= 0xa000];

        switch (reg) {
        case 0:
            p->volume = value & 15;
            p->duty = (value >> 4) & 7;
            p->ignore_duty = (value & 0x80) != 0;
            break;
        case 1:
            p->period = (uint16_t)((p->period & 0xf00) | value);
            break;
        case 2:
            p->period = (uint16_t)((p->period & 0xff) | (value & 15) << 8);
            p->enabled = (value & 0x80) != 0;

            if (!p->enabled)
                p->step = 15;
            if (!p->timer)
                p->timer = 1;
            break;
        }
        return;
    }
    Vrc6Saw *s = &v->saw;

    switch (reg) {
    case 0:
        s->rate = value & 63;
        break;
    case 1:
        s->period = (uint16_t)((s->period & 0xf00) | value);
        break;
    case 2:
        s->period = (uint16_t)((s->period & 0xff) | (value & 15) << 8);
        s->enabled = (value & 0x80) != 0;

        if (!s->enabled) {
            s->accumulator = 0;
            s->step = 0;
        }
        if (!s->timer)
            s->timer = 1;
        break;
    }
}

