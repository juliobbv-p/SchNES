#ifndef NES_H
#define NES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Use static inline for ordinary helpers. Force inlining for constant
// specialization or hot paths, and keep large specialized loops out of callers.
#if defined(_MSC_VER)
#define NES_ALWAYS_INLINE __forceinline
#define NES_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NES_ALWAYS_INLINE inline __attribute__((always_inline))
#define NES_NOINLINE __attribute__((noinline))
#else
#define NES_ALWAYS_INLINE inline
#define NES_NOINLINE
#endif

enum {
    NES_WIDTH = 256, NES_HEIGHT = 240,
    NES_PPU_DOTS_PER_CPU = 3, NES_DOTS_PER_SCANLINE = 341, NES_SCANLINES_PER_FRAME = 262,
    NES_VBLANK_LINE = NES_HEIGHT + 1,
    NES_EXTRA_BLANKING_LINES = 100,
    NES_MAX_SCANLINES_PER_FRAME = NES_SCANLINES_PER_FRAME + NES_EXTRA_BLANKING_LINES,
    // Music engines write the DMC level a few times per frame. Raw PCM streams
    // write it hundreds of times, timed by the CPU clock.
    NES_PCM_STREAM_WRITES = 32,
    // The NTSC odd-frame skip removes one dot every two rendered frames
    NES_NTSC_TWO_FRAME_DOTS = 2 * NES_DOTS_PER_SCANLINE * NES_SCANLINES_PER_FRAME - 1,
    PPU_BACKGROUND_PREFETCH_DOTS = 16,
    PPU_EARLY_DRAW_DOT = 88, PPU_SCROLL_BATCH_LINES = 16,
    NES_CPU_HZ = 1789773, NES_SAMPLE_RATE = 48000,
    NES_AUDIO_CAPACITY = 1024
};

typedef enum {
    MIRROR_HORIZONTAL, MIRROR_VERTICAL, MIRROR_SINGLE_LOW,
    MIRROR_SINGLE_HIGH, MIRROR_FOUR
} Mirroring;

typedef struct CpuTraceCache CpuTraceCache;

typedef struct {
    uint8_t prg_mode, prg[5], ram_slots, ram_banks[4], ram_protect[2];
    uint8_t chr_mode, chr_upper;
    uint16_t chr[12];
    bool last_chr_bg;
    const uint8_t *bg_pages[8];
    const uint64_t *decoded_bg_pages[8];
    uint64_t bg_tags[2];
    uint8_t nametable_map, exram_mode, exram[1024], fill[1024];
    uint8_t irq_target, scanline, multiplier[2];
    bool in_frame, irq_enabled, irq_pending;
} Mmc5;

// Expansion high-pass state in Q30 and low-pass state in Q29
typedef struct {
    int32_t dc, low;
} ExpansionFilter;

typedef struct {
    uint64_t edge;             // FDS: APU cycle when the DAC input last changed
    uint32_t target;           // Current Q24 input level
    union {
        uint32_t start;        // FDS: filtered Q24 level at the edge
        int32_t edge_area;     // Pulse and saw: correction for the next tone window
    };
} ExpansionChannel;

// The chips supply channel levels on the APU clock. The DAC shapes each
// channel before sampling, then filters the sum on the PCM clock.
typedef struct {
    uint64_t cycle;
    unsigned phase, count;
    bool point_sampling;
    ExpansionChannel channel[3];
    ExpansionFilter filter;
    int32_t samples[NES_AUDIO_CAPACITY]; // Q27 before the final PCM store
} ExpansionMix;

// Sunsoft 5B audio: a YM2149-style PSG with three square channels, noise and envelope
typedef struct {
    uint8_t regs[16], select;
    uint8_t divider;          // CPU cycles into the current 16-cycle PSG tick
    uint8_t tone_out;         // One output bit per channel
    uint8_t noise_divider;    // The noise generator advances every second tick
    uint8_t envelope_step;    // Counts down from 31 within the current ramp
    uint8_t envelope_mask;    // XOR applied to the step: $1F while ramping up
    bool envelope_holding, noise_out;
    uint16_t tone_counter[3], noise_counter, envelope_counter;
    uint32_t lfsr;
    unsigned point_output;     // Combined PSG level in Q24 for point sampling
    // PSG ticks so far. Between output changes the counters advance in one
    // step: they are current at state_tick, and event_tick is the next tick
    // that can change the output (UINT64_MAX if none).
    uint64_t ticks, state_tick, event_tick;
    ExpansionMix mix;
} Sunsoft5b;

typedef struct {
    uint8_t volume, duty, step; // The duty step counts down from 15
    bool ignore_duty, enabled;
    uint16_t period, timer;
} Vrc6Pulse;

typedef struct {
    uint8_t rate, step, accumulator;
    bool enabled;
    uint16_t period, timer;
} Vrc6Saw;

typedef struct {
    Vrc6Pulse pulse[2];
    Vrc6Saw saw;
    uint8_t control;          // $9003
    ExpansionMix mix;
} Vrc6Audio;

// State for mappers that need more than the shared register fields: Konami
// VRC2/VRC4/VRC6, Sunsoft FME-7/5B and the FDS. Runahead copies this block with the cartridge.
typedef struct {
    // CPU-cycle IRQ. The counter state is valid at irq_base, and irq_cycle
    // is the first cycle at which the IRQ line asserts (UINT64_MAX if never).
    uint64_t irq_base, irq_cycle;
    uint16_t irq_counter;
    int16_t irq_prescaler;    // VRC scanline prescaler, 341 down by 3 per CPU cycle
    uint8_t irq_latch, irq_control;
    uint8_t prg[4], command;
    uint16_t chr[8];
    // VRC register address lines: A0 and A1 come from these CPU address bits
    uint8_t vrc_a0, vrc_a1;
    bool vrc2, vrc_chr_shift, vrc_swap, vrc_latch;
    bool has_audio;
    struct Fds *fds;
    uint8_t vrc6_banking;     // VRC6 $B003
    // FME-7 banks its RAM at $6000 and keeps Cartridge.ram_size at zero, so the
    // fast paths that assume plain RAM there never treat it as such.
    size_t banked_ram_size;
    // FME-7: the PRG ROM page mapped at $6000, or NULL while RAM or open bus is there
    const uint8_t *wram_rom;
    union {
        Sunsoft5b audio;
        Vrc6Audio vrc6;
    };
} MapperExt;

typedef struct {
    uint8_t wave[64], mod_table[32];
    uint8_t vol_control, mod_control;   // $4080 and $4084
    uint8_t vol_gain, mod_gain, out_gain;
    uint8_t master_speed, master_volume, divider;
    int8_t mod_counter;
    bool wave_write, wave_halt, env_halt, mod_halt, mod_step, enabled;
    uint16_t wave_freq, mod_freq;
    uint32_t wave_acc, mod_acc;         // 24-bit wave and 18-bit mod accumulators
    uint32_t vol_timer, mod_timer;
    uint32_t wave_pitch;
    uint16_t level;                     // Wave sample times gain at the last update
    int32_t point_smooth;               // Adapter low-pass for point sampling, in Q24
    ExpansionMix mix;
} FdsAudio;

enum { FDS_NO_DISK = 0xff, FDS_LOOP_REGION_SHIFT = 7 };

// Famicom Disk System RAM adapter and drive
typedef struct Fds {
    // One bit per PRG RAM byte used by a native trace
    // Prediction clears its pointer after rewriting a traced byte
    uint8_t *code_map;
    uint8_t *disk;                      // All sides back to back
    const uint8_t (*ids)[10];           // Disk IDs as CheckDiskHeader compares them
    uint32_t side_size, position;
    uint64_t disk_writes;
    uint8_t sides, side;
    uint8_t insert_side, notice_side;
    bool insert_auto, announce_pending, notice_missing, request_missing;
    uint32_t notices;
    uint64_t insert_cycle;
    uint16_t timer_reload, timer_counter;
    uint64_t timer_base;
    bool timer_enabled, timer_repeat, timer_irq, disk_irq;
    uint8_t io_enable, control, ext_out, write_data, read_data;
    bool transfer_flag, bad_crc, at_end, head_home, scanning, gap_ended, previous_crc_control;
    bool eject_armed;
    uint16_t poll_frames;
    uint64_t poll_frame;
    uint8_t byte_thirds;
    uint16_t crc;
    uint32_t spin_left;
    uint64_t disk_event;                // Next byte transfer
    uint64_t irq_cycle;
    // One bit per 128-byte RAM region used by a decoded loop
    uint8_t loop_map[0x8000 >> (FDS_LOOP_REGION_SHIFT + 3)];
    FdsAudio audio;
} Fds;

typedef struct {
    uint8_t *prg, *chr, *ram;
    const uint8_t *prg_pages[4], *chr_pages[8];
    uint64_t chr_tags[2], chr_revision;
    size_t prg_size, chr_size, ram_size, nvram_size;
    uint32_t nvram_offset, chr_nvram_size;
    MapperExt *ext;
    char *save_path;
    unsigned mapper, submapper;
    Mirroring mirroring;
    bool chr_is_ram, bus_conflicts, save_dirty, traces_unavailable;
    uint8_t bank;
    // MMC3 family: the MC-ACC prescaler count and the iNES MMC6 detection state
    uint8_t mcacc_count, mmc3_ram_mode;
    // Mapper 10 (MMC4) uses the MMC2 latch path and reports mapper 9 internally
    bool mmc4;
    // $6000-$7FFF needs mapper-specific decoding (MMC6, VRC, FME-7)
    bool special_wram;
    // PRG and CHR page selection lives outside the original mappers' code
    bool extended_banking;
    bool mmc3_reload_blocked;
    struct {
        uint8_t shift, control, chr0, chr1, prg;
        uint64_t last_write;
    } mmc1;
    struct {
        uint8_t select, banks[8], ram_control, irq_latch, irq_counter;
        bool irq_reload, irq_enabled, irq, a12;
        uint64_t a12_low;
        uint64_t irq_ready_cycle; // CPU sample cycle for a rendered edge
    } mmc3;
    CpuTraceCache *trace_cache;
    Mmc5 *mmc5;
    uint64_t *decoded_chr;
    const uint64_t *decoded_chr_pages[8];
    uint64_t changed_chr_tiles[8];
    bool linear_chr_ram;

    struct {
        uint8_t chr[4], latches;
    } mmc2;
    bool banked_chr_ram, chr_rom_tile_cache;
} Cartridge;

typedef struct {
    uint16_t pc;
    uint8_t a, x, y, s, p;
    uint64_t cycles;
    bool jammed, nmi_line, nmi_edge, nmi, nmi_sample, irq, irq_sample;
} Cpu;

typedef struct {
    uint64_t nametable_revision, chr_revision, chr_tag;
    uint32_t scroll;
} BackgroundRowKey;

typedef struct {
    uint64_t setup, nametable_revision;
} Mmc2RowKey;

#if defined(__APPLE__) && defined(__aarch64__) && defined(__ARM_NEON)
// Darwin heap allocations provide 16-byte alignment for the NEON row buffers
#define NES_ALIGN_RASTER_ROW _Alignas(16)
#else
#define NES_ALIGN_RASTER_ROW
#endif

typedef struct {
    uint64_t revision;
    uint8_t bytes[32], mode;
} PpuPaletteState;

typedef struct {
    const uint8_t *chr;
    uint64_t tag;
    uint8_t row, names[32], attributes[8], indices[NES_WIDTH];
} PpuRemappedRow;

_Static_assert(sizeof(PpuPaletteState[2]) + sizeof(PpuRemappedRow) <= sizeof(uint64_t[2][32]),
               "Two-screen caches fit in the first bank's two unused nametable tag slots");

typedef struct {
    uint16_t pattern[2];
    uint8_t attributes[2], palette;
} PpuBackgroundShifters;

// The sprites one line fetches from secondary OAM, kept only when they can
// differ from the usual evaluation of primary OAM. ppu.c steps the
// evaluation that fills secondary OAM as far as the raster has gone.
typedef struct {
    uint8_t secondary[32];
    uint8_t line;      // Line that draws these sprites, or 0xff
    uint8_t compare;   // Low byte of the line that evaluates and fetches them
    uint8_t tall;      // Slots whose fetch saw 8x16 sprites
    uint8_t settled;   // Slots whose fetch already sampled the sprite height
    uint8_t loaded;    // Slots fetched with rendering enabled
    uint8_t address, slot, step; // Primary and secondary OAM addresses, byte of the sprite being copied
    uint16_t dot;      // Next even dot of the evaluation, past 256 once it is complete
    uint16_t overflow; // Dot on which the evaluation set the overflow flag, or zero
    bool zero, full, ninth, wrapped;
} PpuSpriteLine;

typedef struct {
    uint8_t ctrl, mask, status, oam_addr, bus, read_buffer;
    uint8_t render_mask;
    uint8_t overflow_eval_line; // First visible line whose sprite evaluation can still set overflow
    uint16_t v, t;
    uint8_t fine_x;
    bool second_write, suppress_vblank, palette_dirty;
    unsigned scanline, dot, next_dot, hit_dot;
    uint64_t ticks, frames, mask_tick;
    uint64_t bus_low_until, bus_bit5_until, bus_high_until;
    bool odd, no_sprite_limit;
    // No-Lag mode adds extra_blanking_lines to each frame unless the previous
    // frame streamed PCM. blanking_lines holds the lines added to this frame.
    uint8_t extra_blanking_lines, blanking_lines;
    uint8_t oam[256], palette[32], nametable[4096];
    uint32_t oam_revision;
    uint8_t oam_corruption_row; // Pending OAM row corruption from disabling rendering, or zero
    uint64_t overflow_tick; // When a read sees the next pending overflow, or zero to reevaluate
    // Overflow set dot halves for lines with eight or more sprites in range, 0xff if unknown
    uint32_t overflow_memo_key;
    uint8_t overflow_memo[NES_HEIGHT];
    // Sprites in range per row plus line 239's evaluation in the extra overflow row
    uint64_t sprite_rows[2][NES_HEIGHT + 1];
    PpuBackgroundShifters blank_shifters;
    // Each pattern table keeps its decoded rows across mid-scanline bank changes
    NES_ALIGN_RASTER_ROW uint8_t background[2][4][256][NES_WIDTH];
    union {
        uint64_t background_tags[2][4][32];
        struct {
            // Two-screen cartridges use the extra nametable slots for renderer caches
            uint64_t first_nametable_tags[2][32];
            PpuPaletteState palette_cache[2];
            PpuRemappedRow remapped_row;
        };
        struct {
            Mmc2RowKey mmc2_background[NES_HEIGHT];
            Mmc2RowKey mmc2_output[NES_HEIGHT];
        };
    };
    uint64_t background_revision;
    // Track used CHR pages for decoded ROM rows and valid fine-Y lines for other rows
    uint8_t background_valid[2][4][32];
    uint8_t background_blank[2][4][32]; // Fine-Y lines of decoded rows whose tiles are all transparent
    uint8_t scan_low[32], scan_high[32]; // Output bytes for each palette index in the current mode
    bool rgb565; // Ultrafast stores the picture as RGB565
    uint64_t palette_revision, row_palette[NES_HEIGHT];
    // The picture holds the 6-bit NES color in bits 0-5 and the PPUMASK emphasis
    // bits in 6-8, or an RGB565 color when rgb565 is set. Frontends can convert
    // either format with ppu_frame_to_argb.
    NES_ALIGN_RASTER_ROW uint16_t frame[NES_HEIGHT][NES_WIDTH];
    uint64_t nametable_revision;
    BackgroundRowKey rendered_rows[NES_HEIGHT];
    uint64_t early_draw_tick;
    uint8_t mmc2_line_latches;
    uint8_t blank_line, resume_line, resume_fine_x;
    uint32_t chr_tile_rows[256][4];
    uint64_t nametable_row_revision[4][32];
    uint64_t chr_row_revision[2][4][32];
    uint64_t status_deadline; // Cached first event that can change a visible status read
    uint64_t cached_sprite_rows[(NES_HEIGHT + 63) / 64];
    uint32_t cached_oam_revision;
    uint8_t cached_oam[256];
    uint32_t rendered_groups; // Valid eight-line background groups in the framebuffer
    uint8_t deferred_scroll_count;
    uint16_t deferred_scroll[PPU_SCROLL_BATCH_LINES]; // Fine X and horizontal reload for each pending line
    uint16_t resume_dot;
    uint32_t prefetch_since; // Low bits of the most recent prefetch boundary
    // Lines in this frame that draw from an exact evaluation, indexed by line
    // parity. sprite_exact is set while any of them may still be drawn.
    PpuSpriteLine sprite_lines[2];
    bool sprite_exact;
    // Secondary OAM as rendering last left it. The pre-render line fetches it for line 0.
    // When secondary_pending is set, it still has to be filled from line 239's evaluation.
    uint8_t secondary_oam[32];
    bool secondary_zero, secondary_pending, line0_sprites;
    uint8_t eval_line; // First visible line whose sprite evaluation ran since rendering started
    uint16_t eval_enable_dot; // Dot that started rendering on the line before eval_line
    uint64_t exact_row_signature; // Sprites of the last exact line drawn with the composited row cache
} Ppu;

#undef NES_ALIGN_RASTER_ROW

static inline unsigned ppu_frame_scanlines(const Ppu *p)
{
    return NES_SCANLINES_PER_FRAME + p->blanking_lines;
}

static inline unsigned ppu_pre_render_line(const Ppu *p)
{
    return ppu_frame_scanlines(p) - 1;
}

static inline unsigned ppu_vblank_line(const Ppu *p)
{
    return NES_VBLANK_LINE + p->blanking_lines;
}

typedef struct {
    uint8_t period, divider, decay;
    bool constant, loop, start;
} Envelope;

typedef struct {
    uint16_t period, timer;
    uint8_t duty, step, length, sweep_period, sweep_shift, sweep_timer;
    bool sweep_enabled, sweep_negate, sweep_reload;
    Envelope envelope;
} Pulse;

typedef struct {
    uint16_t period, timer;
    uint8_t step, length, linear, reload_value;
    bool control, reload;
} Triangle;

typedef struct {
    uint16_t period, timer, shift;
    uint8_t length;
    bool mode;
    Envelope envelope;
} Noise;

typedef struct {
    uint16_t period, timer, start, address, length, remaining;
    uint32_t stream_read_offset;
    uint64_t dma_halt, dma_enable, dma_abort;
    uint8_t level, shift, bits, buffer;
    bool loop, irq_enabled, irq, silence, empty, stop_after_fetch;
    uint8_t queued_count;
    bool multi_byte; // Length of the active sample, before any $4013 writes
    uint32_t queued_bytes;
} Dmc;

static inline uint64_t dmc_fetch_cycle(const Dmc *d)
{
    return d->dma_halt > d->dma_enable ? d->dma_halt : d->dma_enable;
}
typedef struct {
    int32_t dc, low;
} FixedAudioFilter;

typedef struct {
    uint8_t pulse0, pulse1, noise, duty0, duty1;
    uint8_t tone_flags;
} MixerLevels;

typedef struct {
    uint32_t scanlines, regions, region_lines, redraws;
    uint32_t ppu_syncs, apu_syncs;
    uint32_t bus_reschedules; // Full bus deadline recalculations
} NesRasterStats;

typedef struct {
    Pulse pulse[2];
    Triangle triangle;
    Noise noise;
    Dmc dmc;
    unsigned sample_phase;
    uint8_t length_enabled;
    uint8_t length_clocked; // Counters a write already clocked on this cycle's half frame
    bool wide_pcm; // Keep full precision while generating a frame with expansion audio
    uint64_t cycle, cpu_deadline, tone_cycle;
    // CPU-only time preceding the current pause, plus the next pause interval
    uint64_t clock_offset, pause_start;
    unsigned pause_cycles;
    unsigned dac_writes; // $4011 writes since the current frame began
    // Use APU cycles for the frame sequencer and CPU cycles for DMA timestamps
    uint64_t frame_start, frame_event, frame_reset;
    bool frame_mode, frame_next_mode, frame_inhibit, frame_irq, frame_irq_clear, frame_irq_line;
    bool levels_dirty, stream_strobe, ultrafast, enhanced_clock;
    bool blanking; // The current frame has No-Lag lines, as Ppu.blanking_lines
    MixerLevels levels;
    const Cartridge *stream_cart;
    uint64_t stream_read_cycle;
    FixedAudioFilter filter;
    uint32_t sync_count; // UINT32_MAX when raster statistics are disabled
    unsigned sample_count;
    union {
        int16_t samples[NES_AUDIO_CAPACITY];
        int32_t wide_samples[NES_AUDIO_CAPACITY]; // Q27 before adding expansion audio
    };
} Apu;

static inline bool apu_has_independent_clock(const Apu *a)
{
    return a->enhanced_clock || a->clock_offset;
}

// Pause the APU clock and expansion audio during No-Lag's added lines
static inline uint64_t apu_clock_cycle(const Apu *a, uint64_t cpu)
{
    if (a->pause_cycles && cpu > a->pause_start) {
        uint64_t paused = cpu - a->pause_start;
        cpu -= paused < a->pause_cycles ? paused : a->pause_cycles;
    }
    return cpu - a->clock_offset;
}

static inline bool apu_streams_dmc(const Apu *a)
{
    const Dmc *d = &a->dmc;
    const Cartridge *cart = a->stream_cart;

    // Leave the final IRQ-producing fetch and single-byte samples to the CPU
    if (!cart || !d->multi_byte || d->stop_after_fetch ||
        (d->loop && (d->irq_enabled || d->length == 1)) || (d->irq_enabled && d->remaining <= 1))
        return false;
    if (cart->mapper == 20) {
        // The FDS BIOS is ROM, while writes and disk loads can change the RAM below it
        return !d->loop && d->address >= 0xe000 &&
               (unsigned)d->address + d->remaining <= 0x10000;
    }
    return true;
}

// A projected bus guard can move ahead without moving the actual sample fetch
static inline uint64_t apu_stream_audio_cycle(const Apu *a)
{
    return a->stream_read_cycle - a->dmc.stream_read_offset;
}

// A full buffer fixes the next halt time for a one-byte IRQ sample
static inline bool apu_projects_single_dmc_halt(const Apu *a)
{
    const Dmc *d = &a->dmc;

    return a->ultrafast && !d->multi_byte && d->irq_enabled && !d->loop && !d->stop_after_fetch;
}

static inline bool apu_schedules_dmc_halt(const Apu *a)
{
    // Only the final fetch of a nonlooping sample can raise the DMC IRQ
    const Dmc *d = &a->dmc;

    return !a->ultrafast && !a->stream_cart && (!d->irq_enabled || d->loop || d->remaining > 1) &&
           !apu_has_independent_clock(a);
}

enum { CPU_SHORT_LOOP_BACK = 24, CPU_SHORT_LOOP_CACHE = 256 };

typedef enum {
    CPU_EXEC_NONE, CPU_EXEC_RECOMPILED, CPU_EXEC_LOOP_BATCHED, CPU_EXEC_DIRECT,
    CPU_EXEC_INDIVIDUAL, CPU_EXEC_BUS_HANDLER, CPU_EXEC_DMA,
    CPU_EXEC_INTERRUPT_IDLE, CPU_EXEC_TYPES
} NesCpuExecution;

// Hold a full raster of PPU ticks plus room for an instruction or DMA to cross its boundary
enum { NES_CPU_RASTER_TICKS = 1 << 17 };
typedef struct {
    uint64_t first_tick, end_tick, line_zero_tick;
    bool short_pre_render;
    NesCpuExecution current;
    uint8_t execution[NES_CPU_RASTER_TICKS];
} NesCpuRaster;

typedef struct {
    uint8_t op, mode, operand, compare;
    uint8_t target, cycles, instructions, branch_end;
} ShortLoopStep;

typedef struct {
    uintptr_t code_key;
    uint8_t back, cycles, instructions, first_status_read, indexed_reads;
    uint8_t flags, native_slot;
    ShortLoopStep body[CPU_SHORT_LOOP_BACK + 1];
    uint16_t repeat_bus_offset;
} ShortLoop;

typedef struct {
    Cpu cpu;
    uint64_t ppu_deadline, bus_deadline;
    uint8_t open_bus, cpu_bus;
    uint8_t buttons[2], pad_shift[2], pad_select, pad_bit;
    bool pad_strobe, pad_strobe_write, oam_dma, irq_line, ultrafast, interpreter;
    uint8_t dma_page;
    Cartridge cart;
    void (*poll_buttons)(void *context, uint8_t buttons[2]);
    void *input_context;
    uint8_t ram[2048];
    Apu apu;
    Ppu ppu;
    ShortLoop short_loops[CPU_SHORT_LOOP_CACHE];
    NesRasterStats *raster_stats;
    uint64_t pad_resume_deadline;
    NesCpuRaster *cpu_raster;
} Nes;

static inline bool nes_allows_deferred_io(const Nes *nes)
{
    return !nes->interpreter || nes->ultrafast;
}

typedef struct {
    uint32_t individual;
    uint32_t skipped; // Instructions completed by loop and CHR upload batching
    // Instructions completed in the direct loop or with a final shared bus access
    uint32_t direct, bus_finish;
    uint32_t native_trace; // Instructions completed by native block traces
} NesInstructionStats;

bool cart_load(Cartridge *cart, const char *path, char *error, size_t error_size);
// Loader helpers shared with the FDS. cart_fail formats an error and returns false
bool cart_fail(char *error, size_t error_size, const char *format, ...);
char *cart_path_with_suffix(const char *path, const char *suffix);
// cart_load reports this error when an FDS image has no BIOS beside it or SchNES
extern const char cart_fds_bios_missing[];
bool cart_save(Cartridge *cart, char *error, size_t error_size);
void cart_free(Cartridge *cart);
uint8_t cart_cpu_read(const Cartridge *cart, uint16_t address, uint8_t open_bus);
void cart_commit_mmc1_serial(Cartridge *cart, uint16_t address, uint8_t bits, uint64_t cycle);
void cart_cpu_write(Cartridge *cart, uint16_t address, uint8_t value, uint64_t cycle);
void cart_ppu_write(Cartridge *cart, uint16_t address, uint8_t value);
void cart_mmc3_ppu_address(Cartridge *cart, uint16_t address, uint64_t dot);
void cart_mmc3_clock(Cartridge *cart, unsigned count);
// Check for MMC3 at each call site because only it observes the PPU address bus
static inline void cart_ppu_address(Cartridge *cart, uint16_t address, uint64_t dot)
{
    if (cart->mapper == 4)
        cart_mmc3_ppu_address(cart, address, dot);
}
void cart_mmc2_set_latches(Cartridge *cart, uint8_t latches);
// Bring a CPU-cycle IRQ counter (VRC4, FME-7) up to the given CPU cycle
void cart_timer_irq_advance(Cartridge *cart, uint64_t cycle);
static inline bool cart_is_mcacc(const Cartridge *cart)
{
    return cart->mapper == 4 && cart->submapper == 3;
}

static inline void clock_mmc5_scanlines(Mmc5 *m, unsigned lines)
{
    // The first line starts the counter at zero. Then measure the modular
    // distance to the next IRQ target, including an 8-bit counter wrap.
    if (!m->in_frame) {
        m->in_frame = true;
        m->scanline = 0;

        if (!--lines)
            return;
    }
    if ((uint8_t)(m->irq_target - m->scanline - 1) < lines)
        m->irq_pending = true;
    m->scanline += lines;
}

static inline unsigned mmc3_clocks_to_zero(const Cartridge *cart)
{
    unsigned counter = cart->mmc3.irq_counter;
    return counter && !cart->mmc3.irq_reload ? counter : cart->mmc3.irq_latch + 1u;
}

static inline bool mmc3_zero_asserts_irq(const Cartridge *cart)
{
    // The NEC MMC3 ignores automatic reloads from zero to zero
    return cart->mmc3.irq_latch || cart->mmc3.irq_counter || cart->mmc3.irq_reload || cart->submapper != 4;
}

static inline unsigned mmc3_blocked_reload_bits(const Cartridge *cart)
{
    if (!cart->mmc3_reload_blocked)
        return 0;
    // Sharp MMC3 sets bit 7, while NEC MMC3B/C sets all eight bits
    return cart->submapper == 4 ? 0xff : cart->submapper == 0 ? 0x80 : 0;
}

static inline bool mmc5_prg_remap(uint16_t address)
{
    return (address & 0xfffc) == 0x5114;
}

static inline const uint8_t *cart_prg_page(const Cartridge *cart, uint16_t address)
{
    return cart->prg_pages[(address >> 13) - 4];
}

static inline bool cart_unbanked_ram(const Cartridge *c, bool write)
{
    // MMC3 bit 7 enables RAM and bit 6 write-protects it. FDS writes invalidate cached code.
    return c->mapper != 1 && !c->mmc5 && c->ram_size >= 8192 &&
        (c->mapper != 4 || (c->mmc3.ram_control & (write ? 0xc0 : 0x80)) == 0x80) &&
        !(write && c->mapper == 20);
}

// The size of the RAM buffer, including RAM that the mapper banks
static inline size_t cart_ram_bytes(const Cartridge *c)
{
    return c->ram_size ? c->ram_size : c->ext ? c->ext->banked_ram_size : 0;
}

static inline void cart_store_ram(Cartridge *c, size_t offset, uint8_t value)
{
    if (offset >= c->nvram_offset && offset - c->nvram_offset < c->nvram_size)
        c->save_dirty |= c->ram[offset] != value;
    c->ram[offset] = value;
}

static inline uint8_t cart_prg_read(const Cartridge *cart, uint16_t address)
{
    return cart_prg_page(cart, address)[address & 8191];
}

// A set bit selects the $FE bank for the corresponding 4 KiB window
static inline uint8_t cart_mmc2_latches_after_read(uint8_t latches, unsigned address)
{
    // The left latch matches one row, while the right latch matches all eight
    if (address & 0x1000)
        address &= ~7u;
    switch (address) {
    case 0x0fd8: return latches & ~1u;
    case 0x0fe8: return latches | 1u;
    case 0x1fd8: return latches & ~2u;
    case 0x1fe8: return latches | 2u;
    default: return latches;
    }
}

// The MMC4's left latch also matches all eight rows. The renderer passes a
// constant so the MMC2 code stays as it was.
static NES_ALWAYS_INLINE uint8_t cart_mmc24_latches_after_read(uint8_t latches, unsigned address, bool mmc4)
{
    return cart_mmc2_latches_after_read(latches, mmc4 ? address & ~7u : address);
}

static inline uint8_t cart_ppu_read(Cartridge *cart, uint16_t address)
{
    uint8_t value = cart->chr_pages[address >> 10][address & 1023];

    if (cart->mapper == 9)
        cart_mmc2_set_latches(cart, cart->mmc4 ? cart_mmc24_latches_after_read(cart->mmc2.latches, address, true)
                                               : cart_mmc2_latches_after_read(cart->mmc2.latches, address));
    return value;
}

// Four physical nametable indices fit in a byte, from $2000 to $2c00
static inline unsigned cart_nametable_map(const Cartridge *cart)
{
    static const uint8_t packed[] = {
        [MIRROR_HORIZONTAL] = 0x50, [MIRROR_VERTICAL] = 0x44,
        [MIRROR_SINGLE_LOW] = 0x00, [MIRROR_SINGLE_HIGH] = 0x55,
        [MIRROR_FOUR] = 0xe4
    };
    return cart->mmc5 ? cart->mmc5->nametable_map : packed[cart->mirroring];
}

static inline unsigned cart_nametable(const Cartridge *cart, uint16_t address)
{
    unsigned map = cart_nametable_map(cart);
    return (((map >> ((address >> 9) & 6)) & 3) << 10) | (address & 1023);
}

void cpu_reset(Nes *nes);
void cpu_discard_traces(Cartridge *cart);
// Returns true only when a guest opcode executes
bool cpu_step(Nes *nes);
// Run to the next frame boundary, optionally collecting instruction counts
void cpu_run_frame(Nes *nes);
// Regular and Ultrafast fall back to the interpreter loop without compiled code
bool cpu_recompiler_unavailable(const Nes *nes);
// Counts instructions into stats when given, and records into nes->cpu_raster when attached
void cpu_run_debug_frame(Nes *nes, NesInstructionStats *stats);
void cpu_record_execution(NesCpuRaster *raster, uint64_t cycle, NesCpuExecution execution);
// Called after taking a back edge within one ROM page, including any dummy read
void cpu_batch_short_loop(Nes *nes, const uint8_t *code, unsigned back, NesInstructionStats *stats);
uint8_t cpu_read_slow(Nes *nes, uint16_t address);
// Complete a deadline-safe read after quiet accesses supplied the prefix
bool nes_read_quiet(Nes *nes, uint16_t address, uint64_t cycle, uint8_t prior_bus, uint8_t *value);
static inline bool nes_apu_open_bus(uint16_t address)
{
    return address >= 0x4000 && address <= 0x4014;
}
// Try an I/O write at a projected CPU cycle without advancing either clock
bool nes_write_quiet(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle);
// The same for VRC and FME-7 registers at $8000 and up, except that
// writes that change the PRG mapping return false.
bool nes_write_quiet_fixed_prg(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle);
// The same for a store that compiled code knows goes to PPUADDR, PPUDATA or
// an APU waveform register.
bool nes_write_quiet_ppu_address(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle);
bool nes_write_quiet_ppu_data(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle);
bool nes_write_quiet_waveform(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle);
void nes_write_mmc1_serial(Nes *nes, uint16_t address, uint8_t bits, uint64_t cycle);
uint64_t nes_dmc_trace_deadline(const Nes *nes, uint64_t halt, uint64_t end);
bool nes_dmc_queue_trace(Nes *nes, uint64_t halt, uint64_t raster_limit);
void nes_dmc_finish_trace(Nes *nes, uint64_t halt, uint64_t raster_limit);
void cpu_write_slow(Nes *nes, uint16_t address, uint8_t value);

enum { CPU_CARRY = 1, CPU_ZERO = 2, CPU_OVERFLOW = 64, CPU_NEGATIVE = 128 };

static inline uint8_t cpu_nz_flags(uint8_t flags, uint8_t value)
{
    return (flags & ~(CPU_NEGATIVE | CPU_ZERO)) |
        (value & CPU_NEGATIVE) | (value == 0 ? CPU_ZERO : 0);
}

static inline uint8_t cpu_compare_flags(uint8_t flags, uint8_t reg, uint8_t value)
{
    flags = (flags & ~CPU_CARRY) | (reg >= value ? CPU_CARRY : 0);
    return cpu_nz_flags(flags, (uint8_t)(reg - value));
}

static inline uint8_t cpu_bit_flags(uint8_t flags, uint8_t accumulator, uint8_t value)
{
    const unsigned mask = CPU_NEGATIVE | CPU_OVERFLOW;
    return (flags & ~(mask | CPU_ZERO)) | (value & mask) | ((accumulator & value) ? 0 : CPU_ZERO);
}

static inline bool cpu_is_branch(uint8_t opcode)
{
    return (opcode & 0x1f) == 0x10;
}

static inline uint8_t cpu_branch_mask(uint8_t opcode)
{
    // The high two opcode bits select N, V, C or Z
    static const uint8_t masks[] = {CPU_NEGATIVE, CPU_OVERFLOW, CPU_CARRY, CPU_ZERO};

    return masks[opcode >> 6];
}

static inline bool cpu_branch_taken(uint8_t flags, uint8_t opcode)
{
    return ((flags & cpu_branch_mask(opcode)) != 0) == ((opcode & 0x20) != 0);
}

static inline bool cpu_irq_line(const Nes *nes)
{
    const Mmc5 *mmc5 = nes->cart.mmc5;
    return nes->apu.frame_irq_line | nes->apu.dmc.irq |
           (nes->cart.mmc3.irq && nes->cpu.cycles >= nes->cart.mmc3.irq_ready_cycle) |
           (mmc5 && mmc5->irq_enabled && mmc5->irq_pending);
}

static inline void cpu_sample_interrupts(Nes *nes, bool irq)
{
    nes->cpu.nmi_sample = nes->cpu.nmi_edge;
    nes->cpu.irq_sample = irq;
}

static inline void cpu_drive_bus(Nes *nes, uint8_t value)
{
    nes->open_bus = nes->cpu_bus = value;
    nes->pad_select = 0;
    ++nes->cpu.cycles;
}

static NES_ALWAYS_INLINE uint8_t cpu_read(Nes *nes, uint16_t address)
{
    if (nes->cpu.cycles + 1 < nes->bus_deadline) {
        uint8_t value;

        if (address >= 0x8000)
            value = cart_prg_read(&nes->cart, address);
        else if (address < 0x2000)
            value = nes->ram[address & 2047];
        else
            return cpu_read_slow(nes, address);
        cpu_sample_interrupts(nes, nes->irq_line);
        cpu_drive_bus(nes, value);
        return value;
    }
    return cpu_read_slow(nes, address);
}

static inline void cpu_write(Nes *nes, uint16_t address, uint8_t value)
{
    if (address < 0x2000 && nes->cpu.cycles + 1 < nes->bus_deadline) {
        cpu_sample_interrupts(nes, nes->irq_line);
        nes->ram[address & 2047] = value;
        cpu_drive_bus(nes, value);
    } else {
        cpu_write_slow(nes, address, value);
    }
}

void nes_init(Nes *nes);
// Reset the console between complete frame calls
void nes_reset(Nes *nes);
// Apply after initialization, between complete frame calls
void nes_set_enhanced(Nes *nes, bool enabled);
// A completed frame leaves signed 16-bit PCM in apu.samples
void nes_frame(Nes *nes);
// Report the guest instructions handled during this frame
void nes_frame_with_stats(Nes *nes, NesInstructionStats *stats);
// Report raster work and device synchronization calls during this frame
void nes_frame_with_raster_stats(Nes *nes, NesRasterStats *stats);
// Start with a zeroed raster and reuse it for consecutive frames
void nes_frame_with_cpu_raster(Nes *nes, NesCpuRaster *raster);

typedef struct NesRunahead NesRunahead;
NesRunahead *nes_runahead_create(const Cartridge *cart);
void nes_runahead_destroy(NesRunahead *runahead);
// Zero frames returns the live picture. NULL buttons use the live input source,
// holding its first sample when predicting more than one frame.
const uint16_t (*nes_runahead_frame(NesRunahead *runahead, const Nes *nes,
                                    unsigned frames, const uint8_t buttons[2]))[NES_WIDTH];

// ARGB for each emphasis setting and NES color
extern const uint32_t ppu_colors[8][64];
// Expand two CHR planes into eight 2-bit pixel indices, one per byte from the left
extern const uint64_t chr_plane_bits[256];
static inline uint64_t chr_decode_row(const uint8_t *row)
{
    return chr_plane_bits[row[0]] | (chr_plane_bits[row[8]] << 1);
}

void ppu_init(Ppu *ppu);
void ppu_set_no_sprite_limit(Ppu *ppu, bool enabled);
// Select RGB565 for direct 16-bit scanout between frames
void ppu_set_rgb565(Nes *nes, bool enabled);
void ppu_advance(Nes *nes, unsigned dots);
void ppu_repaint_chr_mapping(Nes *nes, uint64_t tick);
// MC-ACC: after a $C001 write at the current raster position, move this line's IRQ edge
void ppu_mcacc_reset(Nes *nes);
// Convert rows of a picture to ARGB. The picture can come from this PPU or a prediction.
// RGB565 pictures expand each channel by replicating its high bits.
void ppu_frame_to_argb(const Ppu *ppu, const uint16_t (*frame)[NES_WIDTH], unsigned first_line,
                       unsigned lines, uint32_t *argb, size_t pitch);
static inline unsigned ppu_irq_fetch_dot(const Ppu *p)
{
    // Common MMC3 splits use opposite background and sprite pattern banks
    unsigned banks = p->ctrl & 0x18;

    return banks == 0x08 ? 261 : banks == 0x10 ? 325 : 0;
}

static inline uint64_t ppu_cycle_at(const Ppu *p, unsigned line, unsigned dot)
{
    uint64_t ticks = p->ticks - p->dot + (line - p->scanline) * NES_DOTS_PER_SCANLINE + dot;
    return (ticks + NES_PPU_DOTS_PER_CPU - 1) / NES_PPU_DOTS_PER_CPU;
}

uint64_t ppu_irq_cycle(const Nes *nes);
uint64_t ppu_mmc5_irq_cycle(const Nes *nes);
static inline bool ppu_cached_status_supported(const Nes *nes)
{
    return nes->cart.mapper != 5 && nes->cart.mapper != 9;
}

// Called in a visible scanline with vblank clear. Applies pending sprite
// overflow evaluation first.
uint64_t ppu_status_cycle(Nes *nes);
static inline uint64_t ppu_status_read_deadline(Nes *nes)
{
    Ppu *p = &nes->ppu;

    if (!p->status_deadline)
        p->status_deadline = ppu_status_cycle(nes);
    return p->status_deadline;
}
// Called when sprite zero is the only visible status flag that can still change
uint64_t ppu_sprite_zero_cycle(const Nes *nes);
static inline bool ppu_waiting_sprite_zero(const Ppu *p)
{
    // With overflow latched, sprite zero is the only remaining visible status change
    return (p->status & 0xe0) == 0x20 && (p->mask & 0x18) == 0x18 && !p->mask_tick;
}

static inline uint8_t ppu_status_value(const Ppu *p, uint64_t tick)
{
    return (tick < p->bus_low_until ? p->bus & 31 : 0) | (p->status & 0xe0);
}
uint8_t ppu_latch_status_at(Ppu *p, uint64_t tick);
uint8_t ppu_read_register(Nes *nes, uint16_t address);
uint8_t ppu_read_register_at(Nes *nes, uint16_t address, uint64_t tick);
// Preserve the bus timestamp of a deferred PPU write
void ppu_write_register_at(Nes *nes, uint16_t address, uint8_t value, uint64_t tick);
void ppu_write_nametable_at(Nes *nes, uint8_t value, uint16_t next, uint64_t tick);
void ppu_fill_nametable(Nes *nes, uint8_t value, unsigned count, unsigned increment,
                        uint64_t first_tick, uint64_t last_tick);
void ppu_finish_chr_upload(Ppu *p, uint16_t address, uint8_t value, uint64_t tick);
void ppu_copy_oam(Ppu *ppu, const uint8_t *source, unsigned count);

void apu_init(Apu *apu);
void apu_set_clock_pause(Apu *apu, uint64_t cycle, uint64_t start, unsigned clocks);
void apu_advance(Apu *apu, uint64_t cycle);
void apu_advance_before_refill(Apu *apu, uint64_t cycle);
void apu_set_stream_cart(Apu *apu, const Cartridge *cart);
uint8_t apu_read_status(Apu *apu, uint8_t open_bus, uint64_t cycle);
void apu_write(Apu *apu, uint16_t address, uint8_t value, uint64_t cycle);
// Clear $4015 and restart the frame sequencer in its last mode, as the reset line does
void apu_reset(Apu *apu, uint64_t cycle);
// What a write to $4000-$4017 can change for a CPU batch, as WRITE_* flags
// (write_effects.h), if it lands at this CPU cycle.
unsigned apu_write_effects(const Apu *apu, uint16_t address, uint8_t value, uint64_t cycle);
// Apply a control write within the CPU's current bus deadline when possible
bool apu_write_control_quiet(Apu *apu, uint16_t address, uint8_t value, uint64_t cycle);
// Accepts waveform registers $4000 through $400F
void apu_write_waveform(Apu *apu, uint16_t address, uint8_t value, uint64_t cycle);
void apu_dmc_supply(Apu *apu, uint8_t value, uint64_t cycle);
bool apu_queue_dmc_byte(Apu *a, uint8_t value, uint64_t cycle);
void apu_dmc_fetch(Apu *a, uint8_t value, uint64_t get, uint64_t end);

// Sunsoft 5B expansion audio. Render up to a CPU cycle before a register write,
// and mix the frame's samples into the APU output once the APU has finished it.
void nes_expansion_audio_sync(Nes *nes, uint64_t cycle);
void nes_expansion_audio_finish_frame(Nes *nes);
void sunsoft5b_write(Sunsoft5b *s, uint8_t reg, uint8_t value);
// Mapper IRQs driven by CPU cycles report their next assertion through the MMC3
// IRQ deadline. Asserts the IRQ once the CPU has reached it.
uint64_t nes_timer_irq_cycle(Nes *nes);

// The FDS (mapper 20 internally) keeps its state with the mapper extension. It runs
// programs from RAM at $6000-$DFFF, so it compiles only guarded short loops.
static inline bool cart_is_fds(const Cartridge *cart)
{
    return cart->mapper == 20;
}

static inline Fds *cart_fds(const Cartridge *cart)
{
    return cart_is_fds(cart) ? cart->ext->fds : NULL;
}

// VRC6 audio registers $9000-$B002, in VRC6a numbering
void vrc6_audio_write(Vrc6Audio *v, uint16_t address, uint8_t value);
void vrc6_audio_render(Vrc6Audio *v, uint64_t cycle);
void vrc6_audio_render_point(Vrc6Audio *v, uint64_t cycle);

// Famicom Disk System. cart_load looks for the BIOS (disksys.rom) beside the
// image, then in the directories added here.
void cart_add_bios_directory(const char *directory);
bool fds_is_disk_image(const uint8_t start[16]);
bool fds_load(Cartridge *cart, const char *path, char *error, size_t error_size);
void fds_free(Cartridge *cart);
// The save is the header, then the returned number of disk bytes
size_t fds_save_image(const Cartridge *cart, uint8_t header[16], const uint8_t **data);
uint8_t fds_read(Nes *nes, uint16_t address, uint8_t open_bus);
void fds_write(Nes *nes, uint16_t address, uint8_t value);
bool fds_write_quiet(Nes *nes, uint16_t address, uint8_t value);
uint64_t fds_irq_cycle(Nes *nes);
// Between frames, eject the disk and insert a side after about a second if delayed
void fds_change_disk(Nes *nes, unsigned side, bool delay);
unsigned fds_current_side(const Nes *nes);
unsigned fds_side_count(const Nes *nes);
// A count of disk notices for the frontend to poll: each side inserted by hand
// or for the BIOS, and each request for a side the image lacks (missing set).
uint32_t fds_disk_notices(const Nes *nes, unsigned *side, bool *missing);
void fds_audio_reset_timers(FdsAudio *audio);
void fds_audio_enable(FdsAudio *audio, bool enabled);
void fds_audio_render(FdsAudio *audio, uint64_t cycle);
void fds_audio_render_point(FdsAudio *audio, uint64_t cycle);
uint8_t fds_audio_read(Nes *nes, uint16_t address, uint8_t open_bus);
void fds_audio_write(Nes *nes, uint16_t address, uint8_t value);
// Drop decoded loops that include a changed byte of FDS RAM code
void cpu_forget_fds_loops(Nes *nes, uint16_t address);
// Invalidate native execution after a write to a traced FDS RAM byte
void cpu_forget_fds_traces(Cartridge *cart, uint16_t address);
// Handle native quiet RAM stores and route code bytes through the interpreter
bool fds_write_quiet_data(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle);
static inline bool fds_code_byte(const uint8_t *map, uint16_t address)
{
    unsigned offset = address - 0x6000u;
    return map && ((map[offset >> 3] >> (offset & 7)) & 1);
}

static inline bool fds_loop_region(const Fds *fds, unsigned address)
{
    unsigned region = (address - 0x6000u) >> FDS_LOOP_REGION_SHIFT;
    return (fds->loop_map[region >> 3] >> (region & 7)) & 1;
}

#endif
