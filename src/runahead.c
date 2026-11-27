#include "nes.h"

#include <stdlib.h>
#include <string.h>

struct NesRunahead {
    Nes machine;
    Mmc5 mmc5;
    MapperExt ext;
    Fds fds;
    uint8_t *ram, *chr;
    uint64_t *decoded_chr;
    uint8_t *disk;
    uint64_t disk_writes;
    bool disk_stale;
};

typedef struct {
    Nes *prediction;
    const Nes *live;
} PredictionInput;

static void poll_prediction_once(void *context, uint8_t buttons[2])
{
    PredictionInput *input = context;

    input->live->poll_buttons(input->live->input_context, buttons);
    input->prediction->poll_buttons = NULL;
    input->prediction->input_context = NULL;
}

void nes_runahead_destroy(NesRunahead *runahead)
{
    if (runahead) {
        cpu_discard_traces(&runahead->machine.cart);
        free(runahead->ram);
        free(runahead->chr);
        free(runahead->decoded_chr);
        free(runahead->disk);
        free(runahead);
    }
}

NesRunahead *nes_runahead_create(const Cartridge *cart)
{
    NesRunahead *runahead = calloc(1, sizeof *runahead);

    if (!runahead)
        return NULL;

    size_t ram_size = cart_ram_bytes(cart);
    const Fds *fds = cart_fds(cart);

    if (ram_size)
        runahead->ram = malloc(ram_size);
    if (cart->chr_is_ram) {
        runahead->chr = malloc(cart->chr_size);

        if (cart->decoded_chr)
            runahead->decoded_chr = malloc(cart->chr_size / 2 * sizeof *cart->decoded_chr);
    }
    if (fds) {
        runahead->disk = malloc((size_t)fds->sides * fds->side_size);
        runahead->disk_stale = true;
    }
    if ((ram_size && !runahead->ram) || (fds && !runahead->disk) ||
        (cart->chr_is_ram && (!runahead->chr || (cart->decoded_chr && !runahead->decoded_chr)))) {
        nes_runahead_destroy(runahead);
        return NULL;
    }
    return runahead;
}

static inline bool reuse_prediction_tiles(const Nes *prediction, const Nes *live)
{
    const Cartridge *cart = &live->cart;

    if (cart->chr_is_ram) {
        // FDS has fixed CHR pages. Revision counts can agree after different writes.
        return cart_is_fds(cart) && prediction->cart.prg == cart->prg && prediction->cart.chr &&
               prediction->ppu.background_revision == prediction->cart.chr_revision &&
               live->ppu.background_revision == cart->chr_revision &&
               !memcmp(prediction->cart.chr, cart->chr, cart->chr_size);
    }
    return !cart->mmc5 && cart->mapper != 9 && prediction->cart.chr == cart->chr &&
           prediction->cart.prg == cart->prg && cart->mirroring != MIRROR_FOUR &&
           prediction->cart.mirroring != MIRROR_FOUR;
}

static void invalidate_prediction_tiles(Ppu *prediction, const Ppu *live)
{
    if (!memcmp(prediction->nametable, live->nametable, sizeof prediction->nametable))
        return;

    for (unsigned table = 0; table < 2; ++table) {
        const uint8_t *old = prediction->nametable + table * 1024;
        const uint8_t *current = live->nametable + table * 1024;

        if (!memcmp(old, current, 1024))
            continue;
        for (unsigned group = 0; group < 8; ++group) {
            unsigned attr = 960 + group * 8;
            bool changed_palette = memcmp(old + attr, current + attr, 8) != 0;

            for (unsigned row = group * 4; row < group * 4 + 4; ++row) {
                if (changed_palette || memcmp(old + row * 32, current + row * 32, 32)) {
                    prediction->background_valid[0][table][row] = 0;
                    prediction->background_valid[1][table][row] = 0;
                }
            }
        }
    }
}

// Prediction copies skip PCM samples before copying PPU state
_Static_assert(offsetof(Nes, apu) < offsetof(Nes, ppu), "APU samples must precede the PPU tiles");

static bool copy_prediction_state(Nes *prediction, const Nes *live)
{
    const size_t tile_start = offsetof(Nes, ppu) + offsetof(Ppu, background);
    const bool reuse_tiles = reuse_prediction_tiles(prediction, live);
    const size_t tail = reuse_tiles ? offsetof(Nes, ppu) + offsetof(Ppu, scan_low)
                                   : tile_start + sizeof live->ppu.background;
    const size_t sample_start = offsetof(Nes, apu) + offsetof(Apu, samples);
    const size_t after_samples = sample_start + sizeof live->apu.wide_samples;

    if (reuse_tiles)
        invalidate_prediction_tiles(&prediction->ppu, &live->ppu);

    // A frame starts with an empty PCM buffer
    memcpy(prediction, live, sample_start);
    memcpy((uint8_t *)prediction + after_samples, (const uint8_t *)live + after_samples, tile_start - after_samples);
    memcpy((uint8_t *)prediction + tail, (const uint8_t *)live + tail, sizeof *live - tail);

    if (reuse_tiles) {
        // Palette history shares the storage of the unused nametable tags
        memcpy(prediction->ppu.palette_cache, live->ppu.palette_cache, sizeof live->ppu.palette_cache);
        prediction->ppu.background_revision = live->ppu.background_revision;
    } else if (live->cart.mapper == 9) {
        // MMC2 keeps latch-aware rows in one table
        memcpy(&prediction->ppu.background[0][0], &live->ppu.background[0][0],
               sizeof live->ppu.background[0][0]);
    } else {
        memset(prediction->ppu.background_valid, 0, sizeof prediction->ppu.background_valid);
    }
    return reuse_tiles;
}

// Copy writable cartridge state and share ROM
static void copy_prediction_cart(NesRunahead *runahead, const Cartridge *live_cart, bool reuse_tiles)
{
    Cartridge *cart = &runahead->machine.cart;

    if (runahead->ram) {
        cart->ram = runahead->ram;
        memcpy(cart->ram, live_cart->ram, cart_ram_bytes(live_cart));
    }
    if (live_cart->mmc5) {
        runahead->mmc5 = *live_cart->mmc5;
        cart->mmc5 = &runahead->mmc5;
    }
    if (live_cart->ext) {
        runahead->ext = *live_cart->ext;
        cart->ext = &runahead->ext;
    }
    if (cart_is_fds(live_cart)) {
        // Refresh the disk copy after either machine writes
        const Fds *live_disk = live_cart->ext->fds;

        runahead->fds = *live_disk;
        cart->ext->fds = &runahead->fds;

        for (unsigned i = 0; i < 3; ++i)
            cart->prg_pages[i] = cart->ram + 8192 * (i + 1);
        size_t disk_bytes = (size_t)live_disk->sides * live_disk->side_size;

        if (runahead->disk_stale || runahead->disk_writes != live_disk->disk_writes) {
            memcpy(runahead->disk, live_disk->disk, disk_bytes);
            runahead->disk_writes = live_disk->disk_writes;
            runahead->disk_stale = false;
        }
        runahead->fds.disk = runahead->disk;
    }
    if (runahead->chr) {
        cart->chr = runahead->chr;
        cart->decoded_chr = runahead->decoded_chr;
        if (!reuse_tiles) {
            memcpy(cart->chr, live_cart->chr, cart->chr_size);
            if (cart->decoded_chr)
                memcpy(cart->decoded_chr, live_cart->decoded_chr, cart->chr_size / 2 * sizeof *cart->decoded_chr);
        }
        for (unsigned page = 0; page < 8; ++page) {
            cart->chr_pages[page] = cart->chr + (live_cart->chr_pages[page] - live_cart->chr);

            if (cart->decoded_chr)
                cart->decoded_chr_pages[page] = cart->decoded_chr +
                    (live_cart->decoded_chr_pages[page] - live_cart->decoded_chr);
        }
    }
}

const uint16_t (*nes_runahead_frame(NesRunahead *runahead, const Nes *live,
                                    unsigned frames, const uint8_t buttons[2]))[NES_WIDTH]
{
    if (!frames)
        return live->ppu.frame;

    Nes *prediction = &runahead->machine;
    PredictionInput input = {prediction, live};
    // FDS prediction writes can release borrowed traces without invalidating the live cache
    CpuTraceCache *live_traces = live->cart.trace_cache;

    if (live_traces && prediction->cart.trace_cache)
        cpu_discard_traces(&prediction->cart);
    CpuTraceCache *traces = live_traces ? live_traces : prediction->cart.trace_cache;
    bool traces_unavailable = prediction->cart.traces_unavailable;

    bool reuse_tiles = copy_prediction_state(prediction, live);

    prediction->cart.trace_cache = traces;
    prediction->cart.traces_unavailable = traces_unavailable;
    prediction->cart.save_path = NULL;
    prediction->raster_stats = NULL;
    prediction->cpu_raster = NULL;

    if (buttons) {
        prediction->poll_buttons = NULL;
        memcpy(prediction->buttons, buttons, sizeof prediction->buttons);
    } else if (frames > 1 && live->poll_buttons) {
        // Hold the first controller sample for the rest of the prediction
        prediction->poll_buttons = poll_prediction_once;
        prediction->input_context = &input;
    }
    if (prediction->apu.stream_cart)
        prediction->apu.stream_cart = &prediction->cart;

    copy_prediction_cart(runahead, &live->cart, reuse_tiles);
    for (unsigned i = 0; i < frames; ++i)
        nes_frame(prediction);
    if (cart_is_fds(&live->cart) &&
        prediction->cart.ext->fds->disk_writes != live->cart.ext->fds->disk_writes)
        runahead->disk_stale = true;
    // Keep only traces owned by the prediction
    if (live_traces)
        prediction->cart.trace_cache = NULL;
    prediction->poll_buttons = NULL;
    prediction->input_context = NULL;
    return prediction->ppu.frame;
}
