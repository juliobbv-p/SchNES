#include "nes.h"

#include <assert.h>
#include <string.h>

#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#include <arm_neon.h>
#endif

enum { SPRITE_OVERFLOW = 0x20, SPRITE_ZERO = 0x40, VBLANK = 0x80 };
enum { MMC5_IRQ_DOT = 4 };
// A rendered MMC3 edge becomes visible two dots after the fetch completes
enum { MMC3_FIRST_BG_FETCH = 5, MMC3_IRQ_SAMPLE_OFFSET = 3 };
enum { PPU_BUS_HOLD = 20 * NES_DOTS_PER_SCANLINE * NES_SCANLINES_PER_FRAME };

// The 64 NES colors as 0xRRGGBB, applied to an entry macro with the emphasis bits
#define NTSC_PALETTE(X, e) \
    X(e, 0x626262) X(e, 0x001c95) X(e, 0x1904ac) X(e, 0x42009d) \
    X(e, 0x61006b) X(e, 0x6e0025) X(e, 0x650500) X(e, 0x491e00) \
    X(e, 0x223700) X(e, 0x004900) X(e, 0x004f00) X(e, 0x004816) \
    X(e, 0x00355e) X(e, 0x000000) X(e, 0x000000) X(e, 0x000000) \
    X(e, 0xababab) X(e, 0x0c4edb) X(e, 0x3d2eff) X(e, 0x7115f3) \
    X(e, 0x9b0bb9) X(e, 0xb01262) X(e, 0xa92704) X(e, 0x894600) \
    X(e, 0x576600) X(e, 0x237f00) X(e, 0x008900) X(e, 0x008332) \
    X(e, 0x006d90) X(e, 0x000000) X(e, 0x000000) X(e, 0x000000) \
    X(e, 0xffffff) X(e, 0x57a5ff) X(e, 0x8287ff) X(e, 0xb46dff) \
    X(e, 0xdf60ff) X(e, 0xf863c6) X(e, 0xf57568) X(e, 0xd7921b) \
    X(e, 0xa6b100) X(e, 0x72cb00) X(e, 0x46d639) X(e, 0x2ed382) \
    X(e, 0x34bfcf) X(e, 0x484848) X(e, 0x000000) X(e, 0x000000) \
    X(e, 0xffffff) X(e, 0xb4d5ff) X(e, 0xc6c8ff) X(e, 0xdbc0ff) \
    X(e, 0xf0baff) X(e, 0xfbbbf0) X(e, 0xfcc3c5) X(e, 0xf0d0a1) \
    X(e, 0xd9de90) X(e, 0xc2e991) X(e, 0xaef0a7) X(e, 0xa5efc7) \
    X(e, 0xa8e7e8) X(e, 0xb3b3b3) X(e, 0x000000) X(e, 0x000000)

// Emphasis bits 0-2 select red, green and blue. Each set bit dims the other two
// channels to three quarters.
#define DIM(c) ((c) * 3 / 4)
#define DIMMED(c, n) ((n) == 0 ? (c) : (n) == 1 ? DIM(c) : DIM(DIM(c)))
#define CHANNEL(e, rgb, ch) DIMMED(((rgb) >> (16 - 8 * (ch))) & 255, \
    ((ch) != 0 && ((e) & 1)) + ((ch) != 1 && ((e) & 2)) + ((ch) != 2 && ((e) & 4)))
#define ARGB(e, rgb) (UINT32_C(0xff000000) | (uint32_t)CHANNEL(e, rgb, 0) << 16 | \
    (uint32_t)CHANNEL(e, rgb, 1) << 8 | (uint32_t)CHANNEL(e, rgb, 2)),
#define BLUE(e, rgb) CHANNEL(e, rgb, 2),
#define GREEN(e, rgb) CHANNEL(e, rgb, 1),
#define RED(e, rgb) CHANNEL(e, rgb, 0),
#define EMPHASIS_ROWS(ROW) ROW(0), ROW(1), ROW(2), ROW(3), ROW(4), ROW(5), ROW(6), ROW(7)
#define ARGB_ROW(e) {NTSC_PALETTE(ARGB, e)}
#define PLANE_ROW(e) {{NTSC_PALETTE(BLUE, e)}, {NTSC_PALETTE(GREEN, e)}, {NTSC_PALETTE(RED, e)}}

const uint32_t ppu_colors[8][64] = {EMPHASIS_ROWS(ARGB_ROW)};
// Blue, green, and red bytes of ppu_colors for NEON lookups
static const uint8_t color_planes[8][3][64] = {EMPHASIS_ROWS(PLANE_ROW)};

#undef PLANE_ROW
#undef ARGB_ROW
#undef EMPHASIS_ROWS
#undef RED
#undef GREEN
#undef BLUE
#undef ARGB
#undef CHANNEL
#undef DIMMED
#undef DIM
#undef NTSC_PALETTE

// Byte x of an entry holds bit 7 - x of the index, the order of a CHR plane's pixels
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define PIXEL(b, x) ((uint64_t)(((b) >> (7 - (x))) & 1) << (56 - 8 * (x)))
#else
#define PIXEL(b, x) ((uint64_t)(((b) >> (7 - (x))) & 1) << (8 * (x)))
#endif
#define PLANE_BITS(b) PIXEL(b, 0) | PIXEL(b, 1) | PIXEL(b, 2) | PIXEL(b, 3) | \
    PIXEL(b, 4) | PIXEL(b, 5) | PIXEL(b, 6) | PIXEL(b, 7)
#define PLANE_BITS_4(b) PLANE_BITS(b), PLANE_BITS((b) + 1), PLANE_BITS((b) + 2), PLANE_BITS((b) + 3)
#define PLANE_BITS_16(b) PLANE_BITS_4(b), PLANE_BITS_4((b) + 4), PLANE_BITS_4((b) + 8), PLANE_BITS_4((b) + 12)
#define PLANE_BITS_64(b) PLANE_BITS_16(b), PLANE_BITS_16((b) + 16), PLANE_BITS_16((b) + 32), PLANE_BITS_16((b) + 48)
const uint64_t chr_plane_bits[256] = {
    PLANE_BITS_64(0), PLANE_BITS_64(64), PLANE_BITS_64(128), PLANE_BITS_64(192)
};
#undef PLANE_BITS_64
#undef PLANE_BITS_16
#undef PLANE_BITS_4
#undef PLANE_BITS
#undef PIXEL

void ppu_init(Ppu *p)
{
    p->overflow_memo_key = UINT32_MAX;
    p->next_dot = 1;
    p->prefetch_since = (uint32_t)p->ticks;
    p->palette_dirty = true;
    p->rendered_groups = 0;
    p->deferred_scroll_count = 0;
    p->blank_line = p->resume_line = NES_HEIGHT;
    memset(p->cached_sprite_rows, 0, sizeof p->cached_sprite_rows);
    for (unsigned table = 0; table < 4; ++table)
        p->chr_tile_rows[0][table] = UINT32_MAX;
    memset(p->oam, 0xff, sizeof p->oam);
    memset(p->secondary_oam, 0xff, sizeof p->secondary_oam);
    p->sprite_lines[0].line = p->sprite_lines[1].line = 0xff;
    p->sprite_exact = p->line0_sprites = p->secondary_pending = false;
}

void ppu_set_rgb565(Nes *nes, bool enabled)
{
    Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;

    if (p->rgb565 == enabled)
        return;
    p->rgb565 = enabled;

    // Redraw stored rows and palette stamps after a format change
    for (unsigned line = 0; line < NES_HEIGHT; ++line)
        p->row_palette[line] = UINT64_MAX;
    // The palette history shares storage with other caches. Only two-screen carts use it.
    // No stamp equals UINT64_MAX, so the next refresh rebuilds the lookup tables.
    if (cart->mapper != 9 && !cart->mmc5 && cart->mirroring != MIRROR_FOUR) {
        p->palette_cache[0].revision = p->palette_cache[1].revision = 0;
        p->palette_revision = UINT64_MAX;
    }
    p->palette_dirty = true;
    p->rendered_groups = 0;
}

void ppu_set_no_sprite_limit(Ppu *p, bool enabled)
{
    if (p->no_sprite_limit == enabled)
        return;
    p->no_sprite_limit = enabled;
    memset(p->cached_sprite_rows, 0, sizeof p->cached_sprite_rows);
}

static bool rendering(const Ppu *p)
{
    return (p->mask & 0x18) && (p->scanline < NES_HEIGHT || p->scanline == ppu_pre_render_line(p));
}

static bool fetching_sprites_at(const Ppu *p, unsigned dot)
{
    return dot >= 257 && dot <= 320 && rendering(p);
}

static unsigned palette_offset(uint16_t address)
{
    unsigned index = address & 31;
    return (index & 0x13) == 0x10 ? index & 15 : index;
}

static void repaint_scanline(Nes *nes, unsigned previous_render);
static void repaint_background_bank(Nes *nes);
static void repaint_fine_x(Nes *nes, uint64_t tick);
static void repaint_blank_backdrop(Nes *nes, uint8_t previous, uint64_t tick, unsigned delay);
static inline void latch_sprite_overflow(Ppu *p, uint64_t tick);
static void commit_sprite_lines(Ppu *p);

static inline bool in_visible_pixels(const Ppu *p)
{
    return p->scanline < NES_HEIGHT && p->dot > 1 && p->dot <= NES_WIDTH;
}

static NES_ALWAYS_INLINE void schedule_early_draw_at(Nes *nes, uint64_t tick)
{
    Ppu *p = &nes->ppu;

    if (!nes->ultrafast || p->scanline >= NES_HEIGHT)
        return;
    unsigned dot = p->dot + (unsigned)(tick - p->ticks);

    if (dot <= 1 || dot > PPU_EARLY_DRAW_DOT)
        return;

    // Coalesce early raster writes into one update before the vertical scroll advances
    p->early_draw_tick = tick + PPU_EARLY_DRAW_DOT + 1 - dot;
    uint64_t cycle = (p->early_draw_tick + NES_PPU_DOTS_PER_CPU - 1) / NES_PPU_DOTS_PER_CPU;

    if (cycle < nes->ppu_deadline)
        nes->ppu_deadline = cycle;
}

static uint8_t palette_read(const Ppu *p, uint16_t address)
{
    return p->palette[palette_offset(address)] & ((p->mask & 1) ? 0x30 : 0x3f);
}

static inline uint8_t backdrop_index(const Ppu *p, uint16_t v)
{
    return palette_read(p, (v & 0x3f00) == 0x3f00 ? v : 0);
}

static void move_sprite_rows(uint64_t rows[NES_HEIGHT + 1], uint64_t bit, unsigned height,
                             unsigned first, unsigned second)
{
    // Only rows in one of the two spans change sprite membership
    unsigned first_end = first + height;
    unsigned second_end = second + height;
    unsigned end = first_end < second ? first_end : second;

    for (unsigned row = first; row < end && row <= NES_HEIGHT; ++row)
        rows[row] ^= bit;
    unsigned start = first_end > second ? first_end : second;

    for (unsigned row = start; row < second_end && row <= NES_HEIGHT; ++row)
        rows[row] ^= bit;
}

static void latch_bus(Ppu *p, uint8_t value, uint64_t tick)
{
    p->bus = value;
    p->bus_low_until = p->bus_bit5_until = p->bus_high_until = tick + PPU_BUS_HOLD;
}

void ppu_finish_chr_upload(Ppu *p, uint16_t address, uint8_t value, uint64_t tick)
{
    p->v = address;
    latch_bus(p, value, tick);
}

static void update_sprite_rows(Ppu *p, unsigned offset, unsigned value)
{
    unsigned old = p->oam[offset];
    unsigned first = (old < value ? old : value) + 1;
    unsigned second = (old > value ? old : value) + 1;
    uint64_t bit = UINT64_C(1) << (offset >> 2);

    move_sprite_rows(p->sprite_rows[0], bit, 8, first, second);
    move_sprite_rows(p->sprite_rows[1], bit, 16, first, second);
}

static void write_oam(Ppu *p, uint8_t value)
{
    unsigned offset = p->oam_addr++;

    p->overflow_tick = 0;

    if ((offset & 3) == 2)
        value &= 0xe3;
    else if (!(offset & 3) && p->oam[offset] != value)
        update_sprite_rows(p, offset, value);
    ++p->oam_revision;
    p->oam[offset] = value;
}

void ppu_copy_oam(Ppu *p, const uint8_t *source, unsigned count)
{
    p->status_deadline = 0;

    if (!count)
        return;
    latch_sprite_overflow(p, p->ticks);
    p->overflow_tick = 0;

    if (p->sprite_exact)
        commit_sprite_lines(p);
    ++p->oam_revision;
    const uint8_t *end = source + count;

    while ((p->oam_addr & 3) && source < end)
        write_oam(p, *source++);

    // Copy complete sprites and use only the Y byte for the row cache
    uint8_t offset = p->oam_addr;
    const uint8_t sprite_bytes[] = {0xff, 0xff, 0xe3, 0xff};
    uint32_t sprite_mask;

    memcpy(&sprite_mask, sprite_bytes, sizeof sprite_mask);
    if (!(offset & 4)) {
        uint64_t attribute_mask = (uint64_t)sprite_mask << 32 | sprite_mask;
        const uint8_t y_bytes[] = {0xff, 0, 0, 0, 0xff, 0, 0, 0};
        uint64_t y_mask;

        memcpy(&y_mask, y_bytes, sizeof y_mask);
        while (end - source >= 8) {
            uint64_t sprites, previous;

            memcpy(&sprites, source, sizeof sprites);
            memcpy(&previous, p->oam + offset, sizeof previous);
            if ((sprites ^ previous) & y_mask) {
                if (p->oam[offset] != source[0])
                    update_sprite_rows(p, offset, source[0]);
                if (p->oam[offset + 4] != source[4])
                    update_sprite_rows(p, offset + 4, source[4]);
            }
            sprites &= attribute_mask;
            memcpy(p->oam + offset, &sprites, sizeof sprites);
            offset += 8;
            source += 8;
        }
    }
    while (end - source >= 4) {
        if (p->oam[offset] != source[0])
            update_sprite_rows(p, offset, source[0]);
        uint32_t sprite;

        memcpy(&sprite, source, sizeof sprite);
        sprite &= sprite_mask;
        memcpy(p->oam + offset, &sprite, sizeof sprite);
        offset += 4;
        source += 4;
    }
    p->oam_addr = offset;

    while (source < end)
        write_oam(p, *source++);
    latch_bus(p, end[-1], p->ticks);
}

// The secondary OAM address at a dot of a line with rendering enabled. During
// evaluation it is rounded up to the sprite being copied, and wraps to zero
// once secondary OAM is full.
static unsigned secondary_oam_address(const Ppu *p, unsigned dot)
{
    if (dot >= 1 && dot <= 64)
        return (dot / 2) & 31;
    if (dot >= 65 && dot <= 256) {
        if (p->scanline >= NES_HEIGHT)
            return 0;
        unsigned height = (p->ctrl & 0x20) ? 16 : 8, found = 0;

        for (unsigned n = 0, start = 65; n < 64 && start < dot && found < 8; ++n) {
            bool in_range = p->scanline - p->oam[n * 4] < height;

            found += in_range;
            start += in_range ? 8 : 2;
        }
        return (found * 4) & 31;
    }
    if (dot >= 257 && dot <= 320) {
        // Each sprite fetch steps the address for Y, tile and attributes, then
        // after the final pattern byte.
        unsigned step = (dot - 257) & 7;

        return ((dot - 257) / 8 * 4 + (step < 3 ? step + 1 : step < 7 ? 3 : 4)) & 31;
    }
    return 0;
}

// Disabling rendering on a rendered line leaves OAM row corruption pending.
// The first rendered dot afterwards copies OAM row 0 over the row selected
// by the secondary OAM address at the time rendering stopped.
static NES_NOINLINE void corrupt_oam_row(Ppu *p)
{
    latch_sprite_overflow(p, p->ticks);
    uint8_t address = p->oam_addr;

    p->oam_addr = (uint8_t)(p->oam_corruption_row * 8);

    for (unsigned i = 0; i < 8; ++i)
        write_oam(p, p->oam[i]);
    p->oam_addr = address;
    p->oam_corruption_row = 0;
    p->status_deadline = 0;
}

// This spare scroll-key bit marks a rendered row that contains only the backdrop
enum { BG_CLEAR_OUTPUT = 1u << 30 };

static inline uint64_t tile_palette(unsigned attribute, unsigned shift)
{
    return ((attribute >> shift) & 3) * UINT64_C(0x0404040404040404);
}

static void update_cached_tile(Ppu *p, const Cartridge *cart, unsigned address, unsigned tile)
{
    unsigned column = address & 31, row = (address >> 5) & 31, table = address >> 10;
    unsigned attribute = p->nametable[table * 1024 + 960 + (row >> 2) * 8 + column / 4];
    uint64_t palette = tile_palette(attribute, ((row & 2) << 1) | (column & 2));
    unsigned slot = tile >> 6;

    for (unsigned bank = 0; bank < 2; ++bank) {
        uint8_t *valid = &p->background_valid[bank][table][row];

        if (!*valid)
            continue;
        // Use the mapping that produced the cached row
        unsigned page = (uint16_t)(p->background_tags[bank][table][row] >> (16 * slot));
        const uint64_t *pattern = cart->decoded_chr + page * 512 + (tile & 63) * 8;
        uint8_t (*pixels)[NES_WIDTH] = &p->background[bank][table][row * 8];
        uint8_t *blank = &p->background_blank[bank][table][row];

        *valid |= 1u << slot;

        for (unsigned y = 0; y < 8; ++y) {
            uint64_t colors = pattern[y] | palette;

            memcpy(pixels[y] + column * 8, &colors, sizeof colors);
            // A replacement tile can make a previously transparent line visible
            if (pattern[y])
                *blank &= ~(1u << y);
        }
    }
}

static inline bool nametable_row_has_tile(const uint8_t *row, uint8_t tile)
{
#if defined(__aarch64__) && defined(__ARM_NEON)
    uint8x16_t needle = vdupq_n_u8(tile);
    uint8x16_t first = vceqq_u8(vld1q_u8(row), needle);
    uint8x16_t second = vceqq_u8(vld1q_u8(row + 16), needle);

    return vmaxvq_u8(vorrq_u8(first, second)) != 0;
#else
    return memchr(row, tile, 32) != NULL;
#endif
}

static void write_nametable(Nes *nes, unsigned address, uint8_t value)
{
    Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    unsigned old = p->nametable[address];

    if (old == value)
        return;
    p->nametable[address] = value;
    uint64_t revision = ++p->nametable_revision;

    unsigned offset = address & 1023;
    uint64_t *revisions = p->nametable_row_revision[address >> 10];

    revisions[offset / 32] = revision;

    if (offset >= 960) {
        unsigned row = (offset - 960) / 8 * 4;

        for (unsigned i = 0; i < 4; ++i)
            revisions[row + i] = revision;
    }
    if (cart->linear_chr_ram || cart->banked_chr_ram) {
        unsigned table = address >> 10;
        uint32_t bit = UINT32_C(1) << (offset >> 5);

        p->chr_tile_rows[value][table] |= bit;

        if (!nametable_row_has_tile(p->nametable + (address & ~31u), old))
            p->chr_tile_rows[old][table] &= ~bit;
    }
    bool patch = offset < 960 && (cart->chr_is_ram
        ? cart->linear_chr_ram && cart->decoded_chr && p->background_revision == cart->chr_revision
        : cart->chr_rom_tile_cache);

    if (patch) {
        update_cached_tile(p, cart, address, value);
        return;
    }
    for (unsigned bank = 0; bank < 2; ++bank) {
        uint8_t *valid = p->background_valid[bank][address >> 10];

        valid[offset / 32] = 0;

        if (offset >= 960)
            memset(valid + (offset - 960) / 8 * 4, 0, 4);
    }
}

static const uint8_t *nametable_page(const Nes *nes, unsigned table)
{
    static const uint8_t empty[1024] = {0};
    const Mmc5 *m = nes->cart.mmc5;

    if (m && table >= 2) {
        if (table == 3)
            return m->fill;
        return m->exram_mode <= 1 ? m->exram : empty;
    }
    return nes->ppu.nametable + table * 1024u;
}

static void write_mapped_nametable(Nes *nes, uint16_t address, uint8_t value)
{
    unsigned offset = cart_nametable(&nes->cart, address);
    Mmc5 *m = nes->cart.mmc5;

    if (!m || offset < 2048) {
        write_nametable(nes, offset, value);
    } else if (offset < 3072 && m->exram_mode <= 1 && m->exram[offset & 1023] != value) {
        m->exram[offset & 1023] = value;
        ++nes->cart.chr_revision;
    }
}

// Quiet PPUDATA writes use this path when both addresses keep A12 low
void ppu_write_nametable_at(Nes *nes, uint8_t value, uint16_t next, uint64_t tick)
{
    Ppu *p = &nes->ppu;

    if (nes->cart.mmc3.a12)
        cart_ppu_address(&nes->cart, p->v, tick);
    write_mapped_nametable(nes, p->v, value);
    p->v = next & 0x7fff;
    latch_bus(p, value, tick);
}

void ppu_fill_nametable(Nes *nes, uint8_t value, unsigned count, unsigned increment,
                        uint64_t first_tick, uint64_t last_tick)
{
    Ppu *p = &nes->ppu;
    Cartridge *cart = &nes->cart;
    unsigned v = p->v;

    if (cart->mmc3.a12)
        cart_ppu_address(cart, v, first_tick);
    while (count--) {
        write_mapped_nametable(nes, v, value);
        v += increment;
    }
    p->v = v;
    latch_bus(p, value, last_tick);
}

static void increment_y(Ppu *p)
{
    if ((p->v & 0x7000) != 0x7000) {
        p->v += 0x1000;
        return;
    }
    unsigned y = (p->v >> 5) & 31;

    if (y == 29)
        p->v ^= 0x0800;
    y = y == 29 ? 0 : (y + 1) & 31;
    p->v = (p->v & ~0x73e0) | (y << 5);
}

static void begin_sprite_fetches(Ppu *p, unsigned scroll)
{
    p->v = (p->v & ~0x041f) | (scroll & 0x041f);
    p->oam_addr = 0;
}

static void increment_data_address(Nes *nes, uint64_t tick, int previous_color)
{
    Ppu *p = &nes->ppu;

    if (rendering(p)) {
        // PPUDATA clocks both scroll counters while the rendering pipeline runs
        p->prefetch_since = (uint32_t)tick;

        if ((p->v & 31) == 31)
            p->v = (p->v & ~31) ^ 0x0400;
        else
            ++p->v;
        increment_y(p);
    } else {
        uint16_t previous = p->v;

        p->v = (p->v + ((p->ctrl & 4) ? 32 : 1)) & 0x7fff;
        cart_ppu_address(&nes->cart, p->v & 0x3fff, tick);
        if (p->scanline < NES_HEIGHT && ((previous & 0x3f00) == 0x3f00 || (p->v & 0x3f00) == 0x3f00)) {
            uint8_t color = previous_color >= 0 ? (uint8_t)previous_color : backdrop_index(p, previous);

            // PPUDATA updates the displayed palette address after its bus access
            repaint_blank_backdrop(nes, color, tick, 2 * NES_PPU_DOTS_PER_CPU);
        }
    }
}

static unsigned first_sprite(uint64_t sprites)
{
#if defined(__GNUC__) || defined(__clang__)
    return (unsigned)__builtin_ctzll(sprites);
#else
    unsigned index = 0;

    while (!(sprites & 1)) {
        sprites >>= 1;
        ++index;
    }
    return index;
#endif
}

// sprite_rows assumes OAMADDR starts at zero and evaluation and fetches use
// one sprite height. PpuSpriteLine handles other cases, including writes during
// evaluation and line 0's inherited secondary OAM. Advance it before each
// input change, then finish it when drawing the line.

enum { NO_SPRITE_LINE = 0xff, EVALUATION_DONE = 258, PRE_RENDER_COMPARE = (NES_SCANLINES_PER_FRAME - 1) & 0xff };

static inline unsigned sprite_height(const Ppu *p)
{
    return (p->ctrl & 0x20) ? 16 : 8;
}

static inline const PpuSpriteLine *find_sprite_line(const Ppu *p, unsigned line)
{
    const PpuSpriteLine *s = &p->sprite_lines[line & 1];
    return p->sprite_exact && s->line == line ? s : NULL;
}

static inline PpuSpriteLine *exact_sprite_line(Ppu *p, unsigned line)
{
    return (PpuSpriteLine *)find_sprite_line(p, line);
}

// Select slots whose fetch starts at or after the dot, with slot k starting at 257 + 8k
static inline uint8_t slots_fetched_from(unsigned dot)
{
    unsigned skipped = dot <= 257 ? 0 : (dot - 257 + 7) / 8;
    return skipped >= 8 ? 0 : (uint8_t)(0xff << skipped);
}

// Record sprite heights sampled before the dot, with slot k sampling at 262 + 8k
static void settle_sprite_heights(PpuSpriteLine *s, unsigned dot, bool tall)
{
    unsigned count = dot <= 262 ? 0 : (dot - 262 + 7) / 8;
    uint8_t slots = (uint8_t)((1u << (count < 8 ? count : 8)) - 1) & ~s->settled;

    s->settled |= slots;

    if (tall)
        s->tall |= slots;
}

static inline void next_secondary_slot(PpuSpriteLine *s)
{
    if (++s->slot == 32) {
        s->slot = 0;
        s->full = true;
    }
}

// Step the evaluation through the given dot. Each odd dot reads primary OAM
// and each even dot acts on that byte. The even dot writes it to secondary OAM
// until eight sprites are found, so an out-of-range Y also lands in the next
// free slot. After a Y comparison the address steps by one byte when the
// sprite is in range, and otherwise to the next aligned sprite. The X byte is
// compared too, and an out-of-range X realigns the address after one byte.
// Once secondary OAM is full, out-of-range sprites step five bytes, which is
// the hardware's overflow bug. Evaluation that wraps the address stops copying.
static void evaluate_sprites_to(const Ppu *p, PpuSpriteLine *s, unsigned last, unsigned height)
{
    for (; s->dot <= last && s->dot <= 256; s->dot += 2) {
        uint8_t value = p->oam[s->address];
        uint8_t before = s->address;

        if (s->wrapped) {
            s->address = (uint8_t)((s->address + 4) & 0xfc);
            continue;
        }
        if (!s->full)
            s->secondary[s->slot] = value;
        bool in_range = (unsigned)(s->compare - value) < height;

        if (!s->step) {
            if (in_range && !s->ninth) {
                if (s->full) {
                    s->ninth = true;

                    if (!s->overflow)
                        s->overflow = s->dot;
                } else {
                    next_secondary_slot(s);
                    // The sprite compared first is the one that can set the sprite zero hit
                    s->zero |= s->dot == 66;
                }
                ++s->address;
                s->step = 1;
            } else {
                if (s->dot == 66)
                    s->zero = false;
                if (s->full && !s->ninth)
                    s->address += (s->address & 3) == 3 ? 1 : 5;
                else
                    s->address = (uint8_t)((s->address + 4) & 0xfc);
            }
        } else {
            if (s->step == 3)
                s->address = in_range ? (uint8_t)(s->address + (s->full ? 4 : 1)) : (uint8_t)((s->address + 1) & 0xfc);
            else
                ++s->address;
            s->step = (s->step + 1) & 3;

            if (!s->full)
                next_secondary_slot(s);
        }
        if (s->address < before && s->address < 4)
            s->wrapped = true;
    }
}

static inline void finish_sprite_evaluation(const Ppu *p, PpuSpriteLine *s)
{
    if (s->dot <= 256)
        evaluate_sprites_to(p, s, 256, sprite_height(p));
}

// Secondary OAM after the usual evaluation of a line from OAM address zero,
// through the given dot. Returns whether sprite zero was found.
static bool usual_secondary(const Ppu *p, unsigned line, unsigned dot, uint8_t secondary[32])
{
    memset(secondary, 0xff, 32);
    uint64_t found = p->sprite_rows[(p->ctrl & 0x20) != 0][line + 1];
    bool zero = (found & 1) && dot >= 66;
    // Sprite `next` is compared on dot `check`. A miss takes two dots and a copy eight
    unsigned slot = 0, next = 0, check = 66;

    while (found && slot < 32) {
        unsigned sprite = first_sprite(found);
        unsigned at = check + 2 * (sprite - next);

        if (at > dot)
            break;
        for (unsigned byte = 0; byte < 4 && at + 2 * byte <= dot; ++byte)
            secondary[slot + byte] = p->oam[sprite * 4 + byte];
        if (at + 6 > dot)
            return zero;
        slot += 4;
        next = sprite + 1;
        check = at + 8;
        found &= found - 1;
    }
    // The last out-of-range Y before the dot stays in the free slot
    if (slot < 32 && next < 64 && dot >= check) {
        unsigned last = next + (dot - check) / 2;
        secondary[slot] = p->oam[(last < 63 ? last : 63) * 4];
    }
    return zero;
}

// Vblank leaves line 239's evaluation to be filled in only when a later line
// fetches it. OAM writes in vblank can make that differ from the evaluation.
static void fill_pending_secondary(Ppu *p)
{
    if (p->secondary_pending) {
        p->secondary_zero = usual_secondary(p, NES_HEIGHT - 1, NES_DOTS_PER_SCANLINE, p->secondary_oam);
        p->secondary_pending = false;
    }
}

// Start the record for a line. Its evaluating line runs the evaluation from
// the given OAM address unless rendering started after that evaluation began.
static PpuSpriteLine *begin_sprite_line(Ppu *p, unsigned line, unsigned address)
{
    PpuSpriteLine *s = exact_sprite_line(p, line);

    if (s)
        return s;
    s = &p->sprite_lines[line & 1];
    memset(s, 0, sizeof *s);
    s->line = (uint8_t)line;
    s->compare = (uint8_t)(line ? line - 1 : PRE_RENDER_COMPARE);
    s->address = (uint8_t)address;
    s->dot = 66;
    s->loaded = 0xff;

    if (line && line - 1 >= p->eval_line) {
        memset(s->secondary, 0xff, sizeof s->secondary);
    } else {
        // The pre-render line and a line where rendering started late only
        // fetch, and only the slots whose fetch ran with rendering enabled.
        fill_pending_secondary(p);
        memcpy(s->secondary, p->secondary_oam, sizeof s->secondary);
        s->zero = p->secondary_zero;
        s->dot = EVALUATION_DONE;

        if (line)
            s->loaded = line == p->eval_line && p->eval_enable_dot > 64 ? slots_fetched_from(p->eval_enable_dot) : 0;
    }
    p->sprite_exact = true;
    return s;
}

// Keep secondary OAM as rendering leaves it at a dot of a visible line
static NES_NOINLINE void capture_secondary(Ppu *p, unsigned line, unsigned dot)
{
    uint8_t *secondary = p->secondary_oam;
    bool zero = p->secondary_zero;

    if (dot > 64) {
        // A line where rendering started late keeps the earlier contents
        if (line < p->eval_line)
            return;
        PpuSpriteLine *s = exact_sprite_line(p, line + 1);

        if (s) {
            evaluate_sprites_to(p, s, dot - 1, sprite_height(p));
            memcpy(secondary, s->secondary, 32);
            zero = s->zero;
        } else {
            zero = usual_secondary(p, line, dot - 1, secondary);
        }
    } else {
        // Clearing has replaced the start of the previous line's evaluation
        if (line && line - 1 >= p->eval_line) {
            PpuSpriteLine *s = exact_sprite_line(p, line);

            if (s) {
                finish_sprite_evaluation(p, s);
                memcpy(secondary, s->secondary, 32);
                zero = s->zero;
            } else {
                zero = usual_secondary(p, line - 1, NES_DOTS_PER_SCANLINE, secondary);
            }
        } else {
            fill_pending_secondary(p);
            zero = p->secondary_zero;
        }
        if (dot && line >= p->eval_line)
            memset(secondary, 0xff, (dot - 1) / 2);
    }
    p->secondary_zero = zero;
    p->secondary_pending = false;
    // The pre-render line compares each Y with 261 & 255
    bool line0 = false;

    for (unsigned slot = 0; slot < 32; slot += 4)
        line0 |= secondary[slot] <= PRE_RENDER_COMPARE;
    p->line0_sprites = line0;
}

static void begin_line0_sprites(Nes *nes, unsigned dot);

// The line and dot of a write that can run ahead of the raster
static unsigned raster_line_at(const Ppu *p, uint64_t tick, unsigned *dot)
{
    unsigned line = p->scanline, pre_render = ppu_pre_render_line(p);
    unsigned d = p->dot + (tick > p->ticks ? (unsigned)(tick - p->ticks) : 0);

    for (;;) {
        unsigned length = line == pre_render && p->odd && (p->mask & 0x18) ? NES_DOTS_PER_SCANLINE - 1
                                                                           : NES_DOTS_PER_SCANLINE;

        if (d < length)
            break;
        d -= length;
        line = line == pre_render ? 0 : line + 1;
    }
    *dot = d;
    return line;
}

// A write to OAMADDR, or OAMDATA during rendering, set the address that
// evaluation reads next. Usually the address is zero when evaluation starts.
static NES_NOINLINE void sprite_address_change(Ppu *p, uint64_t tick, uint8_t previous)
{
    unsigned dot, line = raster_line_at(p, tick, &dot);
    unsigned pre_render = ppu_pre_render_line(p);
    unsigned target;

    if (line < NES_HEIGHT && dot <= 256)
        target = line + 1;
    else if (dot > 320 && (line + 1 < NES_HEIGHT || line == pre_render))
        target = line == pre_render ? 1 : line + 2;
    else
        return;
    PpuSpriteLine *s = exact_sprite_line(p, target);
    bool started = target == line + 1 && dot > 65;

    if (!s && !p->oam_addr && !started)
        return;
    latch_sprite_overflow(p, tick);
    if (!s)
        s = begin_sprite_line(p, target, started ? previous : p->oam_addr);
    if (started)
        evaluate_sprites_to(p, s, dot - 1, sprite_height(p));
    s->address = p->oam_addr;
    p->overflow_tick = 0;
}

// PPUCTRL is about to change the sprite height. Fetches that already ran keep
// the old height, and so does the evaluation up to this dot.
static NES_NOINLINE void sprite_height_change(Ppu *p, uint64_t tick)
{
    if (!(p->mask & 0x18))
        return;
    unsigned dot, line = raster_line_at(p, tick, &dot);
    unsigned pre_render = ppu_pre_render_line(p);
    bool tall = (p->ctrl & 0x20) != 0;

    if (!dot) {
        // Dot 0 idles after the previous line's fetches
        line = line ? line - 1 : pre_render;
        dot = NES_DOTS_PER_SCANLINE;
    }
    if (line == pre_render) {
        PpuSpriteLine *s = exact_sprite_line(p, 0);

        if (s)
            settle_sprite_heights(s, dot, tall);
        return;
    }
    if (line >= NES_HEIGHT)
        return;
    // Keep the sprites fetched on the previous line when repainting this line
    PpuSpriteLine *s = exact_sprite_line(p, line);

    if (!s && line && dot <= NES_WIDTH && (p->mask & 0x10) && (p->sprite_rows[0][line] | p->sprite_rows[1][line])) {
        s = begin_sprite_line(p, line, 0);
        finish_sprite_evaluation(p, s);
    }
    if (s)
        settle_sprite_heights(s, NES_DOTS_PER_SCANLINE, tall);
    if (dot <= 65)
        return;
    s = begin_sprite_line(p, line + 1, p->oam_addr);
    evaluate_sprites_to(p, s, dot - 1, tall ? 16 : 8);
    settle_sprite_heights(s, dot, tall);
}

// Rendering was enabled or disabled at the current dot
static NES_NOINLINE void sprite_render_change(Nes *nes, bool enabled)
{
    Ppu *p = &nes->ppu;
    unsigned line = p->scanline, dot = p->dot;
    unsigned pre_render = ppu_pre_render_line(p);

    if (!enabled) {
        if (line <= NES_HEIGHT) {
            capture_secondary(p, line < NES_HEIGHT ? line : NES_HEIGHT - 1,
                              line < NES_HEIGHT ? dot : NES_DOTS_PER_SCANLINE);
            // The evaluation and the remaining fetches stop with rendering
            PpuSpriteLine *s = line < NES_HEIGHT ? exact_sprite_line(p, line + 1) : NULL;

            if (s) {
                s->dot = EVALUATION_DONE;
                s->loaded &= (uint8_t)~slots_fetched_from(dot > 7 ? dot - 7 : 0);
            }
        }
        return;
    }
    if (line < NES_HEIGHT) {
        p->eval_line = (uint8_t)(line + (dot > 64));
        p->eval_enable_dot = (uint16_t)dot;
        // Rendering that resumes before evaluation starts runs a new one
        PpuSpriteLine *s = exact_sprite_line(p, line + 1);

        if (s && dot <= 64)
            s->line = NO_SPRITE_LINE;
        else if (s)
            s->loaded |= slots_fetched_from(dot);
        if (p->oam_addr && dot <= 64)
            begin_sprite_line(p, line + 1, p->oam_addr);
        else if (p->oam_addr && dot > 320 && line + 1 < NES_HEIGHT)
            begin_sprite_line(p, line + 2, p->oam_addr);
    } else if (line == pre_render) {
        p->eval_line = 0;

        if (p->oam_addr && dot > 320)
            begin_sprite_line(p, 1, p->oam_addr);
        else if (p->line0_sprites && dot > 257)
            begin_line0_sprites(nes, dot);
    } else {
        p->eval_line = NES_HEIGHT;
    }
}

// Vblank: keep line 239's evaluation for the pre-render fetches, and drop this frame's lines
static NES_NOINLINE void finish_sprite_frame(Ppu *p)
{
    if ((p->mask & 0x18) && p->eval_line < NES_HEIGHT) {
        // Without exact lines, line 0 can only fetch line 239's free slot,
        // which holds sprite 63's Y. Fill secondary OAM only when it is needed.
        if (p->sprite_exact || p->oam[252] <= PRE_RENDER_COMPARE) {
            capture_secondary(p, NES_HEIGHT - 1, NES_DOTS_PER_SCANLINE);
        } else {
            p->secondary_pending = true;
            p->line0_sprites = false;
        }
    }
    p->sprite_lines[0].line = p->sprite_lines[1].line = NO_SPRITE_LINE;
    p->sprite_exact = false;
}

// Regions stop before a line with exact sprites, which the line renderer draws
static NES_NOINLINE unsigned rows_before_sprite_line(const Ppu *p, unsigned rows)
{
    for (unsigned i = 0; i < 2; ++i) {
        unsigned line = p->sprite_lines[i].line;

        if (line >= p->scanline && line < NES_HEIGHT && line - p->scanline < rows)
            rows = line - p->scanline;
    }
    return rows;
}

// The evaluating line of the first exact line at or after the given one, or NES_HEIGHT
static unsigned exact_evaluation_line(const Ppu *p, unsigned from)
{
    unsigned first = NES_HEIGHT;

    for (unsigned i = 0; i < 2; ++i) {
        unsigned line = p->sprite_lines[i].line;

        if (line && line <= NES_HEIGHT && line - 1 >= from && line - 1 < first)
            first = line - 1;
    }
    return first;
}

// The overflow dot of an exact line if nothing else changes before its evaluation ends
static unsigned exact_overflow_dot(const Ppu *p, unsigned eval)
{
    PpuSpriteLine s = *find_sprite_line(p, eval + 1);

    finish_sprite_evaluation(p, &s);
    return s.overflow;
}

// Commit every evaluation up to the raster before primary OAM changes
static NES_NOINLINE void commit_sprite_lines(Ppu *p)
{
    if (p->scanline < NES_HEIGHT && p->dot > 65) {
        PpuSpriteLine *s = exact_sprite_line(p, p->scanline + 1);

        if (s)
            evaluate_sprites_to(p, s, p->dot - 1, sprite_height(p));
    }
}

// Reconstruct the OAM evaluation bus on demand for CPU reads
static NES_NOINLINE uint8_t read_oam_data(const Ppu *p)
{
    if (!rendering(p))
        return p->oam[p->oam_addr];
    unsigned dot = p->dot ? p->dot - 1 : 0;

    if (dot && dot <= 64)
        return 0xff;
    if (p->scanline == ppu_pre_render_line(p) || (!dot && !p->scanline))
        return 0xff;

    // Dot zero still exposes the previous scanline's secondary OAM
    unsigned line = p->scanline - !dot;

    if (!dot)
        dot = 340;
    uint8_t secondary[32];

    memset(secondary, 0xff, sizeof secondary);
    unsigned primary = dot >= 257 ? 0 : p->oam_addr;
    unsigned secondary_addr = 0;
    unsigned overflow_bytes = 0;
    bool done = false;
    unsigned height = (p->ctrl & 0x20) ? 16 : 8;
    uint8_t latch = 0xff;

    for (unsigned cycle = 65; cycle <= dot && cycle <= 256; ++cycle) {
        if (cycle & 1) {
            latch = p->oam[primary];
            continue;
        }
        bool in_range = line >= latch && line - latch < height;
        unsigned next = primary;

        if (done) {
            next = (primary & 0xfc) + 4;
            latch = secondary[secondary_addr & 31];
        } else if (secondary_addr == 32) {
            latch = secondary[0];

            if (in_range || overflow_bytes) {
                ++next;

                if (++overflow_bytes == 4) {
                    next &= ~3u;
                    done = true;
                }
            } else {
                next = (primary & 0xfc) + 4 + ((primary + 1) & 3);
            }
        } else {
            unsigned byte = secondary_addr & 3;

            secondary[secondary_addr] = latch;

            if (byte || in_range) {
                ++secondary_addr;
                ++next;

                if (byte == 3 && !in_range)
                    next &= ~3u;
            } else {
                next = (primary & 0xfc) + 4;
            }
        }
        done |= next >= 256;
        primary = next & 0xff;
    }
    if (dot >= 257) {
        unsigned fetch = dot <= 320 ? dot - 257 : 0;
        unsigned byte = fetch & 7;

        return secondary[(fetch >> 3) * 4 + (byte < 3 ? byte : 3)];
    }
    return latch;
}

static bool fewer_than_eight_sprites(uint64_t row)
{
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_popcountll(row) < 8;
#else
    for (unsigned i = 0; i < 7 && row; ++i)
        row &= row - 1;
    return row == 0;
#endif
}

// The dot on a visible line at which sprite evaluation sets the overflow flag,
// or zero. Once eight sprites are found, the evaluator also steps the byte
// index it compares as a Y coordinate, and stops at the first match.
static unsigned evaluate_full_line(const Ppu *p, unsigned line)
{
    unsigned height = (p->ctrl & 0x20) ? 16 : 8;
    unsigned dot = 65, found = 0, n = 0;

    // Each sprite takes two dots, and an in-range sprite six more to copy
    for (; n < 64 && found < 8; ++n) {
        bool in_range = line - p->oam[n * 4] < height;

        found += in_range;
        dot += in_range ? 8 : 2;
    }
    for (unsigned m = 0; found == 8 && n < 64; ++n, m = (m + 1) & 3, dot += 2) {
        if (line - p->oam[n * 4 + m] < height)
            return dot + 1;
    }
    return 0;
}

// The first line from the given one with at least eight sprites in range, or
// NES_HEIGHT. Fewer sprites can never fill secondary OAM.
static unsigned next_full_line(const Ppu *p, unsigned line)
{
    // Row r lists the sprites that line r - 1 evaluates
    const uint64_t *rows = p->sprite_rows[(p->ctrl & 0x20) != 0];
    unsigned row = line + 1;
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    for (; row + 2 <= NES_HEIGHT + 1; row += 2) {
        uint8x16_t counts = vcntq_u8(vreinterpretq_u8_u64(vld1q_u64(rows + row)));
        uint64x2_t sums = vpaddlq_u32(vpaddlq_u16(vpaddlq_u8(counts)));

        if (vmaxvq_u32(vreinterpretq_u32_u64(vcgtq_u64(sums, vdupq_n_u64(7)))))
            break;
    }
#endif
    while (row <= NES_HEIGHT && fewer_than_eight_sprites(rows[row]))
        ++row;
    return row - 1;
}

// The overflow dot of a line that next_full_line returned
static unsigned sprite_overflow_dot(Ppu *p, unsigned line)
{
    // Cache even overflow dots until OAM or sprite height changes
    bool tall = (p->ctrl & 0x20) != 0;
    uint32_t key = p->oam_revision * 2 + tall;

    if (p->overflow_memo_key != key) {
        p->overflow_memo_key = key;
        memset(p->overflow_memo, 0xff, sizeof p->overflow_memo);
    }
    if (p->overflow_memo[line] == 0xff)
        p->overflow_memo[line] = (uint8_t)(evaluate_full_line(p, line) / 2);
    return p->overflow_memo[line] * 2u;
}

// Renderers do not track the overflow flag. Apply evaluation results when
// $2002 is read or before a change to OAM, sprite size or rendering, and
// record the tick from which a read sees the next pending result.
static NES_NOINLINE void evaluate_sprite_overflow(Ppu *p, uint64_t tick)
{
    unsigned line = NES_HEIGHT, dot = 0;

    p->overflow_tick = UINT64_MAX;

    // Include the last frame's lines until the flag clears at pre-render dot 1
    if (p->scanline == ppu_pre_render_line(p) && p->dot > 1)
        return;
    uint64_t start = p->ticks - p->dot;

    if (p->scanline < NES_HEIGHT) {
        uint64_t elapsed = tick > start ? tick - start : 0;

        if (elapsed < (uint64_t)(NES_HEIGHT - p->scanline) * NES_DOTS_PER_SCANLINE) {
            line = p->scanline + (unsigned)(elapsed / NES_DOTS_PER_SCANLINE);
            dot = (unsigned)(elapsed % NES_DOTS_PER_SCANLINE);
        }
    }
    // An exact line replaces the usual evaluation of its line, and can set
    // the flag with fewer than eight sprites in range.
    unsigned from = p->overflow_eval_line;
    unsigned full = next_full_line(p, from);
    unsigned exact = p->sprite_exact ? exact_evaluation_line(p, from) : NES_HEIGHT;

    for (;;) {
        unsigned eval = full < exact ? full : exact, set;

        if (eval >= NES_HEIGHT)
            break;
        if (eval == exact) {
            set = exact_overflow_dot(p, eval);
            exact = exact_evaluation_line(p, eval + 1);
        } else {
            set = sprite_overflow_dot(p, eval);
        }
        if (full == eval)
            full = next_full_line(p, eval + 1);
        if (!set)
            continue;
        if (eval < line || (eval == line && set < dot)) {
            p->status |= SPRITE_OVERFLOW;
            return;
        }
        p->overflow_tick = start + (eval - p->scanline) * NES_DOTS_PER_SCANLINE + set + 1;
        break;
    }
    if (line > p->overflow_eval_line)
        p->overflow_eval_line = (uint8_t)line;
}

static inline void latch_sprite_overflow(Ppu *p, uint64_t tick)
{
    if ((p->mask & 0x18) && !(p->status & SPRITE_OVERFLOW) && p->overflow_eval_line < NES_HEIGHT &&
        tick >= p->overflow_tick)
        evaluate_sprite_overflow(p, tick);
}

uint8_t ppu_latch_status_at(Ppu *p, uint64_t tick)
{
    latch_sprite_overflow(p, tick);
    p->bus = ppu_status_value(p, tick);
    p->bus_bit5_until = p->bus_high_until = tick + PPU_BUS_HOLD;
    p->second_write = false;
    return p->bus;
}

uint8_t ppu_read_register_at(Nes *nes, uint16_t address, uint64_t tick)
{
    Ppu *p = &nes->ppu;

    switch (address & 7) {
    case 2:
        if (p->scanline == ppu_pre_render_line(p) && p->dot == 1) {
            p->status &= ~(SPRITE_ZERO | SPRITE_OVERFLOW);
            p->overflow_eval_line = NES_HEIGHT;
        }
        ppu_latch_status_at(p, tick);
        if (p->scanline == ppu_vblank_line(p) && p->dot == 1)
            p->suppress_vblank = true;
        p->status &= ~VBLANK;
        break;
    case 4:
        latch_bus(p, read_oam_data(p), tick);
        break;
    case 7: {
        uint16_t v = p->v & 0x3fff;

        if (!rendering(p))
            cart_ppu_address(&nes->cart, v, tick);
        if (v >= 0x3f00) {
            unsigned high = tick < p->bus_high_until ? p->bus & 0xc0 : 0;

            p->bus = high | palette_read(p, v);
            p->bus_low_until = p->bus_bit5_until = tick + PPU_BUS_HOLD;
            v -= 0x1000;
        } else {
            latch_bus(p, p->read_buffer, tick);
        }
        if (v < 0x2000) {
            const Mmc5 *m = nes->cart.mmc5;

            p->read_buffer = m && (p->ctrl & 0x20) && m->last_chr_bg
                ? m->bg_pages[v >> 10][v & 1023] : cart_ppu_read(&nes->cart, v);
        } else {
            unsigned mapped = cart_nametable(&nes->cart, v);
            p->read_buffer = nametable_page(nes, mapped >> 10)[mapped & 1023];
        }
        increment_data_address(nes, tick, -1);
        break;
    }
    default:
        // Undriven bits decay independently of bits refreshed by partial reads
        if (tick >= p->bus_low_until)
            p->bus &= ~0x1f;
        if (tick >= p->bus_bit5_until)
            p->bus &= ~0x20;
        if (tick >= p->bus_high_until)
            p->bus &= ~0xc0;
        break;
    }
    return p->bus;
}

uint8_t ppu_read_register(Nes *nes, uint16_t address)
{
    return ppu_read_register_at(nes, address, nes->ppu.ticks);
}

void ppu_write_register_at(Nes *nes, uint16_t address, uint8_t value, uint64_t tick)
{
    Ppu *p = &nes->ppu;

    p->status_deadline = 0;
    latch_bus(p, value, tick);
    switch (address & 7) {
    case 0: {
        unsigned changed = p->ctrl ^ value;

        if (changed & 0x20) {
            latch_sprite_overflow(p, tick);
            p->overflow_tick = 0;
            sprite_height_change(p, tick);
        }
        p->ctrl = value;
        p->t = (p->t & ~0x0c00) | ((value & 3) << 10);

        if (nes->ultrafast) {
            if (changed & 0x38)
                schedule_early_draw_at(nes, tick);
        } else if (changed & 0x10) {
            repaint_background_bank(nes);
        }
        break;
    }
    case 1: {
        unsigned changed = p->mask ^ value;

        p->palette_dirty |= (changed & 0xe1) != 0;
        p->mask = (p->mask & 0x18) | (value & ~0x18);
        p->render_mask = value & 0x18;
        p->mask_tick = (changed & 0x18) ? tick + NES_PPU_DOTS_PER_CPU : 0;
        // The odd-frame skip samples rendering at dot 339, before the renderer
        // sees a write from late in the pre-render line.
        bool render = (value & 0x18) != 0;

        if (render != ((p->mask & 0x18) != 0) && p->odd && p->scanline == ppu_pre_render_line(p) &&
            p->next_dot >= NES_DOTS_PER_SCANLINE - 1 && p->dot + (tick - p->ticks) <= 338)
            p->next_dot = NES_DOTS_PER_SCANLINE - render;
        if (changed & 0xe7)
            repaint_scanline(nes, p->mask & 0x18);
        break;
    }
    case 3: {
        unsigned dot = p->dot + (unsigned)(tick - p->ticks);
        uint8_t previous = p->oam_addr;

        p->oam_addr = fetching_sprites_at(p, dot) ? 0 : value;
        // A write in vblank before the pre-render line's fetches end has no effect on evaluation
        unsigned pre_render = ppu_pre_render_line(p);
        bool blank = p->scanline >= NES_HEIGHT && p->scanline < pre_render &&
                     dot < (pre_render - p->scanline) * NES_DOTS_PER_SCANLINE + 321;

        if ((p->mask & 0x18) && !blank)
            sprite_address_change(p, tick, previous);
        break;
    }
    case 4:
        if (rendering(p)) {
            uint8_t previous = p->oam_addr;

            p->oam_addr = (p->oam_addr + 4) & 0xfc;
            sprite_address_change(p, tick, previous);
        } else {
            latch_sprite_overflow(p, tick);
            write_oam(p, value);
        }
        break;
    case 5:
        if (p->second_write)
            p->t = (p->t & ~0x73e0) | ((value & 7) << 12) | ((value & 0xf8) << 2);
        else {
            if (p->fine_x != (value & 7)) {
                p->fine_x = value & 7;

                if (nes->ultrafast)
                    schedule_early_draw_at(nes, tick);
                else if ((p->mask & 8) && p->scanline < NES_HEIGHT)
                    repaint_fine_x(nes, tick);
            }
            p->t = (p->t & ~0x001f) | (value >> 3);
        }
        p->second_write = !p->second_write;
        break;
    case 6:
        if (p->second_write) {
            uint16_t previous = p->v;

            p->v = p->t = (p->t & 0x7f00) | value;

            if (nes->ultrafast && p->v != previous)
                schedule_early_draw_at(nes, tick);
            p->prefetch_since = (uint32_t)tick;

            if (!rendering(p)) {
                cart_ppu_address(&nes->cart, p->v, tick);
                if (!nes->ultrafast && p->scanline < NES_HEIGHT && p->v != previous &&
                    ((previous & 0x3f00) == 0x3f00 || (p->v & 0x3f00) == 0x3f00))
                    repaint_blank_backdrop(nes, backdrop_index(p, previous), tick, NES_PPU_DOTS_PER_CPU);
            }
        } else {
            p->t = (p->t & 0xff) | ((value & 63) << 8);
        }
        p->second_write = !p->second_write;
        break;
    case 7: {
        uint16_t v = p->v & 0x3fff;
        int previous_color = -1;

        if (!rendering(p))
            cart_ppu_address(&nes->cart, v, tick);
        if (v < 0x2000)
            cart_ppu_write(&nes->cart, v, value);
        else if (v < 0x3f00)
            write_mapped_nametable(nes, v, value);
        else {
            unsigned offset = palette_offset(v);

            if (!(p->mask & 0x18) && p->scanline < NES_HEIGHT)
                previous_color = palette_read(p, v);
            p->palette_dirty |= p->palette[offset] != (value & 63);
            p->palette[offset] = value & 63;
        }
        increment_data_address(nes, tick, previous_color);
        break;
    }
    }
}

static uint64_t tile_row(const Cartridge *cart, unsigned address)
{
    unsigned page = address >> 10, offset = address & 1023;

    if (cart->decoded_chr)
        return cart->decoded_chr_pages[page][(offset / 16) * 8 + (offset & 7)];
    // Expand the two NES pattern planes into eight byte-sized pixel indices
    return chr_decode_row(cart->chr_pages[page] + offset);
}

// A fully decoded row also remembers which 1 KiB CHR pages its names use
static const uint64_t bg_page_tags[16] = {
    UINT64_C(0x0000000000000000), UINT64_C(0x000000000000ffff),
    UINT64_C(0x00000000ffff0000), UINT64_C(0x00000000ffffffff),
    UINT64_C(0x0000ffff00000000), UINT64_C(0x0000ffff0000ffff),
    UINT64_C(0x0000ffffffff0000), UINT64_C(0x0000ffffffffffff),
    UINT64_C(0xffff000000000000), UINT64_C(0xffff00000000ffff),
    UINT64_C(0xffff0000ffff0000), UINT64_C(0xffff0000ffffffff),
    UINT64_C(0xffffffff00000000), UINT64_C(0xffffffff0000ffff),
    UINT64_C(0xffffffffffff0000), UINT64_C(0xffffffffffffffff)
};

static NES_ALWAYS_INLINE unsigned fill_decoded_background(const uint64_t *const *pages, uint8_t (*rows)[NES_WIDTH],
                                                          const uint8_t *names, const uint8_t *attributes,
                                                          unsigned shift, uint64_t changed, uint8_t *blank)
{
    unsigned used = 0;
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint64x2_t opaque[4] = {vdupq_n_u64(0), vdupq_n_u64(0), vdupq_n_u64(0), vdupq_n_u64(0)};
#else
    uint64_t opaque[8] = {0};
#endif
    // Adjacent tile pairs share an attribute palette and write contiguous pixels
    for (unsigned tile = 0; tile < NES_WIDTH / 8; tile += 2) {
        unsigned left = names[tile], right = names[tile + 1];
        unsigned lpage = left >> 6, rpage = right >> 6;

        used |= (1u << lpage) | (1u << rpage);
        uint64_t lmask = UINT64_C(0xffff) << (16 * lpage);
        uint64_t rmask = UINT64_C(0xffff) << (16 * rpage);
        const uint64_t colors = tile_palette(attributes[tile >> 2], shift | (tile & 2));
        const uint64_t *p0 = pages[lpage] + (left & 63) * 8;
        const uint64_t *p1 = pages[rpage] + (right & 63) * 8;
        bool write0 = (changed & lmask) != 0, write1 = (changed & rmask) != 0;
        // Track transparent lines across every tile, including those kept from an earlier fill
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        uint64x2x4_t lhs = vld1q_u64_x4(p0), rhs = vld1q_u64_x4(p1);

        for (unsigned i = 0; i < 4; ++i)
            opaque[i] = vorrq_u64(opaque[i], vorrq_u64(lhs.val[i], rhs.val[i]));
#else
        for (unsigned i = 0; i < 8; ++i)
            opaque[i] |= p0[i] | p1[i];
#endif
        if (write0 && write1) {
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            uint64x2_t palette = vdupq_n_u64(colors);

            for (unsigned i = 0; i < 4; ++i) {
                uint64x2_t first = vorrq_u64(vzip1q_u64(lhs.val[i], rhs.val[i]), palette);
                uint64x2_t second = vorrq_u64(vzip2q_u64(lhs.val[i], rhs.val[i]), palette);

                vst1q_u8(rows[i * 2] + tile * 8, vreinterpretq_u8_u64(first));
                vst1q_u8(rows[i * 2 + 1] + tile * 8, vreinterpretq_u8_u64(second));
            }
#else
            for (unsigned i = 0; i < 8; ++i) {
                uint64_t pixels[2] = {p0[i] | colors, p1[i] | colors};
                memcpy(rows[i] + tile * 8, pixels, sizeof pixels);
            }
#endif
        } else if (write0 || write1) {
            const uint64_t *pattern = write0 ? p0 : p1;
            unsigned offset = (tile + !write0) * 8;

            for (unsigned i = 0; i < 8; ++i) {
                uint64_t pixels = pattern[i] | colors;
                memcpy(rows[i] + offset, &pixels, sizeof pixels);
            }
        }
    }
    unsigned lines = 0;

    for (unsigned i = 0; i < 8; ++i) {
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        uint64_t bits = (i & 1) ? vgetq_lane_u64(opaque[i / 2], 1) : vgetq_lane_u64(opaque[i / 2], 0);
#else
        uint64_t bits = opaque[i];
#endif
        lines |= (unsigned)!bits << i;
    }
    *blank = (uint8_t)lines;
    return used;
}

static inline const Mmc5 *mmc5_background(const Nes *nes)
{
    return (nes->ppu.ctrl & 0x20) ? nes->cart.mmc5 : NULL;
}

static inline bool mmc5_extended_attributes(const Cartridge *cart)
{
    return cart->mmc5 && cart->mmc5->exram_mode == 1;
}

static inline unsigned mmc5_extended_tile(const Cartridge *cart, unsigned name, unsigned attribute)
{
    unsigned bank = (cart->mmc5->chr_upper << 6) | (attribute & 63);

    return (bank % (cart->chr_size / 4096)) * 256 + name;
}

static inline uint64_t background_chr_tag(const Nes *nes, unsigned bank)
{
    const Mmc5 *m = mmc5_background(nes);
    return m ? m->bg_tags[bank] : nes->cart.chr_tags[bank];
}

static inline bool same_tile_names(const uint8_t *names)
{
#if defined(__aarch64__) && defined(__ARM_NEON)
    uint8x16_t expected = vdupq_n_u8(names[0]);
    uint8x16_t first = veorq_u8(vld1q_u8(names), expected);
    uint8x16_t next = veorq_u8(vld1q_u8(names + 16), expected);

    return !vmaxvq_u8(vorrq_u8(first, next));
#else
    for (unsigned i = 1; i < 32; ++i)
        if (names[i] != names[0])
            return false;
    return true;
#endif
}

static inline bool transparent_tile(const uint64_t *tile)
{
    uint64_t bits = 0;

    for (unsigned y = 0; y < 8; ++y)
        bits |= tile[y];
    return bits == 0;
}

static NES_NOINLINE uint8_t load_extended_background(const Cartridge *cart, uint8_t (*rows)[NES_WIDTH],
                                                     const uint8_t *names, unsigned coarse_y)
{
    const uint8_t *attributes = cart->mmc5->exram + coarse_y * 32;
    unsigned opaque = 0;

    for (unsigned column = 0; column < NES_WIDTH / 8; ++column) {
        unsigned attribute = attributes[column];
        unsigned tile = mmc5_extended_tile(cart, names[column], attribute);
        uint64_t palette = tile_palette(attribute, 6);

        for (unsigned y = 0; y < 8; ++y) {
            uint64_t pattern = cart->decoded_chr ? cart->decoded_chr[tile * 8 + y] :
                chr_decode_row(cart->chr + tile * 16 + y);
            uint64_t pixels = pattern | palette;

            memcpy(rows[y] + column * 8, &pixels, sizeof pixels);
            opaque |= (unsigned)(pattern != 0) << y;
        }
    }
    return (uint8_t)~opaque;
}

static NES_NOINLINE void load_background_row(Nes *nes, unsigned table, unsigned coarse_y,
                                            unsigned fine_y, unsigned bank, uint64_t tag, uint64_t changed)
{
    Ppu *p = &nes->ppu;
    unsigned y = coarse_y * 8 + fine_y;
    uint8_t *row = p->background[bank][table][y];
    const Cartridge *cart = &nes->cart;
    const Mmc5 *m = mmc5_background(nes);
    const uint8_t *nametable = nametable_page(nes, table);
    const uint8_t *names = nametable + coarse_y * 32;

    if (mmc5_extended_attributes(cart)) {
        p->background_blank[bank][table][coarse_y] =
            load_extended_background(cart, &p->background[bank][table][coarse_y * 8], names, coarse_y);
        p->background_valid[bank][table][coarse_y] = UINT8_MAX;
        p->background_tags[bank][table][coarse_y] = tag;
        return;
    }
    const uint8_t *attributes = nametable + 960 + (coarse_y >> 2) * 8;
    unsigned shift = (coarse_y & 2) << 1;

    if (cart->decoded_chr) {
        const uint64_t *const *pages = (m ? m->decoded_bg_pages : cart->decoded_chr_pages) + bank * 4;
        unsigned first_row = coarse_y * 8;
        uint8_t (*rows)[NES_WIDTH] = &p->background[bank][table][first_row];
        unsigned tile = names[0], slot = tile >> 6;
        bool clear = !cart->chr_is_ram && !cart->mmc5 && same_tile_names(names) &&
                     transparent_tile(pages[slot] + (tile & 63) * 8);

        if (clear) {
            // Transparent pattern indices always map to the backdrop, whatever their attribute
            memset(rows, 0, 8 * NES_WIDTH);
            p->background_valid[bank][table][coarse_y] = 1u << slot;
            p->background_blank[bank][table][coarse_y] = UINT8_MAX;
            p->background_tags[bank][table][coarse_y] = tag;
            return;
        }
        uint8_t *blank = &p->background_blank[bank][table][coarse_y];
        unsigned used = changed == UINT64_MAX
            ? fill_decoded_background(pages, rows, names, attributes, shift, UINT64_MAX, blank)
            : fill_decoded_background(pages, rows, names, attributes, shift, changed, blank);

        p->background_valid[bank][table][coarse_y] = cart->chr_is_ram || cart->mmc5 ? UINT8_MAX : used;
    } else {
        unsigned pattern = bank * 4096 + fine_y;

        for (unsigned tile = 0; tile < NES_WIDTH / 8; ++tile) {
            uint64_t palette = tile_palette(attributes[tile >> 2], shift | (tile & 2));
            unsigned address = pattern + names[tile] * 16;
            const uint8_t *const *pages = m ? m->bg_pages : cart->chr_pages;
            uint64_t pixels = chr_decode_row(pages[address >> 10] + (address & 1023)) | palette;

            memcpy(row + tile * 8, &pixels, sizeof pixels);
        }
        p->background_valid[bank][table][coarse_y] |= 1u << fine_y;
    }
    p->background_tags[bank][table][coarse_y] = tag;
}

static NES_ALWAYS_INLINE const uint8_t *background_row_with_tag(Nes *nes, unsigned table,
                                                               unsigned coarse_y, unsigned fine_y,
                                                               unsigned bank, uint64_t tag)
{
    Ppu *p = &nes->ppu;
    unsigned y = coarse_y * 8 + fine_y;
    uint8_t *valid = &p->background_valid[bank][table][coarse_y];
    uint64_t *old_tag = &p->background_tags[bank][table][coarse_y];
    uint64_t changed = *old_tag ^ tag;

    if (nes->cart.decoded_chr) {
        if (!*valid) {
            load_background_row(nes, table, coarse_y, fine_y, bank, tag, UINT64_MAX);
        } else if (changed & bg_page_tags[*valid & 15]) {
            load_background_row(nes, table, coarse_y, fine_y, bank, tag, changed);
        }
    } else {
        if (changed)
            *valid = 0;
        if (!(*valid & (1u << fine_y)))
            load_background_row(nes, table, coarse_y, fine_y, bank, tag, UINT64_MAX);
    }
    return p->background[bank][table][y];
}

static inline const uint8_t *background_row(Nes *nes, unsigned table, unsigned coarse_y,
                                           unsigned fine_y, unsigned bank)
{
    return background_row_with_tag(nes, table, coarse_y, fine_y, bank, background_chr_tag(nes, bank));
}

static void invalidate_changed_rows(Ppu *p, Cartridge *cart)
{
    // The reverse index maps each tile to the nametable rows that use it
    for (unsigned bank = 0; bank < 2; ++bank) {
        uint32_t changed_rows[4] = {0};

        for (unsigned group = 0; group < 4; ++group) {
            unsigned page = bank * 4 + group;
            uint64_t tiles = cart->changed_chr_tiles[page];

            cart->changed_chr_tiles[page] = 0;

            while (tiles) {
                unsigned tile = group * 64 + first_sprite(tiles);

                if (cart->decoded_chr) {
                    unsigned physical = bank * 256 + tile;
                    const uint8_t *source = cart->chr + physical * 16;
                    uint64_t *decoded = cart->decoded_chr + physical * 8;

                    for (unsigned i = 0; i < 8; ++i)
                        decoded[i] = chr_decode_row(source + i);
                }
                for (unsigned table = 0; table < 4; ++table)
                    changed_rows[table] |= p->chr_tile_rows[tile][table];
                tiles &= tiles - 1;
            }
        }
        for (unsigned table = 0; table < 4; ++table) {
            uint32_t rows = changed_rows[table];

            while (rows) {
                unsigned row = first_sprite(rows);

                p->background_valid[bank][table][row] = 0;
                p->chr_row_revision[bank][table][row] = cart->chr_revision;
                rows &= rows - 1;
            }
        }
    }
}

static NES_NOINLINE void invalidate_banked_rows(Ppu *p, Cartridge *cart)
{
    uint64_t pending[8];

    memcpy(pending, cart->changed_chr_tiles, sizeof pending);
    memset(cart->changed_chr_tiles, 0, sizeof cart->changed_chr_tiles);
    unsigned remaining = 8;

    // Flush bulk writes before consulting the row index
    for (unsigned page = 0; page < 8; ++page) {
        uint64_t tiles = pending[page];

        for (; tiles && remaining; --remaining)
            tiles &= tiles - 1;
        if (tiles) {
            memset(p->background_valid, 0, sizeof p->background_valid);
            return;
        }
    }
    for (unsigned page = 0; page < 8; ++page) {
        for (uint64_t tiles = pending[page]; tiles; tiles &= tiles - 1) {
            unsigned tile = first_sprite(tiles);

            for (unsigned slot = 0; slot < 4; ++slot) {
                for (unsigned table = 0; table < 4; ++table) {
                    uint32_t rows = p->chr_tile_rows[slot * 64 + tile][table];

                    for (; rows; rows &= rows - 1) {
                        unsigned row = first_sprite(rows);

                        for (unsigned bank = 0; bank < 2; ++bank) {
                            uint64_t tag = p->background_tags[bank][table][row];

                            if ((uint16_t)(tag >> (16 * slot)) == page)
                                p->background_valid[bank][table][row] = 0;
                        }
                    }
                }
            }
        }
    }
}

static void refresh_background(Nes *nes)
{
    Ppu *p = &nes->ppu;
    Cartridge *cart = &nes->cart;

    if (p->background_revision == cart->chr_revision)
        return;
    if (cart->linear_chr_ram)
        invalidate_changed_rows(p, cart);
    else if (cart->banked_chr_ram)
        invalidate_banked_rows(p, cart);
    else if (cart->chr_is_ram || cart->mmc5)
        memset(p->background_valid, 0, sizeof p->background_valid);
    p->background_revision = cart->chr_revision;
}

static inline void copy_indices(uint8_t *out, const uint8_t *in, unsigned length)
{
    if (length >= 64) {
        unsigned offset = 0;

        do {
            memcpy(out + offset, in + offset, 64);
            offset += 64;
        } while (offset + 64 <= length);
        if (offset != length)
            memcpy(out + length - 64, in + length - 64, 64);
        return;
    }
    if (length >= 32) {
        memcpy(out, in, 32);
        memcpy(out + length - 32, in + length - 32, 32);
        return;
    }
    if (length >= 16) {
        memcpy(out, in, 16);
        memcpy(out + length - 16, in + length - 16, 16);
        return;
    }
    if (length >= 8) {
        memcpy(out, in, 8);
        memcpy(out + length - 8, in + length - 8, 8);
        return;
    }
    if (length >= 4) {
        memcpy(out, in, 4);
        memcpy(out + length - 4, in + length - 4, 4);
        return;
    }
    if (length >= 2) {
        memcpy(out, in, 2);
        memcpy(out + length - 2, in + length - 2, 2);
        return;
    }
    if (length)
        *out = *in;
}

static const uint8_t *background_indices(Nes *nes, uint8_t row[NES_WIDTH])
{
    Ppu *p = &nes->ppu;
    unsigned page = p->v & 0xc00;
    unsigned coarse_y = (p->v >> 5) & 31;
    unsigned fine_y = p->v >> 12;
    unsigned bank = (p->ctrl >> 4) & 1;
    unsigned x = (p->v & 31) * 8 + p->fine_x;
    unsigned table = cart_nametable(&nes->cart, page) >> 10;
    const uint8_t *first = background_row(nes, table, coarse_y, fine_y, bank);

    if (!x)
        return first;

    unsigned next_table = cart_nametable(&nes->cart, page ^ 0x400) >> 10;
    const uint8_t *next = table == next_table ? first :
        background_row(nes, next_table, coarse_y, fine_y, bank);

    copy_indices(row, first + x, NES_WIDTH - x);
    copy_indices(row + NES_WIDTH - x, next, x);
    return row;
}

static void draw_sprite_pixels(Ppu *p, uint8_t *row, uint64_t pattern,
                               unsigned sprite, unsigned attr, unsigned x, unsigned start)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ && \
    (defined(__GNUC__) || defined(__clang__))
    const uint64_t high = UINT64_C(0x8080808080808080);
    const uint64_t low = UINT64_C(0x7f7f7f7f7f7f7f7f);

    if (attr & 0x40)
        pattern = __builtin_bswap64(pattern);
    if (start > x)
        pattern &= UINT64_MAX << ((start - x) * 8);
    // The row has seven extra bytes for the part of a sprite past x = 255

    uint64_t pixels;

    memcpy(&pixels, row + x, sizeof pixels);
    // Adding 127 sets the high bit of each nonzero color index
    uint64_t covered = (pattern + low) & high & ~pixels;
    uint64_t draw = covered;
    bool behind = (attr & 0x20) != 0;
    bool check_hit = sprite == 0 && !p->hit_dot && !(p->status & SPRITE_ZERO);

    if (behind || check_hit) {
        uint64_t background = ((pixels & UINT64_C(0x0303030303030303)) + low) & high;

        if (behind)
            draw &= ~background;
        if (check_hit && (covered & background)) {
            unsigned column = x + first_sprite(covered & background) / 8;

            if (column < NES_WIDTH - 1)
                p->hit_dot = column + 1;
        }
    }
    uint64_t mask = (draw >> 7) * 255;
    uint64_t colors = pattern | ((16 + (attr & 3) * 4) * UINT64_C(0x0101010101010101));

    pixels = (pixels & ~mask) | (colors & mask) | covered;
    memcpy(row + x, &pixels, sizeof pixels);
#else
    uint8_t bits[8];

    memcpy(bits, &pattern, sizeof bits);
    for (unsigned j = 0; j < 8 && x + j < NES_WIDTH; ++j) {
        unsigned column = x + j;
        unsigned color = bits[(attr & 0x40) ? j ^ 7 : j];

        if (!color || column < start || (row[column] & 0x80))
            continue;
        bool background = (row[column] & 3) != 0;

        if (sprite == 0 && background && column != NES_WIDTH - 1 && !p->hit_dot && !(p->status & SPRITE_ZERO))
            p->hit_dot = column + 1;
        if (!background || !(attr & 0x20))
            row[column] = 16 + (attr & 3) * 4 + color;
        row[column] |= 0x80;
    }
#endif
}

static inline unsigned sprite_row_address(const Ppu *p, const uint8_t *sprite, unsigned height)
{
    unsigned y = p->scanline - 1 - sprite[0];
    unsigned tile = sprite[1];

    if (sprite[2] & 0x80)
        y ^= height - 1;
    unsigned address = height == 16
        ? ((tile & 1) << 12) + ((tile & 254) + (y >> 3)) * 16
        : ((p->ctrl & 8) << 9) + tile * 16;

    return address + (y & 7);
}

static inline uint64_t mmc2_peek_tile_row(const Cartridge *cart, unsigned address, uint8_t latches)
{
    unsigned window = address >> 12;
    unsigned reg = window * 2 + ((latches >> window) & 1);
    unsigned offset = cart->mmc2.chr[reg] * 4096u + (address & 4095);

    return cart->decoded_chr
        ? cart->decoded_chr[offset / 16 * 8 + (offset & 7)]
        : chr_decode_row(cart->chr + offset);
}

static NES_ALWAYS_INLINE uint64_t mmc2_tile_row(const Cartridge *cart, unsigned address,
                                               uint8_t *latches, bool mmc4)
{
    uint64_t row = mmc2_peek_tile_row(cart, address, *latches);

    *latches = cart_mmc24_latches_after_read(*latches, address + 8, mmc4);
    return row;
}

static inline unsigned mmc2_decode_tiles(uint8_t *pixels, const uint8_t *names,
                                         const uint8_t *attributes, unsigned shift,
                                         unsigned start, unsigned end, const uint64_t *const rows[2],
                                         unsigned latch, bool triggers)
{
    if ((start & 1) && start < end) {
        unsigned name = names[start];
        uint64_t row = rows[latch][name * 8] |
            tile_palette(attributes[start >> 2], shift | (start & 2));

        memcpy(pixels, &row, sizeof row);
        if (triggers && name - 0xfdu < 2)
            latch = name - 0xfd;
        ++start;
        pixels += 8;
    }
    // An aligned pair shares one attribute palette. The first fetch can
    // still change the latch before the second pattern is read.
    for (; start + 1 < end; start += 2, pixels += 16) {
        unsigned name0 = names[start], name1 = names[start + 1];
        uint64_t palette = tile_palette(attributes[start >> 2], shift | (start & 2));
        uint64_t first = rows[latch][name0 * 8] | palette;

        if (triggers && name0 - 0xfdu < 2)
            latch = name0 - 0xfd;
        uint64_t second = rows[latch][name1 * 8] | palette;

        if (triggers && name1 - 0xfdu < 2)
            latch = name1 - 0xfd;
        memcpy(pixels, &first, sizeof first);
        memcpy(pixels + 8, &second, sizeof second);
    }
    for (unsigned column = start; column < end; ++column, pixels += 8) {
        unsigned name = names[column];
        uint64_t row = rows[latch][name * 8] |
            tile_palette(attributes[column >> 2], shift | (column & 2));

        memcpy(pixels, &row, sizeof row);
        if (triggers && name - 0xfdu < 2)
            latch = name - 0xfd;
    }
    return latch;
}

static uint8_t mmc2_decoded_background(const Nes *nes, uint8_t row[NES_WIDTH], uint8_t latches)
{
    const Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    unsigned window = (p->ctrl >> 4) & 1, fine_y = p->v >> 12;
    unsigned bit = 1u << window, latch = (latches & bit) != 0;
    // All background fetches use one pattern window and one fine Y
    const uint64_t *rows[] = {
        cart->decoded_chr + cart->mmc2.chr[window * 2] * 2048u + fine_y,
        cart->decoded_chr + cart->mmc2.chr[window * 2 + 1] * 2048u + fine_y
    };
    // The MMC2 left latch triggers only on the first row of tiles $FD and $FE
    bool triggers = window || !fine_y || cart->mmc4;
    unsigned column = p->v & 31, coarse_y = (p->v >> 5) & 31;
    unsigned page = p->v & 0xc00, offset = coarse_y * 32, attr = 960 + (coarse_y >> 2) * 8;
    unsigned shift = (coarse_y & 2) << 1;
    const uint8_t *first = p->nametable + cart_nametable(cart, page);
    const uint8_t *second = p->nametable + cart_nametable(cart, page ^ 0x400);
    uint8_t tiles[NES_WIDTH + 8];

    latch = mmc2_decode_tiles(tiles, first + offset, first + attr, shift, column, 32, rows, latch, triggers);
    latch = mmc2_decode_tiles(tiles + (32 - column) * 8, second + offset, second + attr,
                              shift, 0, column + 1, rows, latch, triggers);

    if (triggers) {
        // The final offscreen fetch only affects the latch for the next row
        unsigned last = column == 31 ? first[offset] : second[offset + column + 1];

        if (last - 0xfdu < 2)
            latch = last - 0xfd;
    }
    memcpy(row, tiles + p->fine_x, NES_WIDTH);
    return (latches & ~bit) | latch << window;
}

enum { MMC2_LATCH = 1, MMC2_ROW_VALID = 2, MMC2_ROW_DRAWN = 4 };

static inline uint64_t mmc2_background_layout(const Nes *nes, unsigned window)
{
    const Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    unsigned bank = window * 2;

    return (p->fine_x << 15) | (window << 18) | ((unsigned)cart->mirroring << 19) |
        ((uint64_t)cart->mmc2.chr[bank] << 23) | ((uint64_t)cart->mmc2.chr[bank + 1] << 31) |
        ((uint64_t)((p->mask & 2) != 0) << 39);
}

static inline const uint8_t *mmc2_background_row(Nes *nes, uint8_t *latches, unsigned window,
                                                 uint64_t layout, uint64_t revision)
{
    Ppu *p = &nes->ppu;
    unsigned bit = 1u << window;
    uint8_t *row = p->background[0][0][p->scanline];
    Mmc2RowKey *cached = &p->mmc2_background[p->scanline];
    uint64_t setup = layout | p->v | (((*latches & bit) != 0) << 22);

    setup = (setup << 3) | MMC2_ROW_VALID;

    if ((cached->setup & ~(uint64_t)(MMC2_LATCH | MMC2_ROW_DRAWN)) == setup &&
        cached->nametable_revision == revision) {
        *latches = (*latches & ~bit) | (unsigned)(cached->setup & MMC2_LATCH) << window;
    } else {
        *latches = mmc2_decoded_background(nes, row, *latches);
        cached->setup = setup | ((*latches & bit) != 0);
        cached->nametable_revision = revision;
    }
    return row;
}

static const uint8_t *mmc2_cached_background(Nes *nes, uint8_t *latches)
{
    Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    unsigned window = (p->ctrl >> 4) & 1;
    unsigned page = p->v & 0xc00, coarse_y = (p->v >> 5) & 31;
    unsigned first = cart_nametable(cart, page) >> 10;
    unsigned next = cart_nametable(cart, page ^ 0x400) >> 10;
    uint64_t revision = p->nametable_row_revision[first][coarse_y];

    // Include both source tables, including tiles read offscreen for the latch
    if (p->nametable_row_revision[next][coarse_y] > revision)
        revision = p->nametable_row_revision[next][coarse_y];

    return mmc2_background_row(nes, latches, window, mmc2_background_layout(nes, window), revision);
}

typedef struct {
    uint64_t patterns[8];
    uint8_t latches;
} Mmc2SpriteFetches;

static NES_ALWAYS_INLINE void mmc2_fetch_sprites(const Ppu *p, const Cartridge *cart,
                                                Mmc2SpriteFetches *fetches, uint8_t latches,
                                                const uint64_t *const *decoded_banks, bool mmc4)
{
    // Sprite patterns are fetched before the background for the next row
    bool tall = (p->ctrl & 0x20) != 0;
    uint64_t sprites = p->sprite_rows[tall][p->scanline];
    unsigned slot = 0;

    for (; sprites && slot < 8; ++slot) {
        const uint8_t *sprite = p->oam + first_sprite(sprites) * 4;

        sprites &= sprites - 1;
        unsigned address = sprite_row_address(p, sprite, tall ? 16 : 8);

        if (decoded_banks) {
            unsigned window = address >> 12;
            unsigned reg = window * 2 + ((latches >> window) & 1);

            fetches->patterns[slot] = decoded_banks[reg][((address >> 1) & 0x7f8) | (address & 7)];
            latches = cart_mmc24_latches_after_read(latches, address + 8, mmc4);
        } else {
            fetches->patterns[slot] = mmc2_tile_row(cart, address, &latches, mmc4);
        }
    }
    // An unused 8x16 sprite slot reads the right table's tile $FE
    if (slot < 8 && tall)
        latches = cart_mmc2_latches_after_read(latches, 0x1fe8);
    fetches->latches = latches;
}

static NES_ALWAYS_INLINE const uint8_t *mmc24_scanline(Nes *nes, uint8_t row[NES_WIDTH],
                                                      Mmc2SpriteFetches *fetches, uint8_t *line_latches,
                                                      bool mmc4)
{
    Ppu *p = &nes->ppu;
    Cartridge *cart = &nes->cart;

    mmc2_fetch_sprites(p, cart, fetches, *line_latches, NULL, mmc4);
    uint8_t latches = fetches->latches;

    const uint8_t *indices = row;

    if (cart->decoded_chr) {
        indices = mmc2_cached_background(nes, &latches);
    } else {
        unsigned coarse_y = (p->v >> 5) & 31, column = p->v & 31;
        unsigned page = p->v & 0xc00, pattern = ((p->ctrl & 0x10) << 8) + (p->v >> 12);
        const uint8_t *table = p->nametable + cart_nametable(cart, page);
        uint8_t tiles[NES_WIDTH + 8];

        // Include offscreen fetches because the last tile can switch the next row's bank
        for (unsigned tile = 0; tile < 34; ++tile) {
            uint64_t pixels = mmc2_tile_row(cart, pattern + table[coarse_y * 32 + column] * 16,
                                            &latches, mmc4);

            if (tile < 33) {
                unsigned attribute = table[960 + (coarse_y >> 2) * 8 + (column >> 2)];

                pixels |= tile_palette(attribute, ((coarse_y & 2) << 1) | (column & 2));
                memcpy(tiles + tile * 8, &pixels, sizeof pixels);
            }
            if (++column == 32) {
                column = 0;
                page ^= 0x400;
                table = p->nametable + cart_nametable(cart, page);
            }
        }
        memcpy(row, tiles + p->fine_x, NES_WIDTH);
    }
    *line_latches = latches;
    return indices;
}

static NES_NOINLINE const uint8_t *mmc2_scanline(Nes *nes, uint8_t row[NES_WIDTH],
                                                Mmc2SpriteFetches *fetches, uint8_t *line_latches)
{
    return mmc24_scanline(nes, row, fetches, line_latches, false);
}

// The MMC4 runs its own copy of the latch-aware scanline and region code
static NES_NOINLINE const uint8_t *mmc4_scanline(Nes *nes, uint8_t row[NES_WIDTH],
                                                Mmc2SpriteFetches *fetches, uint8_t *line_latches)
{
    return mmc24_scanline(nes, row, fetches, line_latches, true);
}

static NES_ALWAYS_INLINE uint64_t draw_decoded_sprites(Ppu *p, uint8_t *row, unsigned start,
                                                      uint64_t sprites, unsigned limit,
                                                      const uint64_t *const *pages, bool contiguous, bool tall)
{
    unsigned line = p->scanline - 1;

    for (unsigned count = 0; sprites && count < limit; ++count) {
        unsigned i = first_sprite(sprites);

        sprites &= sprites - 1;
        const uint8_t *sprite = p->oam + 4 * i;
        unsigned x = sprite[3], attr = sprite[2], tile = sprite[1];

        if (x + 8 <= start)
            continue;
        unsigned row_mask = tall ? 15 : 7;
        unsigned y = (line - sprite[0]) ^ ((attr & 0x80) ? row_mask : 0);
        // Tall sprites use the low tile bit to select a pattern table and span two tiles
        unsigned index = ((tall ? tile & 254 : tile) << 3) | (y & row_mask);
        unsigned page = (tile >> 6) | (tall ? (tile & 1) << 2 : 0);
        uint64_t packed = contiguous ? pages[0][index] : pages[page][index & 511];

        if (packed)
            draw_sprite_pixels(p, row, packed, i, attr, x, start);
    }
    return sprites;
}

static NES_NOINLINE uint64_t draw_tall_decoded_sprites(Ppu *p, uint8_t *row, unsigned start, uint64_t sprites,
                                                      unsigned limit, const uint64_t *const *pages)
{
    return draw_decoded_sprites(p, row, start, sprites, limit, pages, false, true);
}

static NES_ALWAYS_INLINE uint64_t draw_sprite_list(Ppu *p, const Cartridge *cart, uint8_t *row,
                                                  unsigned start, unsigned height, uint64_t sprites,
                                                  unsigned limit, const uint64_t *patterns,
                                                  const Mmc2SpriteFetches *extra_mmc2)
{
    if (!patterns && !extra_mmc2 && height == 8 && cart->decoded_chr) {
        unsigned window = (p->ctrl & 8) >> 1;
        const uint64_t *const *pages = cart->decoded_chr_pages + window;
        uint64_t tag = cart->chr_tags[window >> 2];
        // Each 16-bit lane names one 1 KiB physical page. Decoded CHR is at
        // most 512 KiB, so offsets 0–3 cannot carry between lanes.
        uint64_t sequential = (uint16_t)tag * UINT64_C(0x0001000100010001) + UINT64_C(0x0003000200010000);

        if (tag == sequential)
            return draw_decoded_sprites(p, row, start, sprites, limit, pages, true, false);
        return draw_decoded_sprites(p, row, start, sprites, limit, pages, false, false);
    }
    if (!patterns && !extra_mmc2 && height == 16 && cart->decoded_chr)
        return draw_tall_decoded_sprites(p, row, start, sprites, limit, cart->decoded_chr_pages);

    for (unsigned count = 0; sprites && count < limit; ++count) {
        unsigned i = first_sprite(sprites);

        sprites &= sprites - 1;
        const uint8_t *sprite = p->oam + i * 4;
        unsigned attr = sprite[2], x = sprite[3];

        if (x + 8 <= start)
            continue;
        uint64_t packed;

        if (patterns) {
            packed = patterns[count];
        } else {
            unsigned address = sprite_row_address(p, sprite, height);
            packed = extra_mmc2 ? mmc2_peek_tile_row(cart, address, extra_mmc2->latches)
                               : tile_row(cart, address);
        }
        if (!packed)
            continue;
        draw_sprite_pixels(p, row, packed, i, attr, x, start);
    }
    return sprites;
}

static NES_NOINLINE void draw_extra_sprites(Ppu *p, const Cartridge *cart, uint8_t *row,
                                            unsigned start, unsigned height, uint64_t sprites,
                                            const Mmc2SpriteFetches *fetches)
{
    // Extra sprites use the bank left by the real fetches without changing MMC2 latches
    draw_sprite_list(p, cart, row, start, height, sprites, 64, NULL, fetches);
}

static NES_ALWAYS_INLINE void draw_sprites(Ppu *p, const Cartridge *cart, uint8_t row[NES_WIDTH + 7],
                                           unsigned start, uint64_t sprites, const Mmc2SpriteFetches *fetches)
{
    unsigned height = (p->ctrl & 0x20) ? 16 : 8;
    unsigned left = (p->mask & 4) ? 0 : 8;

    if (start < left)
        start = left;
    sprites = draw_sprite_list(p, cart, row, start, height, sprites, 8, fetches ? fetches->patterns : NULL, NULL);

    if (sprites && p->no_sprite_limit)
        draw_extra_sprites(p, cart, row, start, height, sprites, fetches);
}

// The current line's exact sprites with their evaluation complete, or NULL
static NES_NOINLINE const PpuSpriteLine *drawn_sprite_line(Ppu *p)
{
    PpuSpriteLine *s = exact_sprite_line(p, p->scanline);

    if (s) {
        finish_sprite_evaluation(p, s);
        return s;
    }
    // Drop drawn lines whose evaluation can no longer set the overflow flag,
    // so the line renderer stops looking for them. Line 0 has no evaluation.
    bool remaining = false;

    for (unsigned i = 0; i < 2; ++i) {
        PpuSpriteLine *line = &p->sprite_lines[i];

        if (line->line < p->scanline && (!line->line || (p->status & SPRITE_OVERFLOW) ||
                                         p->overflow_eval_line >= line->line))
            line->line = NO_SPRITE_LINE;
        remaining |= line->line != NO_SPRITE_LINE;
    }
    p->sprite_exact = remaining;
    return NULL;
}

// The pattern row a slot fetched, with bit 4 set for an 8x16 sprite, or -1
// when the slot draws nothing. Each fetch compares the Y byte again.
static int sprite_slot_row(const Ppu *p, const PpuSpriteLine *s, unsigned slot)
{
    if (!((s->loaded >> slot) & 1))
        return -1;
    bool tall = ((s->settled >> slot) & 1) ? ((s->tall >> slot) & 1) : (p->ctrl & 0x20) != 0;
    unsigned y = (unsigned)(s->compare - s->secondary[slot * 4]);

    return y < (tall ? 16u : 8u) ? (int)(y | (unsigned)tall << 4) : -1;
}

// Bit 0 marks sprite zero on the line, and bit 1 any other sprite
static uint64_t sprite_line_mask(const Ppu *p, const PpuSpriteLine *s)
{
    uint64_t mask = 0;

    for (unsigned slot = 0; slot < 8; ++slot) {
        if (sprite_slot_row(p, s, slot) >= 0)
            mask |= slot || !s->zero ? 2 : 1;
    }
    return mask;
}

// The pattern row that a slot fetches for a row within the sprite
static uint64_t sprite_slot_pattern(const Ppu *p, const Cartridge *cart, const uint8_t *sprite,
                                    unsigned y, unsigned height)
{
    unsigned tile = sprite[1];

    if (sprite[2] & 0x80)
        y ^= height - 1;
    unsigned address = height == 16 ? ((tile & 1) << 12) + ((tile & 254) + (y >> 3)) * 16
                                     : ((p->ctrl & 8) << 9) + tile * 16;

    return tile_row(cart, address + (y & 7));
}

// Line 0 draws the sprites that the pre-render line fetches on or after the
// dot. Usually the only one in range is line 239's free slot, tile $FF at
// x = 255. When no fetched pixel can reach the screen, line 0 stays with the
// usual renderer.
static NES_NOINLINE void begin_line0_sprites(Nes *nes, unsigned dot)
{
    Ppu *p = &nes->ppu;

    if (!(p->mask & 0x18))
        return;
    PpuSpriteLine *s = begin_sprite_line(p, 0, 0);

    s->loaded = slots_fetched_from(dot);

    for (unsigned slot = 0; slot < 8; ++slot) {
        const uint8_t *sprite = s->secondary + slot * 4;
        unsigned y = (unsigned)(s->compare - sprite[0]);
        // Clip pixels past x = 255, keeping the first pixel on screen
        unsigned shown = NES_WIDTH - sprite[3] < 8 ? NES_WIDTH - sprite[3] : 8;

        // A later height change can still settle either height
        for (unsigned height = 8; ((s->loaded >> slot) & 1) && height <= 16 && y < height; height += 8) {
            uint8_t pixels[8];
            uint64_t pattern = sprite_slot_pattern(p, &nes->cart, sprite, y, height);

            memcpy(pixels, &pattern, sizeof pixels);
            for (unsigned x = 0; x < shown; ++x) {
                if (pixels[(sprite[2] & 0x40) ? 7 - x : x])
                    return;
            }
        }
    }
    s->line = NO_SPRITE_LINE;
    p->sprite_exact = p->sprite_lines[1].line != NO_SPRITE_LINE;
}

static NES_NOINLINE void draw_sprite_line(Ppu *p, const Cartridge *cart, uint8_t row[NES_WIDTH + 7],
                                          unsigned start, const PpuSpriteLine *s)
{
    unsigned left = (p->mask & 4) ? 0 : 8;

    if (start < left)
        start = left;
    // When the pre-render line skips its last dot, line 0 outputs the first
    // pixel of each fetched sprite at x = 0, before the X counters run. The
    // other seven pixels keep their positions. RGB PPUs lack this.
    bool skipped = !s->line && !p->odd;

    for (unsigned slot = 0; slot < 8; ++slot) {
        int fetched = sprite_slot_row(p, s, slot);

        if (fetched < 0)
            continue;
        const uint8_t *sprite = s->secondary + slot * 4;
        unsigned attr = sprite[2], x = sprite[3];
        uint64_t pattern = sprite_slot_pattern(p, cart, sprite, (unsigned)fetched & 15, (fetched & 16) ? 16 : 8);

        if (!pattern)
            continue;
        unsigned id = slot || !s->zero;

        if (skipped) {
            uint8_t pixels[8], first[8] = {0};

            memcpy(pixels, &pattern, sizeof pixels);
            first[0] = pixels[(attr & 0x40) ? 7 : 0];
            pixels[(attr & 0x40) ? 7 : 0] = 0;
            uint64_t lead;

            memcpy(&lead, first, sizeof lead);
            memcpy(&pattern, pixels, sizeof pattern);
            if (lead && start < 8)
                draw_sprite_pixels(p, row, lead, id, attr & ~0x40u, 0, start);
        }
        if (pattern && x + 8 > start)
            draw_sprite_pixels(p, row, pattern, id, attr, x, start);
    }
}

#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static inline void write_color_block(uint16_t *pixels, uint8x16_t indices,
                                     uint8x16x2_t low, uint8x16x2_t high, bool high_table)
{
    indices = vandq_u8(indices, vdupq_n_u8(31));
    // Share the emphasis byte for NES colors and look up both bytes for RGB565
    uint8x16x2_t output = {{vqtbl2q_u8(low, indices), high_table ? vqtbl2q_u8(high, indices) : high.val[0]}};

    vst1q_u8((uint8_t *)pixels, vzip1q_u8(output.val[0], output.val[1]));
    vst1q_u8((uint8_t *)pixels + 16, vzip2q_u8(output.val[0], output.val[1]));
}
#endif

// A drawn row is always stored. One table lookup per byte costs less than
// comparing the row with the previous frame's palette indices.
static NES_ALWAYS_INLINE void write_row_as(const Ppu *p, uint16_t *pixels, const uint8_t *row,
                                           unsigned start, bool high_table)
{
    unsigned x = start;
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint8x16x2_t low = {{vld1q_u8(p->scan_low), vld1q_u8(p->scan_low + 16)}};
    uint8x16x2_t high = {{vld1q_u8(p->scan_high), vld1q_u8(p->scan_high + 16)}};

    for (; x + 16 <= NES_WIDTH; x += 16)
        write_color_block(pixels + x, vld1q_u8(row + x), low, high, high_table);
#endif
    for (; x < NES_WIDTH; ++x)
        pixels[x] = p->scan_low[row[x] & 31] | (uint16_t)p->scan_high[row[x] & 31] << 8;
}

static NES_ALWAYS_INLINE void write_scanline(Ppu *p, const uint8_t *indices, unsigned start, bool force)
{
    unsigned line = p->scanline;

    if (p->rgb565)
        write_row_as(p, p->frame[line], indices, start, true);
    else
        write_row_as(p, p->frame[line], indices, start, false);
    p->row_palette[line] = start && force ? UINT64_MAX : p->palette_revision;
}

// Write a sprite-free line straight from its source rows. Reading the scrolled
// pixels in place avoids storing a composed row and reloading it across the
// seam between its two copies.
static NES_ALWAYS_INLINE void write_background_row_as(const Ppu *p, uint16_t *pixels, const uint8_t *first,
                                                      const uint8_t *next, unsigned x, bool left_hidden,
                                                      bool high_table)
{
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    static const uint8_t lanes[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    uint8x16x2_t low = {{vld1q_u8(p->scan_low), vld1q_u8(p->scan_low + 16)}};
    uint8x16x2_t high = {{vld1q_u8(p->scan_high), vld1q_u8(p->scan_high + 16)}};
    unsigned out = 0, source = x;

    for (; source + 32 <= NES_WIDTH; source += 32, out += 32) {
        write_color_block(pixels + out, vld1q_u8(first + source), low, high, high_table);
        write_color_block(pixels + out + 16, vld1q_u8(first + source + 16), low, high, high_table);
    }
    if (source + 16 <= NES_WIDTH) {
        write_color_block(pixels + out, vld1q_u8(first + source), low, high, high_table);
        source += 16;
        out += 16;
    }
    if (source < NES_WIDTH) {
        // The block that crosses into the next nametable takes bytes from both rows
        uint8x16x2_t seam = {{vld1q_u8(first + NES_WIDTH - 16), vld1q_u8(next)}};
        uint8x16_t select = vaddq_u8(vld1q_u8(lanes), vdupq_n_u8((uint8_t)(source - (NES_WIDTH - 16))));

        write_color_block(pixels + out, vqtbl2q_u8(seam, select), low, high, high_table);
        source += 16;
        out += 16;
    }
    for (; out < NES_WIDTH; source += 16, out += 16)
        write_color_block(pixels + out, vld1q_u8(next + source - NES_WIDTH), low, high, high_table);
#else
    for (unsigned out = 0; out < NES_WIDTH; ++out) {
        unsigned source = x + out;
        uint8_t index = (source < NES_WIDTH ? first[source] : next[source - NES_WIDTH]) & 31;

        pixels[out] = p->scan_low[index] | (uint16_t)p->scan_high[index] << 8;
    }
#endif
    if (left_hidden) {
        // The hidden left edge shows the backdrop
        uint16_t backdrop = p->scan_low[0] | (uint16_t)p->scan_high[0] << 8;

        for (unsigned i = 0; i < 8; ++i)
            pixels[i] = backdrop;
    }
}

static NES_ALWAYS_INLINE void write_background_scanline(Ppu *p, const uint8_t *first, const uint8_t *next,
                                                        unsigned x, bool left_hidden)
{
    unsigned line = p->scanline;

    if (p->rgb565)
        write_background_row_as(p, p->frame[line], first, next, x, left_hidden, true);
    else
        write_background_row_as(p, p->frame[line], first, next, x, left_hidden, false);
    p->row_palette[line] = p->palette_revision;
}

static inline uint64_t visible_sprites(const Ppu *p)
{
    return (p->mask & 0x10) ? p->sprite_rows[(p->ctrl & 0x20) != 0][p->scanline] : 0;
}

// visible_sprites, with bit 0 for sprite zero on a line that has exact sprites
static inline uint64_t line_sprites(Ppu *p)
{
    const PpuSpriteLine *exact;

    if (p->sprite_exact && (p->mask & 0x10) && (exact = drawn_sprite_line(p)) != NULL)
        return sprite_line_mask(p, exact);
    return visible_sprites(p);
}

// A scrolled line reads one tile row from up to two physical nametables
static inline void line_nametables(const Nes *nes, unsigned *table, unsigned *next, unsigned *coarse_y)
{
    const Ppu *p = &nes->ppu;
    unsigned page = p->v & 0xc00;

    *coarse_y = (p->v >> 5) & 31;
    *table = cart_nametable(&nes->cart, page) >> 10;
    *next = ((p->v & 31) || p->fine_x) ? cart_nametable(&nes->cart, page ^ 0x400) >> 10 : *table;
}

static inline bool nametable_rows_unchanged(const Nes *nes, unsigned table, unsigned next,
                                            unsigned coarse_y, uint64_t revision)
{
    const Ppu *p = &nes->ppu;

    // The row stamps record the last write in each source table. A scrolled
    // line can draw from both, even when only a few pixels cross the seam.
    return p->nametable_row_revision[table][coarse_y] <= revision &&
           p->nametable_row_revision[next][coarse_y] <= revision;
}

static bool background_rows_unchanged(const Nes *nes, const BackgroundRowKey *previous)
{
    const Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    bool chr_changed = (cart->chr_is_ram || cart->mmc5) && cart->chr_revision != previous->chr_revision;

    if (chr_changed && !cart->linear_chr_ram)
        return false;
    unsigned table, next, coarse_y;

    line_nametables(nes, &table, &next, &coarse_y);
    if (!nametable_rows_unchanged(nes, table, next, coarse_y, previous->nametable_revision))
        return false;
    if (!chr_changed)
        return true;
    unsigned bank = (p->ctrl >> 4) & 1;

    return p->chr_row_revision[bank][table][coarse_y] <= previous->chr_revision &&
           p->chr_row_revision[bank][next][coarse_y] <= previous->chr_revision;
}

static inline BackgroundRowKey background_key(const Nes *nes)
{
    const Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    // Physical nametable rows retain their decoded pixels across a remap
    unsigned nametable_map = cart->mmc5 ? cart->mmc5->nametable_map : (unsigned)cart->mirroring;

    return (BackgroundRowKey){
        .nametable_revision = p->nametable_revision,
        .chr_revision = cart->chr_is_ram || cart->mmc5 ? cart->chr_revision : 0,
        .chr_tag = background_chr_tag(nes, (p->ctrl >> 4) & 1),
        .scroll = (p->v << 3) | p->fine_x | ((p->mask & 2) << 17) | ((uint32_t)nametable_map << 19)
    };
}

static inline void invalidate_sprite_rows(Ppu *p, unsigned y)
{
    unsigned row = y + 1;

    if (row >= NES_HEIGHT)
        return;
    unsigned word = row >> 6, shift = row & 63;

    // Cover either sprite height, including rows that an earlier Y used
    p->cached_sprite_rows[word] &= ~(UINT64_C(0xffff) << shift);

    if (shift > 48)
        p->cached_sprite_rows[word + 1] &= ~(UINT64_C(0xffff) >> (64 - shift));
}

static NES_NOINLINE void refresh_cached_oam(Ppu *p)
{
    for (unsigned offset = 0; offset < sizeof p->oam; offset += 4) {
        const uint8_t *old = p->cached_oam + offset, *current = p->oam + offset;

        if (!memcmp(old, current, 4))
            continue;
        invalidate_sprite_rows(p, old[0]);
        invalidate_sprite_rows(p, current[0]);
    }
    memcpy(p->cached_oam, p->oam, sizeof p->oam);
    p->cached_oam_revision = p->oam_revision;
}

static inline bool reuse_sprite_row(Ppu *p, uint64_t sprites)
{
    // Keep evaluating sprite zero until it hits, even when the pixels match
    return !(sprites & 1) || (p->status & SPRITE_ZERO);
}

static NES_NOINLINE uint64_t palette_stamp(Ppu *p)
{
    PpuPaletteState *entries = p->palette_cache;
    // A matching palette can reuse rows drawn before an intervening palette split
    uint8_t mode = p->mask & 0xe1;

    for (unsigned slot = 0; slot < 2; ++slot) {
        PpuPaletteState *entry = &entries[slot];

        if (entry->revision && entry->mode == mode && !memcmp(entry->bytes, p->palette, sizeof p->palette))
            return entry->revision;
    }
    // Keep the live palette. A new stamp prevents rows from matching the evicted entry
    PpuPaletteState *entry = &entries[p->palette_revision == entries[0].revision];
    uint64_t latest = entries[0].revision > entries[1].revision ? entries[0].revision : entries[1].revision;

    memcpy(entry->bytes, p->palette, sizeof p->palette);
    entry->mode = mode;
    entry->revision = latest + 1;
    return entry->revision;
}

// The stored value for an NES color under the current emphasis
static inline uint16_t pixel_value(const Ppu *p, const uint32_t *colors, unsigned color, unsigned emphasis)
{
    if (!p->rgb565)
        return (uint16_t)(color | emphasis << 6);
    uint32_t argb = colors[color];

    return (uint16_t)(((argb >> 8) & 0xf800) | ((argb >> 5) & 0x07e0) | ((argb >> 3) & 0x001f));
}

static NES_NOINLINE void update_scan_colors(Nes *nes)
{
    Ppu *p = &nes->ppu;

    p->palette_dirty = false;
    const Cartridge *cart = &nes->cart;
    uint64_t revision = p->palette_revision + 1;

    if (cart->mapper != 9 && !cart->mmc5 && cart->mirroring != MIRROR_FOUR) {
        revision = palette_stamp(p);

        if (p->palette_revision == revision)
            return;
    }
    unsigned emphasis = p->mask >> 5;
    const uint32_t *colors = ppu_colors[emphasis];

    for (unsigned i = 0; i < 32; ++i) {
        uint16_t value = pixel_value(p, colors, palette_read(p, (i & 3) ? i : 0), emphasis);

        p->scan_low[i] = (uint8_t)value;
        p->scan_high[i] = value >> 8;
    }
    p->palette_revision = revision;
}

static inline void refresh_scan_colors(Nes *nes)
{
    if (nes->ppu.palette_dirty)
        update_scan_colors(nes);
}

static inline uint16_t backdrop_value(const Ppu *p, unsigned color)
{
    return pixel_value(p, ppu_colors[p->mask >> 5], color, p->mask >> 5);
}

static inline void write_blank_scanline(Ppu *p, unsigned line, unsigned start, uint16_t color)
{
    uint16_t *pixels = p->frame[line];

    if (start || p->row_palette[line] || pixels[0] != color) {
        for (unsigned x = start; x < NES_WIDTH; ++x)
            pixels[x] = color;
        p->row_palette[line] = start ? UINT64_MAX : 0;
    }
}

// Keep this rare repaint outside the hot raster code layout
#if defined(__GNUC__) || defined(__clang__)
__attribute__((cold))
#endif
static NES_NOINLINE void repaint_blank_backdrop(Nes *nes, uint8_t previous, uint64_t tick, unsigned delay)
{
    Ppu *p = &nes->ppu;
    unsigned dot = p->dot + (unsigned)(tick - p->ticks);

    if (dot <= 1 || dot > NES_WIDTH - delay)
        return;

    // The row was filled at dot 1, so only repaint pixels after the color change
    uint8_t color = backdrop_index(p, p->v);
    const uint32_t *colors = ppu_colors[p->mask >> 5];

    if (colors[color] == colors[previous])
        return;
    if (nes->raster_stats)
        ++nes->raster_stats->redraws;
    unsigned line = p->scanline;
    unsigned start = dot - 1 + delay;
    uint16_t pixel = backdrop_value(p, color);

#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if (start <= NES_WIDTH - 16) {
        uint16_t *pixels = p->frame[line];
        uint16x8_t fill = vdupq_n_u16(pixel);

        for (unsigned x = NES_WIDTH - 16; x > start; x -= 16) {
            vst1q_u16(pixels + x, fill);
            vst1q_u16(pixels + x + 8, fill);
        }
        // The overlap keeps the first store at the exact pixel boundary
        vst1q_u16(pixels + start, fill);
        vst1q_u16(pixels + start + 8, fill);
        p->row_palette[line] = UINT64_MAX;
        return;
    }
#endif
    write_blank_scanline(p, line, start, pixel);
}

static unsigned background_tile_x(unsigned v, unsigned tiles)
{
    unsigned x = (v & 31) + tiles;
    return ((v & ~31u) ^ ((x & 32) << 5)) | (x & 31);
}

static const uint8_t *background_nametable(const Nes *nes, unsigned v)
{
    return nametable_page(nes, cart_nametable(&nes->cart, v) >> 10);
}

static uint8_t background_attribute(const Nes *nes, unsigned v)
{
    if (mmc5_extended_attributes(&nes->cart))
        return nes->cart.mmc5->exram[v & 1023] >> 6;
    unsigned shift = ((v >> 4) & 4) | (v & 2);
    return (background_nametable(nes, v)[0x3c0 | ((v >> 4) & 0x38) | ((v >> 2) & 7)] >> shift) & 3;
}

static const uint8_t *background_pattern(const Nes *nes, unsigned v)
{
    unsigned name = background_nametable(nes, v)[v & 1023];

    if (mmc5_extended_attributes(&nes->cart)) {
        unsigned tile = mmc5_extended_tile(&nes->cart, name, nes->cart.mmc5->exram[v & 1023]);

        return nes->cart.chr + tile * 16 + (v >> 12);
    }
    unsigned address = ((nes->ppu.ctrl & 0x10) << 8) | (name << 4) | (v >> 12);
    const Mmc5 *m = mmc5_background(nes);

    return (m ? m->bg_pages : nes->cart.chr_pages)[address >> 10] + (address & 1023);
}

static void finish_background_fetch(const Nes *nes, PpuBackgroundShifters *s, unsigned *v, unsigned dots)
{
    const uint8_t *pattern = background_pattern(nes, *v);

    s->pattern[0] = ((s->pattern[0] << dots) & 0xff00) | pattern[0];
    s->pattern[1] = ((s->pattern[1] << dots) & 0xff00) | pattern[8];
    unsigned fill = (1u << dots) - 1;

    s->attributes[0] = (s->attributes[0] << dots) | (s->palette & 1 ? fill : 0);
    s->attributes[1] = (s->attributes[1] << dots) | (s->palette & 2 ? fill : 0);
    s->palette = background_attribute(nes, *v);
    *v = background_tile_x(*v, 1);
}

static NES_NOINLINE void capture_blank_shifters(Nes *nes)
{
    Ppu *p = &nes->ppu;
    unsigned x = p->dot - 1, phase = x & 7, fill = (1u << phase) - 1;
    unsigned a = background_tile_x(p->v, x / 8), b = background_tile_x(a, 1);
    unsigned attr_a = background_attribute(nes, a), attr_b = background_attribute(nes, b);
    const uint8_t *first = background_pattern(nes, a);
    const uint8_t *second = background_pattern(nes, b);

    // Freeze the two rows of pattern bits and their attribute bits at the blanking edge
    p->blank_shifters = (PpuBackgroundShifters) {
        .pattern = {((first[0] << 8) | second[0]) << phase, (((first[8] << 8) | second[8]) << phase) | fill},
        .attributes = {((attr_a & 1 ? 0xff : 0) << phase) | (attr_b & 1 ? fill : 0),
                       ((attr_a & 2 ? 0xff : 0) << phase) | (attr_b & 2 ? fill : 0)},
        .palette = attr_b
    };
    p->blank_line = p->scanline;
}

static NES_NOINLINE void resume_background_row(Nes *nes, uint8_t *row, const uint8_t *indices, unsigned first)
{
    const Ppu *p = &nes->ppu;
    unsigned dot = p->resume_dot, start = dot >= 321 ? 0 : dot - 1;
    unsigned shift = dot >= 321 ? (dot - 321) & ~7u : (start & ~7u) + 16;
    // A later fine-X write can change the cached tile stream independently of the prefix
    unsigned x = p->resume_fine_x, end = shift + (x != p->fine_x ? 8 - x : 0);

    if (end > NES_WIDTH)
        end = NES_WIDTH;

    // A later repaint only needs the new tile stream and the bytes a sprite may read before it
    if (first >= end + 7) {
        unsigned begin = first - 7;

        memmove(row + begin, indices + (begin - shift), NES_WIDTH - begin);
        return;
    }

    PpuBackgroundShifters s = p->blank_shifters;
    unsigned v = p->v;

    if (dot >= 321) {
        while (dot <= 336) {
            unsigned dots = 8 - ((dot - 1) & 7);

            finish_background_fetch(nes, &s, &v, dots);
            dot += dots;
        }
    }
    // Freshly fetched tiles take another eight dots to reach the output end
    if (shift < NES_WIDTH)
        memmove(row + shift, indices, NES_WIDTH - shift);
    memset(row, 0, start);
    for (unsigned col = start; col < end;) {
        unsigned dots = 8 - (col & 7), count = end - col;
        unsigned attr0 = (s.attributes[0] << 8) | (s.palette & 1 ? 0xff : 0);
        unsigned attr1 = (s.attributes[1] << 8) | (s.palette & 2 ? 0xff : 0);
        unsigned bits = 8 - x;
        uint64_t pixels = chr_plane_bits[(s.pattern[0] >> bits) & 0xff] |
                          (chr_plane_bits[(s.pattern[1] >> bits) & 0xff] << 1) |
                          (chr_plane_bits[(attr0 >> bits) & 0xff] << 2) |
                          (chr_plane_bits[(attr1 >> bits) & 0xff] << 3);

        copy_indices(row + col, (const uint8_t *)&pixels, dots < count ? dots : count);
        col += dots;

        if (col < end)
            finish_background_fetch(nes, &s, &v, dots);
    }
}

static NES_NOINLINE void draw_restarted_scanline(Nes *nes, unsigned start, uint64_t sprites,
                                                 const PpuSpriteLine *exact)
{
    Ppu *p = &nes->ppu;
    uint8_t row[NES_WIDTH + 7];

    if (p->mask & 8) {
        const uint8_t *indices = background_indices(nes, row);

        resume_background_row(nes, row, indices, start);
        if (!(p->mask & 2))
            memset(row, 0, 8);
    } else {
        memset(row, 0, NES_WIDTH);
    }
    if (sprites) {
        memset(row + NES_WIDTH, 0, 7);
        unsigned sprite_start = p->resume_dot <= NES_WIDTH && start < p->resume_dot - 1u ?
            p->resume_dot - 1u : start;

        if (exact)
            draw_sprite_line(p, &nes->cart, row, sprite_start, exact);
        else
            draw_sprites(p, &nes->cart, row, sprite_start, sprites, NULL);
    }
    if (start && nes->ultrafast)
        return;
    p->rendered_rows[p->scanline].scroll = UINT32_MAX;
    refresh_scan_colors(nes);
    write_scanline(p, row, start, p->row_palette[p->scanline] != p->palette_revision);
}


// Sprite pattern data is fixed with decoded linear CHR RAM or mapper 0, as in regions
static inline bool fixed_sprite_chr(const Cartridge *cart)
{
    bool decoded = cart->linear_chr_ram ? cart->decoded_chr != NULL
                                        : cart->chr_rom_tile_cache && !cart->chr_is_ram;
    return decoded && (cart->linear_chr_ram || !cart->mapper);
}

// Composited sprite rows can be reused with fixed CHR or banked CHR ROM
static inline bool sprite_row_cache_ready(Ppu *p, const Cartridge *cart)
{
    if (!(p->mask & 0x10) || !(fixed_sprite_chr(cart) || (!cart->chr_is_ram && !cart->mmc5)))
        return false;
    if (p->cached_oam_revision != p->oam_revision)
        refresh_cached_oam(p);
    return true;
}

// An exact line, typically line 0 fetching the same stale sprites every
// frame, keeps its composited row under a signature of the fetched sprites
// instead of the OAM check. Its validity bit stays clear so that the usual
// renderers never reuse the row.
static NES_NOINLINE uint64_t sprite_line_signature(const Ppu *p, const PpuSpriteLine *s)
{
    const uint8_t fields[] = {s->line, s->compare, s->tall, s->settled, s->loaded, s->zero, !s->line && !p->odd};
    uint64_t signature = UINT64_C(0xcbf29ce484222325);

    for (unsigned i = 0; i < sizeof s->secondary; ++i)
        signature = (signature ^ s->secondary[i]) * UINT64_C(0x100000001b3);
    for (unsigned i = 0; i < sizeof fields; ++i)
        signature = (signature ^ fields[i]) * UINT64_C(0x100000001b3);
    return signature;
}

// Exact sprite lines use their own copy, so the usual one keeps its code
static NES_ALWAYS_INLINE void draw_line(Nes *nes, unsigned start, bool restart_background, bool fetches,
                                        const PpuSpriteLine *exact)
{
    Ppu *p = &nes->ppu;

    p->rendered_groups &= ~(UINT32_C(1) << (p->scanline / 8));
    uint64_t *row_palette = &p->row_palette[p->scanline];

    p->hit_dot = 0;
    bool mmc2 = nes->cart.mapper == 9;

    if (mmc2) {
        unsigned line = p->scanline;

        p->cached_sprite_rows[line >> 6] &= ~(UINT64_C(1) << (line & 63));

        if (fetches)
            p->mmc2_line_latches = nes->cart.mmc2.latches;
    }
    if (!rendering(p)) {
        write_blank_scanline(p, p->scanline, start, backdrop_value(p, backdrop_index(p, p->v)));
        return;
    }

    if (!mmc2)
        refresh_background(nes);
    uint64_t sprites = exact ? sprite_line_mask(p, exact) : visible_sprites(p);

    if (p->resume_line == p->scanline) {
        draw_restarted_scanline(nes, start, sprites, exact);
        return;
    }
    if (!start && !mmc2) {
        BackgroundRowKey *previous = &p->rendered_rows[p->scanline];

        if (!sprites && (p->mask & 8)) {
            BackgroundRowKey key = background_key(nes);

            // Reuse the row if its source rows survived writes elsewhere
            if (!p->palette_dirty && *row_palette == p->palette_revision &&
                (previous->scroll & ~BG_CLEAR_OUTPUT) == key.scroll && previous->chr_tag == key.chr_tag) {
                if (previous->nametable_revision == key.nametable_revision &&
                    previous->chr_revision == key.chr_revision)
                    return;
                if (background_rows_unchanged(nes, previous)) {
                    *previous = key;
                    return;
                }
            }
            *previous = key;
        } else if (sprites && (p->mask & 8) && sprite_row_cache_ready(p, &nes->cart)) {
            // Match the region renderer's composited sprite-row cache
            const Cartridge *cart = &nes->cart;
            unsigned line = p->scanline, table, next, coarse_y;
            BackgroundRowKey key = background_key(nes);
            uint32_t setup = key.scroll | (UINT32_C(1) << 31) | ((uint32_t)(p->ctrl & 0x28) << 24) |
                             ((uint32_t)(p->mask & 4) << 26);
            uint64_t sprite_chr = key.chr_revision;

            if (!fixed_sprite_chr(cart)) {
                unsigned bank = (p->ctrl >> 4) & 1;
                sprite_chr = cart->chr_tags[(p->ctrl & 0x20) ? bank ^ 1 : (p->ctrl >> 3) & 1];
            }
            uint64_t *valid = &p->cached_sprite_rows[line >> 6];
            uint64_t bit = UINT64_C(1) << (line & 63);
            bool cached = (*valid & bit) != 0;
            uint64_t signature = 0;

            if (exact) {
                signature = sprite_line_signature(p, exact);
                cached = !cached && p->exact_row_signature == signature;
            }
            line_nametables(nes, &table, &next, &coarse_y);
            if (!p->palette_dirty && *row_palette == p->palette_revision && cached &&
                previous->scroll == setup && previous->chr_tag == key.chr_tag &&
                previous->chr_revision == sprite_chr &&
                nametable_rows_unchanged(nes, table, next, coarse_y, previous->nametable_revision) &&
                reuse_sprite_row(p, sprites))
                return;
            *previous = key;
            previous->scroll = setup;
            previous->chr_revision = sprite_chr;

            if (exact) {
                *valid &= ~bit;
                p->exact_row_signature = signature;
            } else {
                *valid |= bit;
            }
        } else {
            previous->scroll = UINT32_MAX;
        }
    }
    uint8_t row[NES_WIDTH + 7];
    Mmc2SpriteFetches fetches_state;
    uint8_t latches = p->mmc2_line_latches;
    const uint8_t *indices = !mmc2 ? row : nes->cart.mmc4 ? mmc4_scanline(nes, row, &fetches_state, &latches)
                                                          : mmc2_scanline(nes, row, &fetches_state, &latches);

    if (mmc2 && fetches)
        cart_mmc2_set_latches(&nes->cart, latches);
    // MMC2 fetches above still update the latch before we reuse the output
    bool mmc2_background = mmc2 && !start && !sprites && (p->mask & 8) && indices != row;

    if (mmc2_background && (p->mmc2_background[p->scanline].setup & MMC2_ROW_DRAWN) &&
        !p->palette_dirty && *row_palette == p->palette_revision)
        return;
    if (p->mask & 8) {
        if (!mmc2)
            indices = background_indices(nes, row);
        if (restart_background || !(p->mask & 2) || sprites) {
            if (indices != row)
                memcpy(row, indices, NES_WIDTH);
            if (restart_background)
                memmove(row + start, row, NES_WIDTH - start);
            if (!(p->mask & 2))
                memset(row, 0, 8);
            indices = row;
        }
    } else {
        memset(row, 0, NES_WIDTH);
        indices = row;
    }
    if (sprites) {
        // The last sprite can load eight bytes at x = 255
        memset(row + NES_WIDTH, 0, 7);
        if (exact)
            draw_sprite_line(p, &nes->cart, row, start, exact);
        else
            draw_sprites(p, &nes->cart, row, start, sprites, mmc2 ? &fetches_state : NULL);
    }
    if (start && nes->ultrafast)
        return;
    if (mmc2) {
        uint64_t *setup = &p->mmc2_background[p->scanline].setup;

        if (mmc2_background)
            *setup |= MMC2_ROW_DRAWN;
        else
            *setup &= ~(uint64_t)MMC2_ROW_DRAWN;
    }

    refresh_scan_colors(nes);
    bool force = *row_palette != p->palette_revision;

    // Give the common full-row SIMD writer a constant starting column
    if (start) {
        if (!mmc2)
            p->rendered_rows[p->scanline].scroll = UINT32_MAX;
        write_scanline(p, indices, start, force);
    } else {
        write_scanline(p, indices, 0, force);
    }
}

static NES_NOINLINE void draw_exact_scanline(Nes *nes, unsigned start, bool restart_background, bool fetches,
                                             const PpuSpriteLine *exact)
{
    draw_line(nes, start, restart_background, fetches, exact);
}

static void draw_scanline(Nes *nes, unsigned start, bool restart_background, bool fetches)
{
    Ppu *p = &nes->ppu;
    const PpuSpriteLine *exact;

    if (p->sprite_exact && (p->mask & 0x10) && (exact = drawn_sprite_line(p)) != NULL)
        draw_exact_scanline(nes, start, restart_background, fetches, exact);
    else
        draw_line(nes, start, restart_background, fetches, NULL);
}

static void repaint_pixels(Nes *nes, unsigned start)
{
    Ppu *p = &nes->ppu;
    unsigned hit = p->hit_dot;
    uint8_t status = p->status;

    if (nes->raster_stats)
        ++nes->raster_stats->redraws;
    draw_scanline(nes, start, false, false);
    p->hit_dot = hit;
    p->status = status;
}

static void repaint_background_bank(Nes *nes)
{
    const Ppu *p = &nes->ppu;

    if (!(p->mask & 8) || !in_visible_pixels(p))
        return;

    // The current fetch group supplies a tile two positions ahead of the beam
    unsigned start = ((p->dot - 1) & ~7u) + 16 - p->fine_x;

    if (start < NES_WIDTH)
        repaint_pixels(nes, start);
}

static NES_NOINLINE void decode_remapped_suffix(const Nes *nes, uint8_t *pixels, unsigned start)
{
    const Ppu *p = &nes->ppu;
    unsigned v = background_tile_x(p->v, (start + p->fine_x) / 8);
    unsigned y = (v >> 5) & 31, fine_y = v >> 12, bank = (p->ctrl & 0x10) >> 2;
    const uint64_t *const *pages = nes->cart.decoded_chr_pages + bank;
    unsigned limit = (NES_WIDTH - start + 7) / 8;

    while (limit) {
        const uint8_t *table = background_nametable(nes, v);
        const uint8_t *names = table + y * 32;
        const uint8_t *attributes = table + 0x3c0 + (y >> 2) * 8;
        unsigned first = v & 31;
        unsigned count = 32 - first < limit ? 32 - first : limit;

        for (unsigned column = first; column < first + count; ++column) {
            unsigned name = names[column], slot = name >> 6;
            uint64_t colors = tile_palette(attributes[column >> 2], ((y & 2) << 1) | (column & 2));
            uint64_t tile = pages[slot][(name & 63) * 8 + fine_y] | colors;

            memcpy(pixels, &tile, sizeof tile);
            pixels += 8;
        }
        limit -= count;
        v ^= 0x400 | first;
    }
}

static const uint8_t *cached_remapped_row(Nes *nes, const uint8_t *names, const uint8_t *attributes)
{
    Ppu *p = &nes->ppu;
    PpuRemappedRow *cache = &p->remapped_row;
    // Rows with the same attribute bytes can still select different quadrants
    unsigned row = (p->v >> 12) | ((p->v >> 3) & 8);
    uint64_t tag = nes->cart.chr_tags[(p->ctrl >> 4) & 1];

    // Check the actual inputs so a runahead prediction can reuse the row
    if (cache->chr != nes->cart.chr || cache->tag != tag || cache->row != row ||
        memcmp(cache->names, names, sizeof cache->names) ||
        memcmp(cache->attributes, attributes, sizeof cache->attributes)) {
        cache->chr = nes->cart.chr;
        cache->tag = tag;
        cache->row = row;
        memcpy(cache->names, names, sizeof cache->names);
        memcpy(cache->attributes, attributes, sizeof cache->attributes);
        decode_remapped_suffix(nes, cache->indices, 0);
    }
    return cache->indices;
}

void ppu_repaint_chr_mapping(Nes *nes, uint64_t tick)
{
    Ppu *p = &nes->ppu;
    unsigned dot = p->dot + (unsigned)(tick - p->ticks);

    // Sprite-free rows can be repainted directly with the new background mapping
    if (dot <= 1 || dot > NES_WIDTH || p->next_dot == 1 || p->resume_line == p->scanline || line_sprites(p))
        return;

    // Keep pixels from tiles whose pattern fetch already started. The
    // next complete fetch supplies a tile two positions ahead of the beam.
    unsigned start = ((dot + 1) & ~7u) + 16 - p->fine_x;

    // Ultrafast uses the settled mapping for the two prefetched tiles too
    if (nes->ultrafast && dot <= PPU_BACKGROUND_PREFETCH_DOTS)
        start = p->mask & 2 ? 0 : 8;
    if (start >= NES_WIDTH)
        return;
    if (nes->raster_stats)
        ++nes->raster_stats->redraws;
    p->rendered_groups &= ~(UINT32_C(1) << (p->scanline / 8));
    p->rendered_rows[p->scanline].scroll = UINT32_MAX;
    uint8_t row[NES_WIDTH + 7];
    const uint8_t *indices = row;

    if (nes->cart.decoded_chr && !nes->cart.chr_is_ram) {
        if (!((p->v & 31) | p->fine_x) && nes->cart.mirroring != MIRROR_FOUR) {
            unsigned y = (p->v >> 5) & 31;
            const uint8_t *table = background_nametable(nes, p->v);

            indices = cached_remapped_row(nes, table + y * 32, table + 0x3c0 + (y >> 2) * 8);
        } else {
            decode_remapped_suffix(nes, row + start, start);
        }
    } else {
        refresh_background(nes);
        indices = background_indices(nes, row);
    }
    refresh_scan_colors(nes);
    write_scanline(p, indices, start, p->row_palette[p->scanline] != p->palette_revision);
}

static unsigned next_pixel_event(const Nes *nes, unsigned after)
{
    const Ppu *p = &nes->ppu;
    unsigned next = p->hit_dot > after ? p->hit_dot : NES_WIDTH;

    if (nes->cart.mmc5 && after < MMC5_IRQ_DOT && next > MMC5_IRQ_DOT && rendering(p))
        return MMC5_IRQ_DOT;
    return next;
}

static void redraw_visible_pixels(Nes *nes, unsigned dot, bool restart_background)
{
    if (nes->raster_stats)
        ++nes->raster_stats->redraws;
    draw_scanline(nes, dot - 1, restart_background, false);
    nes->ppu.next_dot = next_pixel_event(nes, dot - 1);
}

static NES_NOINLINE void repaint_fine_x(Nes *nes, uint64_t tick)
{
    Ppu *p = &nes->ppu;
    unsigned dot = p->dot + (unsigned)(tick - p->ticks);

    if (dot <= 1 || dot > NES_WIDTH)
        return;

    // Fine X selects a different bit from the background shifters immediately
    unsigned previous = p->next_dot;

    redraw_visible_pixels(nes, dot, false);
    if (p->next_dot < previous) {
        uint64_t cycle = ppu_cycle_at(p, p->scanline, p->next_dot + 1);

        if (cycle < nes->ppu_deadline)
            nes->ppu_deadline = cycle;
    }
}

static void repaint_scanline(Nes *nes, unsigned previous_render)
{
    Ppu *p = &nes->ppu;

    if (!in_visible_pixels(p))
        return;

    if (nes->ultrafast) {
        schedule_early_draw_at(nes, p->ticks);
        // Recompute the CPU-visible sprite zero hit at the time of the write
        if ((p->status & SPRITE_ZERO) || !(p->mask & 8) || !(line_sprites(p) & 1)) {
            p->hit_dot = 0;
            p->next_dot = next_pixel_event(nes, p->dot - 1);
            return;
        }
    }
    unsigned dot = p->dot;

    if (previous_render == 8 && !(p->mask & 0x18) && dot > NES_WIDTH - 24 &&
        (p->v & 0x3f00) != 0x3f00) {
        // Preserve the fetched background tile at the right edge of a late split
        unsigned end = (dot - 1 + p->fine_x + 7) & ~7u;

        if (end >= NES_WIDTH + p->fine_x) {
            p->hit_dot = 0;
            p->next_dot = NES_WIDTH;
            return;
        }
        dot = end - p->fine_x + 1;
    }
    redraw_visible_pixels(nes, dot, !previous_render);
}

static unsigned line_end(const Ppu *p)
{
    return NES_DOTS_PER_SCANLINE - (p->scanline == ppu_pre_render_line(p) && p->odd && rendering(p));
}

// The dot at which the MMC3 family clocks its IRQ counter on a rendered line
static unsigned mmc3_irq_fetch_dot(const Nes *nes)
{
    unsigned dot = ppu_irq_fetch_dot(&nes->ppu);

    // With sprites at $1000, eight A12 falling edges follow the sprite pattern
    // fetches at dots 265 through 321. The MC-ACC prescaler clocks on one of
    // them, approximated by the fetch event that starts the same 8-dot slot.
    if (dot == 261 && nes->cart.submapper == 3)
        dot += 8 * ((8u - nes->cart.mcacc_count) & 7);
    return dot;
}

static inline bool mmc3_prerender_bg_clock(const Ppu *p)
{
    // With the background table high and 8x8 sprites low, the first pattern
    // fetch can clock A12 after vblank, before the usual hblank clock.
    // On other lines the preceding pattern fetch keeps the low pulse too short.
    return (p->ctrl & 0x38) == 0x10 && !(p->v & 0x1000);
}

static unsigned irq_dot(const Nes *nes)
{
    if (nes->cart.mapper == 4 && rendering(&nes->ppu)) {
        unsigned dot = mmc3_irq_fetch_dot(nes);

        if (dot)
            return dot;
    }
    return line_end(&nes->ppu);
}

void ppu_mcacc_reset(Nes *nes)
{
    Ppu *p = &nes->ppu;
    bool fetches = (p->scanline < NES_HEIGHT || p->scanline == ppu_pre_render_line(p)) &&
                   rendering(p) && ppu_irq_fetch_dot(p) == 261;
    unsigned dot = p->dot, remaining = 0;

    if (fetches)
        remaining = dot < 265 ? 8 : dot >= 321 ? 0 : 7 - (dot - 265) / 8;
    // MC-ACC: the count at the start of each later line, eight edges apart
    nes->cart.mcacc_count = remaining & 7;

    if (!fetches || dot < 257)
        return;
    // The first falling edge after the write clocks the counter. Replace any
    // event scheduled for the previous phase on this line.
    bool stale = p->next_dot - 261 <= 64 && p->next_dot != 280;
    unsigned event = remaining ? irq_dot(nes) : 0;

    if (event && event <= dot) {
        // The edge falls within the current 8-dot slot
        cart_mmc3_clock(&nes->cart, 1);
        event = 0;
    }
    if (event && (stale || event < p->next_dot))
        p->next_dot = event;
    else if (!event && stale)
        p->next_dot = p->scanline == ppu_pre_render_line(p) && dot < 280 ? 280 : line_end(p);
}

uint64_t ppu_irq_cycle(const Nes *nes)
{
    // VRC4 and FME-7 count CPU cycles and share this deadline
    const Cartridge *cart = &nes->cart;
    if (cart->mapper != 4)
        return nes_timer_irq_cycle((Nes *)nes);
    const Ppu *p = &nes->ppu;

    if (cart->mmc3.irq)
        return cart->mmc3.irq_ready_cycle > nes->cpu.cycles ? cart->mmc3.irq_ready_cycle : UINT64_MAX;
    if (p->mask_tick)
        return nes->ppu_deadline;
    if (!(p->mask & 0x18) || !mmc3_zero_asserts_irq(cart))
        return UINT64_MAX;

    unsigned line = p->scanline, dot = 0, fetch = mmc3_irq_fetch_dot(nes);
    unsigned clocks = mmc3_clocks_to_zero(cart);
    unsigned offset = cart->submapper == 3 ? 1 : MMC3_IRQ_SAMPLE_OFFSET;

    if (line < NES_HEIGHT) {
        if (p->next_dot - 261 <= 64 && p->next_dot != 280)
            dot = p->next_dot;
        else if (p->next_dot <= 257)
            dot = fetch;

        if (dot && clocks == 1)
            return ppu_cycle_at(p, line, dot + offset);
        line += clocks - (dot != 0);

        if (fetch && line < NES_HEIGHT)
            return ppu_cycle_at(p, line, fetch + offset);
    } else {
        unsigned pre_render = ppu_pre_render_line(p);

        if (cart->submapper != 3 && (line < pre_render || p->next_dot <= MMC3_FIRST_BG_FETCH) &&
            mmc3_prerender_bg_clock(p) && !--clocks)
            return ppu_cycle_at(p, pre_render, MMC3_FIRST_BG_FETCH + offset);
        if (clocks == 1) {
            if (line < pre_render || p->next_dot <= 257 || (p->next_dot == 280 && fetch > 280))
                dot = fetch;
            else if (p->next_dot - 261 <= 64 && p->next_dot != 280)
                dot = p->next_dot;

            if (dot)
                return ppu_cycle_at(p, pre_render, dot + offset);
        }
    }
    // The frame boundary wakes the CPU before an event in the next frame
    return UINT64_MAX;
}

uint64_t ppu_mmc5_irq_cycle(const Nes *nes)
{
    const Ppu *p = &nes->ppu;
    const Mmc5 *m = nes->cart.mmc5;

    if (p->mask_tick)
        return nes->ppu_deadline;
    if (p->scanline >= NES_HEIGHT || !rendering(p) || !m->irq_target)
        return UINT64_MAX;
    unsigned next_line = p->scanline + (p->next_dot > MMC5_IRQ_DOT);
    unsigned counter = m->in_frame ? m->scanline + 1u : 0;

    if (m->irq_target < counter)
        return UINT64_MAX;
    unsigned line = next_line + m->irq_target - counter;

    return line < NES_HEIGHT ? ppu_cycle_at(p, line, MMC5_IRQ_DOT + 1) : UINT64_MAX;
}

static inline bool sprite_zero_hit_pending(const Nes *nes)
{
    const Ppu *p = &nes->ppu;
    return p->hit_dot > 1 && (p->next_dot == p->hit_dot ||
        (nes->cart.mmc5 && p->next_dot == MMC5_IRQ_DOT && p->hit_dot > MMC5_IRQ_DOT));
}

// The first line from the given one where an exact line can set the sprite zero hit
static NES_NOINLINE unsigned exact_sprite_zero_row(const Ppu *p, unsigned row)
{
    unsigned first = NES_HEIGHT;

    for (unsigned i = 0; i < 2; ++i) {
        const PpuSpriteLine *s = &p->sprite_lines[i];

        // The flag is known once the first comparison has run
        if ((s->zero || s->dot <= 66) && s->line >= row && s->line < first)
            first = s->line;
    }
    return first;
}

static inline uint64_t sprite_zero_row_cycle(const Ppu *p)
{
    unsigned first = (unsigned)p->oam[0] + 1;
    unsigned row = p->scanline + (p->next_dot != 1);
    unsigned exact = p->sprite_exact ? exact_sprite_zero_row(p, row) : NES_HEIGHT;

    if (row < first)
        row = first;
    if (row < NES_HEIGHT && row - first < ((p->ctrl & 0x20) ? 16u : 8u) && row < exact)
        return ppu_cycle_at(p, row, 2);
    if (exact < NES_HEIGHT)
        return ppu_cycle_at(p, exact, 2);
    return ppu_cycle_at(p, ppu_vblank_line(p), 0);
}

uint64_t ppu_sprite_zero_cycle(const Nes *nes)
{
    const Ppu *p = &nes->ppu;

    if (sprite_zero_hit_pending(nes))
        return ppu_cycle_at(p, p->scanline, p->hit_dot + 1);
    return sprite_zero_row_cycle(p);
}

uint64_t ppu_status_cycle(Nes *nes)
{
    Ppu *p = &nes->ppu;

    if (p->mask_tick)
        return nes->ppu_deadline;
    if (sprite_zero_hit_pending(nes))
        return ppu_cycle_at(p, p->scanline, p->hit_dot + 1);

    // A read can suppress vblank one dot before the status bit is set
    uint64_t event = ppu_cycle_at(p, ppu_vblank_line(p), 1);

    if ((p->mask & 0x18) == 0x18 && !(p->status & SPRITE_ZERO)) {
        uint64_t hit = sprite_zero_row_cycle(p);

        if (hit < event)
            event = hit;
    }
    latch_sprite_overflow(p, p->ticks);
    if ((p->mask & 0x18) && !(p->status & SPRITE_OVERFLOW) && p->overflow_eval_line < NES_HEIGHT &&
        p->overflow_tick != UINT64_MAX) {
        uint64_t overflow = (p->overflow_tick + NES_PPU_DOTS_PER_CPU - 1) / NES_PPU_DOTS_PER_CPU;

        if (overflow < event)
            event = overflow;
    }
    return event;
}

// MMC2 shares latch lookup setup over shorter spans. Keep short MMC5
// IRQ-separated spans on the scanline path.
enum { MIN_REGION_LINES = 4, MIN_MMC2_REGION_LINES = 2, MIN_MMC5_REGION_LINES = 8 };

static inline void count_region(Nes *nes, unsigned lines)
{
    NesRasterStats *stats = nes->raster_stats;

    if (stats) {
        ++stats->regions;
        stats->region_lines += lines;
    }
}

static inline uint32_t background_group_mask(unsigned first, unsigned end)
{
    return ((UINT32_C(1) << ((end + 7) / 8)) - 1) & ~((UINT32_C(1) << (first / 8)) - 1);
}

static inline bool group_has_sprites(const uint64_t *rows)
{
    uint64_t any = 0;

    for (unsigned row = 0; row < 8; ++row)
        any |= rows[row];
    return any != 0;
}

static inline bool background_key_matches(const BackgroundRowKey *previous, const BackgroundRowKey *key,
                                          uint64_t nametable_revision, uint64_t chr_revision, bool linear_chr_ram)
{
    return (previous->scroll & ~BG_CLEAR_OUTPUT) == key->scroll && previous->chr_tag == key->chr_tag &&
           previous->nametable_revision >= nametable_revision &&
           (previous->chr_revision == key->chr_revision ||
            (linear_chr_ram && previous->chr_revision >= chr_revision));
}

// A region shares one horizontal scroll. Deferred scroll writes instead give
// each line its fine X and the horizontal reload that follows it.
static NES_ALWAYS_INLINE void draw_background_lines(Nes *nes, unsigned end, bool decoded, bool fixed_chr,
                                                   const uint16_t *scrolls)
{
    Ppu *p = &nes->ppu;
    const Cartridge *cart = &nes->cart;
    const uint64_t *sprite_rows = p->sprite_rows[(p->ctrl & 0x20) != 0];
    unsigned x = (p->v & 31) * 8 + p->fine_x, bank = (p->ctrl >> 4) & 1;
    bool left_hidden = !(p->mask & 2), sprites_enabled = p->mask & 0x10;

    refresh_background(nes);
    refresh_scan_colors(nes);
    BackgroundRowKey key = background_key(nes);
    // The high bits distinguish composited rows and record their sprite controls
    uint32_t sprite_layout = (UINT32_C(1) << 31) | ((uint32_t)(p->ctrl & 0x28) << 24) |
                              ((uint32_t)(p->mask & 4) << 26);
    bool cache_sprites = sprites_enabled && (fixed_chr || (!cart->chr_is_ram && !cart->mmc5));
    uint64_t sprite_chr = key.chr_revision;

    if (cache_sprites && !fixed_chr) {
        // Tall sprites can use either bank, one of which is already in the background key
        unsigned sprite_bank = (p->ctrl & 0x20) ? bank ^ 1 : (p->ctrl >> 3) & 1;

        sprite_chr = cart->chr_tags[sprite_bank];
    }
    const bool cache_groups = !cart->mmc5 && !scrolls;
    uint32_t previous_groups = p->rendered_groups;
    // Every group touched by this span needs a fresh validity check
    uint32_t groups = previous_groups & ~background_group_mask(p->scanline, end);

    if (cache_sprites && p->cached_oam_revision != p->oam_revision)
        refresh_cached_oam(p);
    const uint32_t scroll_mode = key.scroll & ~((uint32_t)0x7fff << 3) & ~(scrolls ? 7u : 0u);
    const unsigned map = cart_nametable_map(cart);
    unsigned select = (p->v >> 9) & 2;
    unsigned other = x ? select ^ 2 : select;
    unsigned first_table[2] = { (map >> select) & 3, (map >> (select + 4)) & 3 };
    unsigned next_table[2] = { (map >> other) & 3, (map >> (other + 4)) & 3 };

    unsigned source_row = UINT32_MAX, blank_lines = 0;
    uint64_t nametable_revision = 0, chr_revision = 0;
    uint8_t (*first_rows)[NES_WIDTH] = NULL, (*next_rows)[NES_WIDTH] = NULL;

    while (p->scanline < end) {
        unsigned fine_y = p->v >> 12;
        unsigned stop = fine_y + end - p->scanline;

        if (stop > 8)
            stop = 8;
        unsigned coarse_y = (p->v >> 5) & 31;
        unsigned first, next;

        if (scrolls) {
            p->fine_x = *scrolls & 7;
            x = (p->v & 31) * 8 + p->fine_x;
            unsigned shift = (p->v >> 9) & 6;

            first = (map >> shift) & 3;
            next = x ? (map >> (shift ^ 2)) & 3 : first;
            stop = fine_y + 1;
        } else {
            unsigned vertical_page = (p->v >> 11) & 1;

            first = first_table[vertical_page];
            next = next_table[vertical_page];
        }
        unsigned row = (coarse_y << 4) | (first << 2) | next;

        // Deferred lines can change the offset without changing either source row
        if (!scrolls || row != source_row) {
            source_row = row;
            first_rows = next_rows = NULL;
            blank_lines = 0;
            nametable_revision = p->nametable_row_revision[first][coarse_y];

            if (p->nametable_row_revision[next][coarse_y] > nametable_revision)
                nametable_revision = p->nametable_row_revision[next][coarse_y];
            if (cart->linear_chr_ram) {
                chr_revision = p->chr_row_revision[bank][first][coarse_y];

                if (p->chr_row_revision[bank][next][coarse_y] > chr_revision)
                    chr_revision = p->chr_row_revision[bank][next][coarse_y];
            }
        }

        key.scroll = scroll_mode | ((uint32_t)p->v << 3) | (scrolls ? p->fine_x : 0);
        unsigned group_line = p->scanline;

        if (cache_groups && !fine_y && stop == 8 && !(group_line & 7) &&
            (!sprites_enabled || !group_has_sprites(sprite_rows + group_line))) {
            // All eight lines share the same source tiles and palette
            uint32_t bit = UINT32_C(1) << (group_line / 8);
            const BackgroundRowKey *old = &p->rendered_rows[group_line];

            groups |= bit;

            if ((previous_groups & bit) && p->row_palette[group_line] == p->palette_revision &&
                background_key_matches(old, &key, nametable_revision, chr_revision, cart->linear_chr_ram)) {
                p->scanline += 8;
                p->hit_dot = 0;
                p->v |= 0x7000;
                increment_y(p);
                continue;
            }
        }
        for (unsigned y = fine_y; y < stop; ++y, ++p->scanline, key.scroll += 0x8000) {
            unsigned line = p->scanline;
            uint64_t sprites = sprites_enabled ? sprite_rows[line] : 0;
            BackgroundRowKey *previous = &p->rendered_rows[line];
            bool force = p->row_palette[line] != p->palette_revision;

            p->hit_dot = 0;
            bool clear_output = !force && previous->scroll != UINT32_MAX && (previous->scroll & BG_CLEAR_OUTPUT);

            if (!sprites) {
                // Keep the revisions of the last render while its source rows still match
                if (!force && background_key_matches(previous, &key, nametable_revision,
                                                     chr_revision, cart->linear_chr_ram))
                    continue;
                *previous = key;
            } else if (cache_sprites) {
                uint64_t *valid = &p->cached_sprite_rows[line >> 6];
                uint64_t bit = UINT64_C(1) << (line & 63);
                uint32_t setup = key.scroll | sprite_layout;

                if (!force && (*valid & bit) && previous->scroll == setup &&
                    previous->chr_tag == key.chr_tag && previous->nametable_revision >= nametable_revision &&
                    previous->chr_revision == sprite_chr && reuse_sprite_row(p, sprites))
                    continue;
                *previous = key;
                previous->scroll = setup;
                // With banked CHR ROM, this field records the sprite bank mapping
                previous->chr_revision = sprite_chr;
                *valid |= bit;
            } else {
                previous->scroll = UINT32_MAX;
            }

            if (decoded && !first_rows) {
                // A decoded tile row caches all eight scanlines under one mapping tag
                background_row_with_tag(nes, first, coarse_y, 0, bank, key.chr_tag);
                first_rows = &p->background[bank][first][coarse_y * 8];
                next_rows = first_rows;

                if (x && next != first) {
                    background_row_with_tag(nes, next, coarse_y, 0, bank, key.chr_tag);
                    next_rows = &p->background[bank][next][coarse_y * 8];
                }
                // Lines whose source tiles are transparent show only the backdrop, whatever the scroll
                blank_lines = p->background_blank[bank][first][coarse_y] &
                              p->background_blank[bank][next][coarse_y];
            }
            bool clear_background = (blank_lines >> y) & 1;

            if (clear_background && !sprites) {
                if (!clear_output) {
                    write_blank_scanline(p, line, 0, backdrop_value(p, palette_read(p, 0)));
                }
                p->row_palette[line] = p->palette_revision;
                previous->scroll |= BG_CLEAR_OUTPUT;
                continue;
            }
            const uint8_t *indices = decoded ? first_rows[y] :
                background_row_with_tag(nes, first, coarse_y, y, bank, key.chr_tag);

            // An unscrolled, unclipped row is written from its source row as is
            if (!sprites && (x || left_hidden)) {
                const uint8_t *other = !x ? indices : decoded ? next_rows[y] :
                    next == first ? indices : background_row_with_tag(nes, next, coarse_y, y, bank, key.chr_tag);

                write_background_scanline(p, indices, other, x, left_hidden);
                continue;
            }
            uint8_t row[NES_WIDTH + 7];

            if (clear_background) {
                memset(row, 0, NES_WIDTH);
                indices = row;
            } else if (x) {
                const uint8_t *other = decoded ? next_rows[y] :
                    next == first ? indices : background_row_with_tag(nes, next, coarse_y, y, bank, key.chr_tag);

                copy_indices(row, indices + x, NES_WIDTH - x);
                copy_indices(row + NES_WIDTH - x, other, x);
                indices = row;
            } else if (left_hidden || sprites) {
                memcpy(row, indices, NES_WIDTH);
                indices = row;
            }
            if (left_hidden)
                memset(row, 0, 8);
            if (sprites) {
                memset(row + NES_WIDTH, 0, 7);
                draw_sprites(p, cart, row, 0, sprites, NULL);
                if (p->hit_dot)
                    p->status |= SPRITE_ZERO;
            }
            write_scanline(p, indices, 0, force);
        }

        if (scrolls) {
            increment_y(p);
            begin_sprite_fetches(p, *scrolls++ >> 3);
            continue;
        }
        // Only a tile-row boundary can change the coarse scroll and nametable
        p->v += (stop - fine_y - 1) << 12;
        increment_y(p);
    }
    p->rendered_groups = groups;
}

static NES_NOINLINE void draw_background_region(Nes *nes, unsigned lines)
{
    count_region(nes, lines);
    const Cartridge *cart = &nes->cart;
    unsigned end = nes->ppu.scanline + lines;
    bool decoded = cart->linear_chr_ram ? cart->decoded_chr != NULL : cart->chr_rom_tile_cache && !cart->chr_is_ram;

    if (decoded)
        draw_background_lines(nes, end, true, cart->linear_chr_ram || !cart->mapper, NULL);
    else
        draw_background_lines(nes, end, false, false, NULL);
    nes->ppu.oam_addr = 0;
}

// Draw the complete lines left by deferred scroll writes, starting below the current line
static NES_NOINLINE void draw_deferred_lines(Nes *nes, unsigned count)
{
    const Cartridge *cart = &nes->cart;
    Ppu *p = &nes->ppu;
    unsigned end = ++p->scanline + count;
    bool decoded = cart->linear_chr_ram ? cart->decoded_chr != NULL : cart->chr_rom_tile_cache && !cart->chr_is_ram;

    if (decoded)
        draw_background_lines(nes, end, true, cart->linear_chr_ram || !cart->mapper, p->deferred_scroll);
    else
        draw_background_lines(nes, end, false, false, p->deferred_scroll);
    --p->scanline;
}

enum { MMC2_OUTPUT_LATCH_SHIFT = 62 };

static inline uint64_t mmc2_output_layout(const Nes *nes)
{
    const Ppu *p = &nes->ppu;
    uint32_t banks;

    memcpy(&banks, nes->cart.mmc2.chr, sizeof banks);
    // The low 15 bits hold the scroll address and bits 18–19 hold the incoming latches
    return ((uint64_t)banks << 30) | ((uint64_t)nes->cart.mirroring << 27) |
           ((uint64_t)(p->mask & 6) << 24) | ((uint64_t)(p->ctrl & 0x38) << 19) |
           ((uint64_t)p->fine_x << 15);
}

static NES_ALWAYS_INLINE void draw_mmc24_region(Nes *nes, unsigned lines, bool mmc4)
{
    count_region(nes, lines);
    Ppu *p = &nes->ppu;
    Cartridge *cart = &nes->cart;
    const uint64_t *sprite_rows = p->sprite_rows[(p->ctrl & 0x20) != 0];
    bool sprites_enabled = p->mask & 0x10, left_hidden = !(p->mask & 2);
    uint8_t latches = cart->mmc2.latches;
    unsigned end = p->scanline + lines;
    uint8_t row[NES_WIDTH + 7];
    Mmc2SpriteFetches fetches;
    // Bank addresses stay fixed through a region while latch selection can change
    const uint64_t *decoded_banks[4];

    for (unsigned i = 0; i < 4; ++i)
        decoded_banks[i] = cart->decoded_chr + cart->mmc2.chr[i] * (4096 / 2);
    refresh_scan_colors(nes);

    unsigned window = (p->ctrl >> 4) & 1;
    uint64_t layout = mmc2_background_layout(nes, window);
    uint64_t sprite_layout = mmc2_output_layout(nes);

    if (sprites_enabled && p->cached_oam_revision != p->oam_revision)
        refresh_cached_oam(p);
    const unsigned map = cart_nametable_map(cart);
    unsigned select = (p->v >> 9) & 2, other = select ^ 2;
    unsigned first_table[2] = { (map >> select) & 3, (map >> (select + 4)) & 3 };
    unsigned next_table[2] = { (map >> other) & 3, (map >> (other + 4)) & 3 };

    while (p->scanline < end) {
        unsigned stop = p->scanline + 8 - (p->v >> 12);

        if (stop > end)
            stop = end;
        unsigned page_y = (p->v >> 11) & 1, coarse_y = (p->v >> 5) & 31;
        unsigned first = first_table[page_y], next = next_table[page_y];
        uint64_t revision = p->nametable_row_revision[first][coarse_y];

        if (p->nametable_row_revision[next][coarse_y] > revision)
            revision = p->nametable_row_revision[next][coarse_y];
        for (; p->scanline < stop; ++p->scanline, p->v += 0x1000) {
            unsigned line = p->scanline;
            uint64_t sprites = sprites_enabled ? sprite_rows[line] : 0;

            p->hit_dot = 0;
            p->mmc2_line_latches = latches;
            bool force = p->row_palette[line] != p->palette_revision;
            uint64_t bit = UINT64_C(1) << (line & 63);
            uint64_t *valid = &p->cached_sprite_rows[line >> 6];
            Mmc2RowKey *output = &p->mmc2_output[line];
            uint64_t setup = sprite_layout | p->v | ((uint64_t)latches << 18);

            if (sprites && !force && (*valid & bit) &&
                (output->setup & ((UINT64_C(1) << MMC2_OUTPUT_LATCH_SHIFT) - 1)) == setup &&
                output->nametable_revision == revision && reuse_sprite_row(p, sprites)) {
                latches = output->setup >> MMC2_OUTPUT_LATCH_SHIFT;
                continue;
            }
            *valid &= ~bit;
            mmc2_fetch_sprites(p, cart, &fetches, latches, decoded_banks, mmc4);
            latches = fetches.latches;
            const uint8_t *indices = mmc2_background_row(nes, &latches, window, layout, revision);
            uint64_t *cached = &p->mmc2_background[line].setup;
            bool reusable = !sprites;

            if (reusable && !force && (*cached & MMC2_ROW_DRAWN))
                continue;
            if (left_hidden || sprites) {
                memcpy(row, indices, NES_WIDTH);
                if (left_hidden)
                    memset(row, 0, 8);
                indices = row;
            }
            if (sprites) {
                memset(row + NES_WIDTH, 0, 7);
                draw_sprites(p, cart, row, 0, sprites, &fetches);
                if (p->hit_dot)
                    p->status |= SPRITE_ZERO;
                *output = (Mmc2RowKey){setup | ((uint64_t)latches << MMC2_OUTPUT_LATCH_SHIFT), revision};
                *valid |= bit;
            }
            if (reusable)
                *cached |= MMC2_ROW_DRAWN;
            else
                *cached &= ~(uint64_t)MMC2_ROW_DRAWN;
            write_scanline(p, indices, 0, force);
        }
        // Use the full vertical reload only at the end of the tile-row span
        p->v -= 0x1000;
        increment_y(p);
    }
    // Publish the result of the ordered fetches before any CPU access
    cart_mmc2_set_latches(cart, latches);
    p->oam_addr = 0;
}

static NES_NOINLINE void draw_mmc2_region(Nes *nes, unsigned lines)
{
    draw_mmc24_region(nes, lines, false);
}

static NES_NOINLINE void draw_mmc4_region(Nes *nes, unsigned lines)
{
    draw_mmc24_region(nes, lines, true);
}

static NES_NOINLINE void draw_blank_region(Nes *nes, unsigned rows)
{
    Ppu *p = &nes->ppu;

    count_region(nes, rows);
    uint16_t color = backdrop_value(p, backdrop_index(p, p->v));
    unsigned end = p->scanline + rows;

    p->rendered_groups &= ~background_group_mask(p->scanline, end);

    for (; p->scanline < end; ++p->scanline)
        write_blank_scanline(p, p->scanline, 0, color);
    p->hit_dot = 0;

    if (nes->cart.mapper == 9)
        p->mmc2_line_latches = nes->cart.mmc2.latches;
}

static NES_ALWAYS_INLINE void advance_raster_span(Nes *nes, unsigned dots, bool draw_regions)
{
    Ppu *p = &nes->ppu;
    Mmc5 *mmc5 = nes->cart.mmc5;
    unsigned pre_render = ppu_pre_render_line(p);
    unsigned vblank = ppu_vblank_line(p);
    unsigned dot = p->dot + dots;
    unsigned mmc3_clocks = 0, mmc3_irq_delay = 0;
    bool mmc2_region = draw_regions && nes->cart.mapper == 9;
    unsigned minimum = mmc2_region ? MIN_MMC2_REGION_LINES :
                       mmc5 ? MIN_MMC5_REGION_LINES : MIN_REGION_LINES;
    bool mmc3_render_clock = nes->cart.mapper == 4 && ppu_irq_fetch_dot(p);

    if (p->early_draw_tick && dot > PPU_EARLY_DRAW_DOT) {
        // Pixel output can sample the settled state without changing sprite event timing
        repaint_pixels(nes, 0);
        p->early_draw_tick = 0;
    }
    while (dot >= p->next_dot) {
        unsigned event = p->next_dot;

        if (event >= NES_DOTS_PER_SCANLINE - 1) {
            // Blank lines between vblank and pre-render have no dot events
            if (p->scanline >= NES_HEIGHT && p->scanline < pre_render) {
                unsigned next = p->scanline < vblank ? vblank : pre_render;
                unsigned lines = next - p->scanline;

                if (dot >= lines * NES_DOTS_PER_SCANLINE) {
                    dot -= lines * NES_DOTS_PER_SCANLINE;
                    p->scanline = next;
                    p->next_dot = 1;
                    continue;
                }
                unsigned elapsed = dot / NES_DOTS_PER_SCANLINE;

                p->scanline += elapsed;
                dot -= elapsed * NES_DOTS_PER_SCANLINE;
                break;
            }
            dot -= event;

            if (++p->scanline > pre_render) {
                p->scanline = 0;
                p->odd = !p->odd;
                // CPU-timed PCM must keep NTSC speed, but the APU pauses during
                // added lines. Leave them out after a frame that streamed PCM.
                Apu *a = &nes->apu;

                p->blanking_lines = a->dac_writes < NES_PCM_STREAM_WRITES ? p->extra_blanking_lines : 0;
                a->blanking = p->blanking_lines != 0;
                a->dac_writes = 0;
                pre_render = ppu_pre_render_line(p);
                vblank = ppu_vblank_line(p);
            }
            p->next_dot = (p->scanline < NES_HEIGHT || p->scanline == vblank ||
                           p->scanline == pre_render) ? 1 : NES_DOTS_PER_SCANLINE;
            continue;
        }
        if (dot == event)
            break;
        switch (event) {
        case 1:
            if (p->scanline < NES_HEIGHT) {
                // Complete lines can share a backdrop or a stable horizontal reload
                bool blank = !(p->mask & 0x18);

                // A line with exact sprites uses the line renderer
                if (draw_regions && dot >= minimum * NES_DOTS_PER_SCANLINE && p->scanline + minimum <= NES_HEIGHT &&
                    (blank || ((p->mask & 8) && !((p->v ^ p->t) & 0x41f))) && p->scanline != p->resume_line &&
                    (blank || !p->sprite_exact || rows_before_sprite_line(p, dot / NES_DOTS_PER_SCANLINE) >= minimum)) {
                    unsigned rows = dot / NES_DOTS_PER_SCANLINE;

                    if (rows > NES_HEIGHT - p->scanline)
                        rows = NES_HEIGHT - p->scanline;
                    if (p->sprite_exact && !blank)
                        rows = rows_before_sprite_line(p, rows);
                    if (blank) {
                        draw_blank_region(nes, rows);
                    } else {
                        if (mmc2_region && nes->cart.mmc4)
                            draw_mmc4_region(nes, rows);
                        else if (mmc2_region)
                            draw_mmc2_region(nes, rows);
                        else
                            draw_background_region(nes, rows);
                        if (mmc5)
                            clock_mmc5_scanlines(mmc5, rows);
                        if (mmc3_render_clock)
                            mmc3_clocks += rows;
                    }
                    dot -= rows * NES_DOTS_PER_SCANLINE;
                    p->next_dot = p->scanline < NES_HEIGHT ? 1 : NES_DOTS_PER_SCANLINE;
                    break;
                }
                if (nes->raster_stats)
                    ++nes->raster_stats->scanlines;
                draw_scanline(nes, 0, false, true);
                // All visible-line events are due when the span covers the whole line
                if (dot >= NES_DOTS_PER_SCANLINE) {
                    if (p->hit_dot)
                        p->status |= SPRITE_ZERO;
                    if (mmc5 && (p->mask & 0x18))
                        clock_mmc5_scanlines(mmc5, 1);
                    goto finish_visible_line;
                }
                if (p->hit_dot == 1)
                    p->status |= SPRITE_ZERO;
                p->next_dot = next_pixel_event(nes, 1);
            } else if (p->scanline == vblank) {
                if (mmc5) {
                    mmc5->in_frame = false;

                    if ((p->ctrl & 0x80) && !p->suppress_vblank)
                        mmc5->irq_pending = false;
                }
                if (!p->suppress_vblank)
                    p->status |= VBLANK;
                finish_sprite_frame(p);
                p->blank_line = p->resume_line = NES_HEIGHT;
                ++p->frames;
                // Keep elapsed time from a prefetch boundary within 32 bits
                p->prefetch_since = (uint32_t)p->ticks;
                p->next_dot = NES_DOTS_PER_SCANLINE;
            } else {
                p->status &= ~(SPRITE_ZERO | SPRITE_OVERFLOW | VBLANK);
                p->overflow_eval_line = 0;
                p->eval_line = 0;
                p->overflow_tick = 0;

                if (p->oam_corruption_row && rendering(p))
                    corrupt_oam_row(p);
                p->suppress_vblank = false;
                p->next_dot = nes->cart.mapper == 4 && nes->cart.submapper != 3 ? MMC3_FIRST_BG_FETCH : 256;
            }
            break;
        case MMC3_FIRST_BG_FETCH:
            if (p->scanline != pre_render)
                goto sprite_zero;
            p->next_dot = 256;
            if (rendering(p) && mmc3_prerender_bg_clock(p))
                goto clock_mmc3;
            break;
        case 256:
            // The pixel events already ran during an earlier synchronization
            if (dot >= NES_DOTS_PER_SCANLINE && p->scanline < NES_HEIGHT)
                goto finish_visible_line;
            if (rendering(p))
                increment_y(p);
            p->next_dot = 257;
            break;
        case 257:
            if (rendering(p))
                begin_sprite_fetches(p, p->t);
            if (p->line0_sprites && p->scanline == pre_render)
                begin_line0_sprites(nes, 257);
            p->next_dot = irq_dot(nes);

            if (p->scanline == pre_render && p->next_dot > 280)
                p->next_dot = 280;
            break;
        case 261:
        case 269:
        case 277:
        case 285:
        case 293:
        case 301:
        case 309:
        case 317:
        case 325:
            p->next_dot = event < 280 && p->scanline == pre_render ? 280 : line_end(p);
            if (!rendering(p))
                break;
        clock_mmc3:
            ++mmc3_clocks;
            // The mapper counts the fetch immediately. Its IRQ can miss the
            // current CPU sample if this span stops just after the edge.
            if (dot - event < MMC3_IRQ_SAMPLE_OFFSET && nes->cart.submapper != 3)
                mmc3_irq_delay = MMC3_IRQ_SAMPLE_OFFSET - (dot - event);
            break;
        case 280:
            if (rendering(p))
                p->v = (p->v & ~0x7be0) | (p->t & 0x7be0);
            {
                unsigned irq = irq_dot(nes);
                p->next_dot = irq > 280 && irq < NES_DOTS_PER_SCANLINE - 1 ? irq : line_end(p);
            }
            break;
        case MMC5_IRQ_DOT:
            if (mmc5 && rendering(p))
                clock_mmc5_scanlines(mmc5, 1);
            if (p->hit_dot == event)
                p->status |= SPRITE_ZERO;
            p->next_dot = next_pixel_event(nes, event);
            break;
        default:
        sprite_zero:
            p->status |= SPRITE_ZERO;
            p->next_dot = next_pixel_event(nes, event);
            break;
        }
        continue;

    finish_visible_line:
        if (p->mask & 0x18) {
            increment_y(p);
            begin_sprite_fetches(p, p->t);
            if (mmc3_render_clock)
                ++mmc3_clocks;
        }
        dot -= NES_DOTS_PER_SCANLINE;
        p->next_dot = ++p->scanline < NES_HEIGHT ? 1 : NES_DOTS_PER_SCANLINE;
    }
    // The CPU observes mapper IRQs after this span, at its scheduled deadline
    if (mmc3_clocks) {
        Cartridge *cart = &nes->cart;
        bool late_irq = mmc3_irq_delay && !cart->mmc3.irq && mmc3_clocks == mmc3_clocks_to_zero(cart);

        cart_mmc3_clock(cart, mmc3_clocks);
        if (late_irq && cart->mmc3.irq) {
            uint64_t ticks = p->ticks + dots + mmc3_irq_delay;

            cart->mmc3.irq_ready_cycle = (ticks + NES_PPU_DOTS_PER_CPU - 1) / NES_PPU_DOTS_PER_CPU;
        }
    }
    p->dot = dot;
}

static NES_NOINLINE void advance_raster_lines(Nes *nes, unsigned dots)
{
    advance_raster_span(nes, dots, false);
}

static NES_NOINLINE void advance_raster_regions(Nes *nes, unsigned dots)
{
    advance_raster_span(nes, dots, true);
}

static inline void advance_raster(Nes *nes, unsigned dots)
{
    const Cartridge *cart = &nes->cart;
    unsigned span = nes->ppu.dot + dots;

    if (span >= MIN_MMC2_REGION_LINES * NES_DOTS_PER_SCANLINE &&
        (cart->mapper == 9 ? cart->decoded_chr != NULL : span >= MIN_REGION_LINES * NES_DOTS_PER_SCANLINE)) {
        advance_raster_regions(nes, dots);
        return;
    }
    advance_raster_lines(nes, dots);
}

static void resume_mmc3_sprite_fetch(Nes *nes)
{
    Ppu *p = &nes->ppu;

    if (p->next_dot != NES_DOTS_PER_SCANLINE || p->dot <= 257 || p->dot > 317 ||
        p->scanline >= NES_HEIGHT || ppu_irq_fetch_dot(p) != 261 || (p->v & 0x1000) ||
        nes->cart.submapper == 3)
        return;

    // A remaining sprite fetch can raise A12 after rendering resumes in hblank
    unsigned fetch = p->dot + ((261 - p->dot) & 7);
    uint64_t edge = p->ticks + fetch - p->dot;
    uint64_t low = p->ticks - ((uint32_t)p->ticks - p->prefetch_since);

    if (nes->cart.mmc3.a12_low > low)
        low = nes->cart.mmc3.a12_low;
    if (edge / NES_PPU_DOTS_PER_CPU >= low / NES_PPU_DOTS_PER_CPU + 3)
        p->next_dot = fetch;
}

static NES_NOINLINE void apply_render_mask(Nes *nes)
{
    Ppu *p = &nes->ppu;
    unsigned dots = (unsigned)(p->mask_tick - p->ticks);
    unsigned skip = 0;

    if (nes->ultrafast && !(p->mask & 0x18) && (p->render_mask & 8) &&
        p->blank_line < NES_HEIGHT - 1 && (p->v & 0x3f00) != 0x3f00 &&
        (p->scanline == p->blank_line || (p->scanline == p->blank_line + 1u && p->next_dot == 1))) {
        unsigned origin = p->scanline == p->blank_line ? NES_DOTS_PER_SCANLINE : 0;
        unsigned restart_dot = p->dot + dots;

        if (restart_dot > origin + 1 && restart_dot <= origin + PPU_BACKGROUND_PREFETCH_DOTS)
            skip = restart_dot - origin - 1;
    }
    // Stop at dot 1 to skip the temporary blank paint. The mask update schedules
    // the settled draw, and the brief blank interval has no raster events.
    advance_raster(nes, dots - skip);
    if (skip) {
        p->dot += skip;
        p->next_dot = NES_WIDTH;
        p->hit_dot = 0;
    }
    p->ticks = p->mask_tick;
    p->mask_tick = 0;
    unsigned previous_render = p->mask & 0x18;
    bool was_blank = !previous_render;

    latch_sprite_overflow(p, p->ticks);
    p->mask = (p->mask & ~0x18) | p->render_mask;
    bool rendered_line = p->scanline < NES_HEIGHT || p->scanline == ppu_pre_render_line(p);

    if (rendered_line && !was_blank && !(p->mask & 0x18))
        p->oam_corruption_row = (uint8_t)(secondary_oam_address(p, p->dot));
    if (was_blank == ((p->mask & 0x18) != 0))
        sprite_render_change(nes, was_blank);
    if (was_blank) {
        if ((p->mask & 0x18) && nes->cart.mapper == 4)
            resume_mmc3_sprite_fetch(nes);
        p->prefetch_since = (uint32_t)p->ticks;
    }
    if (!(p->mask & 0x18)) {
        // The row renderer skips the two tile prefetch clocks in late hblank.
        // If blanking interrupts that span, expose their effect on the VRAM address.
        if (!was_blank && p->dot > 328 &&
            (p->scanline < NES_HEIGHT || p->scanline == ppu_pre_render_line(p))) {
            uint32_t elapsed = (uint32_t)p->ticks - p->prefetch_since;
            unsigned tiles = (elapsed >= p->dot - 328) +
                             (p->dot > 336 && elapsed >= p->dot - 336);

            p->v = background_tile_x(p->v, tiles);
        }
        p->blank_line = p->resume_line = NES_HEIGHT;

        if (!was_blank && in_visible_pixels(p) && nes->cart.mapper != 9)
            capture_blank_shifters(nes);
        if (nes->cart.mapper == 4) {
            p->prefetch_since = (uint32_t)p->ticks;

            // Drop a late edge if rendering stops again before the fetch
            if (p->scanline < NES_HEIGHT && p->next_dot - 269 <= 48)
                p->next_dot = NES_DOTS_PER_SCANLINE;
        }
    } else if (was_blank && (p->mask & 8) && p->blank_line < NES_HEIGHT - 1 && (p->v & 0x3f00) != 0x3f00) {
        // This restart missed at least part of the next line's background prefetch
        if ((p->scanline == p->blank_line && p->dot >= 321) ||
            (p->scanline == p->blank_line + 1u && p->dot <= NES_WIDTH)) {
            p->resume_line = p->blank_line + 1;
            p->resume_dot = p->dot ? p->dot : 1;
            p->resume_fine_x = p->fine_x;
        }
        p->blank_line = NES_HEIGHT;
    }
    // Evaluation resumes with the first line whose sprite scan starts with rendering on
    if (was_blank && (p->mask & 0x18)) {
        p->overflow_tick = 0;
        p->overflow_eval_line = p->scanline < NES_HEIGHT ? p->scanline + (p->dot > 65) :
                                p->scanline == ppu_pre_render_line(p) && p->dot > 1 ? 0 : NES_HEIGHT;

        if (rendered_line && p->oam_corruption_row)
            corrupt_oam_row(p);
    }
    if (p->scanline == ppu_pre_render_line(p) && p->next_dot >= NES_DOTS_PER_SCANLINE - 1)
        p->next_dot = line_end(p);
    if (nes->cart.mmc5 && !(p->mask & 0x18))
        nes->cart.mmc5->in_frame = false;
    if (p->oam_addr && fetching_sprites_at(p, p->dot))
        p->oam_addr = 0;
    repaint_scanline(nes, previous_render);
}

static NES_NOINLINE unsigned advance_deferred_scroll(Nes *nes, unsigned dots)
{
    Ppu *p = &nes->ppu;
    unsigned count = p->deferred_scroll_count;
    // The last write occurred after the final line's horizontal reload
    unsigned elapsed = count * NES_DOTS_PER_SCANLINE + NES_WIDTH + 2 - p->dot;

    assert(elapsed <= dots);
    unsigned new_x = p->fine_x;

    unsigned resume = p->resume_line - p->scanline - 1;
    bool shared = (p->mask & 8) && resume >= count && !p->sprite_exact;

    if (shared && count > 1)
        count_region(nes, count);
    else if (nes->raster_stats)
        nes->raster_stats->scanlines += count;

    if (shared) {
        // Without a restarted line, every pending line has its background on.
        // Lines with exact sprites use the line renderer below.
        draw_deferred_lines(nes, count);
        count = 0;
    }
    for (unsigned i = 0; i < count; ++i) {
        unsigned scroll = p->deferred_scroll[i];

        p->fine_x = scroll & 7;
        ++p->scanline;
        draw_scanline(nes, 0, false, true);
        if (p->hit_dot)
            p->status |= SPRITE_ZERO;
        increment_y(p);
        begin_sprite_fetches(p, scroll >> 3);
    }
    p->fine_x = new_x;
    p->deferred_scroll_count = 0;
    p->dot = NES_WIDTH + 2;
    p->next_dot = NES_DOTS_PER_SCANLINE;
    p->ticks += elapsed;
    return dots - elapsed;
}

void ppu_advance(Nes *nes, unsigned dots)
{
    Ppu *p = &nes->ppu;

    p->status_deadline = 0;
    uint64_t target = p->ticks + dots;

    if (p->deferred_scroll_count)
        dots = advance_deferred_scroll(nes, dots);
    if (p->mask_tick && p->mask_tick <= target) {
        apply_render_mask(nes);
        dots = (unsigned)(target - p->ticks);
    }
    advance_raster(nes, dots);
    p->ticks = target;
}


void ppu_frame_to_argb(const Ppu *p, const uint16_t (*frame)[NES_WIDTH], unsigned first_line,
                       unsigned lines, uint32_t *argb, size_t pitch)
{
    if (p->rgb565) {
        for (unsigned y = 0; y < lines; ++y, argb += pitch) {
            const uint16_t *row = frame[first_line + y];

            for (unsigned x = 0; x < NES_WIDTH; ++x) {
                unsigned v = row[x], r = v >> 11, g = (v >> 5) & 63, b = v & 31;
                argb[x] = 0xff000000u | (r << 3 | r >> 2) << 16 | (g << 2 | g >> 4) << 8 | (b << 3 | b >> 2);
            }
        }
        return;
    }
    for (unsigned y = 0; y < lines; ++y, argb += pitch) {
        const uint16_t *row = frame[first_line + y];
        unsigned x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        for (; x + 16 <= NES_WIDTH; x += 16) {
            uint16x8_t low = vld1q_u16(row + x), high = vld1q_u16(row + x + 8);
            // Look up a run that shares one emphasis setting in the byte planes
            uint16x8_t modes = vorrq_u16(vshrq_n_u16(low, 6), vshlq_n_u16(vshrq_n_u16(high, 6), 8));
            unsigned emphasis = vgetq_lane_u16(low, 0) >> 6;

            if (vmaxvq_u16(vabdq_u16(modes, vdupq_n_u16(emphasis | emphasis << 8))))
                break;
            uint8x16_t index = vandq_u8(vcombine_u8(vmovn_u16(low), vmovn_u16(high)), vdupq_n_u8(63));
            const uint8_t (*planes)[64] = color_planes[emphasis];
            uint8x16x4_t blue = vld1q_u8_x4(planes[0]), green = vld1q_u8_x4(planes[1]),
                         red = vld1q_u8_x4(planes[2]);
            uint8x16x4_t output = {{
                vqtbl4q_u8(blue, index), vqtbl4q_u8(green, index), vqtbl4q_u8(red, index), vdupq_n_u8(255)
            }};

            vst4q_u8((uint8_t *)(argb + x), output);
        }
#endif
        for (; x < NES_WIDTH; ++x)
            argb[x] = ppu_colors[(row[x] >> 6) & 7][row[x] & 63];
    }
}
