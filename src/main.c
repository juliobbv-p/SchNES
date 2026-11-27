#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include "nes.h"
#include "png_write.h"

#include <SDL.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

uint32_t crc32_update_argb8888(uint32_t crc, const uint32_t *pixels, size_t count);

#ifdef __APPLE__
void schnes_set_dock_icon(void);
#endif

static const char app_name[] = "SchNES";
enum { CROPPED_HEIGHT = 224 };

typedef enum {
    MODE_REGULAR, MODE_ULTRAFAST, MODE_INTERPRETER, MODE_COUNT
} EmulationMode;

static const struct {
    const char *name, *window_suffix;
    SDL_Color color;
} emulation_modes[MODE_COUNT] = {
    [MODE_REGULAR] = {"REGULAR", "", {137, 227, 208, 255}},
    [MODE_ULTRAFAST] = {"ULTRAFAST", " Ultrafast", {255, 69, 0, 255}},
    [MODE_INTERPRETER] = {"INTERPRETER", " Interpreter", {104, 232, 116, 255}},
};

static EmulationMode emulation_mode(const Nes *nes)
{
    return nes->interpreter ? MODE_INTERPRETER : nes->ultrafast ? MODE_ULTRAFAST : MODE_REGULAR;
}

typedef enum {
    RUNAHEAD_OFF, RUNAHEAD_ONE_FRAME, RUNAHEAD_TWO_FRAMES
} RunaheadDepth;

static const char *cycle_runahead(RunaheadDepth *depth)
{
    static const char *const notices[] = {
        [RUNAHEAD_OFF] = "RUNAHEAD OFF",
        [RUNAHEAD_ONE_FRAME] = "RUNAHEAD 1 FRAME",
        [RUNAHEAD_TWO_FRAMES] = "RUNAHEAD 2 FRAMES",
    };

    *depth = *depth == RUNAHEAD_OFF ? RUNAHEAD_TWO_FRAMES : *depth - 1;
    return notices[*depth];
}

enum {
    PAD_B = 2, PAD_UP = 16, PAD_DOWN = 32, PAD_LEFT = 64, PAD_RIGHT = 128
};

static void usage(FILE *stream, const char *name)
{
    fprintf(stream, "Usage: %s [-u|-i] [-n] [-s] [-f] [-r] [ROM.nes]\n"
            "       %s [-u|-i] [-n] [-s] --headless FRAMES ROM.nes\n"
            "  -u  Use ultrafast emulation at reduced accuracy\n"
            "  -i  Use Interpreter mode\n"
            "  -n  Enable No-Lag mode with 100 extra blanking scanlines at the normal NTSC frame rate\n"
            "  -s  Draw all sprites on each scanline, bypassing the eight-sprite display limit\n"
            "  -f  Show the full 256x240 view (default: 256x224 crop)\n"
            "  -r  Disable the default one frame of runahead\n", name, name);
}

static bool load_rom(Nes *nes, const char *path, NesRunahead **runahead, const char **failure)
{
    Cartridge cart = {0};
    char error[512];

    if (!cart_load(&cart, path, error, sizeof error)) {
        fprintf(stderr, "%s\n", error);
        if (failure)
            *failure = strcmp(error, cart_fds_bios_missing) ? "CANNOT OPEN ROM" : "FDS BIOS DISKSYS.ROM NOT FOUND";
        return false;
    }
    const bool ultrafast = nes->ultrafast, interpreter = nes->interpreter;
    const bool no_sprite_limit = nes->ppu.no_sprite_limit;
    const uint8_t extra_blanking_lines = nes->ppu.extra_blanking_lines;

    if (runahead) {
        nes_runahead_destroy(*runahead);
        *runahead = NULL;
    }
    cart_free(&nes->cart);
    memset(nes, 0, sizeof *nes);
    nes->cart = cart;
    nes->ultrafast = ultrafast;
    nes->interpreter = interpreter;
    nes->ppu.extra_blanking_lines = extra_blanking_lines;
    nes->ppu.no_sprite_limit = no_sprite_limit;
    nes_init(nes);
    return true;
}

static bool save_battery_ram(Nes *nes)
{
    char error[512];

    if (cart_save(&nes->cart, error, sizeof error))
        return true;
    fprintf(stderr, "%s\n", error);
    return false;
}

static SDL_Texture *begin_screenshot(SDL_Renderer *renderer)
{
    int width, height;

    SDL_RenderGetLogicalSize(renderer, &width, &height);
    SDL_Texture *target = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                                            SDL_TEXTUREACCESS_TARGET, width, height);

    if (target && SDL_SetTextureBlendMode(target, SDL_BLENDMODE_NONE) == 0 &&
        SDL_SetRenderTarget(renderer, target) == 0)
        return target;
    fprintf(stderr, "Cannot create screenshot target (%s)\n", SDL_GetError());
    SDL_DestroyTexture(target);
    return NULL;
}

static SDL_Surface *read_screenshot(SDL_Renderer *renderer, SDL_Texture *target)
{
    int width, height;

    if (SDL_QueryTexture(target, NULL, NULL, &width, &height) != 0)
        return NULL;
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ARGB8888);

    if (surface && SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_ARGB8888,
                                        surface->pixels, surface->pitch) == 0)
        return surface;
    fprintf(stderr, "Cannot read screenshot (%s)\n", SDL_GetError());
    SDL_FreeSurface(surface);
    return NULL;
}

static bool save_screenshot(const char *rom_path, const SDL_Surface *surface)
{
    size_t capacity = strlen(rom_path) + sizeof ".4294967295.png";
    char *path = malloc(capacity);
    FILE *file = NULL;
    bool saved = false;

    if (!path)
        return false;

    // Exclusive creation also handles captures from multiple emulator windows
    for (unsigned number = 1; number; ++number) {
        if (number == 1)
            snprintf(path, capacity, "%s.png", rom_path);
        else
            snprintf(path, capacity, "%s.%u.png", rom_path, number);
        file = fopen(path, "wbx");

        if (file || errno != EEXIST)
            break;
    }

    if (file) {
        saved = png_write_argb8888(file, surface->pixels, surface->w, surface->h,
                                   surface->pitch / sizeof(uint32_t));

        if (fclose(file) != 0)
            saved = false;
        if (!saved)
            remove(path);
    }
    if (!saved)
        fprintf(stderr, "Cannot save screenshot to %s\n", path);
    free(path);
    return saved;
}

static NesRunahead *open_runahead(const Cartridge *cart, RunaheadDepth *depth)
{
    if (*depth == RUNAHEAD_OFF)
        return NULL;
    NesRunahead *runahead = nes_runahead_create(cart);

    if (!runahead) {
        fprintf(stderr, "Cannot allocate runahead state. Continuing with runahead disabled.\n");
        *depth = RUNAHEAD_OFF;
    }
    return runahead;
}

static uint32_t frame_crc(const Ppu *p)
{
    // Checksum the displayed ARGB colors, one row at a time
    uint32_t crc = UINT32_MAX;
    uint32_t row[NES_WIDTH];

    for (unsigned y = 0; y < NES_HEIGHT; ++y) {
        ppu_frame_to_argb(p, p->frame, y, 1, row, NES_WIDTH);
        crc = crc32_update_argb8888(crc, row, NES_WIDTH);
    }
    return ~crc;
}

static void run_headless(Nes *nes, unsigned long long frames)
{
    uint64_t samples = 0;

    for (unsigned long long i = 0; i < frames; ++i) {
        nes_frame(nes);
        samples += nes->apu.sample_count;
    }
    printf("frames=%" PRIu64 " cycles=%" PRIu64 " video=%08" PRIx32 " samples=%" PRIu64 "\n",
           nes->ppu.frames, nes->cpu.cycles, frame_crc(&nes->ppu), samples);
}

static uint8_t read_keyboard_buttons(const Uint8 *keys, unsigned port)
{
    static const SDL_Scancode bindings[2][8] = {
        {SDL_SCANCODE_X, SDL_SCANCODE_Z, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_RETURN,
         SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT},
        {SDL_SCANCODE_G, SDL_SCANCODE_F, SDL_SCANCODE_LSHIFT, SDL_SCANCODE_TAB,
         SDL_SCANCODE_W, SDL_SCANCODE_S, SDL_SCANCODE_A, SDL_SCANCODE_D}
    };
    uint8_t state = 0;

    for (unsigned bit = 0; bit < 8; ++bit) {
        if (keys[bindings[port][bit]])
            state |= 1u << bit;
    }
    return state;
}

static uint8_t read_gamepad_buttons(SDL_GameController *pad)
{
    if (!pad)
        return 0;
    static const SDL_GameControllerButton buttons[] = {
        SDL_CONTROLLER_BUTTON_A, SDL_CONTROLLER_BUTTON_X, SDL_CONTROLLER_BUTTON_BACK,
        SDL_CONTROLLER_BUTTON_START, SDL_CONTROLLER_BUTTON_DPAD_UP, SDL_CONTROLLER_BUTTON_DPAD_DOWN,
        SDL_CONTROLLER_BUTTON_DPAD_LEFT, SDL_CONTROLLER_BUTTON_DPAD_RIGHT
    };
    uint8_t state = 0;

    for (unsigned bit = 0; bit < 8; ++bit) {
        if (SDL_GameControllerGetButton(pad, buttons[bit]))
            state |= 1u << bit;
    }
    if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_B))
        state |= PAD_B;
    Sint16 x = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTX);
    Sint16 y = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTY);

    if (x < -14000) state |= PAD_LEFT;
    if (x >  14000) state |= PAD_RIGHT;
    if (y < -14000) state |= PAD_UP;
    if (y >  14000) state |= PAD_DOWN;
    return state;
}

static uint8_t clear_opposite_directions(uint8_t state)
{
    if ((state & (PAD_LEFT | PAD_RIGHT)) == (PAD_LEFT | PAD_RIGHT))
        state &= ~(PAD_LEFT | PAD_RIGHT);
    if ((state & (PAD_UP | PAD_DOWN)) == (PAD_UP | PAD_DOWN))
        state &= ~(PAD_UP | PAD_DOWN);
    return state;
}

static void add_gamepad(SDL_GameController *pads[2], int index)
{
    if (!SDL_IsGameController(index))
        return;
    SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(index);

    for (unsigned i = 0; i < 2; ++i) {
        if (pads[i] && SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(pads[i])) == id)
            return;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (!pads[i]) {
            pads[i] = SDL_GameControllerOpen(index);
            return;
        }
    }
}

static void scan_gamepads(SDL_GameController *pads[2])
{
    for (int i = 0; i < SDL_NumJoysticks(); ++i)
        add_gamepad(pads, i);
}

static void remove_gamepad(SDL_GameController *pads[2], SDL_JoystickID id)
{
    for (unsigned i = 0; i < 2; ++i) {
        if (pads[i] && SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(pads[i])) == id) {
            SDL_GameControllerClose(pads[i]);
            pads[i] = NULL;
        }
    }
    scan_gamepads(pads);
}

typedef struct {
    SDL_GameController *pads[2];
    Uint64 next_poll, poll_interval;
    uint8_t buttons[2];
} Input;

static void refresh_input(Input *input, Uint64 now)
{
    if (now < input->next_poll)
        return;
    // Limit SDL event pumping during repeated controller strobes
    input->next_poll = now + input->poll_interval;
    SDL_PumpEvents();
    const Uint8 *keys = SDL_GetKeyboardState(NULL);

    for (unsigned port = 0; port < 2; ++port) {
        uint8_t buttons = read_keyboard_buttons(keys, port) | read_gamepad_buttons(input->pads[port]);
        input->buttons[port] = clear_opposite_directions(buttons);
    }
}

static void poll_buttons(void *context, uint8_t buttons[2])
{
    Input *input = context;

    refresh_input(input, SDL_GetPerformanceCounter());
    memcpy(buttons, input->buttons, sizeof input->buttons);
}

static void attach_input(Nes *nes, Input *input)
{
    nes->input_context = input;
    nes->poll_buttons = poll_buttons;
    input->next_poll = 0;
}

static SDL_AudioDeviceID open_audio(SDL_AudioSpec *actual)
{
    SDL_AudioSpec request = {0};
    SDL_AudioDeviceID device;

    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        fprintf(stderr, "Audio unavailable (%s)\n", SDL_GetError());
        return 0;
    }
    request.freq = NES_SAMPLE_RATE;
    request.format = AUDIO_S16SYS;
    request.channels = 1;
    request.samples = 256;
    device = SDL_OpenAudioDevice(NULL, 0, &request, actual, SDL_AUDIO_ALLOW_SAMPLES_CHANGE);

    if (!device)
        fprintf(stderr, "Audio unavailable (%s)\n", SDL_GetError());
    return device;
}

static void set_window_title(SDL_Window *window, const char *path, const Nes *nes, bool paused)
{
    const char *name = path;

    for (const char *p = path; *p; ++p) {
        if (*p == '/' || *p == '\\')
            name = p + 1;
    }
    char title[256];
    const char *mode = emulation_modes[emulation_mode(nes)].window_suffix;
    const char *no_lag = nes->ppu.extra_blanking_lines ? " No-Lag" : "";

    snprintf(title, sizeof title, "%s%s%s - %s%s", app_name, mode, no_lag,
             name, paused ? " (Paused)" : "");
    SDL_SetWindowTitle(window, title);
}

static const char *cycle_cpu_mode(Nes *nes)
{
    const EmulationMode next = (emulation_mode(nes) + 1) % MODE_COUNT;

    nes->ultrafast = next == MODE_ULTRAFAST;
    nes->interpreter = next == MODE_INTERPRETER;
    return emulation_modes[next].name;
}

static const char *toggle_sprite_limit(Ppu *p)
{
    ppu_set_no_sprite_limit(p, !p->no_sprite_limit);
    return p->no_sprite_limit ? "SPRITE LIMIT OFF" : "SPRITE LIMIT ON";
}

static double display_refresh_period(SDL_Window *window, double frequency)
{
    SDL_DisplayMode mode;
    int display = SDL_GetWindowDisplayIndex(window);

    if (display >= 0 && SDL_GetCurrentDisplayMode(display, &mode) == 0 && mode.refresh_rate > 0)
        return frequency / mode.refresh_rate;
    return 0;
}

static void sleep_until(double target, double ticks_per_ms)
{
    double ms = (target - SDL_GetPerformanceCounter()) / ticks_per_ms;

    if (ms <= 0)
        return;
#ifdef _WIN32
    Uint32 whole_ms = (Uint32)ms;

    SDL_Delay(whole_ms + (whole_ms < ms));
#else
    Uint64 ns = (Uint64)(ms * 1000000.0) + 1;
    struct timespec delay = {(time_t)(ns / 1000000000), (long)(ns % 1000000000)};

    while (nanosleep(&delay, &delay) == -1 && errno == EINTR) {}
#endif
}

enum {
    STATS_RECOMPILED, STATS_LOOP_BATCHED, STATS_DIRECT, STATS_INDIVIDUAL,
    STATS_BUS_HANDLER, STATS_ROWS
};

enum {
    STATS_FRAMES = 60,
    SHARE_LENGTH = sizeof "100.0%", AVERAGE_LENGTH = sizeof "4294967295.0",
    RASTER_STATS_ROWS = 9
};

typedef enum {
    OSD_HIDDEN, OSD_INFO, OSD_CPU, OSD_RASTER, OSD_PAGE_COUNT
} OsdPage;

typedef struct {
    uint32_t totals[STATS_ROWS];
    unsigned frames;
    bool ready;

    struct {
        char count[AVERAGE_LENGTH], share[SHARE_LENGTH];
    } rows[STATS_ROWS];
} InstructionOverlay;

typedef struct {
    uint32_t totals[RASTER_STATS_ROWS];
    char averages[RASTER_STATS_ROWS][AVERAGE_LENGTH];
    unsigned frames;
    bool ready;
} RasterOverlay;

typedef struct {
    Uint64 window_start;
    uint64_t start_frame;
    double fps;
} FrameRate;

typedef struct {
    FrameRate emulation, display;
    uint64_t presented_frames;
} FrameRates;

static void sample_frame_rate(FrameRate *rate, uint64_t frame, Uint64 now, Uint64 frequency)
{
    if (rate->window_start) {
        Uint64 elapsed = now - rate->window_start;

        if (elapsed < frequency)
            return;
        rate->fps = (double)(frame - rate->start_frame) * frequency / elapsed;
    }
    rate->window_start = now;
    rate->start_frame = frame;
}

static void format_instruction_share(char share[SHARE_LENGTH], uint32_t count, uint32_t total)
{
    if (!total) {
        memcpy(share, "N/A", sizeof "N/A");
        return;
    }
    unsigned tenths = ((uint64_t)count * 1000 + total / 2) / total;

    if (tenths > 1000)
        tenths = 1000;
    snprintf(share, SHARE_LENGTH, "%u.%u%%", tenths / 10, tenths % 10);
}

static void format_stats_average(char average[AVERAGE_LENGTH], uint32_t total)
{
    uint32_t tenths = ((uint64_t)total * 10 + STATS_FRAMES / 2) / STATS_FRAMES;

    snprintf(average, AVERAGE_LENGTH, "%" PRIu32 ".%" PRIu32, tenths / 10, tenths % 10);
}

static void sample_instruction_stats(Nes *nes, InstructionOverlay *overlay)
{
    NesInstructionStats frame;

    nes_frame_with_stats(nes, &frame);
    const uint32_t counts[STATS_ROWS] = {
        [STATS_RECOMPILED] = frame.native_trace,
        [STATS_LOOP_BATCHED] = frame.skipped,
        [STATS_DIRECT] = frame.direct,
        [STATS_INDIVIDUAL] = frame.individual,
        [STATS_BUS_HANDLER] = frame.bus_finish
    };

    for (unsigned i = 0; i < STATS_ROWS; ++i)
        overlay->totals[i] += counts[i];
    if (++overlay->frames != STATS_FRAMES)
        return;

    uint32_t instructions = 0;

    for (unsigned i = 0; i < STATS_ROWS; ++i)
        instructions += overlay->totals[i];
    for (unsigned i = 0; i < STATS_ROWS; ++i) {
        format_stats_average(overlay->rows[i].count, overlay->totals[i]);
        format_instruction_share(overlay->rows[i].share, overlay->totals[i], instructions);
    }
    memset(overlay->totals, 0, sizeof overlay->totals);
    overlay->frames = 0;
    overlay->ready = true;
}

static void sample_raster_stats(Nes *nes, RasterOverlay *overlay)
{
    NesRasterStats frame;

    nes_frame_with_raster_stats(nes, &frame);
    const uint32_t counts[RASTER_STATS_ROWS] = {
        frame.scanlines + frame.regions + frame.redraws,
        frame.regions, frame.region_lines, frame.scanlines, frame.redraws,
        frame.ppu_syncs + frame.apu_syncs, frame.ppu_syncs, frame.apu_syncs, frame.bus_reschedules
    };

    for (unsigned i = 0; i < RASTER_STATS_ROWS; ++i)
        overlay->totals[i] += counts[i];
    if (++overlay->frames != STATS_FRAMES)
        return;

    for (unsigned i = 0; i < RASTER_STATS_ROWS; ++i) {
        format_stats_average(overlay->averages[i], overlay->totals[i]);
        overlay->totals[i] = 0;
    }
    overlay->frames = 0;
    overlay->ready = true;
}

enum {
    GLYPH_WIDTH = 3, GLYPH_HEIGHT = 5, GLYPH_STEP = GLYPH_WIDTH + 1,
    OSD_MAX_GLYPHS = NES_WIDTH / GLYPH_STEP,
    OSD_MAX_PIXELS = OSD_MAX_GLYPHS * GLYPH_WIDTH * GLYPH_HEIGHT,
    TITLE_GLYPH_WIDTH = 6, TITLE_GLYPH_HEIGHT = 10, TITLE_GLYPH_STEP = 8
};

// A bitmap font for the OSD text and decimal counters
static const uint8_t osd_font[128][GLYPH_HEIGHT] = {
    ['0'] = {7, 5, 5, 5, 7}, ['1'] = {2, 6, 2, 2, 7},
    ['2'] = {7, 1, 7, 4, 7}, ['3'] = {7, 1, 7, 1, 7},
    ['4'] = {5, 5, 7, 1, 1}, ['5'] = {7, 4, 7, 1, 7},
    ['6'] = {7, 4, 7, 5, 7}, ['7'] = {7, 1, 2, 2, 2},
    ['8'] = {7, 5, 7, 5, 7}, ['9'] = {7, 5, 7, 1, 7},
    ['A'] = {2, 5, 7, 5, 5}, ['B'] = {6, 5, 6, 5, 6}, ['C'] = {3, 4, 4, 4, 3},
    ['D'] = {6, 5, 5, 5, 6}, ['E'] = {7, 4, 6, 4, 7},
    ['F'] = {7, 4, 6, 4, 4}, ['G'] = {3, 4, 5, 5, 3}, ['H'] = {5, 5, 7, 5, 5},
    ['I'] = {7, 2, 2, 2, 7}, ['K'] = {5, 5, 6, 5, 5}, ['L'] = {4, 4, 4, 4, 7},
    ['M'] = {5, 7, 7, 5, 5}, ['N'] = {5, 7, 7, 7, 5},
    ['O'] = {7, 5, 5, 5, 7}, ['P'] = {6, 5, 6, 4, 4},
    ['Q'] = {7, 5, 5, 7, 1},
    ['R'] = {6, 5, 6, 5, 5}, ['S'] = {7, 4, 7, 1, 7},
    ['T'] = {7, 2, 2, 2, 2}, ['U'] = {5, 5, 5, 5, 7},
    ['V'] = {5, 5, 5, 5, 2}, ['W'] = {5, 5, 7, 7, 5}, ['X'] = {5, 5, 2, 5, 5},
    ['Y'] = {5, 5, 2, 2, 2}, ['Z'] = {7, 1, 2, 4, 7}, ['-'] = {0, 0, 7, 0, 0},
    ['.'] = {0, 0, 0, 0, 2}, ['/'] = {1, 1, 2, 4, 4}, ['%'] = {5, 1, 2, 4, 5},
    ['a'] = {0, 3, 5, 5, 3}, ['d'] = {1, 1, 3, 5, 3}, ['e'] = {0, 7, 7, 4, 3},
    ['f'] = {3, 2, 7, 2, 2}, ['g'] = {0, 3, 5, 3, 6}, ['h'] = {4, 4, 6, 5, 5},
    ['i'] = {2, 0, 2, 2, 2}, ['l'] = {6, 2, 2, 2, 3}, ['n'] = {0, 6, 5, 5, 5},
    ['o'] = {0, 2, 5, 5, 2}, ['p'] = {0, 6, 5, 6, 4}, ['r'] = {0, 3, 4, 4, 4}
};

// The drop headline uses two-pixel stems and separate rows for descenders
static const uint8_t title_font[128][TITLE_GLYPH_HEIGHT] = {
    ['D'] = {60, 54, 51, 51, 51, 51, 54, 60,  0,  0},
    ['a'] = { 0,  0, 30,  3, 31, 51, 51, 31,  0,  0},
    ['d'] = { 3,  3, 31, 51, 51, 51, 51, 31,  0,  0},
    ['e'] = { 0,  0, 30, 51, 63, 48, 48, 31,  0,  0},
    ['f'] = {14, 24, 24, 62, 24, 24, 24, 24,  0,  0},
    ['g'] = { 0,  0, 31, 51, 51, 51, 31,  3,  3, 30},
    ['h'] = {48, 48, 62, 51, 51, 51, 51, 51,  0,  0},
    ['i'] = {12,  0, 28, 12, 12, 12, 12, 30,  0,  0},
    ['l'] = {28, 12, 12, 12, 12, 12, 12,  7,  0,  0},
    ['n'] = { 0,  0, 62, 51, 51, 51, 51, 51,  0,  0},
    ['o'] = { 0,  0, 30, 51, 51, 51, 51, 30,  0,  0},
    ['p'] = { 0,  0, 62, 51, 51, 51, 62, 48, 48, 48},
    ['r'] = { 0,  0, 54, 59, 48, 48, 48, 48,  0,  0},
    ['-'] = { 0,  0,  0,  0, 63, 63,  0,  0,  0,  0},
};

typedef struct {
    const unsigned char *rows;
    unsigned width, height, step;
} BitmapFont;

static const BitmapFont osd_bitmap = {
    (const unsigned char *)&osd_font, GLYPH_WIDTH, GLYPH_HEIGHT, GLYPH_STEP
};
static const BitmapFont title_bitmap = {
    (const unsigned char *)&title_font, TITLE_GLYPH_WIDTH, TITLE_GLYPH_HEIGHT, TITLE_GLYPH_STEP
};

static int bitmap_text_points(SDL_Point *points, const BitmapFont *font, int x, int y, const char *text)
{
    const unsigned max_glyphs = NES_WIDTH / font->step;
    int count = 0;

    for (unsigned i = 0; *text && i < max_glyphs; ++i, ++text, x += (int)font->step) {
        unsigned ch = (unsigned char)*text;

        if (ch >= 128)
            continue;
        const unsigned char *glyph = font->rows + ch * font->height;

        for (unsigned row = 0; row < font->height; ++row) {
            unsigned bits = glyph[row];

            for (unsigned col = 0; col < font->width; ++col) {
                if (bits & (1u << (font->width - 1 - col)))
                    points[count++] = (SDL_Point){x + (int)col, y + (int)row};
            }
        }
    }
    return count;
}

static int draw_osd_text(SDL_Renderer *renderer, int x, int y, const char *text)
{
    SDL_Point points[OSD_MAX_PIXELS];
    int count = bitmap_text_points(points, &osd_bitmap, x, y, text);

    return SDL_RenderDrawPoints(renderer, points, count);
}

static int osd_text_width(const char *text)
{
    return *text ? (int)strlen(text) * GLYPH_STEP - 1 : 0;
}

enum { CPU_LEGEND_WIDTH = 94, CPU_DEBUG_WIDTH = NES_DOTS_PER_SCANLINE + CPU_LEGEND_WIDTH };
typedef struct {
    SDL_Texture *texture;
    SDL_Rect view;
    NesCpuRaster raster;
    uint32_t pixels[NES_DOTS_PER_SCANLINE * NES_MAX_SCANLINES_PER_FRAME];
    SDL_Rect previous_window;
    int previous_view_height;
    EmulationMode mode;
} CpuDebugView;

static const struct { const char *label; uint32_t color; } cpu_execution_legend[CPU_EXEC_TYPES] = {
    [CPU_EXEC_NONE]           = {"NO SAMPLE",      0xff082446},
    [CPU_EXEC_RECOMPILED]     = {"RECOMPILED",     0xff008cff},
    [CPU_EXEC_LOOP_BATCHED]   = {"LOOP-BATCHED",   0x00000000},
    [CPU_EXEC_DIRECT]         = {"DIRECT",         0xffffe000},
    [CPU_EXEC_INDIVIDUAL]     = {"INDIVIDUAL",     0xffff1588},
    [CPU_EXEC_BUS_HANDLER]    = {"BUS HANDLER",    0xff00df65},
    [CPU_EXEC_DMA]            = {"DMA",            0xff00efff},
    [CPU_EXEC_INTERRUPT_IDLE] = {"INTERRUPT/IDLE", 0xffff2200}
};

static void close_cpu_debug(CpuDebugView *debug)
{
    if (debug) {
        SDL_DestroyTexture(debug->texture);
        free(debug);
    }
}

static CpuDebugView *open_cpu_debug(SDL_Renderer *renderer, int height)
{
    CpuDebugView *debug = calloc(1, sizeof *debug);

    if (debug) {
        debug->view = (SDL_Rect){0, 0, NES_DOTS_PER_SCANLINE, height};
        debug->texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                           NES_DOTS_PER_SCANLINE, NES_MAX_SCANLINES_PER_FRAME);

        if (debug->texture)
            return debug;
        free(debug);
    }
    return NULL;
}

// Upload RGB565 directly and convert other formats into the streaming texture
static int upload_frame(SDL_Texture *texture, const SDL_Rect *view, const Ppu *ppu,
                        const uint16_t (*frame)[NES_WIDTH])
{
    if (ppu->rgb565)
        return SDL_UpdateTexture(texture, view, frame[view->y], NES_WIDTH * (int)sizeof **frame);
    void *pixels;
    int pitch;

    if (SDL_LockTexture(texture, view, &pixels, &pitch) != 0)
        return -1;
    ppu_frame_to_argb(ppu, frame, (unsigned)view->y, (unsigned)view->h, pixels, (size_t)pitch / sizeof(uint32_t));
    SDL_UnlockTexture(texture);
    return 0;
}

static uint32_t tint_cpu_dot(uint32_t pixel, uint32_t color)
{
    // Blend the execution color at 50% opacity over the game pixel
    const uint32_t rb = 0x00ff00ff, g = 0x0000ff00;

    return 0xff000000 | ((((pixel & rb) + (color & rb)) >> 1) & rb) |
           ((((pixel & g) + (color & g)) >> 1) & g);
}

static void compose_cpu_debug(CpuDebugView *debug, const Ppu *ppu)
{
    const NesCpuRaster *trace = &debug->raster;
    uint64_t first = trace->first_tick;

    if (trace->end_tick - first > NES_CPU_RASTER_TICKS)
        first = trace->end_tick - NES_CPU_RASTER_TICKS;
    const unsigned lines = debug->view.h;
    // The view keeps room for No-Lag lines that a frame can leave out. Vblank
    // and pre-render always fill its last rows.
    const unsigned vblank_line = lines - (NES_SCANLINES_PER_FRAME - NES_VBLANK_LINE);

    for (unsigned y = 0; y < lines; ++y) {
        uint32_t picture[NES_WIDTH];

        if (y < NES_HEIGHT)
            ppu_frame_to_argb(ppu, ppu->frame, y, 1, picture, NES_WIDTH);
        bool leading = y >= vblank_line;
        // Vblank and pre-render lead into the current picture. Post-render,
        // including the extra No-Lag scanlines, follows it before the next vblank.
        unsigned before = leading ? (lines - y) * NES_DOTS_PER_SCANLINE -
                                     trace->short_pre_render : 0;
        uint64_t row = leading ? trace->line_zero_tick - before :
                                trace->line_zero_tick + y * NES_DOTS_PER_SCANLINE;
        unsigned length = NES_DOTS_PER_SCANLINE -
                          (y == lines - 1 && trace->short_pre_render);

        for (unsigned x = 0; x < NES_DOTS_PER_SCANLINE; ++x) {
            uint64_t tick = row + x;
            NesCpuExecution execution = CPU_EXEC_NONE;

            if (trace->line_zero_tick >= before && x < length && tick >= first && tick < trace->end_tick)
                execution = trace->execution[tick & (NES_CPU_RASTER_TICKS - 1)];
            uint32_t color = cpu_execution_legend[execution].color;

            if (y < NES_HEIGHT && x >= 1 && x <= NES_WIDTH) {
                uint32_t pixel = picture[x - 1];
                color = execution == CPU_EXEC_NONE || !(color >> 24) ? pixel : tint_cpu_dot(pixel, color);
            }
            debug->pixels[y * NES_DOTS_PER_SCANLINE + x] = color;
        }
    }
}

static int draw_cpu_legend(SDL_Renderer *renderer, const Nes *nes, const CpuDebugView *debug)
{
    const int left = NES_DOTS_PER_SCANLINE, legend_top = 23, legend_step = 9;
    const int range_top = legend_top + CPU_EXEC_TYPES * legend_step + 10;
    const unsigned lines = NES_SCANLINES_PER_FRAME + nes->ppu.extra_blanking_lines;
    const SDL_Rect panel = {left, 0, CPU_LEGEND_WIDTH, (int)lines};
    SDL_Color previous;

    if (SDL_GetRenderDrawColor(renderer, &previous.r, &previous.g, &previous.b, &previous.a) != 0)
        return -1;
    int result = SDL_SetRenderDrawColor(renderer, 12, 18, 27, 255);

    result |= SDL_RenderFillRect(renderer, &panel);
    EmulationMode mode = debug->raster.end_tick ? debug->mode : emulation_mode(nes);
    const SDL_Color title = emulation_modes[mode].color;

    result |= SDL_SetRenderDrawColor(renderer, title.r, title.g, title.b, title.a);
    result |= draw_osd_text(renderer, left + 7, 9, "CPU RASTER");

    for (unsigned i = 0; i < CPU_EXEC_TYPES; ++i) {
        unsigned category = (i + 1) % CPU_EXEC_TYPES;
        uint32_t color = cpu_execution_legend[category].color;
        SDL_Rect swatch = {left + 7, legend_top + (int)i * legend_step, 5, GLYPH_HEIGHT};

        result |= SDL_SetRenderDrawColor(renderer, (Uint8)(color >> 16), (Uint8)(color >> 8), (Uint8)color, 255);
        result |= SDL_RenderFillRect(renderer, &swatch);
        result |= SDL_SetRenderDrawColor(renderer, 218, 227, 239, 255);
        result |= draw_osd_text(renderer, left + 16, swatch.y, cpu_execution_legend[category].label);
    }
    result |= SDL_SetRenderDrawColor(renderer, 164, 182, 205, 255);
    result |= draw_osd_text(renderer, left + 7, range_top, "HBLANK 257-340");
    unsigned pre_render = lines - 1, vblank = NES_VBLANK_LINE + nes->ppu.extra_blanking_lines;
    char range[24];

    if (vblank > NES_VBLANK_LINE)
        snprintf(range, sizeof range, "POST-RENDER %u-%u", NES_HEIGHT, vblank - 1);
    else
        snprintf(range, sizeof range, "POST-RENDER %u", NES_HEIGHT);
    result |= draw_osd_text(renderer, left + 7, range_top + 13, range);
    snprintf(range, sizeof range, "VBLANK %u-%u", vblank, pre_render - 1);
    result |= draw_osd_text(renderer, left + 7, range_top + 22, range);
    snprintf(range, sizeof range, "PRE-RENDER %u", pre_render);
    result |= draw_osd_text(renderer, left + 7, range_top + 31, range);
    result |= SDL_SetRenderDrawColor(renderer, previous.r, previous.g, previous.b, previous.a);
    return result;
}

static bool window_is_floating(SDL_Window *window)
{
    return !(SDL_GetWindowFlags(window) & (SDL_WINDOW_FULLSCREEN | SDL_WINDOW_MAXIMIZED));
}

static const char *toggle_crop(SDL_Window *window, SDL_Renderer *renderer, SDL_Rect *view, bool resize)
{
    int height = view->h == NES_HEIGHT ? CROPPED_HEIGHT : NES_HEIGHT;

    if (resize) {
        if (SDL_RenderSetLogicalSize(renderer, NES_WIDTH, height) != 0) {
            fprintf(stderr, "Cannot toggle crop (%s)\n", SDL_GetError());
            return NULL;
        }
        int window_width, window_height;

        SDL_GetWindowSize(window, &window_width, &window_height);
        SDL_SetWindowMinimumSize(window, NES_WIDTH, height);
        if (window_is_floating(window))
            SDL_SetWindowSize(window, window_width, (window_height * height + view->h / 2) / view->h);
    }
    view->y = (NES_HEIGHT - height) / 2;
    view->h = height;
    return height == CROPPED_HEIGHT ? "CROP ON" : "CROP OFF";
}

static void fit_cpu_debug_window(SDL_Window *window, SDL_Rect previous, int view_height, int height)
{
    const int width = CPU_DEBUG_WIDTH;
    double scale = SDL_min((double)previous.w / NES_WIDTH, (double)previous.h / view_height);
    SDL_Rect bounds;
    int display = SDL_GetWindowDisplayIndex(window);
    int top = 0, left = 0, bottom = 0, right = 0;
#if SDL_VERSION_ATLEAST(2, 0, 5)
    bool have_bounds = display >= 0 && SDL_GetDisplayUsableBounds(display, &bounds) == 0;

    SDL_GetWindowBordersSize(window, &top, &left, &bottom, &right);
#else
    bool have_bounds = false;
#endif
    if (!have_bounds)
        have_bounds = display >= 0 && SDL_GetDisplayBounds(display, &bounds) == 0;
    if (have_bounds) {
        bounds.x += left;
        bounds.y += top;
        bounds.w = SDL_max(1, bounds.w - left - right);
        bounds.h = SDL_max(1, bounds.h - top - bottom);
        scale = SDL_min(scale, SDL_min((double)bounds.w / width, (double)bounds.h / height));
    }
    int w = SDL_max(1, (int)(width * scale + 0.5));
    int h = SDL_max(1, (int)(height * scale + 0.5));

    SDL_SetWindowMinimumSize(window, SDL_min(width, w), SDL_min(height, h));
    SDL_SetWindowSize(window, w, h);
    if (have_bounds) {
        int x = SDL_max(bounds.x, SDL_min(previous.x, bounds.x + bounds.w - w));
        int y = SDL_max(bounds.y, SDL_min(previous.y, bounds.y + bounds.h - h));

        SDL_SetWindowPosition(window, x, y);
    }
}

static bool set_cpu_debug_layout(SDL_Window *window, SDL_Renderer *renderer,
                                 CpuDebugView *debug, bool enabling, int view_height)
{
    int width = enabling ? CPU_DEBUG_WIDTH : NES_WIDTH;
    int height = enabling ? debug->view.h : view_height;

    if (SDL_RenderSetLogicalSize(renderer, width, height) != 0)
        return false;

    bool floating = window_is_floating(window);

    if (floating && enabling) {
        SDL_Rect *previous = &debug->previous_window;

        SDL_GetWindowPosition(window, &previous->x, &previous->y);
        SDL_GetWindowSize(window, &previous->w, &previous->h);
        debug->previous_view_height = view_height;
        fit_cpu_debug_window(window, *previous, view_height, height);
    } else {
        SDL_SetWindowMinimumSize(window, width, height);
        if (floating && debug->previous_window.w) {
            SDL_Rect previous = debug->previous_window;

            SDL_SetWindowSize(window, previous.w,
                              (previous.h * view_height + debug->previous_view_height / 2) /
                              debug->previous_view_height);
            SDL_SetWindowPosition(window, previous.x, previous.y);
        }
    }
    return true;
}

static bool set_no_lag_mode(Nes *nes, SDL_Window *window, SDL_Renderer *renderer,
                            CpuDebugView *debug, bool enabled)
{
    int lines = enabled ? NES_MAX_SCANLINES_PER_FRAME : NES_SCANLINES_PER_FRAME;

    if (debug && SDL_RenderSetLogicalSize(renderer, CPU_DEBUG_WIDTH, lines) != 0) {
        fprintf(stderr, "Cannot resize raster view (%s)\n", SDL_GetError());
        return false;
    }
    nes_set_enhanced(nes, enabled);
    if (debug) {
        debug->view.h = lines;
        memset(&debug->raster, 0, sizeof debug->raster);
        if (debug->previous_window.w && window_is_floating(window))
            fit_cpu_debug_window(window, debug->previous_window, debug->previous_view_height, lines);
        else
            SDL_SetWindowMinimumSize(window, CPU_DEBUG_WIDTH, lines);
    }
    return true;
}

static int draw_notice(SDL_Renderer *renderer, int left, int view_height, const char *text, const Nes *mode)
{
    const int padding = 3, margin = 8;
    const int height = GLYPH_HEIGHT + 2 * padding;
    const char suffix[] = " MODE";
    const int text_width = osd_text_width(text);
    const int suffix_width = mode ? (sizeof suffix - 1) * GLYPH_STEP : 0;
    const SDL_Rect panel = {left, view_height - margin - height,
                            text_width + suffix_width + 2 * padding, height};
    const int x = panel.x + padding, y = panel.y + padding;
    const SDL_Color white = {249, 251, 255, 255};
    const SDL_Color color = mode ? emulation_modes[emulation_mode(mode)].color : white;
    SDL_Color previous;

    if (SDL_GetRenderDrawColor(renderer, &previous.r, &previous.g, &previous.b, &previous.a) != 0)
        return -1;

    int result = SDL_SetRenderDrawColor(renderer, 12, 18, 27, 255);

    result |= SDL_RenderFillRect(renderer, &panel);
    result |= SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    result |= draw_osd_text(renderer, x, y, text);

    if (mode) {
        result |= SDL_SetRenderDrawColor(renderer, white.r, white.g, white.b, white.a);
        result |= draw_osd_text(renderer, x + text_width + 1, y, suffix);
    }
    result |= SDL_SetRenderDrawColor(renderer, previous.r, previous.g, previous.b, previous.a);
    return result;
}

enum {
    INFO_EMULATION_FPS, INFO_DISPLAY_FPS, INFO_MAPPER, INFO_PRG_ROM, INFO_CHR_ROM, INFO_CHR_RAM,
    INFO_EXTERNAL_SRAM, INFO_ROWS, INFO_VALUE_LENGTH = 24
};

static const char *mapper_name(const Cartridge *cart)
{
    switch (cart->mapper) {
    case 0: return "NROM";
    case 1: return "MMC1";
    case 2: return "UXROM";
    case 3: return "CNROM";
    case 4: return cart->submapper == 1 ? "MMC6" : cart->submapper == 3 ? "MC-ACC" : "MMC3";
    case 5: return "MMC5";
    case 7: return "AXROM";
    case 9: return cart->mmc4 ? "MMC4" : "MMC2";
    case 21: case 23: case 25:
        return cart->ext && cart->ext->vrc2 ? "KONAMI VRC2" : "KONAMI VRC4";
    case 20: return "FAMICOM DISK SYSTEM";
    case 22: return "KONAMI VRC2";
    case 24: case 26: return "KONAMI VRC6";
    case 66: return "GXROM";
    case 69: return "SUNSOFT FME-7";
    case 71: return "CAMERICA BF9093";
    case 206: return "NAMCO 108";
    default: return "UNKNOWN";
    }
}

static void format_cart_size(char value[INFO_VALUE_LENGTH], size_t bytes)
{
    if (bytes % 1024)
        snprintf(value, INFO_VALUE_LENGTH, "%zu B", bytes);
    else
        snprintf(value, INFO_VALUE_LENGTH, "%zu KiB", bytes / 1024);
}

static void format_frame_rate(char value[INFO_VALUE_LENGTH], const FrameRate *rate, bool paused)
{
    if (paused || rate->fps > 0)
        snprintf(value, INFO_VALUE_LENGTH, "%.2f", paused ? 0.0 : rate->fps);
    else
        memcpy(value, "--", sizeof "--");
}

static void format_info_values(char values[INFO_ROWS][INFO_VALUE_LENGTH], const Cartridge *cart,
                               const FrameRates *rates, bool paused)
{
    format_frame_rate(values[INFO_EMULATION_FPS], &rates->emulation, paused);
    format_frame_rate(values[INFO_DISPLAY_FPS], &rates->display, paused);
    // The MMC4 shares the MMC2 code path under mapper 9
    snprintf(values[INFO_MAPPER], INFO_VALUE_LENGTH, "%u %s", cart->mmc4 ? 10u : cart->mapper, mapper_name(cart));
    format_cart_size(values[INFO_PRG_ROM], cart->prg_size);
    format_cart_size(values[INFO_CHR_ROM], cart->chr_is_ram ? 0 : cart->chr_size);
    format_cart_size(values[INFO_CHR_RAM], cart->chr_is_ram ? cart->chr_size : 0);
    format_cart_size(values[INFO_EXTERNAL_SRAM], cart_ram_bytes(cart));
}

static int draw_stats(SDL_Renderer *renderer, OsdPage page, const InstructionOverlay *overlay,
                      const RasterOverlay *raster, const FrameRates *rates, bool paused, const Nes *nes)
{
    static const char *const cpu_labels[STATS_ROWS] = {
        "RECOMPILED", "LOOP-BATCHED", "DIRECT", "INDIVIDUAL", "BUS HANDLER"
    };
    static const char *const raster_labels[RASTER_STATS_ROWS] = {
        "SECTIONS", "REGIONS", "BATCHED LINES", "SINGLE LINES", "REDRAWS",
        "TOTAL SYNCS", "PPU SYNCS", "APU SYNCS", "BUS RESCHEDULES"
    };
    static const char *const info_labels[INFO_ROWS] = {
        "EMULATION FPS", "DISPLAY FPS", "MAPPER", "PRG ROM", "CHR ROM", "CHR RAM", "EXTERNAL SRAM"
    };
    const int padding = 3, line_height = GLYPH_HEIGHT + 1, column_gap = GLYPH_STEP;
    const int title_padding = 2;
    bool cpu_page = page == OSD_CPU;
    bool info_page = page == OSD_INFO;
    bool ready = info_page ? rates != NULL : cpu_page ? overlay && overlay->ready : raster && raster->ready;
    const char *heading = info_page ? "INFO" : cpu_page ? "CPU" : "RASTER";
    EmulationMode mode = emulation_mode(nes);
    const SDL_Color title_color = emulation_modes[mode].color;
    char title[sizeof "RASTER - INTERPRETER"];

    if (mode == MODE_REGULAR)
        snprintf(title, sizeof title, "%s", heading);
    else
        snprintf(title, sizeof title, "%s - %s", heading, emulation_modes[mode].name);

    enum { MAX_ROWS = SDL_max(INFO_ROWS, SDL_max(STATS_ROWS, RASTER_STATS_ROWS)) };
    struct { const char *label, *count, *share; } rows[MAX_ROWS] = {0};
    char values[INFO_ROWS][INFO_VALUE_LENGTH];
    int row_count = 0;

    if (!ready) {
        rows[0].label = info_page ? "LOAD ROM FOR INFO" :
                        !rates ? "LOAD ROM TO SAMPLE" : paused ? "RESUME TO SAMPLE" : "COLLECTING STATS";
        row_count = 1;
    } else if (info_page) {
        format_info_values(values, &nes->cart, rates, paused);
        for (unsigned i = 0; i < INFO_ROWS; ++i) {
            if (i == INFO_CHR_RAM && !nes->cart.chr_is_ram)
                continue;
            rows[row_count].label = info_labels[i];
            rows[row_count].count = values[i];
            ++row_count;
        }
    } else if (cpu_page) {
        unsigned first_stat = nes->interpreter ? STATS_LOOP_BATCHED : STATS_RECOMPILED;

        for (unsigned i = first_stat; i < STATS_ROWS; ++i) {
            rows[row_count].label = cpu_labels[i];
            rows[row_count].count = overlay->rows[i].count;
            rows[row_count].share = overlay->rows[i].share;
            ++row_count;
        }
    } else {
        for (unsigned i = 0; i < RASTER_STATS_ROWS; ++i) {
            rows[row_count].label = raster_labels[i];
            rows[row_count].count = raster->averages[i];
            ++row_count;
        }
    }

    int label_width = 0, share_width = 0;
    int count_width = info_page && ready ? osd_text_width("999999.99") : 0;

    for (int i = 0; i < row_count; ++i) {
        int label = osd_text_width(rows[i].label);
        int count = rows[i].count ? osd_text_width(rows[i].count) : 0;
        int share = rows[i].share ? osd_text_width(rows[i].share) : 0;

        label_width = SDL_max(label_width, label);
        count_width = SDL_max(count_width, count);
        share_width = SDL_max(share_width, share);
    }
    int share_column = share_width ? column_gap + share_width : 0;
    int table_width = label_width + (count_width ? column_gap + count_width : 0) + share_column;
    int title_width = osd_text_width(title);
    int content_width = SDL_max(title_width, table_width);
    int lines = 1 + row_count;
    const SDL_Rect panel = {8, 8, content_width + 2 * padding,
                            lines * line_height - 1 + 2 * padding + title_padding};
    const int left = panel.x + padding, right = left + content_width;
    const int count_right = right - share_column;
    int y = panel.y + padding + line_height + title_padding;
    Uint8 r, g, b, a;
    SDL_BlendMode blend;

    if (SDL_GetRenderDrawColor(renderer, &r, &g, &b, &a) != 0 ||
        SDL_GetRenderDrawBlendMode(renderer, &blend) != 0)
        return -1;

    int result = SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    result |= SDL_SetRenderDrawColor(renderer, 12, 18, 27, 232);
    result |= SDL_RenderFillRect(renderer, &panel);
    result |= SDL_SetRenderDrawColor(renderer, title_color.r, title_color.g, title_color.b, title_color.a);
    result |= draw_osd_text(renderer, left, panel.y + padding, title);

    for (int i = 0; i < row_count; ++i, y += line_height) {
        result |= SDL_SetRenderDrawColor(renderer, 186, 198, 212, 255);
        result |= draw_osd_text(renderer, left, y, rows[i].label);

        if (rows[i].count) {
            result |= SDL_SetRenderDrawColor(renderer, 249, 251, 255, 255);
            result |= draw_osd_text(renderer, count_right - osd_text_width(rows[i].count), y, rows[i].count);

            if (rows[i].share)
                result |= draw_osd_text(renderer, right - osd_text_width(rows[i].share), y, rows[i].share);
        }
    }
    result |= SDL_SetRenderDrawColor(renderer, r, g, b, a);
    result |= SDL_SetRenderDrawBlendMode(renderer, blend);
    return result;
}

static int draw_drop_prompt(SDL_Renderer *renderer, int height, const char *error)
{
    static const char prompt[] = "Drag-and-drop file here";
    static const char *const mode_prefixes[MODE_COUNT] = {"CYCLE ", " / ", " / "};

    static const struct { const char *key, *action; } shortcuts[] = {
        {"BACKTICK", "HOLD TO FAST-FORWARD"},
        {"1", "CYCLE INFO / CPU STATS / RASTER STATS / HIDDEN"},
        {"2", NULL},
        {"3", "TOGGLE CROP / FULL VIEW"},
        {"4", "CYCLE RUNAHEAD 1 FRAME / OFF / 2 FRAMES"},
        {"5", "TOGGLE NO-LAG MODE"},
        {"6", "TOGGLE 8-SPRITE LIMIT"},
        {"7", "TOGGLE CPU RASTER DEBUGGER"},
        {"8", "SAVE PNG SCREENSHOT"},
        {"ESC", "QUIT"},
    };
    static const char *const controls[][4] = {
        {"CONTROLS", "PLAYER 1", "PLAYER 2", "GAMEPAD"},
        {"D-PAD", "ARROW KEYS", "W A S D", "D-PAD OR LEFT STICK"},
        {"A", "X", "G", "SOUTH"},
        {"B", "Z", "F", "WEST OR EAST"},
        {"SELECT", "RIGHT SHIFT", "LEFT SHIFT", "BACK"},
        {"START", "ENTER", "TAB", "START"},
    };
    const int row_step = GLYPH_HEIGHT + 4, gap = 3 * GLYPH_STEP;
    const int title_width = (sizeof prompt - 2) * TITLE_GLYPH_STEP + TITLE_GLYPH_WIDTH;
    const int title_height = TITLE_GLYPH_HEIGHT;
    const int guide_offset = title_height + GLYPH_HEIGHT + row_step;
    const int controls_offset = guide_offset + (SDL_arraysize(shortcuts) + 1) * row_step;
    const int content_height = controls_offset + (SDL_arraysize(controls) - 1) * row_step + GLYPH_HEIGHT;
    const int y = (height - content_height) / 2;
    int mode_chars = 0;

    for (unsigned i = 0; i < MODE_COUNT; ++i)
        mode_chars += (int)(strlen(mode_prefixes[i]) + strlen(emulation_modes[i].name));
    int key_width = 0, action_width = mode_chars * GLYPH_STEP - 1;

    for (unsigned i = 0; i < SDL_arraysize(shortcuts); ++i) {
        key_width = SDL_max(key_width, osd_text_width(shortcuts[i].key));

        if (shortcuts[i].action)
            action_width = SDL_max(action_width, osd_text_width(shortcuts[i].action));
    }
    int column_widths[SDL_arraysize(controls[0])] = {0};
    int controls_width = (SDL_arraysize(column_widths) - 1) * gap;

    for (unsigned col = 0; col < SDL_arraysize(column_widths); ++col) {
        for (unsigned row = 0; row < SDL_arraysize(controls); ++row)
            column_widths[col] = SDL_max(column_widths[col], osd_text_width(controls[row][col]));
        controls_width += column_widths[col];
    }
    const int left = (NES_WIDTH - SDL_max(key_width + gap + action_width, controls_width)) / 2;
    const int action_x = left + key_width + gap;

    int result = SDL_SetRenderDrawColor(renderer, 249, 251, 255, 255);
    SDL_Point title_points[(sizeof prompt - 1) * TITLE_GLYPH_WIDTH * TITLE_GLYPH_HEIGHT];
    int title_count = bitmap_text_points(title_points, &title_bitmap, (NES_WIDTH - title_width) / 2, y, prompt);

    result |= SDL_RenderDrawPoints(renderer, title_points, title_count);

    if (error)
        result |= draw_osd_text(renderer, (NES_WIDTH - osd_text_width(error)) / 2,
                                y + title_height + 3, error);
    int row_y = y + guide_offset;

    for (unsigned i = 0; i < SDL_arraysize(shortcuts); ++i, row_y += row_step) {
        result |= SDL_SetRenderDrawColor(renderer, 249, 251, 255, 255);
        result |= draw_osd_text(renderer, left, row_y, shortcuts[i].key);

        if (shortcuts[i].action) {
            result |= SDL_SetRenderDrawColor(renderer, 186, 198, 212, 255);
            result |= draw_osd_text(renderer, action_x, row_y, shortcuts[i].action);
        } else {
            int x = action_x;

            for (unsigned mode = 0; mode < MODE_COUNT; ++mode) {
                SDL_Color color = emulation_modes[mode].color;

                result |= SDL_SetRenderDrawColor(renderer, 186, 198, 212, 255);
                result |= draw_osd_text(renderer, x, row_y, mode_prefixes[mode]);
                x += osd_text_width(mode_prefixes[mode]) + 1;
                result |= SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
                result |= draw_osd_text(renderer, x, row_y, emulation_modes[mode].name);
                x += osd_text_width(emulation_modes[mode].name) + 1;
            }
        }
    }
    row_y = y + controls_offset;

    for (unsigned row = 0; row < SDL_arraysize(controls); ++row, row_y += row_step) {
        SDL_Color color = row ? (SDL_Color){186, 198, 212, 255} : (SDL_Color){249, 251, 255, 255};

        result |= SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        int x = left;

        for (unsigned col = 0; col < SDL_arraysize(column_widths); ++col) {
            result |= draw_osd_text(renderer, x, row_y, controls[row][col]);
            x += column_widths[col] + gap;
        }
    }
    return result;
}

static int wait_for_rom(SDL_Window *window, SDL_Renderer *renderer, Nes *nes, char **path,
                        SDL_Rect *view, OsdPage *osd_page, RunaheadDepth *runahead_depth)
{
    const char *notice = NULL;
    bool notice_is_mode = false, redraw = true;
    const char *load_error = NULL; // Shown under the prompt after a failed drop

    for (;;) {
        if (redraw) {
            if (SDL_SetRenderDrawColor(renderer, 12, 18, 27, 255) != 0 ||
                SDL_RenderClear(renderer) != 0 ||
                draw_drop_prompt(renderer, view->h, load_error) != 0 ||
                (*osd_page != OSD_HIDDEN &&
                 draw_stats(renderer, *osd_page, NULL, NULL, NULL, false, nes) != 0) ||
                (notice && draw_notice(renderer, 8, view->h, notice, notice_is_mode ? nes : NULL) != 0) ||
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255) != 0) {
                fprintf(stderr, "Cannot draw ROM prompt (%s)\n", SDL_GetError());
                return 1;
            }
            SDL_RenderPresent(renderer);
            redraw = false;
        }

        SDL_Event event;

        if (!SDL_WaitEvent(&event)) {
            fprintf(stderr, "Cannot wait for a ROM (%s)\n", SDL_GetError());
            return 1;
        }
        if (event.type == SDL_QUIT ||
            (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) ||
            (event.type == SDL_KEYDOWN && event.key.keysym.scancode == SDL_SCANCODE_ESCAPE))
            return 0;
        if (event.type == SDL_KEYDOWN && !event.key.repeat) {
            redraw = true;

            switch (event.key.keysym.scancode) {
            case SDL_SCANCODE_1:
                *osd_page = (*osd_page + 1) % OSD_PAGE_COUNT;

                if (notice_is_mode)
                    notice = NULL;
                break;
            case SDL_SCANCODE_2:
                notice = cycle_cpu_mode(nes);
                notice_is_mode = true;

                if (*osd_page != OSD_HIDDEN)
                    notice = NULL;
                break;
            case SDL_SCANCODE_3:
                notice = toggle_crop(window, renderer, view, true);
                notice_is_mode = false;
                break;
            case SDL_SCANCODE_4:
                notice = cycle_runahead(runahead_depth);
                notice_is_mode = false;
                break;
            case SDL_SCANCODE_5:
                nes->ppu.extra_blanking_lines = nes->ppu.extra_blanking_lines ? 0 : NES_EXTRA_BLANKING_LINES;
                notice = nes->ppu.extra_blanking_lines ? "NO-LAG ON" : "NO-LAG OFF";
                notice_is_mode = false;
                break;
            case SDL_SCANCODE_6:
                notice = toggle_sprite_limit(&nes->ppu);
                notice_is_mode = false;
                break;
            default:
                redraw = false;
                break;
            }
        } else if (event.type == SDL_DROPFILE) {
            if (load_rom(nes, event.drop.file, NULL, &load_error)) {
                *path = event.drop.file;
                return 0;
            }
            redraw = true;
        } else if ((event.type == SDL_WINDOWEVENT && (event.window.event == SDL_WINDOWEVENT_EXPOSED ||
                    event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
                    event.window.event == SDL_WINDOWEVENT_RESTORED)) ||
                   event.type == SDL_RENDER_DEVICE_RESET || event.type == SDL_RENDER_TARGETS_RESET) {
            redraw = true;
        }
        if (event.type == SDL_DROPFILE || event.type == SDL_DROPTEXT)
            SDL_free(event.drop.file);
    }
}

typedef struct {
    const char *text;
    Uint64 until;
    bool is_mode;
} Notice;

static void show_notice(Notice *notice, const char *text, bool is_mode, Uint64 duration)
{
    *notice = (Notice){text, SDL_GetPerformanceCounter() + duration, is_mode};
}

typedef struct {
    unsigned side;
    uint32_t notices;
    char text[40];
} DiskStatus;

static const char *change_disk(Nes *nes, DiskStatus *disk, bool next)
{
    unsigned sides = fds_side_count(nes), current = fds_current_side(nes);

    if (!sides)
        return NULL;
    if (current != FDS_NO_DISK)
        disk->side = current;
    if (!next && current != FDS_NO_DISK) {
        fds_change_disk(nes, FDS_NO_DISK, false);
        return "DISK EJECTED";
    }
    if (next)
        disk->side = (disk->side + 1) % sides;
    // The core reports the insertion
    fds_change_disk(nes, disk->side, next);
    return NULL;
}

static const char *disk_notice(const Nes *nes, DiskStatus *disk)
{
    unsigned side;
    bool missing;
    uint32_t notices = fds_disk_notices(nes, &side, &missing);

    if (notices == disk->notices)
        return NULL;
    disk->notices = notices;

    if (side == FDS_NO_DISK)
        return "REQUESTED DISK NOT IN IMAGE";
    if (!missing)
        disk->side = side;
    snprintf(disk->text, sizeof disk->text, "DISK %u SIDE %c %s", side / 2 + 1, 'A' + (int)(side & 1),
             missing ? "NOT IN IMAGE" : "INSERTED");
    return disk->text;
}

static void add_bios_directories(void)
{
    char *base = SDL_GetBasePath();

    if (!base)
        return;
    cart_add_bios_directory(base);
    const char *bundle = strstr(base, ".app/Contents/Resources/");

    if (bundle) {
        size_t length = (size_t)(bundle - base);

        while (length && base[length - 1] != '/')
            --length;
        base[length] = 0;
        cart_add_bios_directory(base);
    }
    SDL_free(base);
}

static int run_interactive(Nes *nes, const char *rom_path, bool crop, RunaheadDepth runahead_depth)
{
    const int view_height = crop ? CROPPED_HEIGHT : NES_HEIGHT;
    SDL_Rect view = {0, (NES_HEIGHT - view_height) / 2, NES_WIDTH, view_height};
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture_argb, *texture_rgb565;
    DiskStatus disk = {0};
    int result = 0;
    char *dropped_rom = NULL;
    OsdPage osd_page = OSD_HIDDEN;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        fprintf(stderr, "Cannot initialize SDL (%s)\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    // Disable text input before SDL opens a window and activates the macOS IME
    SDL_StopTextInput();
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
    SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
    window = SDL_CreateWindow(app_name, SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, NES_WIDTH * 3, view_height * 3,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    renderer = window ? SDL_CreateRenderer(window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC) : NULL;

    if (window && !renderer)
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE | SDL_RENDERER_PRESENTVSYNC);
    texture_argb = renderer ? SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, NES_WIDTH, NES_HEIGHT) : NULL;
    // Ultrafast pictures are already RGB565
    texture_rgb565 = texture_argb ? SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB565,
        SDL_TEXTUREACCESS_STREAMING, NES_WIDTH, NES_HEIGHT) : NULL;

    if (!texture_argb || !texture_rgb565 || SDL_RenderSetLogicalSize(renderer, NES_WIDTH, view_height) != 0) {
        fprintf(stderr, "Cannot initialize video (%s)\n", SDL_GetError());
        result = 1;
        goto cleanup_video;
    }
    SDL_SetWindowMinimumSize(window, NES_WIDTH, view_height);
#ifdef __APPLE__
    schnes_set_dock_icon();
#endif
    if (!rom_path) {
        result = wait_for_rom(window, renderer, nes, &dropped_rom, &view, &osd_page, &runahead_depth);

        if (!dropped_rom)
            goto cleanup_video;
        rom_path = dropped_rom;
    }

    NesRunahead *runahead = open_runahead(&nes->cart, &runahead_depth);
    CpuDebugView *debug = NULL;
    const uint16_t (*display_frame)[NES_WIDTH] = nes->ppu.frame;
    Uint64 counter_hz;
    SDL_AudioSpec audio_spec = {0};
    SDL_AudioDeviceID audio;

    set_window_title(window, rom_path, nes, false);
    counter_hz = SDL_GetPerformanceFrequency();
    Input input = {.poll_interval = (counter_hz + 999) / 1000};

    scan_gamepads(input.pads);
    attach_input(nes, &input);
    audio = open_audio(&audio_spec);

    const double frequency = (double)counter_hz;
    const double ticks_per_ms = frequency / 1000.0;
    const double frame_ticks = frequency * NES_NTSC_TWO_FRAME_DOTS /
        (2 * NES_PPU_DOTS_PER_CPU * NES_CPU_HZ);
    const size_t audio_frame_bytes = NES_SAMPLE_RATE * frame_ticks / frequency * sizeof(int16_t);
    const size_t audio_prefill = audio_frame_bytes + audio_spec.size;
    const size_t audio_high_water = audio_prefill + audio_frame_bytes;
    const double audio_pace_gain = frequency / (8.0 * NES_SAMPLE_RATE * sizeof(int16_t));
    const double audio_pace_limit = frame_ticks / 4;
    SDL_RendererInfo renderer_info;
    const Uint32 display_sync_flags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC;
    const bool display_sync = SDL_GetRendererInfo(renderer, &renderer_info) == 0 &&
                              (renderer_info.flags & display_sync_flags) == display_sync_flags;
    double refresh_ticks = display_refresh_period(window, frequency);
    double last_refresh = 0, frame_work = 0, prediction_work = 0;
    double prediction_target = 0;
    const double frame_margin = 1.5 * ticks_per_ms;
    const double prediction_margin = 1.75 * ticks_per_ms;
    double deadline = (double)SDL_GetPerformanceCounter();
    double next_present = 0;
    bool running = true, paused = false, audio_playing = false;
    bool fast_forward_held = false, fast_forward = false;
    bool refresh_texture = true;
    bool screenshot_pending = false;
    Notice notice = {0};
    // Regular and Ultrafast say once when they must interpret, after any other notice
    bool recompiler_notice_due = true;
    const Uint64 notice_duration = 2 * counter_hz;
    InstructionOverlay overlay = {0};
    RasterOverlay raster = {0};
    FrameRates rates = {0};

    while (running) {
        double frame_start = (double)SDL_GetPerformanceCounter();
        bool was_paused = paused, was_fast_forward = fast_forward;
        bool reset_requested = false;
        bool rom_reloaded = false;
        char *pending_rom = NULL;
        bool was_no_lag = nes->ppu.extra_blanking_lines != 0;
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_DROPFILE) {
                SDL_free(pending_rom);
                pending_rom = event.drop.file;
            } else if (event.type == SDL_DROPTEXT) {
                SDL_free(event.drop.file);
            } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
                add_gamepad(input.pads, event.cdevice.which);
            } else if (event.type == SDL_CONTROLLERDEVICEREMOVED) {
                remove_gamepad(input.pads, event.cdevice.which);
            } else if ((event.type == SDL_KEYUP && event.key.keysym.scancode == SDL_SCANCODE_GRAVE) ||
                       (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)) {
                fast_forward_held = false;
            } else if (event.type == SDL_WINDOWEVENT &&
                       (event.window.event == SDL_WINDOWEVENT_SHOWN ||
                        event.window.event == SDL_WINDOWEVENT_HIDDEN ||
                        event.window.event == SDL_WINDOWEVENT_MINIMIZED ||
                        event.window.event == SDL_WINDOWEVENT_RESTORED)) {
                rates.display = (FrameRate){0};
            } else if (event.type == SDL_WINDOWEVENT &&
                       (event.window.event == SDL_WINDOWEVENT_MOVED ||
                        event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED
#if SDL_VERSION_ATLEAST(2, 0, 18)
                        || event.window.event == SDL_WINDOWEVENT_DISPLAY_CHANGED
#endif
                       )) {
                refresh_ticks = display_refresh_period(window, frequency);
                last_refresh = prediction_target = 0;
            } else if (event.type == SDL_KEYDOWN && !event.key.repeat) {
                switch (event.key.keysym.scancode) {
                case SDL_SCANCODE_ESCAPE: running = false; break;
                case SDL_SCANCODE_1:
                    if (debug)
                        break;
                    osd_page = (osd_page + 1) % OSD_PAGE_COUNT;

                    if (notice.is_mode)
                        notice.text = NULL;
                    if (osd_page == OSD_CPU)
                        overlay = (InstructionOverlay){0};
                    if (osd_page == OSD_RASTER)
                        raster = (RasterOverlay){0};
                    break;
                case SDL_SCANCODE_2: {
                    const char *mode = cycle_cpu_mode(nes);

                    recompiler_notice_due = true;
                    overlay = (InstructionOverlay){0};
                    raster = (RasterOverlay){0};
                    rates = (FrameRates){0};

                    if (osd_page == OSD_HIDDEN)
                        show_notice(&notice, mode, true, notice_duration);
                    set_window_title(window, rom_path, nes, paused);
                    break;
                }
                case SDL_SCANCODE_3: {
                    const char *setting = toggle_crop(window, renderer, &view, !debug);

                    if (!setting)
                        break;
                    refresh_texture = true;
                    show_notice(&notice, setting, false, notice_duration);
                    break;
                }
                case SDL_SCANCODE_GRAVE: fast_forward_held = true; break;
                case SDL_SCANCODE_4: {
                    const char *setting = cycle_runahead(&runahead_depth);

                    if (!runahead)
                        runahead = open_runahead(&nes->cart, &runahead_depth);
                    if (paused) {
                        display_frame = nes->ppu.frame;
                        refresh_texture = true;
                    }
                    prediction_target = prediction_work = frame_work = 0;
                    show_notice(&notice, runahead ? setting : "RUNAHEAD UNAVAILABLE", false, notice_duration);
                    break;
                }
                case SDL_SCANCODE_6:
                    show_notice(&notice, toggle_sprite_limit(&nes->ppu), false, notice_duration);
                    break;
                case SDL_SCANCODE_MINUS: case SDL_SCANCODE_EQUALS: {
                    const char *status = change_disk(nes, &disk,
                                                     event.key.keysym.scancode == SDL_SCANCODE_EQUALS);

                    if (status)
                        show_notice(&notice, status, false, notice_duration);
                    break;
                }
                case SDL_SCANCODE_7: {
                    int lines = NES_SCANLINES_PER_FRAME + nes->ppu.extra_blanking_lines;
                    CpuDebugView *next = debug ? NULL : open_cpu_debug(renderer, lines);
                    bool ok = (debug || next) &&
                        set_cpu_debug_layout(window, renderer, next ? next : debug, next != NULL, view.h);

                    if (!ok) {
                        close_cpu_debug(next);
                        show_notice(&notice, "DEBUG UNAVAILABLE", false, notice_duration);
                        break;
                    }
                    close_cpu_debug(debug);
                    debug = next;
                    display_frame = nes->ppu.frame;
                    refresh_texture = true;
                    last_refresh = prediction_target = 0;
                    frame_work = prediction_work = 0;
                    break;
                }
                case SDL_SCANCODE_5: {
                    bool enable = !nes->ppu.extra_blanking_lines;

                    if (!set_no_lag_mode(nes, window, renderer, debug, enable))
                        break;
                    overlay = (InstructionOverlay){0};
                    raster = (RasterOverlay){0};
                    display_frame = nes->ppu.frame;
                    refresh_texture = true;
                    show_notice(&notice, enable ? "NO-LAG ON" : "NO-LAG OFF", false, notice_duration);
                    set_window_title(window, rom_path, nes, paused);
                    break;
                }
                case SDL_SCANCODE_F11: {
                    Uint32 mode = SDL_GetWindowFlags(window) & SDL_WINDOW_FULLSCREEN_DESKTOP;

                    if (SDL_SetWindowFullscreen(window, mode ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP) != 0)
                        fprintf(stderr, "Cannot toggle fullscreen (%s)\n", SDL_GetError());
                    break;
                }
                case SDL_SCANCODE_P:
                    paused = !paused;
                    set_window_title(window, rom_path, nes, paused);
                    break;
                case SDL_SCANCODE_8: screenshot_pending = true; break;
                case SDL_SCANCODE_R: reset_requested = true; break;
                default: break;
                }
            }
        }
        if (!running) {
            SDL_free(pending_rom);
            break;
        }
        if (reset_requested || pending_rom) {
            // Flush the live save before loading, including when reopening the same ROM
            bool saved = save_battery_ram(nes);
            const char *load_error = "CANNOT OPEN ROM";

            rom_reloaded = saved && load_rom(nes, pending_rom ? pending_rom : rom_path, &runahead, &load_error);
            const char *status;

            if (!saved)
                status = "CANNOT SAVE GAME";
            else if (pending_rom)
                status = rom_reloaded ? "GAME LOADED" : load_error;
            else
                status = rom_reloaded ? "RESET" : "RESET FAILED";
            show_notice(&notice, status, false, notice_duration);
            recompiler_notice_due = true;

            if (rom_reloaded) {
                if (pending_rom) {
                    SDL_free(dropped_rom);
                    rom_path = dropped_rom = pending_rom;
                    pending_rom = NULL;
                    paused = false;
                    set_window_title(window, rom_path, nes, paused);
                }
                runahead = open_runahead(&nes->cart, &runahead_depth);
                disk = (DiskStatus){0};
                attach_input(nes, &input);
                overlay = (InstructionOverlay){0};
                raster = (RasterOverlay){0};
                display_frame = nes->ppu.frame;

                if (debug)
                    memset(&debug->raster, 0, sizeof debug->raster);
                refresh_texture = true;
            }
            SDL_free(pending_rom);
        }
        bool timing_changed = was_no_lag != (nes->ppu.extra_blanking_lines != 0);

        if (rom_reloaded || timing_changed) {
            frame_start = deadline = (double)SDL_GetPerformanceCounter();
            next_present = last_refresh = prediction_target = 0;
            frame_work = prediction_work = 0;
        }
        fast_forward = fast_forward_held && !paused;

        if (fast_forward != was_fast_forward) {
#if SDL_VERSION_ATLEAST(2, 0, 18)
            SDL_RenderSetVSync(renderer, !fast_forward);
#endif
            deadline = (double)SDL_GetPerformanceCounter();
            next_present = 0;
            last_refresh = prediction_target = 0;
        }
        if (paused != was_paused)
            last_refresh = prediction_target = 0;
        // No-Lag keeps the frame rate and the APU clock, so its queued audio stays valid
        if (audio && (rom_reloaded || ((paused || fast_forward) && !was_paused && !was_fast_forward))) {
            SDL_PauseAudioDevice(audio, 1);
            SDL_ClearQueuedAudio(audio);
            audio_playing = false;
        }
        const bool show_info = osd_page == OSD_INFO && !debug;

        if (rates.emulation.window_start &&
            (!show_info || paused || rom_reloaded || timing_changed || fast_forward != was_fast_forward))
            rates = (FrameRates){0};
        if (!paused) {
            double run_until = 0;

            if (fast_forward) {
                // Batch frames between short event polls, stopping early when a present is due
                run_until = (double)SDL_GetPerformanceCounter() + input.poll_interval;

                if (next_present < run_until)
                    run_until = next_present;
            } else {
                // Poll fresh input at the first controller latch of this frame
                input.next_poll = 0;
            }
            do {
                if (debug) {
                    nes_frame_with_cpu_raster(nes, &debug->raster);
                    debug->mode = emulation_mode(nes);
                } else if (osd_page == OSD_CPU) {
                    sample_instruction_stats(nes, &overlay);
                } else if (osd_page == OSD_RASTER) {
                    sample_raster_stats(nes, &raster);
                } else {
                    nes_frame(nes);
                }
            } while (fast_forward && SDL_GetPerformanceCounter() < run_until);
            // Only the live machine reports disk changes, never a runahead prediction
            const char *changed = disk_notice(nes, &disk);

            if (changed)
                show_notice(&notice, changed, false, notice_duration);
            if (recompiler_notice_due && !nes->interpreter && cpu_recompiler_unavailable(nes) &&
                SDL_GetPerformanceCounter() >= notice.until) {
                show_notice(&notice, "RECOMPILER UNAVAILABLE", false, notice_duration);
                recompiler_notice_due = false;
            }
            if (show_info)
                sample_frame_rate(&rates.emulation, nes->ppu.frames, SDL_GetPerformanceCounter(), counter_hz);
            refresh_texture = true;
        }
        if (fast_forward) {
            double now = (double)SDL_GetPerformanceCounter();

            if (now < next_present)
                continue;
            next_present = now + (refresh_ticks ? refresh_ticks : frequency / 60);
        }
        double audio_pace = 0;

        if (audio && !paused && !fast_forward && nes->apu.sample_count) {
            // Submit PCM before any renderer stall. Prime the queue after startup,
            // a pause, or an underrun so the next frame has some timing headroom.
            size_t queued = SDL_GetQueuedAudioSize(audio);

            if (audio_playing && !queued) {
                SDL_PauseAudioDevice(audio, 1);
                audio_playing = false;
            }
            Uint32 bytes = (Uint32)(nes->apu.sample_count * sizeof nes->apu.samples[0]);

            if (SDL_QueueAudio(audio, nes->apu.samples, bytes) != 0) {
                fprintf(stderr, "Audio stopped (%s)\n", SDL_GetError());
                SDL_CloseAudioDevice(audio);
                audio = 0;
            } else {
                queued += bytes;

                if (!audio_playing && queued >= audio_prefill) {
                    SDL_PauseAudioDevice(audio, 0);
                    audio_playing = true;
                } else if (audio_playing) {
                    // Gradually correct clock drift when the queue leaves its target range
                    if (queued < audio_prefill)
                        audio_pace = (audio_prefill - queued) * -audio_pace_gain;
                    else if (queued > audio_high_water)
                        audio_pace = (queued - audio_high_water) * audio_pace_gain;
                    if (audio_pace > audio_pace_limit)
                        audio_pace = audio_pace_limit;
                    else if (audio_pace < -audio_pace_limit)
                        audio_pace = -audio_pace_limit;
                }
            }
        }
        if (!fast_forward) {
            deadline += frame_ticks + audio_pace;
            double lateness = SDL_GetPerformanceCounter() - deadline;

            // A slow refresh can leave another emulation frame due. Catch up
            // without waiting for a second refresh or predicting a stale frame.
            if (!paused && lateness >= 0 && lateness <= frame_ticks * 2)
                continue;
        }
        double prediction_start = 0, prediction_wait = 0, targeted_refresh = 0;

        if (!paused) {
            display_frame = nes->ppu.frame;

            if (runahead_depth != RUNAHEAD_OFF && !fast_forward && !debug) {
                // The committed frame and its audio are ready. Start prediction
                // near the display deadline so it can use later input.
                double predict_at = prediction_target - prediction_work - prediction_margin;

                prediction_start = (double)SDL_GetPerformanceCounter();

                if (predict_at - prediction_start >= ticks_per_ms) {
                    double ready = prediction_start;

                    sleep_until(predict_at, ticks_per_ms);
                    prediction_start = (double)SDL_GetPerformanceCounter();
                    prediction_wait = prediction_start - ready;
                    targeted_refresh = prediction_target;
                }
                // Poll at the prediction's controller latch, after the state copy
                input.next_poll = 0;
                display_frame = nes_runahead_frame(runahead, nes, runahead_depth, NULL);
            }
        }
        if (notice.text && SDL_GetPerformanceCounter() >= notice.until)
            notice.text = NULL;
        if (debug && refresh_texture)
            compose_cpu_debug(debug, &nes->ppu);
        SDL_Texture *frame_texture = debug ? debug->texture : nes->ppu.rgb565 ? texture_rgb565 : texture_argb;
        const SDL_Rect *frame_view = debug ? &debug->view : &view;
        int debug_pitch = debug ? debug->view.w * (int)sizeof(uint32_t) : 0;
        // Draw at logical resolution for screenshots, then scale that same image to the window
        SDL_Texture *capture_target = screenshot_pending ? begin_screenshot(renderer) : NULL;

        if ((refresh_texture && (debug ? SDL_UpdateTexture(frame_texture, frame_view, debug->pixels, debug_pitch)
                                       : upload_frame(frame_texture, &view, &nes->ppu, display_frame)) != 0) ||
            SDL_RenderClear(renderer) != 0 ||
            SDL_RenderCopy(renderer, frame_texture, frame_view, debug ? frame_view : NULL) != 0 ||
            (debug && draw_cpu_legend(renderer, nes, debug) != 0) ||
            (!debug && osd_page != OSD_HIDDEN &&
             draw_stats(renderer, osd_page, &overlay, &raster, &rates, paused, nes) != 0) ||
            (notice.text && draw_notice(renderer, debug ? debug->view.w + 4 : 8, frame_view->h,
                                        notice.text, notice.is_mode ? nes : NULL) != 0)) {
            fprintf(stderr, "Video stopped (%s)\n", SDL_GetError());
            SDL_DestroyTexture(capture_target);
            result = 1;
            break;
        }
        SDL_Surface *screenshot = NULL;

        if (capture_target) {
            screenshot = read_screenshot(renderer, capture_target);
            bool display_ok = SDL_SetRenderTarget(renderer, NULL) == 0 &&
                              SDL_RenderClear(renderer) == 0 &&
                              SDL_RenderCopy(renderer, capture_target, NULL, NULL) == 0;

            SDL_DestroyTexture(capture_target);
            if (!display_ok) {
                fprintf(stderr, "Cannot restore video after screenshot (%s)\n", SDL_GetError());
                SDL_FreeSurface(screenshot);
                result = 1;
                break;
            }
        }
        bool measure_display = show_info && !paused && refresh_texture;

        refresh_texture = false;
        double submitted = (double)SDL_GetPerformanceCounter();

        SDL_RenderPresent(renderer);
        if (fast_forward && !measure_display && !screenshot_pending)
            continue;

        Uint64 presented_at = SDL_GetPerformanceCounter();

        if (screenshot_pending) {
            bool saved = screenshot && save_screenshot(rom_path, screenshot);

            SDL_FreeSurface(screenshot);
            screenshot_pending = false;
            show_notice(&notice, saved ? "SCREENSHOT SAVED" : "SCREENSHOT FAILED", false, notice_duration);
        }
        if (measure_display) {
            if (SDL_GetWindowFlags(window) & (SDL_WINDOW_HIDDEN | SDL_WINDOW_MINIMIZED))
                rates.display = (FrameRate){0};
            else
                sample_frame_rate(&rates.display, ++rates.presented_frames, presented_at, counter_hz);
        }
        if (fast_forward)
            continue;

        double now = (double)presented_at;

        if (now - deadline > frame_ticks * 2)
            deadline = now;
        double wake = deadline;

        prediction_target = 0;

        if (!paused && display_sync && refresh_ticks &&
            !(SDL_GetWindowFlags(window) & (SDL_WINDOW_HIDDEN | SDL_WINDOW_MINIMIZED))) {
            frame_work = SDL_max(submitted - frame_start - prediction_wait, frame_work * 0.95);
            // SDL reports no scanout timestamp. A blocking present provides a
            // recent estimate of display phase. Do not time from queued presents.
            bool blocking = now - submitted >= 0.75 * ticks_per_ms;

            if (blocking)
                last_refresh = now;
            // A slower display needs the full frame budget for audio catch-up
            bool late_prediction = prediction_start && refresh_ticks <= frame_ticks + prediction_margin;

            if (late_prediction)
                prediction_work = SDL_max(submitted - prediction_start, prediction_work * 0.95);
            if (last_refresh && now - last_refresh <= frame_ticks * 2) {
                double earliest = SDL_max(deadline, now);
                unsigned refreshes = (unsigned)((earliest - last_refresh) / refresh_ticks) + 1;
                double refresh = last_refresh + refreshes * refresh_ticks;

                // A quick present can still have queued a frame for its target
                if (!blocking && targeted_refresh && refresh <= targeted_refresh)
                    refresh += refresh_ticks;
                double lead = frame_work + (prediction_start ? prediction_margin : frame_margin);
                double latest = deadline + frame_ticks + SDL_min(audio_pace, 0.0) - lead;

                wake = SDL_max(deadline, SDL_min(refresh - lead, latest));

                if (late_prediction && refresh <= earliest + frame_ticks + prediction_margin)
                    prediction_target = refresh;
            }
        }
        sleep_until(wake, ticks_per_ms);
    }

    nes->poll_buttons = NULL;
    nes->input_context = NULL;
    nes_runahead_destroy(runahead);
    close_cpu_debug(debug);
    if (audio)
        SDL_CloseAudioDevice(audio);
    for (unsigned i = 0; i < 2; ++i) {
        if (input.pads[i])
            SDL_GameControllerClose(input.pads[i]);
    }
cleanup_video:
    SDL_free(dropped_rom);
    if (texture_argb) SDL_DestroyTexture(texture_argb);
    if (texture_rgb565) SDL_DestroyTexture(texture_rgb565);
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
    return result;
}

int main(int argc, char **argv)
{
    const char *path = NULL;
    unsigned long long frames = 0;
    bool ultrafast = false, interpreter = false, no_lag = false, no_sprite_limit = false;
    bool crop = true;
    RunaheadDepth runahead = RUNAHEAD_ONE_FRAME;

    if (argc == 2 && (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h"))) {
        usage(stdout, argv[0]);
        return 0;
    }
    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];

        if (!strcmp(arg, "-u")) {
            ultrafast = true;
        } else if (!strcmp(arg, "-i")) {
            interpreter = true;
        } else if (!strcmp(arg, "-n")) {
            no_lag = true;
        } else if (!strcmp(arg, "-s")) {
            no_sprite_limit = true;
        } else if (!strcmp(arg, "-f")) {
            crop = false;
        } else if (!strcmp(arg, "-r")) {
            runahead = RUNAHEAD_OFF;
        } else if (!strcmp(arg, "--headless") && !frames && i + 1 < argc) {
            const char *count = argv[++i];
            char *end;

            errno = 0;
            frames = strtoull(count, &end, 10);

            if (errno || count[0] < '0' || count[0] > '9' || *end || !frames) {
                fprintf(stderr, "Frame count must be a positive integer\n");
                return 2;
            }
        } else if (!path && arg[0] != '-') {
            path = arg;
        } else {
            usage(stderr, argv[0]);
            return 2;
        }
    }
    if (frames && !path) {
        usage(stderr, argv[0]);
        return 2;
    }
    if (ultrafast && interpreter) {
        fprintf(stderr, "Choose either -u or -i.\n");
        return 2;
    }

    Nes *nes = calloc(1, sizeof *nes);

    if (!nes) {
        fprintf(stderr, "Out of memory\n");
        return 1;
    }
    add_bios_directories();
    nes->ultrafast = ultrafast;
    nes->interpreter = interpreter;
    nes->ppu.extra_blanking_lines = no_lag ? NES_EXTRA_BLANKING_LINES : 0;
    nes->ppu.no_sprite_limit = no_sprite_limit;
    int result = 1;

    if (!path || load_rom(nes, path, NULL, NULL)) {
        result = 0;

        if (frames)
            run_headless(nes, frames);
        else
            result = run_interactive(nes, path, crop, runahead);
        if (!save_battery_ram(nes))
            result = 1;
    }
    cart_free(&nes->cart);
    free(nes);
    return result;
}
