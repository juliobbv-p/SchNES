#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include "nes.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

enum { MAX_ROM = 64 * 1024 * 1024, MAX_RAM = 1024 * 1024 };
// iNES mapper 4 starts as MMC3 and switches to open MMC6-compatible RAM on detection
enum { MMC3_RAM_NORMAL, MMC3_RAM_DETECT, MMC3_RAM_OPEN };

static void map_prg(Cartridge *c);
static void map_chr(Cartridge *c);
static void timer_irq_schedule(Cartridge *c);
static void vrc6_mirroring(Cartridge *c);

static inline bool mmc6(const Cartridge *c)
{
    return c->mapper == 4 && c->submapper == 1;
}

static void init_ext(Cartridge *c)
{
    MapperExt *x = c->ext;

    x->irq_cycle = UINT64_MAX;
    x->irq_prescaler = 341;

    if (c->mapper == 69) {
        // Only the 5B has audio, but FME-7 and 5A games never write its ports
        x->has_audio = true;
        x->audio.lfsr = 1;
        return;
    }
    if (c->mapper == 24 || c->mapper == 26) {
        // VRC6b swaps A0 and A1
        x->vrc_a0 = c->mapper == 24 ? 1 : 2;
        x->vrc_a1 = c->mapper == 24 ? 2 : 1;
        x->has_audio = true;
        return;
    }
    // Each board wires two CPU address lines to the VRC register select inputs.
    // Submapper 0 combines both wirings of the iNES mapper.
    static const uint8_t lines[4][4][2] = {
        {{0x42, 0x84}, {0x02, 0x04}, {0x40, 0x80}, {0x02, 0x04}}, // 21: VRC4a/VRC4c
        {{0x02, 0x01}, {0x02, 0x01}, {0x02, 0x01}, {0x02, 0x01}}, // 22: VRC2a
        {{0x05, 0x0a}, {0x01, 0x02}, {0x04, 0x08}, {0x01, 0x02}}, // 23: VRC2b/VRC4f, VRC4e
        {{0x0a, 0x05}, {0x02, 0x01}, {0x08, 0x04}, {0x02, 0x01}}, // 25: VRC2c/VRC4b, VRC4d
    };
    unsigned board = c->mapper == 25 ? 3 : c->mapper - 21;

    x->vrc_a0 = lines[board][c->submapper][0];
    x->vrc_a1 = lines[board][c->submapper][1];
    x->vrc2 = c->mapper == 22 || c->submapper == 3;
    x->vrc_chr_shift = c->mapper == 22;
}

bool cart_fail(char *error, size_t size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(error, size, format, args);
    va_end(args);
    return false;
}

static size_t decode_rom_size(uint8_t low, unsigned high, size_t unit)
{
    if (high == 15) {
        unsigned exponent = low >> 2;

        if (exponent > 26)
            return MAX_ROM + 1;
        return ((size_t)1 << exponent) * (2 * (low & 3) + 1);
    }
    return ((high << 8) | low) * unit;
}

static size_t decode_ram_size(unsigned shift)
{
    return shift ? (size_t)64 << shift : 0;
}

char *cart_path_with_suffix(const char *path, const char *suffix)
{
    size_t length = strlen(path) + strlen(suffix) + 1;
    char *result = malloc(length);

    if (result)
        snprintf(result, length, "%s%s", path, suffix);
    return result;
}

static bool load_battery_ram(Cartridge *cart, const char *path, char *error, size_t size)
{
    if (!cart->nvram_size && !cart->chr_nvram_size)
        return true;
    cart->save_path = cart_path_with_suffix(path, ".sav");

    if (!cart->save_path)
        return cart_fail(error, size, "Out of memory");
    FILE *file = fopen(cart->save_path, "rb");

    if (!file) {
        if (errno == ENOENT)
            return true;
        return cart_fail(error, size, "Cannot read %s (%s)", cart->save_path, strerror(errno));
    }
    // Leave unread bytes of a short save zero-filled
    size_t prg_size = cart->nvram_size;

    if ((!prg_size || fread(cart->ram + cart->nvram_offset, 1, prg_size, file) == prg_size) &&
        cart->chr_nvram_size)
        fread(cart->chr, 1, cart->chr_nvram_size, file);
    bool ok = !ferror(file);

    fclose(file);
    return ok || cart_fail(error, size, "Cannot read %s", cart->save_path);
}

static bool valid_mapper_layout(const Cartridge *cart, size_t chr_rom_size, size_t chr_ram_size)
{
    const size_t prg_size = cart->prg_size;
    const size_t chr_size = cart->chr_size;

    switch (cart->mapper) {
    case 0: case 3:
        return prg_size <= 32768;
    case 1:
        return prg_size <= 524288 && (cart->submapper != 5 || prg_size == 32768);
    case 4:
        return prg_size >= 32768;
    case 5:
        return prg_size >= 32768 && chr_rom_size && cart->ram_size <= 65536 && !(cart->ram_size % 8192);
    case 9:
        return prg_size >= 32768 && prg_size <= 131072 && chr_rom_size &&
               chr_rom_size <= 131072 && !cart->ram_size;
    case 10:
        return prg_size >= 32768 && prg_size <= 262144 && chr_rom_size && chr_rom_size <= 131072;
    case 21: case 22: case 23: case 25:
        return prg_size >= 16384 && prg_size <= 262144 && chr_size <= 524288;
    case 24: case 26:
        return prg_size >= 16384 && prg_size <= 262144 && chr_size <= 262144;
    case 66:
        return prg_size <= 131072 && chr_size <= 32768;
    case 69:
        return prg_size >= 16384 && prg_size <= 524288;
    case 71:
        return (prg_size == 65536 || prg_size == 131072 || prg_size == 262144) &&
               !chr_rom_size && chr_ram_size == 8192 && !cart->chr_nvram_size && !cart->ram_size;
    case 206:
        return cart->submapper == 1 ? prg_size == 32768 : prg_size >= 16384;
    default:
        return true;
    }
}

static bool read_ines(Cartridge *cart, FILE *file, char *error, size_t size)
{
    uint8_t header[16];

    if (fread(header, 1, sizeof header, file) != sizeof header || memcmp(header, "NES\x1a", 4))
        return cart_fail(error, size, "Expected an iNES or NES 2.0 cartridge image");

    bool nes2 = (header[7] & 0x0c) == 8;
    bool clean_ines = (header[7] & 0x0c) == 0 && !(header[12] || header[13] || header[14] || header[15]);

    cart->mapper = header[6] >> 4;

    if (nes2 || clean_ines)
        cart->mapper |= header[7] & 0xf0;
    if (nes2) {
        cart->mapper |= (header[8] & 15) << 8;
        cart->submapper = header[8] >> 4;
    }
    if (((nes2 || clean_ines) && (header[7] & 3)) ||
        (nes2 && ((header[12] & 3) == 1 || (header[12] & 3) == 3)) ||
        (clean_ines && (header[9] & 1)))
        return cart_fail(error, size, "This cartridge requires hardware other than an NTSC NES");

    unsigned prg_unit = 16384, chr_unit = 8192;

    switch (cart->mapper) {
    case 0: case 2: case 3: case 71: break;
    case 1: chr_unit = 4096; break;
    case 4: case 5: case 21: case 22: case 23: case 24: case 25: case 26: case 69: case 206:
        prg_unit = 8192; chr_unit = 1024; break;
    case 7: case 66: prg_unit = 32768; break;
    case 9: prg_unit = 8192; chr_unit = 4096; break;
    case 10: chr_unit = 4096; break;
    default: return cart_fail(error, size, "Unsupported mapper %u", cart->mapper);
    }

    bool discrete = cart->mapper == 2 || cart->mapper == 3 || cart->mapper == 7;
    bool vrc = cart->mapper == 21 || cart->mapper == 22 || cart->mapper == 23 || cart->mapper == 25;
    bool vrc6 = cart->mapper == 24 || cart->mapper == 26;
    bool supported_submapper;

    switch (cart->mapper) {
    case 1: supported_submapper = cart->submapper == 1 || cart->submapper == 2 ||
                                  cart->submapper == 4 || cart->submapper == 5; break;
    case 2: case 3: case 7: supported_submapper = cart->submapper <= 2; break;
    // MMC6, Acclaim MC-ACC and NEC MMC3
    case 4: supported_submapper = cart->submapper == 1 || cart->submapper == 3 || cart->submapper == 4; break;
    case 21: supported_submapper = cart->submapper <= 2; break;
    case 23: case 25: supported_submapper = cart->submapper <= 3; break;
    case 71: case 206: supported_submapper = cart->submapper <= 1; break;
    default: supported_submapper = false; break;
    }
    if (cart->submapper && !supported_submapper)
        return cart_fail(error, size, "Unsupported mapper %u submapper %u", cart->mapper, cart->submapper);
    cart->bus_conflicts = cart->mapper == 66 || (discrete && (cart->submapper == 2 ||
        (!cart->submapper && cart->mapper != 7)));

    cart->prg_size = decode_rom_size(header[4], nes2 ? header[9] & 15 : 0, 16384);
    size_t chr_rom_size = decode_rom_size(header[5], nes2 ? header[9] >> 4 : 0, 8192);

    cart->chr_nvram_size = nes2 ? decode_ram_size(header[11] >> 4) : 0;
    size_t chr_ram_size = nes2 ? decode_ram_size(header[11] & 15) : (chr_rom_size ? 0 : 8192);

    cart->chr_is_ram = !chr_rom_size;
    cart->chr_size = chr_rom_size ? chr_rom_size : chr_ram_size + cart->chr_nvram_size;
    cart->nvram_size = nes2 ? decode_ram_size(header[10] >> 4) : 0;
    cart->ram_size = nes2 ? decode_ram_size(header[10] & 15) + cart->nvram_size : (header[8] ? header[8] : 1) * 8192u;

    // These boards have no PRG RAM
    if (!nes2 && (cart->mapper == 9 || cart->mapper == 66 || cart->mapper == 71 || cart->mapper == 206))
        cart->ram_size = 0;
    if (!nes2 && (header[6] & 2))
        cart->nvram_size = cart->ram_size;
    if (cart->mapper == 1 && cart->chr_is_ram && cart->ram_size == 16384 && cart->nvram_size == 8192)
        cart->nvram_offset = 8192; // SOROM battery-backs the second RAM bank
    bool has_trainer = (header[6] & 4) != 0;

    if (has_trainer && cart->ram_size < 8192)
        cart->ram_size = 8192;
    if (cart->mapper == 4 && cart->submapper == 1) {
        // MMC6 has 1 KiB of internal RAM, mirrored at $7000-$7FFF
        cart->nvram_size = (cart->nvram_size || (header[6] & 2)) ? 1024 : 0;
        cart->ram_size = 1024;
        cart->nvram_offset = 0;
    } else if (cart->mapper == 4 && !nes2) {
        // Detect iNES MMC6 from $8000 bit 5 and the $A001 half-bank enables
        cart->mmc3_ram_mode = MMC3_RAM_DETECT;
    }

    if (!cart->prg_size || cart->prg_size > MAX_ROM || cart->prg_size % prg_unit ||
        !cart->chr_size || cart->chr_size > MAX_ROM || cart->chr_size % chr_unit ||
        (chr_rom_size && (chr_ram_size || cart->chr_nvram_size)) || cart->ram_size > MAX_RAM ||
        !valid_mapper_layout(cart, chr_rom_size, chr_ram_size))
        return cart_fail(error, size, "Unsupported PRG or CHR memory layout for mapper %u", cart->mapper);

    // Reuse MMC2 latches while retaining the MMC4 RAM and PRG layout
    if (cart->mapper == 10) {
        cart->mapper = 9;
        cart->mmc4 = true;
    }
    if (header[6] & 8)
        cart->mirroring = MIRROR_FOUR;
    else if (cart->mapper == 1 || cart->mapper == 7)
        cart->mirroring = MIRROR_SINGLE_LOW;
    else
        cart->mirroring = (header[6] & 1) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
    cart->mmc1.shift = 0x10;
    cart->mmc1.control = 0x0c;
    cart->mmc1.last_write = UINT64_MAX;
    cart->mmc3.ram_control = 0x80;
    cart->mmc2.latches = 3;

    if (vrc || vrc6 || cart->mapper == 69) {
        cart->ext = calloc(1, sizeof *cart->ext);

        if (!cart->ext)
            return cart_fail(error, size, "Out of memory");
        init_ext(cart);
    }
    cart->special_wram = cart->ext || mmc6(cart);
    cart->extended_banking = cart->ext || cart->mmc4 || cart->mapper == 66 || cart->mapper == 206;

    if (cart->mapper == 5) {
        cart->mmc5 = calloc(1, sizeof *cart->mmc5);

        if (!cart->mmc5)
            return cart_fail(error, size, "Out of memory");
        cart->mmc5->prg_mode = 3;
        cart->mmc5->prg[4] = 0xff;
        cart->mmc5->exram_mode = 3;
    }
    cart->prg = malloc(cart->prg_size);
    cart->chr = calloc(cart->chr_size, 1);
    cart->ram = cart->ram_size ? calloc(cart->ram_size, 1) : NULL;

    if (!cart->prg || !cart->chr || (cart->ram_size && !cart->ram))
        return cart_fail(error, size, "Out of memory");
    if ((has_trainer && fread(cart->ram + 4096, 1, 512, file) != 512) ||
        fread(cart->prg, 1, cart->prg_size, file) != cart->prg_size ||
        (chr_rom_size && fread(cart->chr, 1, chr_rom_size, file) != chr_rom_size))
        return cart_fail(error, size, "Truncated cartridge image");
    if (cart->mapper == 69) {
        cart->ext->banked_ram_size = cart->ram_size;
        cart->ram_size = 0;
    }
    map_prg(cart);
    return true;
}

static void decode_chr(Cartridge *c)
{
    // Keep the decoded copy bounded for cartridges with large CHR ROMs
    if (c->chr_size > 512 * 1024)
        return;
    c->decoded_chr = malloc(c->chr_size / 2 * sizeof *c->decoded_chr);

    if (!c->decoded_chr)
        return;
    for (size_t tile = 0; tile < c->chr_size / 16; ++tile)
        for (unsigned row = 0; row < 8; ++row)
            c->decoded_chr[tile * 8 + row] = chr_decode_row(c->chr + tile * 16 + row);
}

bool cart_load(Cartridge *c, const char *path, char *error, size_t size)
{
    FILE *file = fopen(path, "rb");

    if (!file)
        return cart_fail(error, size, "Cannot open %s (%s)", path, strerror(errno));

    uint8_t start[16] = {0};
    bool disk = fread(start, 1, sizeof start, file) == sizeof start && fds_is_disk_image(start);
    bool ok;

    if (disk) {
        fclose(file);
        ok = fds_load(c, path, error, size);
    } else {
        rewind(file);
        ok = read_ines(c, file, error, size);
        fclose(file);
        if (ok)
            ok = load_battery_ram(c, path, error, size);
    }
    if (ok) {
        unsigned mapper = c->mapper;

        c->linear_chr_ram = c->chr_is_ram && c->chr_size == 8192 &&
            (mapper == 0 || mapper == 2 || mapper == 3 || mapper == 7 || mapper == 71 || cart_is_fds(c));
        decode_chr(c);
        if (c->decoded_chr) {
            c->chr_rom_tile_cache = !c->chr_is_ram && mapper != 5 && mapper != 9;
            c->banked_chr_ram = c->chr_is_ram && c->chr_size == 8192 && (mapper == 1 || mapper == 4);
        }
        map_chr(c);
    } else {
        cart_free(c);
    }
    return ok;
}

bool cart_save(Cartridge *c, char *error, size_t size)
{
    if (!c->save_path || !c->save_dirty)
        return true;
#ifdef _WIN32
    char temp_suffix[48];

    snprintf(temp_suffix, sizeof temp_suffix, ".tmp-%lu-%llu", (unsigned long)GetCurrentProcessId(),
             (unsigned long long)GetTickCount64());
#else
    const char temp_suffix[] = ".tmp-XXXXXX";
#endif
    char *temp = cart_path_with_suffix(c->save_path, temp_suffix);

    if (!temp)
        return cart_fail(error, size, "Out of memory");
#ifdef _WIN32
    FILE *file = fopen(temp, "wbx");
#else
    int fd = mkstemp(temp);
    FILE *file = fd >= 0 ? fdopen(fd, "wb") : NULL;

    if (fd >= 0 && !file) {
        int reason = errno;

        close(fd);
        remove(temp);
        errno = reason;
    }
#endif
    if (!file) {
        cart_fail(error, size, "Cannot create %s (%s)", temp, strerror(errno));
        free(temp);
        return false;
    }
    bool ok;

    if (cart_is_fds(c)) {
        uint8_t header[16];
        const uint8_t *disk;
        size_t bytes = fds_save_image(c, header, &disk);

        ok = fwrite(header, 1, sizeof header, file) == sizeof header &&
             fwrite(disk, 1, bytes, file) == bytes;
    } else {
        ok = !c->nvram_size ||
            fwrite(c->ram + c->nvram_offset, 1, c->nvram_size, file) == c->nvram_size;

        if (ok && c->chr_nvram_size)
            ok = fwrite(c->chr, 1, c->chr_nvram_size, file) == c->chr_nvram_size;
    }
    if (fclose(file))
        ok = false;
    if (ok) {
#ifdef _WIN32
        ok = MoveFileExA(temp, c->save_path, MOVEFILE_REPLACE_EXISTING) != 0;
#else
        ok = rename(temp, c->save_path) == 0;
#endif
    }
    if (!ok) {
        remove(temp);
        cart_fail(error, size, "Cannot save %s", c->save_path);
    } else {
        c->save_dirty = false;
    }
    free(temp);
    return ok;
}

void cart_free(Cartridge *c)
{
    cpu_discard_traces(c);
    free(c->prg);
    free(c->chr);
    free(c->mmc5);
    fds_free(c);
    free(c->ext);
    free(c->decoded_chr);
    free(c->ram);
    free(c->save_path);
    *c = (Cartridge){0};
}

static size_t ram_offset(const Cartridge *c, uint16_t address)
{
    unsigned bank = 0;

    if (c->mapper == 1 && c->chr_is_ram) {
        if (c->ram_size > 16384)
            bank = (c->mmc1.chr0 >> 2) & 3;
        else if (c->ram_size > 8192)
            bank = (c->mmc1.chr0 >> 3) & 1;
    }
    size_t offset = bank * 8192u + (address & 8191);

    return offset < c->ram_size ? offset : offset % c->ram_size;
}

static bool ram_enabled(const Cartridge *c)
{
    return c->ram_size && (c->mapper != 1 || !(c->mmc1.prg & 16)) &&
        (c->mapper != 4 || (c->mmc3.ram_control & 128));
}

// Mappers added after the original set use their own page functions, which
// keeps the code generated for the common mappers' remaps unchanged.
static unsigned extended_prg_page(const Cartridge *c, unsigned page)
{
    unsigned last = (unsigned)(c->prg_size / 8192) - 1;

    switch (c->mapper) {
    case 9: {
        // MMC4: one 16 KiB bank at $8000 and the last 16 KiB fixed
        unsigned bank = page >= 2 ? (last - 1) / 2 : c->bank;

        return bank * 2 + (page & 1);
    }
    case 21: case 22: case 23: case 25: {
        // VRC4 swap mode exchanges the switchable $8000 bank with the fixed $C000 bank
        const MapperExt *x = c->ext;
        unsigned banks[4] = {x->prg[0] & 31u, x->prg[1] & 31u, last - 1, last};

        if (x->vrc_swap && !(page & 1))
            page ^= 2;
        return banks[page];
    }
    case 24: case 26: {
        const MapperExt *x = c->ext;
        return page < 2 ? (x->prg[0] & 15u) * 2 + page : page == 2 ? x->prg[1] & 31u : last;
    }
    case 66: return ((c->bank >> 4) & 3) * 4 + page;
    case 69: return page < 3 ? c->ext->prg[page + 1] & 63u : last;
    case 206: {
        if (c->submapper == 1)
            return page;
        unsigned banks[4] = {c->mmc3.banks[6] & 15u, c->mmc3.banks[7] & 15u, last - 1, last};

        return banks[page];
    }
    default: return page;
    }
}

static unsigned prg_page_number(const Cartridge *c, unsigned page)
{
    unsigned bank, slot = page >> 1;
    unsigned count = (unsigned)(c->prg_size / 16384);

    switch (c->mapper) {
    case 2: case 71: bank = slot ? count - 1 : c->bank; break;
    case 4: {
        unsigned last = (unsigned)(c->prg_size / 8192) - 1;
        unsigned banks[4] = {c->mmc3.banks[6] & 63u, c->mmc3.banks[7] & 63u, last - 1, last};

        if ((c->mmc3.select & 0x40) && !(page & 1))
            page ^= 2;
        return banks[page];
    }
    case 7: return (c->bank & 7) * 4 + page;
    case 9: return page ? (unsigned)(c->prg_size / 8192) - 4 + page : c->bank;
    default: return page;
    }
    return bank * 2 + (page & 1);
}

static size_t mmc5_ram_offset(const Mmc5 *m, uint16_t address)
{
    unsigned bank = address < 0x8000 ? m->prg[0] & 7 : m->ram_banks[(address >> 13) - 4];
    return bank * 8192u + (address & 8191);
}

static uint8_t read_mmc5(const Cartridge *c, uint16_t address, uint8_t bus)
{
    Mmc5 *m = c->mmc5;

    if (address >= 0x8000 && !(m->ram_slots & (1u << ((address >> 13) - 4))))
        return cart_prg_read(c, address);
    if (address >= 0x6000) {
        size_t offset = mmc5_ram_offset(m, address);
        return offset < c->ram_size ? c->ram[offset] : bus;
    }
    if (address >= 0x5c00)
        return m->exram_mode >= 2 ? m->exram[address & 1023] : bus;
    switch (address) {
    case 0x5015: return 0;
    case 0x5204: {
        uint8_t status = bus & 0x3f;

        if (m->in_frame)
            status |= 0x40;
        if (m->irq_pending)
            status |= 0x80;
        m->irq_pending = false;
        return status;
    }
    case 0x5205: return (uint8_t)(m->multiplier[0] * m->multiplier[1]);
    case 0x5206: return (m->multiplier[0] * m->multiplier[1]) >> 8;
    default: return bus;
    }
}

// MMC6 maps two 512-byte halves of its RAM with separate read and write enables
static unsigned mmc6_permissions(const Cartridge *c, uint16_t address)
{
    if (address < 0x7000 || !(c->mmc3.select & 0x20))
        return 0;
    unsigned bits = c->mmc3.ram_control >> ((address & 0x200) ? 6 : 4);

    // Writes need the half to be readable as well
    return (bits & 2) ? 2 | (bits & 1) : 0;
}

// $6000-$7FFF for mappers that do not map plain 8 KiB RAM there
static uint8_t read_special_wram(const Cartridge *c, uint16_t address, uint8_t bus)
{
    if (mmc6(c)) {
        if (mmc6_permissions(c, address))
            return c->ram[address & 1023];
        // With only the other half readable, this half reads as zero
        return address >= 0x7000 && (c->mmc3.select & 0x20) && (c->mmc3.ram_control & 0xa0) ? 0 : bus;
    }
    const MapperExt *x = c->ext;

    if (c->mapper == 69) {
        unsigned reg = x->prg[0];

        if (x->wram_rom)
            return x->wram_rom[address & 8191];
        if (!(reg & 0x80) || !x->banked_ram_size)
            return bus;
        return c->ram[((reg & 63u) * 8192u + (address & 8191)) % x->banked_ram_size];
    }
    // VRC: the chip decodes only $6000-$6FFF. A VRC2 without RAM has a one-bit latch there
    if (address >= 0x7000 && c->ram_size < 8192)
        return bus;
    if (!c->ram_size)
        return x->vrc2 ? (uint8_t)((bus & 0xfe) | x->vrc_latch) : bus;
    return c->ram[(address & 8191) % c->ram_size];
}

static void write_special_wram(Cartridge *c, uint16_t address, uint8_t value)
{
    if (mmc6(c)) {
        if (mmc6_permissions(c, address) & 1)
            cart_store_ram(c, address & 1023, value);
        return;
    }
    MapperExt *x = c->ext;

    if (c->mapper == 69) {
        unsigned reg = x->prg[0];

        if ((reg & 0xc0) == 0xc0 && x->banked_ram_size)
            cart_store_ram(c, ((reg & 63u) * 8192u + (address & 8191)) % x->banked_ram_size, value);
        return;
    }
    if (address >= 0x7000 && c->ram_size < 8192)
        return;
    if (!c->ram_size)
        x->vrc_latch = value & 1;
    else
        cart_store_ram(c, (address & 8191) % c->ram_size, value);
}


uint8_t cart_cpu_read(const Cartridge *c, uint16_t address, uint8_t bus)
{
    if (c->mmc5)
        return read_mmc5(c, address, bus);
    if (address >= 0x8000)
        return cart_prg_read(c, address);
    if (address >= 0x6000 && c->special_wram)
        return read_special_wram(c, address, bus);
    if (address >= 0x6000 && ram_enabled(c))
        return c->ram[ram_offset(c, address)];
    return bus;
}

static void map_mmc5_prg(Cartridge *c, unsigned pages)
{
    // Each CPU window selects a register and a position within its bank group
    static const uint8_t registers[4][4] = {{4, 4, 4, 4}, {2, 2, 4, 4}, {2, 2, 3, 4}, {1, 2, 3, 4}};
    static const uint8_t masks[4][4] = {{3, 3, 3, 3}, {1, 1, 1, 1}, {1, 1, 0, 0}, {0, 0, 0, 0}};
    Mmc5 *m = c->mmc5;

    m->ram_slots = 0;

    for (unsigned page = 0; page < 4; ++page) {
        unsigned reg = registers[m->prg_mode][page];
        unsigned mask = masks[m->prg_mode][page];
        unsigned bank = (m->prg[reg] & 127 & ~mask) | (page & mask);

        if (reg != 4 && !(m->prg[reg] & 0x80)) {
            m->ram_slots |= 1u << page;
            m->ram_banks[page] = bank & 7;
            // The timing interpreter handles CPU reads while a RAM slot is mapped
            c->prg_pages[page] = c->prg;
        } else {
            c->prg_pages[page] = c->prg + (bank % pages) * 8192u;
        }
    }
}

static void map_mmc1_prg(Cartridge *c, unsigned count)
{
    unsigned lower = 0, upper = 1;

    if (c->submapper != 5) {
        unsigned region = count > 16 ? c->mmc1.chr0 & 16 : 0;
        unsigned selected = c->mmc1.prg & 15;

        switch ((c->mmc1.control >> 2) & 3) {
        case 0: case 1: lower = region + (selected & 14); upper = lower + 1; break;
        case 2: lower = region; upper = region + selected; break;
        default: lower = region + selected; upper = count > 16 ? region + 15 : count - 1; break;
        }
    }
    if (count & (count - 1)) {
        lower %= count;
        upper %= count;
    } else {
        lower &= count - 1;
        upper &= count - 1;
    }
    const uint8_t *lo = c->prg + lower * 16384u;
    const uint8_t *hi = c->prg + upper * 16384u;

    c->prg_pages[0] = lo;
    c->prg_pages[1] = lo + 8192;
    c->prg_pages[2] = hi;
    c->prg_pages[3] = hi + 8192;
}

static void map_fme7_wram(Cartridge *c)
{
    unsigned reg = c->ext->prg[0];

    c->ext->wram_rom = reg & 0x40 ? NULL : c->prg + ((reg & 63u) % (c->prg_size / 8192)) * 8192;
}

static NES_NOINLINE void map_extended_prg(Cartridge *c, unsigned pages)
{
    for (unsigned i = 0; i < 4; ++i)
        c->prg_pages[i] = c->prg + (extended_prg_page(c, i) % pages) * 8192u;
    if (c->mapper == 69)
        map_fme7_wram(c);
}

static void map_prg(Cartridge *c)
{
    unsigned pages = (unsigned)(c->prg_size / 8192);

    if (c->mapper == 1) {
        map_mmc1_prg(c, pages / 2);
        return;
    }
    if (c->mmc5) {
        map_mmc5_prg(c, pages);
        return;
    }
    if (c->extended_banking) {
        map_extended_prg(c, pages);
        return;
    }
    for (unsigned i = 0; i < 4; ++i)
        c->prg_pages[i] = c->prg + (prg_page_number(c, i) % pages) * 8192u;
}

static void commit_mmc1(Cartridge *c, uint16_t address, uint8_t value)
{
    switch (address >> 13) {
    case 4: {
        static const Mirroring modes[] = {
            MIRROR_SINGLE_LOW, MIRROR_SINGLE_HIGH, MIRROR_VERTICAL, MIRROR_HORIZONTAL
        };

        c->mmc1.control = value;

        if (c->mirroring != MIRROR_FOUR)
            c->mirroring = modes[value & 3];
        map_prg(c);
        map_chr(c);
        break;
    }
    case 5: {
        bool remap_prg = ((c->mmc1.chr0 ^ value) & 16) && c->prg_size > 262144;

        c->mmc1.chr0 = value;

        if (remap_prg)
            map_prg(c);
        map_chr(c);
        break;
    }
    case 6: c->mmc1.chr1 = value; map_chr(c); break;
    case 7: {
        unsigned changed = (c->mmc1.prg ^ value) & 15;

        c->mmc1.prg = value;

        if (changed && c->submapper != 5)
            map_prg(c);
        break;
    }
    }
}

static void write_mmc1(Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (c->mmc1.last_write != UINT64_MAX && cycle == c->mmc1.last_write + 1)
        return;
    c->mmc1.last_write = cycle;

    if (value & 128) {
        c->mmc1.shift = 0x10;
        c->mmc1.control |= 0x0c;
        map_prg(c);
        return;
    }
    bool complete = (c->mmc1.shift & 1) != 0;

    c->mmc1.shift = (c->mmc1.shift >> 1) | ((value & 1) << 4);

    if (!complete)
        return;
    value = c->mmc1.shift;
    c->mmc1.shift = 0x10;
    commit_mmc1(c, address, value);
}

void cart_commit_mmc1_serial(Cartridge *c, uint16_t address, uint8_t bits, uint64_t cycle)
{
    c->mmc1.last_write = cycle;
    commit_mmc1(c, address, bits & 31);
}

static void map_mmc3_chr_register(Cartridge *c, unsigned reg, unsigned value)
{
    unsigned slot = (reg < 2 ? reg * 2 : reg + 2) ^ ((c->mmc3.select >> 5) & 4);
    unsigned pages = (unsigned)(c->chr_size / 1024);
    unsigned count = reg < 2 ? 2 : 1;

    if (count == 2)
        value &= 254;
    uint64_t tag = c->chr_tags[slot >> 2];

    for (unsigned i = 0; i < count; ++i) {
        unsigned page = slot + i, mapped = (value + i) % pages;

        c->chr_pages[page] = c->chr + mapped * 1024u;

        if (c->decoded_chr)
            c->decoded_chr_pages[page] = c->decoded_chr + mapped * 512u;
        unsigned shift = (page & 3) * 16;

        tag = (tag & ~(UINT64_C(0xffff) << shift)) | ((uint64_t)mapped << shift);
    }
    c->chr_tags[slot >> 2] = tag;
}

static void swap_mmc3_chr_banks(Cartridge *c)
{
    for (unsigned i = 0; i < 4; ++i) {
        const uint8_t *page = c->chr_pages[i];

        c->chr_pages[i] = c->chr_pages[i + 4];
        c->chr_pages[i + 4] = page;
        const uint64_t *decoded = c->decoded_chr_pages[i];

        c->decoded_chr_pages[i] = c->decoded_chr_pages[i + 4];
        c->decoded_chr_pages[i + 4] = decoded;
    }
    uint64_t tag = c->chr_tags[0];

    c->chr_tags[0] = c->chr_tags[1];
    c->chr_tags[1] = tag;
}

static inline void map_mmc2_window(Cartridge *c, unsigned window)
{
    unsigned slot = window * 4;
    unsigned reg = window * 2 + ((c->mmc2.latches >> window) & 1);
    unsigned first = c->mmc2.chr[reg] * 4u;
    const uint8_t *raw = c->chr + first * 1024u;

    for (unsigned i = 0; i < 4; ++i)
        c->chr_pages[slot + i] = raw + i * 1024u;
    if (c->decoded_chr) {
        const uint64_t *decoded = c->decoded_chr + first * 512u;

        for (unsigned i = 0; i < 4; ++i)
            c->decoded_chr_pages[slot + i] = decoded + i * 512u;
    }
    // Repeat the base page in each 16-bit field and add the page offsets
    c->chr_tags[window] = first * UINT64_C(0x0001000100010001) + UINT64_C(0x0003000200010000);
}

static void write_mmc2(Cartridge *c, uint16_t address, uint8_t value)
{
    switch (address >> 12) {
    case 0xa:
        c->bank = value & 15;
        map_prg(c);
        break;
    case 0xb: case 0xc: case 0xd: case 0xe: {
        unsigned reg = (address >> 12) - 0xb;
        unsigned bank = (value & 31) % (c->chr_size / 4096);

        if (c->mmc2.chr[reg] == bank)
            break;
        c->mmc2.chr[reg] = bank;

        if ((reg & 1) == ((c->mmc2.latches >> (reg >> 1)) & 1u))
            map_mmc2_window(c, reg >> 1);
        break;
    }
    case 0xf:
        c->mirroring = (value & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
        break;
    }
}

static void write_mmc6_protect(Cartridge *c, uint8_t value)
{
    if (mmc6(c)) {
        // While $8000 bit 5 disables the RAM, the MMC6 holds $A001 at zero
        c->mmc3.ram_control = (c->mmc3.select & 0x20) ? value : 0;
        return;
    }
    if (c->mmc3_ram_mode == MMC3_RAM_DETECT) {
        // MMC3 ignores $8000 bit 5 and $A001 bits 4-5. An MMC6 game sets the RAM
        // enable first and then its half-bank enables. Keep the RAM enabled
        // and writable from then on, as iNES MMC6 support usually does.
        if (!((c->mmc3.select & 0x20) && (value & 0x30))) {
            c->mmc3.ram_control = value;
            return;
        }
        c->mmc3_ram_mode = MMC3_RAM_OPEN;
    }
    c->mmc3.ram_control = 0x80;
}

// VRC IRQ registers: 0 and 1 write latch nibbles, 2 the control, 3 acknowledges, 4 writes the latch
static void write_vrc_irq(Cartridge *c, unsigned reg, uint8_t value, uint64_t cycle)
{
    MapperExt *x = c->ext;

    // Register writes take effect from the next CPU cycle
    cart_timer_irq_advance(c, cycle + 1);
    switch (reg) {
    case 0: x->irq_latch = (uint8_t)((x->irq_latch & 0xf0) | (value & 15)); break;
    case 1: x->irq_latch = (uint8_t)((x->irq_latch & 15) | (value << 4)); break;
    case 2:
        x->irq_control = value & 7;

        if (value & 2) {
            x->irq_counter = x->irq_latch;
            x->irq_prescaler = 341;
        }
        c->mmc3.irq = false;
        break;
    case 3:
        // Acknowledge, and copy the A bit to the E bit
        x->irq_control = (uint8_t)((x->irq_control & 5) | ((x->irq_control & 1) << 1));
        c->mmc3.irq = false;
        break;
    default: x->irq_latch = value; break;
    }
    timer_irq_schedule(c);
}

static void write_vrc(Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    MapperExt *x = c->ext;
    unsigned reg = ((address & x->vrc_a0) ? 1u : 0) | ((address & x->vrc_a1) ? 2u : 0);

    switch (address >> 12) {
    case 0x8:
        x->prg[0] = value & 31;
        map_prg(c);
        break;
    case 0x9:
        // VRC4 selects PRG swap mode at $9002, while VRC2 supports only H/V mirroring
        if (x->vrc2 || reg < 2) {
            static const Mirroring modes[] = {
                MIRROR_VERTICAL, MIRROR_HORIZONTAL, MIRROR_SINGLE_LOW, MIRROR_SINGLE_HIGH
            };

            if (c->mirroring != MIRROR_FOUR)
                c->mirroring = modes[value & (x->vrc2 ? 1 : 3)];
        } else if (reg == 2 && x->vrc_swap != ((value & 2) != 0)) {
            x->vrc_swap = (value & 2) != 0;
            map_prg(c);
        }
        break;
    case 0xa:
        x->prg[1] = value & 31;
        map_prg(c);
        break;
    case 0xb: case 0xc: case 0xd: case 0xe: {
        unsigned slot = ((address >> 12) - 0xb) * 2 + (reg >> 1);
        unsigned old = x->chr[slot];
        unsigned bank = reg & 1 ? (old & 15) | ((value & (x->vrc2 ? 15 : 31)) << 4)
                                : (old & ~15u) | (value & 15);

        if (bank == old)
            break;
        x->chr[slot] = (uint16_t)bank;
        unsigned pages = (unsigned)(c->chr_size / 1024);
        unsigned mapped = (x->vrc_chr_shift ? bank >> 1 : bank) % pages;
        unsigned shift = (slot & 3) * 16;
        uint64_t tag = c->chr_tags[slot >> 2];

        c->chr_pages[slot] = c->chr + mapped * 1024u;

        if (c->decoded_chr)
            c->decoded_chr_pages[slot] = c->decoded_chr + mapped * 512u;
        c->chr_tags[slot >> 2] = (tag & ~(UINT64_C(0xffff) << shift)) | ((uint64_t)mapped << shift);
        break;
    }
    case 0xf:
        if (!x->vrc2)
            write_vrc_irq(c, reg, value, cycle);
        break;
    }
}

static void write_vrc6(Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    MapperExt *x = c->ext;
    unsigned reg = ((address & x->vrc_a0) ? 1u : 0) | ((address & x->vrc_a1) ? 2u : 0);

    switch (address >> 12) {
    case 0x8:
        x->prg[0] = value & 15;
        map_prg(c);
        break;
    case 0x9: case 0xa: case 0xb:
        if ((address >> 12) == 0xb && reg == 3) {
            x->vrc6_banking = value;
            map_chr(c);
            vrc6_mirroring(c);
        } else {
            vrc6_audio_write(&x->vrc6, (uint16_t)((address & 0xf000) | reg), value);
        }
        break;
    case 0xc:
        x->prg[1] = value & 31;
        map_prg(c);
        break;
    case 0xd: case 0xe:
        x->chr[((address >> 12) - 0xd) * 4 + reg] = value;
        map_chr(c);
        vrc6_mirroring(c);
        break;
    case 0xf:
        if (reg < 3)
            write_vrc_irq(c, reg ? reg + 1 : 4, value, cycle);
        break;
    }
}

static void write_fme7(Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    MapperExt *x = c->ext;

    switch (address >> 13) {
    case 4:
        x->command = value & 15;
        break;
    case 5:
        switch (x->command) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7:
            x->chr[x->command] = value;
            map_chr(c);
            break;
        case 8:
            x->prg[0] = value;
            map_fme7_wram(c);
            break;
        case 9: case 10: case 11:
            x->prg[x->command - 8] = value;
            map_prg(c);
            break;
        case 12: {
            static const Mirroring modes[] = {
                MIRROR_VERTICAL, MIRROR_HORIZONTAL, MIRROR_SINGLE_LOW, MIRROR_SINGLE_HIGH
            };

            if (c->mirroring != MIRROR_FOUR)
                c->mirroring = modes[value & 3];
            break;
        }
        default:
            cart_timer_irq_advance(c, cycle + 1);
            if (x->command == 13) {
                // Every control write acknowledges the IRQ
                x->irq_control = value;
                c->mmc3.irq = false;
            } else if (x->command == 14) {
                x->irq_counter = (uint16_t)((x->irq_counter & 0xff00) | value);
            } else {
                x->irq_counter = (uint16_t)((x->irq_counter & 0xff) | (value << 8));
            }
            timer_irq_schedule(c);
            break;
        }
        break;
    case 6:
        x->audio.select = value;
        break;
    case 7:
        // A nonzero upper nibble in the select register disables writes
        if (!(x->audio.select & 0xf0))
            sunsoft5b_write(&x->audio, x->audio.select, value);
        break;
    }
}

static NES_NOINLINE void write_extended_mapper(Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    switch (c->mapper) {
    case 21: case 22: case 23: case 25: write_vrc(c, address, value, cycle); break;
    case 24: case 26: write_vrc6(c, address, value, cycle); break;
    case 66:
        c->bank = value;
        map_prg(c);
        map_chr(c);
        break;
    case 69: write_fme7(c, address, value, cycle); break;
    case 206:
        if (address < 0xa000) {
            if (!(address & 1)) {
                c->mmc3.select = value & 7;
            } else {
                c->mmc3.banks[c->mmc3.select & 7] = value;
                map_prg(c);
                map_chr(c);
            }
        }
        break;
    }
}

static void write_mmc5(Cartridge *c, uint16_t address, uint8_t value)
{
    Mmc5 *m = c->mmc5;

    if (address >= 0x6000) {
        if (m->ram_protect[0] == 2 && m->ram_protect[1] == 1 &&
            (address < 0x8000 || (m->ram_slots & (1u << ((address >> 13) - 4))))) {
            size_t offset = mmc5_ram_offset(m, address);

            if (offset < c->ram_size)
                cart_store_ram(c, offset, value);
        }
    } else if (address >= 0x5c00) {
        if (m->exram_mode != 3) {
            uint8_t *byte = &m->exram[address & 1023];
            uint8_t stored = (m->exram_mode == 2 || m->in_frame) ? value : 0;

            if (*byte != stored) {
                *byte = stored;
                ++c->chr_revision;
            }
        }
    } else if (address >= 0x5113 && address <= 0x5117) {
        m->prg[address - 0x5113] = value;
        map_prg(c);
    } else if (address >= 0x5120 && address <= 0x512b) {
        unsigned bank = address - 0x5120;

        m->last_chr_bg = bank >= 8;
        uint16_t selected = value | (m->chr_upper << 8);

        if (m->chr[bank] != selected) {
            m->chr[bank] = selected;
            map_chr(c);
        }
    } else {
        switch (address) {
        case 0x5100: m->prg_mode = value & 3; map_prg(c); break;
        case 0x5101: m->chr_mode = value & 3; map_chr(c); break;
        case 0x5102: case 0x5103: m->ram_protect[address & 1] = value & 3; break;
        case 0x5104: m->exram_mode = value & 3; ++c->chr_revision; break;
        case 0x5105: m->nametable_map = value; break;
        case 0x5106: memset(m->fill, value, 960); ++c->chr_revision; break;
        case 0x5107: memset(m->fill + 960, (value & 3) * 0x55, 64); ++c->chr_revision; break;
        case 0x5130:
            if (m->chr_upper != (value & 3)) {
                m->chr_upper = value & 3;
                if (m->exram_mode == 1)
                    ++c->chr_revision;
            }
            break;
        case 0x5203: m->irq_target = value; break;
        case 0x5204: m->irq_enabled = (value & 0x80) != 0; break;
        case 0x5205: case 0x5206: m->multiplier[address - 0x5205] = value; break;
        }
    }
}

void cart_cpu_write(Cartridge *c, uint16_t address, uint8_t value, uint64_t cycle)
{
    if (c->mmc5) {
        write_mmc5(c, address, value);
        return;
    }
    if (address < 0x8000) {
        if (address >= 0x6000 && c->special_wram)
            write_special_wram(c, address, value);
        else if (address >= 0x6000 && ram_enabled(c) &&
            (c->mapper != 4 || !(c->mmc3.ram_control & 0x40))) {
            cart_store_ram(c, ram_offset(c, address), value);
        }
        return;
    }
    if (c->bus_conflicts)
        value &= cart_prg_read(c, address);
    switch (c->mapper) {
    case 1: write_mmc1(c, address, value, cycle); break;
    case 2: c->bank = value; map_prg(c); break;
    case 3: c->bank = value; map_chr(c); break;
    case 4:
        switch (address & 0xe001) {
        case 0x8000: {
            unsigned changed = c->mmc3.select ^ value;

            c->mmc3.select = value;

            if (changed & 0x40) {
                const uint8_t *page = c->prg_pages[0];

                c->prg_pages[0] = c->prg_pages[2];
                c->prg_pages[2] = page;
            }
            if (changed & 0x80) swap_mmc3_chr_banks(c);
            break;
        }
        case 0x8001: {
            unsigned bank = c->mmc3.select & 7;

            if (c->mmc3.banks[bank] == value)
                break;
            c->mmc3.banks[bank] = value;

            if (bank < 6) {
                map_mmc3_chr_register(c, bank, value);
            } else {
                unsigned page = bank == 7 ? 1 : (c->mmc3.select >> 5) & 2;
                size_t offset = (value & 63) % (c->prg_size / 8192) * 8192;

                c->prg_pages[page] = c->prg + offset;
            }
            break;
        }
        case 0xa000:
            if (c->mirroring != MIRROR_FOUR)
                c->mirroring = (value & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            break;
        case 0xa001:
            if (c->mmc3_ram_mode != MMC3_RAM_NORMAL || mmc6(c))
                write_mmc6_protect(c, value);
            else
                c->mmc3.ram_control = value;
            break;
        case 0xc000: c->mmc3.irq_latch = value; break;
        case 0xc001: {
            unsigned bits = mmc3_blocked_reload_bits(c);

            if (bits) {
                c->mmc3.irq_counter |= bits;
            } else {
                c->mmc3.irq_reload = true;
                c->mmc3.irq_counter = 0;
            }
            if (c->submapper == 3)
                c->mcacc_count = 0;
            break;
        }
        case 0xe000: c->mmc3.irq_enabled = c->mmc3.irq = false; break;
        case 0xe001: c->mmc3.irq_enabled = true; break;
        }
        break;
    case 7:
        c->bank = value;
        map_prg(c);
        if (c->mirroring != MIRROR_FOUR)
            c->mirroring = (value & 16) ? MIRROR_SINGLE_HIGH : MIRROR_SINGLE_LOW;
        break;
    case 9: write_mmc2(c, address, value); break;
    case 71:
        // Fire Hawk selects a one-screen nametable at $8000-$9FFF
        if (address < 0xa000 && c->submapper == 1 && c->mirroring != MIRROR_FOUR)
            c->mirroring = (value & 16) ? MIRROR_SINGLE_HIGH : MIRROR_SINGLE_LOW;
        if (address >= 0xc000 && c->bank != (value & 15)) {
            c->bank = value & 15;
            // Mapper 71 has 4, 8, or 16 banks with the upper 16 KiB fixed
            unsigned bank = c->bank & (c->prg_size / 16384 - 1);
            const uint8_t *page = c->prg + bank * 16384u;

            c->prg_pages[0] = page;
            c->prg_pages[1] = page + 8192;
        }
        break;
    default: write_extended_mapper(c, address, value, cycle); break;
    }
}

static unsigned mmc5_chr_page(const Mmc5 *m, unsigned page, bool background)
{
    unsigned shift = 3 - m->chr_mode;
    unsigned mask = (1u << shift) - 1;
    unsigned reg = page | mask;

    if (background)
        reg = 8 + (reg & 3);
    return (m->chr[reg] << shift) | (page & mask);
}

// With $B003 bit 5 set, a 2 KiB bank passes PPU A10 through
static unsigned vrc6_chr_page(const MapperExt *x, unsigned page)
{
    unsigned mode = x->vrc6_banking & 3, pass = (x->vrc6_banking & 0x20) ? 1 : 0;
    unsigned reg;

    if (mode == 0 || (mode >= 2 && page < 4))
        return x->chr[page];
    reg = mode == 1 ? page >> 1 : 4 + ((page - 4) >> 1);
    return (x->chr[reg] & (pass ? 0xfe : 0xff)) | (pass & page);
}

// Approximate other mirroring layouts and use CIRAM for CHR ROM nametables ($B003 bit 4)
static void vrc6_mirroring(Cartridge *c)
{
    const MapperExt *x = c->ext;
    unsigned banking = x->vrc6_banking, pages;

    switch (banking & 0x2f) {
    case 0x20: case 0x27: pages = 0xa; break;
    case 0x23: case 0x24: pages = 0xc; break;
    case 0x28: case 0x2f: pages = 0x0; break;
    case 0x2b: case 0x2c: pages = 0xf; break;
    default: {
        static const uint8_t regs[8][4] = {
            {6, 6, 7, 7}, {4, 5, 6, 7}, {6, 7, 6, 7}, {6, 7, 6, 7},
            {6, 7, 6, 7}, {4, 5, 6, 7}, {6, 6, 7, 7}, {6, 6, 7, 7}
        };

        pages = 0;

        for (unsigned i = 0; i < 4; ++i)
            pages |= (x->chr[regs[banking & 7][i]] & 1u) << i;
        break;
    }
    }
    Mirroring mirroring;

    if (pages == 0x0 || pages == 0xf)
        mirroring = pages ? MIRROR_SINGLE_HIGH : MIRROR_SINGLE_LOW;
    else if ((pages & 3) == 0 || (pages & 3) == 3)
        mirroring = MIRROR_HORIZONTAL;
    else
        mirroring = MIRROR_VERTICAL;
    if (c->mirroring != MIRROR_FOUR)
        c->mirroring = mirroring;
}

static unsigned extended_chr_page(const Cartridge *c, unsigned page)
{
    switch (c->mapper) {
    case 21: case 22: case 23: case 25:
        return c->ext->vrc_chr_shift ? c->ext->chr[page] >> 1 : c->ext->chr[page];
    case 24: case 26: return vrc6_chr_page(c->ext, page);
    case 66: return page + (c->bank & 3) * 8u;
    case 69: return c->ext->chr[page];
    case 206:
        // The left pattern table always uses the two 2 KiB banks
        return page < 4 ? (c->mmc3.banks[page >> 1] & 62u) + (page & 1) : c->mmc3.banks[page - 2] & 63u;
    default: return page;
    }
}

static unsigned chr_page_number(const Cartridge *c, unsigned page)
{
    switch (c->mapper) {
    case 1:
        if (c->mmc1.control & 16)
            return (page & 3) + 4u * (page & 4 ? c->mmc1.chr1 : c->mmc1.chr0);
        return page + (c->mmc1.chr0 & 30) * 4u;
    case 3: return page + c->bank * 8u;
    case 4: {
        unsigned slot = page ^ ((c->mmc3.select & 128) ? 4 : 0);
        return slot < 4 ? (c->mmc3.banks[slot >> 1] & 254u) + (slot & 1)
                        : c->mmc3.banks[slot - 2];
    }
    case 5: return mmc5_chr_page(c->mmc5, page, false);
    default: return page;
    }
}

enum { CHR_ORIGINAL, CHR_MMC5_BACKGROUND, CHR_EXTENDED };

static NES_ALWAYS_INLINE void map_chr_pages(const Cartridge *c, const uint8_t **raw,
                                            const uint64_t **decoded, uint64_t *tags, unsigned kind)
{
    unsigned pages = (unsigned)(c->chr_size / 1024);

    for (unsigned bank = 0; bank < 2; ++bank) {
        uint64_t tag = 0;

        for (unsigned i = 0; i < 4; ++i) {
            unsigned page = bank * 4 + i;
            unsigned mapped = (kind == CHR_MMC5_BACKGROUND ? mmc5_chr_page(c->mmc5, page, true) :
                               kind == CHR_EXTENDED ? extended_chr_page(c, page) : chr_page_number(c, page)) % pages;

            raw[page] = c->chr + mapped * 1024u;

            if (c->decoded_chr)
                decoded[page] = c->decoded_chr + mapped * 512u;
            // Up to 64 MiB of CHR fits in four 16-bit page numbers per bank
            tag |= (uint64_t)mapped << (i * 16);
        }
        tags[bank] = tag;
    }
}

static NES_NOINLINE void map_extended_chr(Cartridge *c)
{
    map_chr_pages(c, c->chr_pages, c->decoded_chr_pages, c->chr_tags, CHR_EXTENDED);
}

static void map_chr(Cartridge *c)
{
    if (c->mapper == 9) {
        map_mmc2_window(c, 0);
        map_mmc2_window(c, 1);
        return;
    }
    if (c->extended_banking) {
        map_extended_chr(c);
        return;
    }
    map_chr_pages(c, c->chr_pages, c->decoded_chr_pages, c->chr_tags, CHR_ORIGINAL);
    Mmc5 *m = c->mmc5;

    if (m)
        map_chr_pages(c, m->bg_pages, m->decoded_bg_pages, m->bg_tags, CHR_MMC5_BACKGROUND);
}

void cart_mmc2_set_latches(Cartridge *c, uint8_t latches)
{
    unsigned changed = c->mmc2.latches ^ latches;

    if (!changed)
        return;
    c->mmc2.latches = latches;

    if (changed & 1)
        map_mmc2_window(c, 0);
    if (changed & 2)
        map_mmc2_window(c, 1);
}

void cart_ppu_write(Cartridge *c, uint16_t address, uint8_t value)
{
    if (c->chr_is_ram) {
        size_t offset = (size_t)(c->chr_pages[address >> 10] - c->chr) + (address & 1023);

        if (c->chr[offset] != value) {
            c->save_dirty |= offset < c->chr_nvram_size;
            c->chr[offset] = value;
            ++c->chr_revision;
            size_t tile = offset / 16, row = offset & 7;

            if (c->decoded_chr && !c->linear_chr_ram)
                c->decoded_chr[tile * 8 + row] = chr_decode_row(c->chr + tile * 16 + row);
            if (c->linear_chr_ram || c->banked_chr_ram)
                c->changed_chr_tiles[tile >> 6] |= UINT64_C(1) << (tile & 63);
        }
    }
}

void cart_mmc3_clock(Cartridge *c, unsigned count)
{
    unsigned period = c->mmc3.irq_latch + 1u;
    unsigned first = mmc3_clocks_to_zero(c);

    // A manual reload blocks another request until the following A12 clock
    c->mmc3_reload_blocked = count == 1 && c->mmc3.irq_reload;
    if (count < first) {
        c->mmc3.irq_reload = false;
        c->mmc3.irq_counter = first - count;
        return;
    }

    // The IRQ stays asserted after the first zero, even if the counter wraps
    if (!c->mmc3.irq && c->mmc3.irq_enabled && mmc3_zero_asserts_irq(c)) {
        c->mmc3.irq = true;
        c->mmc3.irq_ready_cycle = 0;
    }
    c->mmc3.irq_reload = false;
    unsigned remaining = count - first;

    if (remaining >= period)
        remaining %= period;
    c->mmc3.irq_counter = remaining ? period - remaining : 0;
}

void cart_mmc3_ppu_address(Cartridge *c, uint16_t address, uint64_t dot)
{
    bool high = (address & 0x1000) != 0;

    if (!high && c->mmc3.a12) {
        c->mmc3.a12_low = dot;

        // The MC-ACC clocks on the first of every eight falling edges, without a filter
        if (c->submapper == 3 && (c->mcacc_count = (c->mcacc_count + 1) & 7) == 1)
            cart_mmc3_clock(c, 1);
    }
    // M2 falls every three PPU clocks, and short A12 dips during tile fetches are ignored
    if (high && !c->mmc3.a12 && c->submapper != 3 &&
        dot / NES_PPU_DOTS_PER_CPU - c->mmc3.a12_low / NES_PPU_DOTS_PER_CPU >= 3)
        cart_mmc3_clock(c, 1);
    c->mmc3.a12 = high;
}

// VRC4 and FME-7 IRQs count CPU cycles. Their counters are advanced lazily
// whenever a register is written, and the next assertion cycle is published
// through the MMC3 IRQ deadline so CPU batches stop there.
void cart_timer_irq_advance(Cartridge *c, uint64_t cycle)
{
    MapperExt *x = c->ext;

    if (!c->mmc3.irq && x->irq_cycle <= cycle)
        c->mmc3.irq = true;
    if (cycle <= x->irq_base)
        return;
    uint64_t elapsed = cycle - x->irq_base;

    x->irq_base = cycle;

    if (c->mapper == 69) {
        if (x->irq_control & 0x80)
            x->irq_counter = (uint16_t)(x->irq_counter - elapsed);
        return;
    }
    if (!(x->irq_control & 2))
        return;
    uint64_t clocks = elapsed;

    if (!(x->irq_control & 4)) {
        // Scanline mode: subtract 3 per cycle and clock the counter at zero or below
        uint64_t total = 3 * elapsed, prescaler = (uint64_t)x->irq_prescaler;

        if (total < prescaler) {
            x->irq_prescaler = (int16_t)(prescaler - total);
            return;
        }
        clocks = (total - prescaler) / 341 + 1;
        x->irq_prescaler = (int16_t)(341 - (total - prescaler) % 341);
    }
    // The counter trips at $FF and reloads from the latch
    unsigned first = 256u - x->irq_counter;

    if (clocks < first) {
        x->irq_counter = (uint16_t)(x->irq_counter + clocks);
        return;
    }
    unsigned period = 256u - x->irq_latch;

    x->irq_counter = (uint16_t)(x->irq_latch + (clocks - first) % period);
}

static void timer_irq_schedule(Cartridge *c)
{
    MapperExt *x = c->ext;

    x->irq_cycle = UINT64_MAX;

    if (c->mapper == 69) {
        // The IRQ fires when the counter decrements from $0000 to $FFFF
        if ((x->irq_control & 0x81) == 0x81)
            x->irq_cycle = x->irq_base + x->irq_counter + 1u;
    } else if (x->irq_control & 2) {
        uint64_t clocks = 256u - x->irq_counter;

        // In scanline mode the counter clocks once the prescaler has dropped by 341 per step
        if (!(x->irq_control & 4))
            clocks = ((uint64_t)x->irq_prescaler + 341 * (clocks - 1) + 2) / 3;
        x->irq_cycle = x->irq_base + clocks;
    }
    // Leave the deadline to the pending flag once the IRQ is asserted
    c->mmc3.irq_enabled = x->irq_cycle != UINT64_MAX && !c->mmc3.irq;
}
