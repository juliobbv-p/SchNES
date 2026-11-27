#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include "mapper_writes.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FDS maps PRG RAM at $6000-$DFFF. Writes invalidate decoded code in that RAM.
//
// Each .fds side is expanded to the stream the drive reads: a lead-in gap, then
// each block after a gap end mark ($80) and followed by its CRC-16/KERMIT and a gap.

enum {
    FDS_SIDE_BYTES = 65500,
    FDS_LEAD_IN = 28300 / 8,
    FDS_BLOCK_GAP = 976 / 8,
    // About 7.1 s per pass, close to a real disk
    FDS_SIDE_IMAGE = FDS_LEAD_IN + FDS_SIDE_BYTES + 16384,
    FDS_MAX_SIDES = 32,
    // The byte period is 149 1/3 CPU cycles
    FDS_SPIN_UP = 50000,
    FDS_BYTE_THIRDS = 448,
    FDS_BIOS_SIZE = 8192,
    FDS_RAM_SIZE = 32768,
    FDS_SAVE_HEADER = 16,
    FDS_EJECT_POLL_FRAMES = 60,
};

#define FDS_CHANGE_DELAY ((uint64_t)NES_CPU_HZ)

static const char save_magic[10] = {'S', 'C', 'H', 'N', 'E', 'S', '-', 'F', 'D', 'S'};
enum { BIOS_DIRECTORIES = 4 };
static char bios_directories[BIOS_DIRECTORIES][1024];

const char cart_fds_bios_missing[] = "The FDS BIOS (disksys.rom) was not found beside the disk image or SchNES";

void cart_add_bios_directory(const char *directory)
{
    for (unsigned i = 0; i < BIOS_DIRECTORIES && directory && directory[0]; ++i) {
        if (!bios_directories[i][0]) {
            snprintf(bios_directories[i], sizeof bios_directories[i], "%s", directory);
            return;
        }
    }
}

bool fds_is_disk_image(const uint8_t start[16])
{
    return !memcmp(start, "FDS\x1a", 4) || !memcmp(start, "\x01*NINTENDO-HVC*", 15);
}

static uint16_t crc_add(uint16_t crc, uint8_t value)
{
    crc ^= value;

    for (unsigned bit = 0; bit < 8; ++bit)
        crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    return crc;
}

static void build_side(const uint8_t *raw, uint8_t *out)
{
    size_t at = FDS_LEAD_IN, j = 0;
    unsigned file_size = 0;

    while (j < FDS_SIDE_BYTES) {
        size_t length;

        switch (raw[j]) {
        case 1: length = 56; break;
        case 2: length = 2; break;
        case 3:
            length = 16;

            if (j + 14 < FDS_SIDE_BYTES)
                file_size = raw[j + 13] | raw[j + 14] << 8;
            break;
        case 4: length = 1 + (size_t)file_size; break;
        case 0: return;
        default:
            // Present nonstandard data as one block
            length = FDS_SIDE_BYTES - j;
            break;
        }
        if (j + length > FDS_SIDE_BYTES || at + length + 3 + FDS_BLOCK_GAP > FDS_SIDE_IMAGE)
            return;
        uint16_t crc = crc_add(0, 0x80);

        out[at++] = 0x80;

        for (size_t i = 0; i < length; ++i) {
            out[at++] = raw[j + i];
            crc = crc_add(crc, raw[j + i]);
        }
        out[at++] = (uint8_t)crc;
        out[at++] = (uint8_t)(crc >> 8);
        at += FDS_BLOCK_GAP;
        j += length;
    }
}

static bool read_file(const char *path, uint8_t **data, size_t *size, size_t limit)
{
    FILE *file = fopen(path, "rb");

    if (!file)
        return false;
    size_t capacity = 65536, used = 0;
    uint8_t *buffer = malloc(capacity);

    while (buffer) {
        if (used == capacity) {
            if (capacity >= limit)
                break;
            uint8_t *grown = realloc(buffer, capacity * 2);

            if (!grown) {
                free(buffer);
                buffer = NULL;
                break;
            }
            buffer = grown;
            capacity *= 2;
        }
        size_t count = fread(buffer + used, 1, capacity - used, file);

        used += count;

        if (count == 0)
            break;
    }
    bool ok = buffer && !ferror(file);

    fclose(file);
    if (!ok) {
        free(buffer);
        return false;
    }
    *data = buffer;
    *size = used;
    return true;
}

static bool load_bios_from(const char *directory, size_t length, uint8_t *bios)
{
    static const char *const names[] = {"disksys.rom", "DISKSYS.ROM", "Disksys.rom"};

    for (unsigned i = 0; i < sizeof names / sizeof *names; ++i) {
        char path[1200];
        int written = length ? snprintf(path, sizeof path, "%.*s%s", (int)length, directory, names[i])
                             : snprintf(path, sizeof path, "%s", names[i]);

        if (written < 0 || (size_t)written >= sizeof path)
            continue;
        uint8_t *data;
        size_t size;

        if (!read_file(path, &data, &size, 1 << 20))
            continue;
        // Read the last 8 KiB to skip any dump header
        bool ok = size >= FDS_BIOS_SIZE;

        if (ok)
            memcpy(bios, data + size - FDS_BIOS_SIZE, FDS_BIOS_SIZE);
        free(data);
        if (ok)
            return true;
    }
    return false;
}

static size_t directory_length(const char *path)
{
    size_t length = strlen(path);

    while (length && path[length - 1] != '/' && path[length - 1] != '\\')
        --length;
    return length;
}

static bool load_bios(const char *disk_path, uint8_t *bios)
{
    if (load_bios_from(disk_path, directory_length(disk_path), bios))
        return true;
    for (unsigned i = 0; i < BIOS_DIRECTORIES && bios_directories[i][0]; ++i) {
        const char *base = bios_directories[i];
        size_t length = strlen(base);
        char directory[sizeof bios_directories[i] + 1];

        snprintf(directory, sizeof directory, "%s%s", base,
                 base[length - 1] == '/' || base[length - 1] == '\\' ? "" : "/");
        if (load_bios_from(directory, strlen(directory), bios))
            return true;
    }
    return false;
}

static bool load_disk_save(Cartridge *cart, char *error, size_t error_size)
{
    Fds *drive = cart->ext->fds;
    uint8_t *data;
    size_t length;

    if (!read_file(cart->save_path, &data, &length, (size_t)FDS_MAX_SIDES * FDS_SIDE_IMAGE + FDS_SAVE_HEADER)) {
        if (errno == ENOENT)
            return true;
        return cart_fail(error, error_size, "Cannot read %s (%s)", cart->save_path, strerror(errno));
    }
    size_t disk_bytes = (size_t)drive->sides * drive->side_size;
    bool valid = length == FDS_SAVE_HEADER + disk_bytes &&
                 !memcmp(data, save_magic, sizeof save_magic) &&
                 data[10] == 1 && data[11] == drive->sides;

    if (valid) {
        uint32_t side_size = data[12] | data[13] << 8 | data[14] << 16 | (uint32_t)data[15] << 24;

        valid = side_size == drive->side_size;

        if (valid)
            memcpy(drive->disk, data + FDS_SAVE_HEADER, disk_bytes);
    }
    free(data);
    return valid || cart_fail(error, error_size, "%s does not match this disk image", cart->save_path);
}

size_t fds_save_image(const Cartridge *c, uint8_t header[16], const uint8_t **data)
{
    const Fds *f = c->ext->fds;

    memcpy(header, save_magic, sizeof save_magic);
    header[10] = 1;
    header[11] = f->sides;

    for (unsigned i = 0; i < 4; ++i)
        header[12 + i] = (uint8_t)(f->side_size >> (8 * i));
    *data = f->disk;
    return (size_t)f->sides * f->side_size;
}

static void reset_audio(FdsAudio *a)
{
    *a = (FdsAudio){.master_speed = 0xe8, .enabled = true, .vol_control = 0x80, .mod_control = 0x80};
    fds_audio_reset_timers(a);
}

bool fds_load(Cartridge *c, const char *path, char *error, size_t size)
{
    uint8_t *file;
    size_t length, offset, sides;

    if (!read_file(path, &file, &length, (size_t)FDS_MAX_SIDES * FDS_SIDE_BYTES + 16))
        return cart_fail(error, size, "Cannot open %s (%s)", path, strerror(errno));
    offset = length >= 16 && !memcmp(file, "FDS\x1a", 4) ? 16 : 0;
    sides = length > offset ? (length - offset) / FDS_SIDE_BYTES : 0;

    // Missing sides read as empty
    if (offset && file[4])
        sides = file[4];
    if (!sides || sides > FDS_MAX_SIDES || length < offset + 16 ||
        memcmp(file + offset, "\x01*NINTENDO-HVC*", 15)) {
        free(file);
        return cart_fail(error, size, "Expected a Famicom Disk System image");
    }

    c->mapper = 20;
    c->mirroring = MIRROR_VERTICAL;
    c->prg_size = FDS_BIOS_SIZE;
    c->chr_size = 8192;
    c->chr_is_ram = true;
    c->ram_size = FDS_RAM_SIZE;

    c->prg = malloc(FDS_BIOS_SIZE);
    c->chr = calloc(c->chr_size, 1);
    c->ram = calloc(FDS_RAM_SIZE, 1);
    c->ext = calloc(1, sizeof *c->ext);
    Fds *f = calloc(1, sizeof *f);
    uint8_t *raw = calloc(FDS_SIDE_BYTES, 1);
    uint8_t (*ids)[10] = calloc(sides, sizeof *ids);
    uint8_t *disk = calloc(sides, FDS_SIDE_IMAGE);

    c->save_path = cart_path_with_suffix(path, ".sav");

    if (!c->prg || !c->chr || !c->ram || !c->ext || !f || !raw || !ids || !disk || !c->save_path) {
        free(f);
        free(file);
        free(raw);
        free(ids);
        free(disk);
        return cart_fail(error, size, "Out of memory");
    }
    c->ext->fds = f;
    f->disk = disk;
    f->ids = (const uint8_t (*)[10])ids;
    f->sides = (uint8_t)sides;
    f->side_size = FDS_SIDE_IMAGE;

    for (size_t s = 0; s < sides; ++s) {
        size_t start = offset + s * FDS_SIDE_BYTES;
        size_t available = start < length ? length - start : 0;

        memset(raw, 0, FDS_SIDE_BYTES);
        memcpy(raw, file + start, available < FDS_SIDE_BYTES ? available : FDS_SIDE_BYTES);
        // CheckDiskHeader compares block 1 bytes $0F-$18
        memcpy(ids[s], raw + 15, 10);
        build_side(raw, disk + s * FDS_SIDE_IMAGE);
    }
    free(raw);
    free(file);

    if (!load_bios(path, c->prg))
        return cart_fail(error, size, "%s", cart_fds_bios_missing);
    if (!load_disk_save(c, error, size))
        return false;

    f->side = 0;
    f->insert_side = FDS_NO_DISK;
    f->insert_cycle = UINT64_MAX;
    f->disk_event = UINT64_MAX;
    f->irq_cycle = UINT64_MAX;
    f->head_home = true;
    f->io_enable = 0x83;
    f->control = 0x26;
    reset_audio(&f->audio);
    c->ext->irq_cycle = UINT64_MAX;
    c->ext->has_audio = true;

    for (unsigned i = 0; i < 3; ++i)
        c->prg_pages[i] = c->ram + 8192 * (i + 1);
    c->prg_pages[3] = c->prg;
    return true;
}

void fds_free(Cartridge *c)
{
    Fds *f = c->ext ? c->ext->fds : NULL;

    if (!f)
        return;
    free(f->disk);
    free((void *)f->ids);
    free(f);
    c->ext->fds = NULL;
}

static void notify(Fds *f, unsigned side, bool missing)
{
    ++f->notices;
    f->notice_side = (uint8_t)side;
    f->notice_missing = missing;
    f->announce_pending = false;
}

// From timer_base, each cycle trips at zero or decrements the timer
static uint64_t timer_trip(const Fds *f)
{
    return f->timer_enabled ? f->timer_base + f->timer_counter + 1 : UINT64_MAX;
}

static void advance_timer(Fds *f, uint64_t cycle)
{
    if (!f->timer_enabled || cycle <= f->timer_base) {
        if (cycle > f->timer_base)
            f->timer_base = cycle;
        return;
    }
    uint64_t trip = timer_trip(f);

    if (cycle < trip) {
        f->timer_counter = (uint16_t)(f->timer_counter - (cycle - f->timer_base));
    } else {
        f->timer_irq = true;

        if (!f->timer_repeat) {
            f->timer_enabled = false;
            f->timer_counter = f->timer_reload;
        } else {
            uint64_t period = f->timer_reload + UINT64_C(1);
            f->timer_counter = (uint16_t)(f->timer_reload - (cycle - trip) % period);
        }
    }
    f->timer_base = cycle;
}

static int peek(const Nes *nes, unsigned address)
{
    address &= 0xffff;

    if (address < 0x2000)
        return nes->ram[address & 2047];
    if (address >= 0xe000)
        return nes->cart.prg[address & 8191];
    if (address >= 0x6000)
        return nes->cart.ram[address - 0x6000];
    return -1;
}

// CheckDiskHeader ($E445) starts the motor with ($00) pointing to the requested
// disk ID, where $FF matches anything. Insert the only side that matches.
static void select_requested_side(Nes *nes, Fds *f)
{
    unsigned pointer = nes->ram[0] | nes->ram[1] << 8;
    uint8_t id[10];

    for (unsigned i = 0; i < 10; ++i) {
        int value = peek(nes, pointer + i);

        if (value < 0)
            return;
        id[i] = (uint8_t)value;
    }
    unsigned matches = 0, match = 0;
    bool same_game = false;

    for (unsigned s = 0; s < f->sides; ++s) {
        bool equal = true, game = true;

        for (unsigned i = 0; i < 10; ++i) {
            bool byte = id[i] == 0xff || id[i] == f->ids[s][i];

            equal &= byte;
            game &= byte || i == 6 || i == 7;
        }
        same_game |= game;

        if (!equal)
            continue;
        if (s == f->side) {
            // An automatic reinsertion is announced once the BIOS accepts it
            if (f->announce_pending)
                notify(f, s, false);
            return;
        }
        ++matches;
        match = s;
    }
    if (matches == 1) {
        f->side = (uint8_t)match;
        notify(f, match, false);
    } else if (!matches && same_game) {
        // The game requests another side using byte 6 for the side and byte 7 for the disk
        if (!f->request_missing)
            notify(f, id[6] < 2 && id[7] < 0x80 ? id[7] * 2u + id[6] : FDS_NO_DISK, true);
        f->request_missing = true;
    }
}

static void update_drive(Nes *nes, Fds *f, uint64_t cycle)
{
    bool motor = f->control & 1, reset = f->control & 2;

    if (f->side == FDS_NO_DISK || !motor) {
        f->head_home = true;
        f->scanning = false;
        f->disk_event = UINT64_MAX;
        f->spin_left = 0;
        return;
    }
    if (f->scanning)
        return;
    if (reset) {
        if (f->disk_event != UINT64_MAX) {
            f->spin_left = (uint32_t)(f->disk_event - cycle);
            f->disk_event = UINT64_MAX;
        }
        return;
    }
    if (f->head_home) {
        select_requested_side(nes, f);
        f->eject_armed = !f->request_missing;
        f->head_home = false;
        f->at_end = false;
        f->position = 0;
        f->gap_ended = false;
        f->byte_thirds = 0;
        f->disk_event = cycle + FDS_SPIN_UP;
    } else if (f->disk_event == UINT64_MAX) {
        f->disk_event = cycle + (f->spin_left ? f->spin_left : 1);
        f->spin_left = 0;
    }
}

static void transfer_byte(Nes *nes, Fds *f)
{
    uint8_t *data = f->disk + (size_t)f->side * f->side_size;
    bool irq = (f->control & 0x80) != 0;
    bool crc_control = (f->control & 0x10) != 0, ready = (f->control & 0x40) != 0;

    f->scanning = true;

    if (f->control & 4) {
        uint8_t value = data[f->position];

        if (!f->previous_crc_control)
            f->crc = crc_add(f->crc, value);
        if (!ready) {
            f->gap_ended = false;
            f->crc = 0;
            f->bad_crc = false;
        } else if (value && !f->gap_ended) {
            // The gap end mark raises no IRQ
            f->gap_ended = true;
            irq = false;
        }
        if (f->gap_ended) {
            f->transfer_flag = true;
            f->read_data = value;
            f->disk_irq |= irq;
        }
        if (!f->previous_crc_control && crc_control)
            f->bad_crc = f->crc != 0;
    } else {
        uint8_t value = 0;

        if (!crc_control) {
            f->transfer_flag = true;
            value = f->write_data;
            f->disk_irq |= irq;
        }
        if (!ready) {
            value = 0;
            f->crc = 0;
        }
        if (!crc_control) {
            f->crc = crc_add(f->crc, value);
        } else {
            value = (uint8_t)f->crc;
            f->crc >>= 8;
        }
        if (data[f->position] != value) {
            data[f->position] = value;
            ++f->disk_writes;
            nes->cart.save_dirty = true;
        }
        f->gap_ended = false;
        f->bad_crc = false;
    }
    f->previous_crc_control = crc_control;

    if (++f->position >= f->side_size) {
        // The drive stops at the end until the program restarts it
        f->at_end = true;
        f->head_home = true;
        f->scanning = false;
        f->disk_event = UINT64_MAX;
        f->disk_irq |= (f->control & 0x80) != 0;
        return;
    }
    unsigned thirds = f->byte_thirds + FDS_BYTE_THIRDS;

    f->disk_event += thirds / 3;
    f->byte_thirds = (uint8_t)(thirds % 3);
}

static void sync_adapter(Nes *nes, uint64_t cycle)
{
    Fds *f = nes->cart.ext->fds;

    if (f->insert_cycle <= cycle) {
        uint64_t at = f->insert_cycle;

        f->side = f->insert_side;

        if (f->insert_auto)
            f->announce_pending = true;
        else
            notify(f, f->side, false);
        f->insert_cycle = UINT64_MAX;
        f->insert_side = FDS_NO_DISK;
        update_drive(nes, f, at);
    }
    advance_timer(f, cycle);
    while (f->disk_event <= cycle)
        transfer_byte(nes, f);
}

static void schedule_irq(Nes *nes)
{
    Cartridge *c = &nes->cart;
    Fds *f = c->ext->fds;

    c->mmc3.irq = f->timer_irq || f->disk_irq;
    uint64_t next = timer_trip(f);

    if ((f->control & 0x80) && f->disk_event < next)
        next = f->disk_event;
    if (f->insert_cycle < next)
        next = f->insert_cycle;
    f->irq_cycle = next;
    c->mmc3.irq_enabled = next != UINT64_MAX && !c->mmc3.irq;
}

uint64_t fds_irq_cycle(Nes *nes)
{
    Fds *f = nes->cart.ext->fds;

    if (nes->cpu.cycles < f->irq_cycle)
        return f->irq_cycle;
    sync_adapter(nes, nes->cpu.cycles);
    schedule_irq(nes);
    nes->irq_line = cpu_irq_line(nes);
    return nes->cart.mmc3.irq_enabled ? f->irq_cycle : UINT64_MAX;
}

static inline bool store_ram(Nes *nes, uint16_t address, uint8_t value, bool from_trace)
{
    uint8_t *byte = nes->cart.ram + (address - 0x6000);

    if (*byte == value)
        return true;
    bool changes_code = fds_code_byte(nes->cart.ext->fds->code_map, address);

    // Exit to C before a write can invalidate the running trace
    if (from_trace && changes_code)
        return false;
    *byte = value;

    if (changes_code)
        cpu_forget_fds_traces(&nes->cart, address);
    if (fds_loop_region(nes->cart.ext->fds, address))
        cpu_forget_fds_loops(nes, address);
    return true;
}

bool fds_write_quiet_data(Nes *nes, uint16_t address, uint8_t value, uint64_t cycle)
{
    return cycle + 1 < nes->bus_deadline && store_ram(nes, address, value, true);
}

// C code can drop the traces of a changed code byte itself
bool fds_write_quiet(Nes *nes, uint16_t address, uint8_t value)
{
    if (fds_register(address))
        return false;
    if (address < 0xe000)
        store_ram(nes, address, value, false);
    return true;
}

static void set_control(Nes *nes, Fds *f, uint8_t value, uint64_t cycle)
{
    f->control = value;
    nes->cart.mirroring = (value & 8) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
    f->disk_irq = false;
    update_drive(nes, f, cycle);
}

void fds_write(Nes *nes, uint16_t address, uint8_t value)
{
    Fds *f = nes->cart.ext->fds;
    uint64_t cycle = nes->cpu.cycles;

    if (address >= 0x6000) {
        if (address < 0xe000)
            store_ram(nes, address, value, false);
        return;
    }
    if (address >= 0x4040) {
        if (address <= 0x408a)
            fds_audio_write(nes, address, value);
        return;
    }
    if (address > 0x4026)
        return;
    sync_adapter(nes, cycle);
    bool disk_io = (f->io_enable & 1) != 0;

    switch (address) {
    case 0x4020:
        f->timer_reload = (uint16_t)((f->timer_reload & 0xff00) | value);
        break;
    case 0x4021:
        f->timer_reload = (uint16_t)((f->timer_reload & 0xff) | value << 8);
        break;
    case 0x4022:
        if (!disk_io)
            break;
        f->timer_repeat = value & 1;
        f->timer_enabled = (value & 2) != 0;
        f->timer_base = cycle;

        if (f->timer_enabled)
            f->timer_counter = f->timer_reload;
        else
            f->timer_irq = false;
        break;
    case 0x4023:
        if ((value ^ f->io_enable) & 2) {
            nes_expansion_audio_sync(nes, cycle);
            fds_audio_enable(&f->audio, (value & 2) != 0);
        }
        f->io_enable = value;

        if (!(value & 1)) {
            f->timer_enabled = false;
            f->timer_irq = false;
            f->transfer_flag = false;
            f->ext_out = 0x7f;
            set_control(nes, f, 0x06, cycle);
        }
        break;
    case 0x4024:
        if (!disk_io)
            break;
        f->write_data = value;
        f->transfer_flag = false;
        f->disk_irq = false;
        break;
    case 0x4025:
        if (disk_io)
            set_control(nes, f, value, cycle);
        break;
    case 0x4026:
        if (disk_io)
            f->ext_out = value;
        break;
    }
    schedule_irq(nes);
}

// A program asking for another side polls $4032 with the drive idle. After a
// second of that, eject and reinsert the disk. CheckDiskHeader then picks the side.
static void watch_disk_polling(Nes *nes, Fds *f)
{
    uint64_t frame = nes->ppu.frames;

    if (frame == f->poll_frame)
        return;
    f->poll_frames = frame == f->poll_frame + 1 ? f->poll_frames + 1 : 1;
    f->poll_frame = frame;

    if (f->poll_frames < FDS_EJECT_POLL_FRAMES)
        return;
    f->poll_frames = 0;
    f->eject_armed = false;
    f->insert_auto = true;
    f->insert_side = f->side;
    f->insert_cycle = nes->cpu.cycles + FDS_CHANGE_DELAY;
    f->side = FDS_NO_DISK;
    update_drive(nes, f, nes->cpu.cycles);
}

uint8_t fds_read(Nes *nes, uint16_t address, uint8_t bus)
{
    Cartridge *c = &nes->cart;
    Fds *f = c->ext->fds;

    if (address >= 0x6000)
        return address < 0x8000 ? c->ram[address - 0x6000] : cart_prg_read(c, address);
    if (address >= 0x4040)
        return address < 0x4098 ? fds_audio_read(nes, address, bus) : bus;
    if (address < 0x4030 || address > 0x4033)
        return bus;
    sync_adapter(nes, nes->cpu.cycles);
    uint8_t value;

    switch (address) {
    case 0x4030:
        value = (uint8_t)((bus & 0x24) | f->timer_irq | (f->control & 8) |
                          (f->bad_crc ? 0x10 : 0) | (f->at_end ? 0x40 : 0) | (f->transfer_flag ? 0x80 : 0));
        f->timer_irq = false;
        break;
    case 0x4031:
        value = f->read_data;
        f->transfer_flag = false;
        f->disk_irq = false;
        break;
    case 0x4032: {
        bool empty = f->side == FDS_NO_DISK;

        value = (uint8_t)((bus & 0xf8) | empty | ((empty || !f->scanning) ? 2 : 0) | (empty ? 4 : 0));

        if (f->eject_armed && !empty && !f->scanning && f->disk_event == UINT64_MAX)
            watch_disk_polling(nes, f);
        break;
    }
    default:
        // Battery good
        value = (uint8_t)(0x80 | (f->ext_out & 0x7f));
        break;
    }
    schedule_irq(nes);
    return value;
}

void fds_change_disk(Nes *nes, unsigned side, bool delay)
{
    Fds *f = cart_fds(&nes->cart);

    if (!f)
        return;
    uint64_t cycle = nes->cpu.cycles;

    sync_adapter(nes, cycle);
    f->eject_armed = f->request_missing = f->insert_auto = f->announce_pending = false;
    f->side = FDS_NO_DISK;
    f->insert_side = FDS_NO_DISK;
    f->insert_cycle = UINT64_MAX;
    update_drive(nes, f, cycle);
    if (side < f->sides) {
        f->insert_side = (uint8_t)side;
        f->insert_cycle = cycle + (delay ? FDS_CHANGE_DELAY : 1);
    }
    schedule_irq(nes);
    nes->bus_deadline = cycle;
}

unsigned fds_current_side(const Nes *nes)
{
    const Fds *f = cart_fds(&nes->cart);

    if (!f)
        return FDS_NO_DISK;
    return f->side != FDS_NO_DISK ? f->side : f->insert_side;
}

uint32_t fds_disk_notices(const Nes *nes, unsigned *side, bool *missing)
{
    const Fds *f = cart_fds(&nes->cart);

    if (!f)
        return 0;
    *side = f->notice_side;
    *missing = f->notice_missing;
    return f->notices;
}

unsigned fds_side_count(const Nes *nes)
{
    const Fds *f = cart_fds(&nes->cart);
    return f ? f->sides : 0;
}
