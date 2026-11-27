#ifndef WRITE_EFFECTS_H
#define WRITE_EFFECTS_H

#include "nes.h"

// What a CPU write can change, as far as a CPU batch is concerned.
//
// A batch runs up to Nes.bus_deadline without servicing devices. A write may
// stay inside it only if it leaves the raster, that deadline, the IRQ line and
// the code being run as they were, or if the quiet path can account for the
// change itself. Each device classifies its own writes once: mappers in
// mapper_writes.h, the PPU in ppu_writes.h and the APU in apu_write_effects.
// The quiet path applies a write or turns it down from these flags, the bus
// handler checks them in debug builds, and the recompiler picks its callouts
// from the same register knowledge. The bus handler applies every write
// exactly.
//
// A classification describes the device as it is before the write, at the
// cycle the write lands on. Classifiers may report an effect that a write
// turns out not to have, but never miss one.
enum {
    WRITE_UNCHANGED = 0,
    // A raster input changes: a CHR bank, the nametable mapping, or an input
    // to the next reload of a scanline counter. No raster event may be pending
    // before the write.
    WRITE_RASTER = 1 << 0,
    // The next CPU-visible event can move earlier, or the IRQ line can change
    WRITE_DEADLINE = 1 << 1,
    // A CPU-cycle IRQ counter changes. Apply the write, then keep it only if
    // the IRQ line and the bus deadline stay as they were.
    WRITE_IRQ = 1 << 2,
    // PRG remaps can affect streamed DMC reads and running code
    WRITE_PRG = 1 << 3,
    // Only the sound changes. Render it up to the write first. Ultrafast
    // may leave some APU renders out, which moves only audio.
    WRITE_AUDIO = 1 << 4,
    // A byte that native code was compiled from changes
    WRITE_CODE = 1 << 5,
    // Regular and interpreter bring the raster up to writes to this register,
    // so a write after a pending raster event takes the bus handler.
    WRITE_TIMED = 1 << 6,
    // Only the bus handler applies this write
    WRITE_BUS = 1 << 7,
};

// The APU's waveform registers, $4000-$400F, only change the sound. The DMC
// and frame counter registers above them can also move CPU events.
static inline bool apu_waveform_register(uint16_t address)
{
    return (address & 0xfff0) == 0x4000;
}

#endif
