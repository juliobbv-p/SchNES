#ifndef SCHNES_PNG_WRITE_H
#define SCHNES_PNG_WRITE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Write opaque RGB from packed 0xAARRGGBB pixels
bool png_write_argb8888(FILE *file, const uint32_t *pixels,
                       unsigned width, unsigned height, size_t stride);

#endif
