#include "png_write.h"

#include <stdlib.h>

static void put_be32(uint8_t out[4], uint32_t value)
{
    out[0] = value >> 24;
    out[1] = value >> 16;
    out[2] = value >> 8;
    out[3] = value;
}

static inline uint32_t crc32_add_byte(uint32_t crc, uint8_t byte)
{
    crc ^= byte;

    for (unsigned bit = 0; bit < 8; ++bit)
        crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xedb88320) : 0);
    return crc;
}

uint32_t crc32_update_argb8888(uint32_t crc, const uint32_t *pixels, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        for (unsigned component = 0; component < 24; component += 8)
            crc = crc32_add_byte(crc, (uint8_t)(pixels[i] >> component));
    }
    return crc;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i)
        crc = crc32_add_byte(crc, data[i]);
    return crc;
}

static bool write_chunk(FILE *file, const char type[4], const uint8_t *data, size_t length)
{
    uint8_t size[4], checksum[4];
    uint32_t crc = crc32_update(UINT32_MAX, (const uint8_t *)type, 4);

    put_be32(size, (uint32_t)length);
    put_be32(checksum, ~crc32_update(crc, data, length));
    return fwrite(size, 1, 4, file) == 4 && fwrite(type, 1, 4, file) == 4 &&
           (!length || fwrite(data, 1, length, file) == length) &&
           fwrite(checksum, 1, 4, file) == 4;
}

bool png_write_argb8888(FILE *file, const uint32_t *pixels,
                       unsigned width, unsigned height, size_t stride)
{
    enum { ZLIB_HEADER = 2, BLOCK_HEADER = 5, ADLER_SIZE = 4, ADLER_MOD = 65521 };
    static const uint8_t signature[] = {137, 'P', 'N', 'G', 13, 10, 26, 10};
    uint8_t header[13] = {[8] = 8, [9] = 2}; // 8-bit RGB
    uint32_t adler_a = 1, adler_b = 0;

    if (!file || !pixels || !width || width > (UINT16_MAX - 1) / 3 ||
        !height || height > INT32_MAX || stride < width || height > SIZE_MAX / sizeof *pixels / stride)
        return false;

    unsigned row_size = 1 + 3 * width;
    uint8_t *block = malloc(ZLIB_HEADER + BLOCK_HEADER + row_size + ADLER_SIZE);

    if (!block)
        return false;
    uint8_t *deflate = block + ZLIB_HEADER;
    uint8_t *row = deflate + BLOCK_HEADER;

    block[0] = 0x78;
    block[1] = 0x01; // zlib header for stored DEFLATE blocks
    deflate[1] = row_size;
    deflate[2] = row_size >> 8;
    deflate[3] = ~row_size;
    deflate[4] = ~row_size >> 8;
    row[0] = 0; // PNG filter type None

    put_be32(header, width);
    put_be32(header + 4, height);
    bool ok = fwrite(signature, 1, sizeof signature, file) == sizeof signature &&
              write_chunk(file, "IHDR", header, sizeof header);

    for (unsigned y = 0; ok && y < height; ++y) {
        const uint32_t *source = pixels + y * stride;
        uint8_t *rgb = row + 1;
        bool last = y + 1 == height;
        size_t prefix = y == 0 ? ZLIB_HEADER : 0;

        for (unsigned x = 0; x < width; ++x) {
            uint32_t pixel = source[x];

            *rgb++ = pixel >> 16;
            *rgb++ = pixel >> 8;
            *rgb++ = pixel;
        }
        for (unsigned i = 0; i < row_size; ++i) {
            adler_a = (adler_a + row[i]) % ADLER_MOD;
            adler_b = (adler_b + adler_a) % ADLER_MOD;
        }

        deflate[0] = last;

        if (last)
            put_be32(row + row_size, (adler_b << 16) | adler_a);
        ok = write_chunk(file, "IDAT", deflate - prefix,
                         prefix + BLOCK_HEADER + row_size + (last ? ADLER_SIZE : 0));
    }
    free(block);
    return ok && write_chunk(file, "IEND", NULL, 0);
}
