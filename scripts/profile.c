#include "nes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void __llvm_profile_reset_counters(void);

#ifdef _WIN32
uint64_t __llvm_profile_get_size_for_buffer(void);
int __llvm_profile_write_buffer(char *buffer);

// The Windows runner supports writing the profile through stdio
int __llvm_profile_runtime;
#endif

static void poll_input(void *context, uint8_t buttons[2])
{
    memcpy(buttons, context, 2);
}

int main(int argc, char **argv)
{
    if (argc != 8)
        return 2;

    unsigned first = strtoul(argv[3], NULL, 10), end = strtoul(argv[4], NULL, 10);
    unsigned depth = strtoul(argv[6], NULL, 10);
    char mode = argv[5][0];

    if (first >= end || depth > 2 || (mode != 'r' && mode != 'u' && mode != 'i') || argv[5][1])
        return 2;

    FILE *input = fopen(argv[2], "rb");
    uint8_t *tape = malloc(end);

    if (!input || !tape || fread(tape, 1, end, input) != end || fclose(input))
        return 2;

    static Nes nes;
    char error[256];

    if (!cart_load(&nes.cart, argv[1], error, sizeof error)) {
        fprintf(stderr, "%s\n", error);
        return 2;
    }
    nes.ultrafast = mode == 'u';
    nes.interpreter = mode == 'i';
    nes_init(&nes);
    NesRunahead *runahead = depth ? nes_runahead_create(&nes.cart) : NULL;

    if (depth && !runahead)
        return 2;

    uint8_t buttons[2] = {0};

    if (depth) {
        nes.poll_buttons = poll_input;
        nes.input_context = buttons;
    }
    for (unsigned frame = 0; frame < end; ++frame) {
        if (frame == first)
            __llvm_profile_reset_counters();
        nes.buttons[0] = buttons[0] = tape[frame];
        nes_frame(&nes);
        if (depth && frame + 60 >= first)
            nes_runahead_frame(runahead, &nes, depth, NULL);
        if (nes.cpu.jammed || nes.ppu.frames != frame + 1)
            return 3;
    }
    nes_runahead_destroy(runahead);
    cart_free(&nes.cart);
    free(tape);

#ifdef _WIN32
    size_t size = (size_t)__llvm_profile_get_size_for_buffer();
    char *profile = malloc(size);

    if (!size || !profile || __llvm_profile_write_buffer(profile))
        return 3;
    FILE *output = fopen(argv[7], "wb");

    if (!output)
        return 3;
    bool written = fwrite(profile, 1, size, output) == size;

    free(profile);
    return fclose(output) || !written;
#else
    return 0; // The native LLVM runtime writes to LLVM_PROFILE_FILE at exit
#endif
}
