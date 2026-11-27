# Scripts

| Path | Purpose |
| --- | --- |
| `publish.py` | Packages the Apple silicon and Windows ARM64 release and can build a separate Windows x64 release. |
| `profile.c` | Training driver for profile-guided optimization. CMake builds it as `schnes_profile` when `publish.py` asks for it. |

## Release builds

The default publish command stages the Apple silicon and Windows ARM64 builds in
`bin/`, including `SchNES.zip`. Pass a path as the first argument to include
a showcase video in the archive. Publishing targets macOS 11 when an installed
SDK supports it and otherwise uses the oldest target supported by the installed SDKs.

Both release paths use Python, CMake 3.24 or later, Ninja, `patch`, LLVM-MinGW,
the SDL2 archive at `build/SDL2-2.32.10.tar.gz`, and the replay corpus. PGO uses the
ROMs and tape windows from `build/replay-inputs/active-gameplay/workloads.json`.

Run `python3 scripts/publish.py --windows-x64` to build only Windows x64.
Training executes natively on Windows. On another host, add Wine to `PATH` or
pass `--wine /path/to/wine64`. Use a Wine prefix that maps the host filesystem
to drive `Z:`. The x64 build uses its own gameplay profile and outputs
`build/publish/windows-x64/SchNES.exe`.
