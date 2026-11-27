cmake_minimum_required(VERSION 3.16)
project(SchNES LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(APPLE)
    enable_language(OBJC)
elseif(WIN32)
    enable_language(RC)
endif()

option(SCHNES_STATIC_SDL "Link the SDL2 static library" OFF)
set(SCHNES_SDL2_ARCHIVE "" CACHE FILEPATH "SDL2 2.32.10 source archive for a lean static build")
if(SCHNES_SDL2_ARCHIVE)
    include(cmake/LeanSDL2.cmake)
    schnes_add_lean_sdl2("${SCHNES_SDL2_ARCHIVE}")
else()
    find_package(SDL2 CONFIG REQUIRED)
endif()

if(SCHNES_STATIC_SDL OR SCHNES_SDL2_ARCHIVE)
    if(NOT TARGET SDL2::SDL2-static)
        message(FATAL_ERROR
            "SCHNES_STATIC_SDL requires an SDL2 installation exporting SDL2::SDL2-static. "
            "Set SDL2_DIR to its CMake package directory.")
    endif()
    set(sdl_target SDL2::SDL2-static)
else()
    set(sdl_target SDL2::SDL2)
endif()

# Compile raster tracing separately to preserve the normal CPU code layout
add_library(schnes_cpu_debug OBJECT src/cpu.c)
target_compile_definitions(schnes_cpu_debug PRIVATE SCHNES_CPU_DEBUG)

add_library(schnes_core STATIC
    src/apu.c
    src/cartridge.c
    src/cpu.c
    src/fds.c
    src/fds_audio.c
    src/machine.c
    src/ppu.c
    src/runahead.c
    src/sunsoft5b.c
    src/vrc6.c
    $<TARGET_OBJECTS:schnes_cpu_debug>
)
set_target_properties(schnes_core PROPERTIES OUTPUT_NAME schnes)
target_include_directories(schnes_core PUBLIC src)

option(SCHNES_PGO_GENERATE "Build a profile-guided optimization trainer" OFF)
set(SCHNES_PGO_PROFILE "" CACHE FILEPATH "Merged LLVM profile for the release core")
if(SCHNES_PGO_GENERATE OR SCHNES_PGO_PROFILE)
    if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang" OR (SCHNES_PGO_GENERATE AND SCHNES_PGO_PROFILE))
        message(FATAL_ERROR "Use Clang and select either profile generation or a merged profile.")
    endif()
    if(MINGW)
        target_compile_options(schnes_core PRIVATE -mllvm -disable-vp)
    endif()
    if(SCHNES_PGO_GENERATE)
        target_compile_options(schnes_core PRIVATE -fprofile-generate)
        target_link_options(schnes_core INTERFACE -fprofile-generate)
        add_executable(schnes_profile EXCLUDE_FROM_ALL scripts/profile.c)
        target_link_libraries(schnes_profile PRIVATE schnes_core)
        if(MINGW)
            target_link_options(schnes_profile PRIVATE -static)
        endif()
    else()
        target_compile_options(schnes_core PRIVATE "-fprofile-use=${SCHNES_PGO_PROFILE}"
            -Werror=profile-instr-out-of-date -Werror=profile-instr-unprofiled)
    endif()
endif()

add_executable(SchNES src/main.c src/png_write.c)
if(TARGET SDL2::SDL2main)
    target_link_libraries(SchNES PRIVATE SDL2::SDL2main)
endif()
target_link_libraries(SchNES PRIVATE schnes_core ${sdl_target})
get_target_property(sdl_type ${sdl_target} TYPE)
set(sdl_static FALSE)
if(SCHNES_STATIC_SDL OR sdl_type STREQUAL "STATIC_LIBRARY")
    set(sdl_static TRUE)
endif()
if(APPLE)
    find_library(APPKIT_FRAMEWORK AppKit)
    target_sources(SchNES PRIVATE src/macos_icon.m assets/SchNES.icns)
    target_link_libraries(SchNES PRIVATE "${APPKIT_FRAMEWORK}")
    set_source_files_properties(src/macos_icon.m PROPERTIES COMPILE_OPTIONS -fobjc-arc)
    set_source_files_properties(assets/SchNES.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # Re-sign the bundle when an icon-only rebuild changes its sealed resource
    set_property(TARGET SchNES APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/assets/SchNES.icns")
    set_target_properties(SchNES PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME SchNES
        MACOSX_BUNDLE_BUNDLE_VERSION 1
        MACOSX_BUNDLE_SHORT_VERSION_STRING 1.0
        MACOSX_BUNDLE_ICON_FILE SchNES.icns
        MACOSX_BUNDLE_GUI_IDENTIFIER org.schnes.SchNES)
    target_link_options(SchNES PRIVATE "$<$<CONFIG:Release>:LINKER:-x,-no_exported_symbols>")
    if(sdl_static)
        # SDL's package lists dependencies for every backend in the archive.
        # Let the linker keep the code and frameworks used by this executable.
        target_link_options(SchNES PRIVATE "LINKER:-dead_strip,-dead_strip_dylibs")
        set_target_properties(SchNES PROPERTIES SKIP_BUILD_RPATH TRUE)
    endif()
    add_custom_command(TARGET SchNES POST_BUILD
        COMMAND /usr/bin/codesign --force --sign - "$<TARGET_BUNDLE_DIR:SchNES>"
        COMMAND "${CMAKE_COMMAND}" -E create_symlink
                "$<TARGET_BUNDLE_DIR_NAME:SchNES>/Contents/MacOS/$<TARGET_FILE_NAME:SchNES>"
                "$<TARGET_BUNDLE_DIR:SchNES>/../SchNES"
        VERBATIM)
elseif(WIN32)
    set_target_properties(SchNES PROPERTIES WIN32_EXECUTABLE TRUE)
    target_sources(SchNES PRIVATE assets/SchNES.rc)
    set_property(SOURCE assets/SchNES.rc APPEND PROPERTY OBJECT_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/SchNES.ico")
    if(MINGW AND sdl_static)
        target_link_options(SchNES PRIVATE -static)
    endif()
    if(MINGW AND SCHNES_SDL2_ARCHIVE)
        target_link_options(SchNES PRIVATE "$<$<CONFIG:Release>:LINKER:--gc-sections,--strip-all>")
    endif()
endif()

if(MSVC)
    target_compile_options(schnes_cpu_debug PRIVATE /W4 /wd4505)
    target_compile_options(schnes_core PRIVATE /W4)
    target_compile_options(SchNES PRIVATE /W4)
else()
    target_compile_options(schnes_cpu_debug PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-function)
    target_compile_options(schnes_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(SchNES PRIVATE -Wall -Wextra -Wpedantic)
endif()
