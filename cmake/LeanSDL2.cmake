include(FetchContent)

function(schnes_add_lean_sdl2 archive)
    if(NOT CMAKE_SYSTEM_NAME MATCHES "^(Darwin|Windows)$")
        message(FATAL_ERROR "SCHNES_SDL2_ARCHIVE is supported on macOS and desktop Windows.")
    endif()
    if(CMAKE_VERSION VERSION_LESS 3.24)
        message(FATAL_ERROR "Building SDL2 from the source archive requires CMake 3.24 or later.")
    endif()
    get_filename_component(archive "${archive}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
    if(NOT EXISTS "${archive}")
        message(FATAL_ERROR "SDL2 source archive not found: ${archive}")
    endif()
    find_program(SCHNES_PATCH_PROGRAM patch)
    if(NOT SCHNES_PATCH_PROGRAM)
        message(FATAL_ERROR "The patch command is required to prepare the SDL2 archive.")
    endif()

    # Keep the profile local to this build. SDL's options honor normal variables
    # with CMP0077 enabled.
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
    set(SDL_STATIC ON)
    set(SDL2_DISABLE_INSTALL ON)
    set(SDL2_DISABLE_UNINSTALL ON)
    foreach(feature IN ITEMS SHARED TEST TESTS OPENGL OPENGLES VULKAN HAPTIC POWER
                             LOCALE MISC FILESYSTEM OFFSCREEN DISKAUDIO)
        set(SDL_${feature} OFF)
    endforeach()
    if(APPLE)
        set(SDL2_DISABLE_SDL2MAIN ON)
        set(SDL_RENDER_METAL ON)
    elseif(WIN32)
        # Keep SDL's Windows entry point, D3D11, WASAPI and WinMM. SDL
        # 2.32.10 enables the other DirectX backends from these checks.
        set(SDL2_DISABLE_SDL2MAIN OFF)
        set(HAVE_D3D_H OFF)
        set(HAVE_D3D12_H OFF)
        set(HAVE_DSOUND_H OFF)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|x86_64)$")
            # Keep SDL at the x86-64 baseline
            set(SDL_SSE3 OFF)
        endif()
    endif()

    set(sdl_patch "${PROJECT_SOURCE_DIR}/cmake/SDL2-2.32.10-schnes.patch")
    # A changed patch requires a fresh extraction, since patching mutates the source tree
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${sdl_patch}")
    file(SHA256 "${sdl_patch}" patch_hash)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(schnes_sdl2
        URL "${archive}"
        URL_HASH SHA256=5f5993c530f084535c65a6879e9b26ad441169b3e25d789d83287040a9ca5165
        SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/sdl2-${patch_hash}-src"
        PATCH_COMMAND "${SCHNES_PATCH_PROGRAM}" -p1 --batch --forward -i "${sdl_patch}"
    )
    FetchContent_MakeAvailable(schnes_sdl2)
    # SchNES uploads and captures ARGB8888 textures. Keep the packed RGB
    # renderers and drop conversion code for formats the frontend never submits.
    target_compile_definitions(SDL2-static PRIVATE
        SDL_HAVE_YUV=0
        SDL_HAVE_BLIT_0=0
        SDL_HAVE_BLIT_1=0
        SDL_HAVE_BLIT_N_RGB565=0
    )
    if(MSVC)
        target_compile_options(SDL2-static PRIVATE "$<$<CONFIG:Release>:/O1>")
    else()
        target_compile_options(SDL2-static PRIVATE "$<$<CONFIG:Release>:-Os>")
    endif()
    if(MINGW)
        target_compile_options(SDL2-static PRIVATE -ffunction-sections -fdata-sections)
    endif()
endfunction()
