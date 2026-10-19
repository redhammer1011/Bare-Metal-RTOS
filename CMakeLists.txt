cmake_minimum_required(VERSION 3.20)

project(BareMetalRTOS LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PLATFORM_DIR ${CMAKE_SOURCE_DIR}/platform/mps2_an385)
set(LINKER_SCRIPT ${PLATFORM_DIR}/linker.ld)

add_executable(fw.elf
    ${PLATFORM_DIR}/startup.cpp
    ${PLATFORM_DIR}/uart.cpp
    app/main.cpp
)

target_include_directories(fw.elf PRIVATE ${PLATFORM_DIR})

# -mcpu/-mthumb appear in both lists: the compiler needs them to generate
# Cortex-M3 Thumb code, the linker to pick the matching runtime libraries.
set(CPU_FLAGS -mcpu=cortex-m3 -mthumb)

target_compile_options(fw.elf PRIVATE
    ${CPU_FLAGS}
    -ffreestanding
    -fno-exceptions
    -fno-rtti
    -Wall
    -Wextra
)

target_link_options(fw.elf PRIVATE
    ${CPU_FLAGS}
    -nostdlib
    -T ${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_BINARY_DIR}/fw.map
    -Wl,--no-warn-rwx-segments
)

# Relink when only the linker script changes.
set_target_properties(fw.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

# Print flash/RAM usage after every build.
add_custom_command(TARGET fw.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:fw.elf>
    VERBATIM
)
