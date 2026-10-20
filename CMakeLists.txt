cmake_minimum_required(VERSION 3.20)

project(BareMetalRTOS LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CORE_DIR ${CMAKE_SOURCE_DIR}/core)

# One project, two builds, chosen by preset:
#   arm    -> cross-compiled firmware (cmake/arm-none-eabi.cmake sets Generic)
#   native -> unit tests for core/ on the laptop with the host g++
if(CMAKE_SYSTEM_NAME STREQUAL "Generic")

    # ------------------------------------------------------------------ firmware
    set(PLATFORM_DIR ${CMAKE_SOURCE_DIR}/platform/mps2_an385)
    set(LINKER_SCRIPT ${PLATFORM_DIR}/linker.ld)

    add_executable(fw.elf
        ${PLATFORM_DIR}/startup.cpp
        ${PLATFORM_DIR}/uart.cpp
        ${PLATFORM_DIR}/systick.cpp
        ${PLATFORM_DIR}/fault.cpp
        app/main.cpp
    )

    target_include_directories(fw.elf PRIVATE ${PLATFORM_DIR} ${CORE_DIR})

    # GCC 15+ enables libstdc++ assertions at -O0; they call into the full C++
    # runtime, which -nostdlib firmware doesn't link (e.g. std::atomic::load).
    target_compile_definitions(fw.elf PRIVATE _GLIBCXX_NO_ASSERTIONS)

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

else()

    # ------------------------------------------------------------- native tests
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(core_tests
        tests/test_ring_buffer.cpp
    )
    target_include_directories(core_tests PRIVATE ${CORE_DIR})
    target_link_libraries(core_tests PRIVATE GTest::gtest_main)
    target_compile_options(core_tests PRIVATE -Wall -Wextra)
    # Static link so the test .exe runs without MinGW DLLs on PATH.
    target_link_options(core_tests PRIVATE -static)

    include(GoogleTest)
    gtest_discover_tests(core_tests)

endif()
