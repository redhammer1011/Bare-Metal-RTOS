# arm-none-eabi.cmake - CMake toolchain file for bare-metal ARM Cortex-M
#
# Tells CMake to cross-compile with the Arm GNU Toolchain instead of the
# laptop's native compiler. Selected with -DCMAKE_TOOLCHAIN_FILE=... or the
# "arm" preset in CMakePresets.json.

set(CMAKE_SYSTEM_NAME Generic)          # no operating system ("none" in arm-none-eabi)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_SIZE         arm-none-eabi-size)

# CMake checks the compiler by building a test program. A bare-metal program
# can't link without a vector table and linker script, so only compile it.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Search for programs on the host, but libraries/headers only in the target
# environment, so nothing from the laptop gets linked into firmware.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
