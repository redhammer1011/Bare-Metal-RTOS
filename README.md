# Bare-Metal-RTOS

Flight-controller-style firmware for an ARM Cortex-M3, built from the reset
vector up and run entirely in emulation (QEMU, ARM MPS2-AN385 board). No vendor
SDK or HAL: startup code, linker script, and drivers are written from scratch.

## Status

| Phase | Scope | Status |
|---|---|---|
| 0 | Toolchain, startup code, linker script, polled UART, CMake, QEMU + GDB | ✅ Done |
| 1 | Full vector table + fault handler, SysTick, interrupt-driven UART with ring buffer, unit tests | ✅ Done |
| 2 | Own cooperative/preemptive scheduler, then FreeRTOS; jitter and priority-inversion experiments | ⬜ Next |
| 3 | Framed serial protocol (COBS + CRC), Python plant simulation, complementary filter + PID | ⬜ |
| 4 | CI, documentation, results | ⬜ |

## Boot sequence

1. On reset the CPU loads the stack pointer from address `0x0` and jumps to the
   address at `0x4`, both from the vector table in `startup.cpp`.
2. `Reset_Handler` copies `.data` from flash to RAM, zeroes `.bss`, and runs
   C++ global constructors (`.init_array`).
3. `main()` runs.

Memory layout (from AN385 Table 3-1), defined in `linker.ld`:

| Region | Address | Size | Holds |
|---|---|---|---|
| Code ("FLASH") | `0x00000000` | 4 MB | vector table, `.text`, `.rodata`, `.data` initial values |
| RAM | `0x20000000` | 4 MB | `.data`, `.bss`, stack (top of RAM, grows down) |
| UART0 | `0x40004000` | | CMSDK APB UART, RX interrupt = IRQ 0 |
| SysTick | `0xE000E010` | | 1 ms tick from the 25 MHz core clock |

## Interrupts

| Source | Vector | Handler | Notes |
|---|---|---|---|
| SysTick | 15 | `SysTick_Handler` | increments `millis()` / `millis64()` |
| UART0 RX | 16 (IRQ 0) | `UART0RX_Handler` | clears flag, pushes byte into a lock-free SPSC ring buffer |
| HardFault | 3 | `HardFault_Handler` | prints faulting PC and fault status registers, halts |
| everything else | | `Default_Handler` | weak aliases; override by defining the named handler |

RX interrupts are enabled from `main()`, never from startup code: the ISR writes
`.bss`, which `Reset_Handler` zeroes after `uart_init()` runs.

## Layout

```
app/                    application code (main.cpp)
core/                   hardware-independent code (ring buffer), unit-tested on the host
platform/mps2_an385/    board-specific: startup, linker script, UART, SysTick, NVIC,
                        critical sections, fault handler
tests/                  GoogleTest unit tests for core/
cmake/                  arm-none-eabi toolchain file
scripts/                QEMU launch script
```

## Build and run

Requires the Arm GNU Toolchain (`arm-none-eabi-*`), CMake 3.20+, Ninja, and
QEMU (`qemu-system-arm`) on `PATH`.

```powershell
cmake --preset arm              # configure (once)
cmake --build --preset arm      # build -> build-arm/fw.elf, prints flash/RAM usage
.\scripts\run_qemu.ps1          # run; exit with Ctrl+A then X
```

Expected output: a `Tick N` line every second, and anything you type is echoed
back by the firmware.

```
R
Hello from main
Tick 1
Tick 2
```

## Unit tests

Hardware-independent code in `core/` is tested on the host with GoogleTest
(downloaded automatically by CMake). Requires a host `g++` on `PATH`.

```powershell
cmake --preset native           # configure (once)
cmake --build --preset native   # build tests
ctest --preset native           # run
```

## Debug

```powershell
.\scripts\run_qemu.ps1 -Debug                                  # terminal 1: paused at reset
arm-none-eabi-gdb build-arm/fw.elf -ex "target remote :1234"   # terminal 2
```

A fault prints a report and halts, for example after reading an unmapped address:

```
*** HardFault ***
 PC    = 0x00000858
 CFSR  = 0x00008200      <- BFARVALID + PRECISERR: precise bus fault
 HFSR  = 0x40000000      <- FORCED: escalated to HardFault
 BFAR  = 0xfffffff0      <- the address that was accessed
```

Resolve the PC to a source line with
`arm-none-eabi-addr2line -e build-arm/fw.elf -f -C 0x00000858`.

## Memory usage

Flash = `.isr_vector` + `.text` + `.data`; RAM = `.data` + `.bss`, plus an 8 KB
main stack reserved by `linker.ld`. Measured with `arm-none-eabi-size -A`.

| Milestone | Build | Flash | RAM (globals) | Notes |
|---|---|---|---|---|
| Phase 0 | Debug (`-O0`) | 344 B | 0 B | startup + polled UART |
| Phase 1 | Debug (`-O0`) | 2,460 B | 96 B | + SysTick, IRQ-driven RX, ring buffer, fault handler |
| Phase 1 | Release (`-O3`) | 1,716 B | 96 B | |
| Phase 1 | MinSizeRel (`-Os`) | 1,124 B | 96 B | |

Phase 1 RAM breakdown (from `build-arm/fw.map` / `arm-none-eabi-nm -S`): UART RX
ring buffer 72 B (64 data + two 4-byte indices), RX drop counter 4 B,
`millis` counter 4 B, `millis64` counter 8 B, plus 8 B of alignment padding.

Observations:
- At `-O0`, `RingBuffer::push`/`pop` are ~320 B each because every
  `std::atomic` access is an out-of-line call; at `-O2`+ they inline to a few
  instructions with `dmb` barriers.
- `-O3` unrolls `uart_put_u32`'s digit loop (146 B → 408 B). `-Os` is the
  usual firmware choice when flash is tight.

## QEMU caveats

Measured differences between the emulator and real hardware:
- **Timing:** without `-icount shift=auto` (set in `scripts/run_qemu.ps1`), QEMU
  on Windows can't deliver a 1 kHz SysTick on time, and the firmware clock runs
  ~45% slow. icount ties emulated time to instructions executed instead.
- **UART RX can't overrun:** QEMU holds the next byte until firmware reads
  `DATA`, so a 208-byte burst into a 64-slot buffer loses nothing. A real UART
  at 115200 baud delivers a byte every ~87 µs regardless; a slow ISR overruns
  (STATE bit 3) and a slow consumer fills the ring buffer (`uart_rx_dropped_count()`).
