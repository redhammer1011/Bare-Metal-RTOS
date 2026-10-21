# Bare-Metal-RTOS

Flight-controller-style firmware for an ARM Cortex-M3, built from the reset
vector up and run entirely in emulation (QEMU, ARM MPS2-AN385 board). No vendor
SDK or HAL: startup code, linker script, and drivers are written from scratch.

## Status

| Phase | Scope | Status |
|---|---|---|
| 0 | Toolchain, startup code, linker script, polled UART, CMake, QEMU + GDB | ✅ Done |
| 1 | Full vector table + fault handler, SysTick, interrupt-driven UART with ring buffer, unit tests | ✅ Done |
| 2 | Own cooperative/preemptive scheduler, then FreeRTOS; jitter and priority-inversion experiments | 🔄 Cooperative scheduler done |
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

## Scheduling

`main()` hands work to a time-triggered cooperative scheduler
(`core/scheduler.hpp`): a fixed table of tasks, each a plain function that runs
to completion every `period_ms`.

- **No drift:** a task's next deadline advances by exactly one period, so a
  late run doesn't shift later ones.
- **Overruns:** if a task falls whole periods behind, the missed deadlines are
  counted (`overruns(i)`) and skipped instead of run back-to-back.
- **Wrap-safe:** due checks use the signed difference of 32-bit timestamps, so
  scheduling continues correctly across the 49.7-day `millis()` wrap.
- **Testable:** the scheduler never reads the clock; `run(now)` takes the time
  as an argument, so host tests drive it with a fake clock (including across
  the wrap) in milliseconds of real time.

| Task | Period | Job |
|---|---|---|
| `echo_task` | 1 ms | echo every byte the RX ISR has queued |
| `tick_task` | 1000 ms | print `Tick N` |

**The cooperative limitation, measured.** Setting `kSlowTaskDemo = true` in
`app/main.cpp` adds a task that busy-waits 30 ms every 100 ms. Echo latency for
120 single characters sent ~8 ms apart:

| | Median | 90th percentile | Max | Echoes > 5 ms |
|---|---|---|---|---|
| Normal | 0.7 ms | 1.4 ms | 2.1 ms | 0 / 120 |
| With slow task | 0.9 ms | 29.2 ms | 63.9 ms | 17 / 120 |

Bytes still arrive and are buffered by the ISR, but `echo_task` can't run until
`slow_task` returns, so any byte landing in that window waits for it. A task's
worst-case start delay is the sum of the other tasks' run times, which is what a
preemptive scheduler removes.

## Layout

```
app/                    application code (main.cpp)
core/                   hardware-independent code (ring buffer, scheduler), unit-tested on the host
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
| Phase 2: cooperative scheduler | Debug (`-O0`) | 2,948 B | 184 B | + 84 B scheduler table (4 tasks) |

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
