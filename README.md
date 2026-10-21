# Bare-Metal-RTOS

Flight-controller-style firmware for an ARM Cortex-M3, built from the reset
vector up and run entirely in emulation (QEMU, ARM MPS2-AN385 board). No vendor
SDK or HAL: startup code, linker script, and drivers are written from scratch.

## Status

| Phase | Scope | Status |
|---|---|---|
| 0 | Toolchain, startup code, linker script, polled UART, CMake, QEMU + GDB | ✅ Done |
| 1 | Full vector table + fault handler, SysTick, interrupt-driven UART with ring buffer, unit tests | ✅ Done |
| 2 | Cooperative scheduler, then FreeRTOS; latency and priority-inversion experiments | 🔄 Cooperative scheduler + FreeRTOS port done |
| 3 | Framed serial protocol (COBS + CRC), Python plant simulation, complementary filter + PID | ⬜ |
| 4 | CI, documentation, results | ⬜ |

## Boot sequence

1. On reset the CPU loads the stack pointer from address `0x0` and jumps to the
   address at `0x4`, both from the vector table in `startup.cpp`.
2. `Reset_Handler` copies `.data` from flash to RAM, zeroes `.bss`, and runs
   C++ global constructors (`.init_array`).
3. `main()` creates the FreeRTOS tasks and calls `vTaskStartScheduler()`,
   which programs SysTick, switches to the process stack, and starts the
   highest-priority task. It never returns.

Memory layout (from AN385 Table 3-1), defined in `linker.ld`:

| Region | Address | Size | Holds |
|---|---|---|---|
| Code ("FLASH") | `0x00000000` | 4 MB | vector table, `.text`, `.rodata`, `.data` initial values |
| RAM | `0x20000000` | 4 MB | `.data`, `.bss`, stack (top of RAM, grows down) |
| UART0 | `0x40004000` | | CMSDK APB UART, RX interrupt = IRQ 0 |
| SysTick | `0xE000E010` | | 1 ms kernel tick from the 25 MHz core clock (LOAD = 24999) |

## Interrupts

| Source | Vector | Handler | NVIC priority | Notes |
|---|---|---|---|---|
| HardFault | 3 | `HardFault_Handler` | fixed (−1) | prints faulting PC and fault status registers, halts; reads the frame from MSP or PSP |
| SVCall | 11 | FreeRTOS `vPortSVCHandler` | — | starts the first task |
| PendSV | 14 | FreeRTOS `xPortPendSVHandler` | `0xFF` (least urgent) | context switch |
| SysTick | 15 | FreeRTOS `xPortSysTickHandler` | `0xFF` | kernel tick; calls `vApplicationTickHook()` → `millis()` |
| UART0 RX | 16 (IRQ 0) | `UART0RX_Handler` | `0xC0` | clears flag, pushes byte into a lock-free SPSC ring buffer, notifies `echo_task` |
| everything else | | `Default_Handler` | | weak aliases; override by defining the named handler |

The three FreeRTOS handlers are renamed to our vector table names in
`FreeRTOSConfig.h`, so they override the weak aliases in `startup.cpp`.

**Priority rule:** an ISR that calls a FreeRTOS `...FromISR()` API must have an
NVIC priority numerically ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY` (`0xA0`).
UART RX runs at `0xC0`. QEMU implements all 8 priority bits (measured by writing
`0xFF` to a priority register and reading it back); real Cortex-M3 parts
usually implement 3 or 4.

RX interrupts are enabled from inside `echo_task`, once the scheduler is
running: never from startup code (the ISR writes `.bss`, which `Reset_Handler`
zeroes after `uart_init()`), and not before `vTaskStartScheduler()` (the ISR
can request a context switch, which needs a running task).

## Scheduling

### FreeRTOS (current)

The firmware runs on FreeRTOS V11.3.2 (`rtos/FreeRTOS-Kernel`, a git
submodule), configured in `app/FreeRTOSConfig.h`: preemptive, 1 kHz tick,
static allocation only (no heap), stack-overflow checking on every context
switch.

| Task | Priority | Wakes on | Job |
|---|---|---|---|
| `echo_task` | 3 | task notification from the UART RX ISR | echo every queued byte |
| `tick_task` | 2 | `xTaskDelayUntil`, every 1000 ms | print `Tick N` |
| `slow_task` (demo) | 1 | `xTaskDelayUntil`, every 100 ms | busy-wait 30 ms |

UART output is serialized by a FreeRTOS mutex (`UartLock`, RAII) so lines from
different tasks don't interleave. Each task has a statically allocated 1 KB
stack; measured high-water marks at `-O0` are 124 B (echo) and 164 B (tick).

**Echo latency** for 120 single characters sent ~8 ms apart, with
`kSlowTaskDemo = true` adding a task that busy-waits 30 ms every 100 ms:

| Scheduler | Slow task | Median | 90th percentile | Max | Echoes > 5 ms |
|---|---|---|---|---|---|
| Cooperative | off | 0.7 ms | 1.4 ms | 2.1 ms | 0 / 120 |
| Cooperative | on | 0.9 ms | 29.2 ms | 63.9 ms | 17 / 120 |
| FreeRTOS | off | 0.3 ms | 0.5 ms | 2.2 ms | 0 / 120 |
| FreeRTOS | on, priority 1 (below echo) | 0.3 ms | 0.5 ms | 2.4 ms | 0 / 120 |
| FreeRTOS | on, priority 4 (above echo) | 0.4 ms | 23.3 ms | 34.7 ms | 12 / 120 |

- Under FreeRTOS, `echo_task` blocks until the ISR notifies it instead of
  polling every 1 ms, so even the unloaded median drops.
- With the slow task below echo, the kernel preempts it the moment a byte
  arrives: latency is unchanged by the 30% CPU hog.
- With the slow task above echo, the stalls return. Preemption only helps a
  task that outranks the work in its way: priority assignment is the design
  decision.

### Cooperative scheduler (Phase 2, step 1)

Before FreeRTOS, `main()` ran a time-triggered cooperative scheduler
(`core/scheduler.hpp`), kept as a unit-tested library: a fixed table of tasks,
each a plain function that runs to completion every `period_ms`.

- **No drift:** a task's next deadline advances by exactly one period, so a
  late run doesn't shift later ones.
- **Overruns:** if a task falls whole periods behind, the missed deadlines are
  counted (`overruns(i)`) and skipped instead of run back-to-back.
- **Wrap-safe:** due checks use the signed difference of 32-bit timestamps, so
  scheduling continues correctly across the 49.7-day `millis()` wrap.
- **Testable:** the scheduler never reads the clock; `run(now)` takes the time
  as an argument, so host tests drive it with a fake clock (including across
  the wrap) in milliseconds of real time.

Its measured limitation (cooperative rows in the table above): bytes still
arrive and are buffered by the ISR, but `echo_task` can't run until `slow_task`
returns, so a task's worst-case start delay is the sum of the other tasks' run
times. That is what moving to a preemptive kernel removed.

## Layout

```
app/                    application: tasks (main.cpp), FreeRTOSConfig.h, kernel hooks
core/                   hardware-independent code (ring buffer, scheduler), unit-tested on the host
rtos/FreeRTOS-Kernel/   FreeRTOS kernel, git submodule pinned to V11.3.2
platform/mps2_an385/    board-specific: startup, linker script, UART, SysTick, NVIC,
                        critical sections, fault handler
tests/                  GoogleTest unit tests for core/
cmake/                  arm-none-eabi toolchain file
scripts/                QEMU launch script
```

## Build and run

Requires the Arm GNU Toolchain (`arm-none-eabi-*`), CMake 3.20+, Ninja, and
QEMU (`qemu-system-arm`) on `PATH`.

Clone with the FreeRTOS submodule (`git clone --recursive ...`), or after a
plain clone run `git submodule update --init`.

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
| Phase 2: FreeRTOS | Debug (`-O0`) | 19,508 B | 4,332 B | kernel ≈ 16 KB flash; 3 × 1 KB task stacks + TCBs, idle task, mutex |

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
