# run_qemu.ps1 - run the firmware in QEMU on the emulated MPS2-AN385 board
#
#   .\scripts\run_qemu.ps1          run; UART0 (console) appears in this terminal
#   .\scripts\run_qemu.ps1 -Debug   start frozen at reset and wait for GDB on :1234
#
# UART1 (binary packet link) listens on TCP 127.0.0.1:5560 for the Python side,
# e.g.  python sim/link_test.py
#
# Exit QEMU with Ctrl+A, then X.

param(
    [switch]$Debug,
    [string]$Elf = "$PSScriptRoot\..\build-arm\fw.elf",
    [int]$LinkPort = 5560
)

if (-not (Test-Path $Elf)) {
    Write-Error "Firmware not found at $Elf. Build first: cmake --build --preset arm"
    exit 1
}

# -icount shift=auto ties emulated time to instructions executed instead of the
# host clock. Without it, QEMU on Windows cannot deliver a 1 kHz SysTick on time
# and the firmware's clock runs ~45% slow.
#
# Serial ports are assigned in order: the first -serial is UART0, the second
# UART1. Giving any -serial disables -nographic's default, so UART0 is named
# explicitly (mon:stdio keeps Ctrl+A shortcuts). wait=off lets the firmware run
# before a client connects; bytes sent with no client are discarded.
$qemuArgs = @(
    "-M", "mps2-an385", "-nographic", "-icount", "shift=auto",
    "-serial", "mon:stdio",
    "-serial", "tcp:127.0.0.1:${LinkPort},server=on,wait=off",
    "-kernel", $Elf
)
if ($Debug) {
    $qemuArgs += @("-s", "-S")
    Write-Host "QEMU paused at reset. In another terminal: arm-none-eabi-gdb build-arm/fw.elf -ex 'target remote :1234'"
}

& qemu-system-arm @qemuArgs
