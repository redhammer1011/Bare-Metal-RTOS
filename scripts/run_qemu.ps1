# run_qemu.ps1 - run the firmware in QEMU on the emulated MPS2-AN385 board
#
#   .\scripts\run_qemu.ps1          run; UART0 output appears in this terminal
#   .\scripts\run_qemu.ps1 -Debug   start frozen at reset and wait for GDB on :1234
#
# Exit QEMU with Ctrl+A, then X.

param(
    [switch]$Debug,
    [string]$Elf = "$PSScriptRoot\..\build-arm\fw.elf"
)

if (-not (Test-Path $Elf)) {
    Write-Error "Firmware not found at $Elf. Build first: cmake --build --preset arm"
    exit 1
}

$qemuArgs = @("-M", "mps2-an385", "-nographic", "-kernel", $Elf)
if ($Debug) {
    $qemuArgs += @("-s", "-S")
    Write-Host "QEMU paused at reset. In another terminal: arm-none-eabi-gdb build-arm/fw.elf -ex 'target remote :1234'"
}

& qemu-system-arm @qemuArgs
