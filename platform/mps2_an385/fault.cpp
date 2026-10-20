// fault.cpp - HardFault handler that reports where and why the CPU faulted
//
// On any exception the hardware pushes r0-r3, r12, LR, PC, xPSR onto the
// stack that was active. HardFault_Handler finds that frame and passes it to
// hardfault_report(), which prints the crashing PC and the fault status
// registers over UART, then halts. Turn the PC into a source line with:
//     arm-none-eabi-addr2line -e build-arm/fw.elf -f -C <PC>
//
// Defining HardFault_Handler here overrides the weak alias in startup.cpp.

#include "uart.hpp"

#include <cstdint>

namespace {
    // System Control Block fault registers (ARMv7-M).
    constexpr std::uint32_t SCB_CFSR  = 0xE000ED28;   // configurable fault status: what kind of fault
    constexpr std::uint32_t SCB_HFSR  = 0xE000ED2C;   // HardFault status: bit 30 = escalated
    constexpr std::uint32_t SCB_MMFAR = 0xE000ED34;   // MemManage fault address
    constexpr std::uint32_t SCB_BFAR  = 0xE000ED38;   // bus fault address

    std::uint32_t read_reg(std::uint32_t address){
        return *reinterpret_cast<volatile std::uint32_t*>(address);
    }

    // Indices into the exception stack frame, in the order the hardware pushes it.
    enum FrameIndex { R0 = 0, R1, R2, R3, R12, LR, PC, XPSR };

    void print_line(const char* label, std::uint32_t value){
        uart_puts(label);
        uart_put_hex32(value);
        uart_putc('\n');
    }
}

// Called (via a branch, with the frame address in r0) from HardFault_Handler.
// Uses only polled UART output, which doesn't depend on interrupts.
extern "C" void hardfault_report(const std::uint32_t* frame){
    uart_puts("\n*** HardFault ***\n");
    print_line(" PC    = ", frame[PC]);     // instruction that faulted
    print_line(" LR    = ", frame[LR]);     // return address in the faulting function
    print_line(" xPSR  = ", frame[XPSR]);
    print_line(" CFSR  = ", read_reg(SCB_CFSR));
    print_line(" HFSR  = ", read_reg(SCB_HFSR));
    print_line(" MMFAR = ", read_reg(SCB_MMFAR));
    print_line(" BFAR  = ", read_reg(SCB_BFAR));
    uart_puts("halted\n");
    while(true){}
}

// naked: no compiler-generated prologue, so LR and SP still hold exactly what
// the hardware left on exception entry. LR bit 2 says which stack the frame
// is on: 0 = MSP (main stack), 1 = PSP (process stack, used by RTOS tasks).
extern "C" __attribute__((naked)) void HardFault_Handler(){
    __asm volatile(
        "tst   lr, #4          \n"
        "ite   eq              \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "b     hardfault_report\n"
    );
}
