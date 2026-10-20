// startup.cpp - boot code for the MPS2-AN385 (Cortex-M3)
//
// Runs between power-on and main(): the vector table tells the CPU where the
// stack is and where Reset_Handler is; Reset_Handler prepares memory for C++
// and calls main().

#include <cstdint>

#include "uart.hpp"

// ---------------------------------------------------------------------------
// Part 1: symbols defined by linker.ld
//
// These are ADDRESSES the linker chose, not variables. Always use &_sdata,
// never _sdata. Declaring them as uint32_t makes &_sdata a uint32_t*, so
// startup loops can move memory one 32-bit word at a time.
// ---------------------------------------------------------------------------
extern "C" {
    extern std::uint32_t _estack;                  // one past the top of RAM: initial SP

    extern std::uint32_t _sidata;                  // .data initial values (in FLASH)
    extern std::uint32_t _sdata;                   // .data start (in RAM)
    extern std::uint32_t _edata;                   // .data end   (in RAM)

    extern std::uint32_t _sbss;                    // .bss start  (in RAM)
    extern std::uint32_t _ebss;                    // .bss end    (in RAM)

    // .init_array is a list of pointers to the constructors of C++ globals.
    using InitFunc = void (*)();
    extern InitFunc __init_array_start[];
    extern InitFunc __init_array_end[];

    void Reset_Handler();
    void NMI_Handler() __attribute__((weak, alias("Default_Handler")));
    void HardFault_Handler() __attribute__((weak, alias("Default_Handler")));
    void MemManage_Handler() __attribute__((weak, alias("Default_Handler")));
    void BusFault_Handler() __attribute__((weak, alias("Default_Handler")));
    void UsageFault_Handler() __attribute__((weak, alias("Default_Handler")));
    void SVC_Handler() __attribute__((weak, alias("Default_Handler")));
    void DebugMon_Handler() __attribute__((weak, alias("Default_Handler")));
    void PendSV_Handler() __attribute__((weak, alias("Default_Handler")));
    void SysTick_Handler() __attribute__((weak, alias("Default_Handler")));
    void UART0RX_Handler() __attribute__((weak, alias("Default_Handler")));
}

int main();

extern "C" void Default_Handler(){
    while(true){}
}
// ---------------------------------------------------------------------------
// Part 2: Reset_Handler
// ---------------------------------------------------------------------------
extern "C" void Reset_Handler(){
    uart_init();
    uart_putc('R');
    uart_putc('\n');
    // copy .data from FLASH to RAM
    std::uint32_t* src = &_sidata;
    std::uint32_t* dst = &_sdata;
    while(dst < &_edata){
        *dst++ = *src++;
    }

    // zero .bss
    dst = &_sbss;
    while(dst < &_ebss){
        *dst++ = 0;
    }

    // call C++ global constructors
    for(InitFunc* ctor = __init_array_start; ctor < __init_array_end; ++ctor){
        (*ctor)();
    }

    // call main()
    main();

    // if main() returns, loop forever
    while(true){}
}

// ---------------------------------------------------------------------------
// Part 3: vector table (placed at 0x00000000 by linker.ld)
// ---------------------------------------------------------------------------
__attribute__((section(".isr_vector"), used))
const void* const vector_table[] = {
    &_estack,                                         // initial SP
    reinterpret_cast<const void*>(Reset_Handler),     // Reset_Handler
    reinterpret_cast<const void*>(NMI_Handler),       // NMI_Handler
    reinterpret_cast<const void*>(HardFault_Handler), // HardFault_Handler
    reinterpret_cast<const void*>(MemManage_Handler), // MemManage_Handler
    reinterpret_cast<const void*>(BusFault_Handler),  // BusFault_Handler
    reinterpret_cast<const void*>(UsageFault_Handler),// UsageFault_Handler
    nullptr,                                         // reserved
    nullptr,                                         // reserved
    nullptr,                                         // reserved
    nullptr,                                         // reserved
    reinterpret_cast<const void*>(SVC_Handler),       // SVC_Handler
    reinterpret_cast<const void*>(DebugMon_Handler),  // DebugMon_Handler
    nullptr,                                         // reserved
    reinterpret_cast<const void*>(PendSV_Handler),    // PendSV_Handler
    reinterpret_cast<const void*>(SysTick_Handler),   // SysTick_Handler
    // External interrupts: index 16 + IRQ number. Sources per QEMU hw/arm/mps2.c (AN385).
    reinterpret_cast<const void*>(UART0RX_Handler),    // IRQ 0:  UART0 RX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 1:  UART0 TX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 2:  UART1 RX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 3:  UART1 TX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 4:  UART2 RX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 5:  UART2 TX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 6:  (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 7:  (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 8:  TIMER0
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 9:  TIMER1
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 10: dual timer
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 11: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 12: UART 0-2 overflow (combined)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 13: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 14: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 15: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 16: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 17: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 18: UART3 RX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 19: UART3 TX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 20: UART4 RX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 21: UART4 TX
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 22: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 23: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 24: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 25: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 26: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 27: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 28: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 29: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 30: (unused)
    reinterpret_cast<const void*>(Default_Handler),    // IRQ 31: (unused)
};
static_assert(sizeof(vector_table) / sizeof(vector_table[0]) == 48, "16 system + 32 IRQ entries");
