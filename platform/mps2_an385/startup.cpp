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
}

int main();

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
};

extern "C" void Default_Handler(){
    while(true){}
}
