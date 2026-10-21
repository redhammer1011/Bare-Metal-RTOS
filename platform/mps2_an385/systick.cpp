// systick.cpp - millisecond time base
//
// FreeRTOS owns SysTick: vTaskStartScheduler() programs it from
// configCPU_CLOCK_HZ / configTICK_RATE_HZ (25 MHz / 1 kHz -> LOAD = 24999) and
// its SysTick_Handler runs the kernel tick. The kernel calls
// vApplicationTickHook() from that handler once per tick, and the counters
// here advance from it. Both read 0 until the scheduler starts.

#include "systick.hpp"
#include "critical_section.hpp"

namespace{
    volatile std::uint32_t millis_counter = 0; // Counter for milliseconds

    // 64-bit copy for millis64(). Reading it takes two 32-bit loads, so an
    // unguarded read can be torn by the tick interrupt between the two halves.
    volatile std::uint64_t millis64_counter = 0;
}

std::uint32_t millis(){
    return millis_counter;
}

std::uint64_t millis64(){
    CriticalSection cs;          // no tick between the low and high halves
    return millis64_counter;
}

// Called by the FreeRTOS tick (configUSE_TICK_HOOK = 1), in interrupt context.
extern "C" void vApplicationTickHook(){
    millis_counter++;
    millis64_counter = millis64_counter + 1;  // the tick can't interrupt itself, so no guard needed here
}
