#include "systick.hpp"
#include <cstddef>

namespace{
    struct SysTickRegs{
        volatile std::uint32_t CTRL;      // 0x00
        volatile std::uint32_t LOAD;      // 0x04
        volatile std::uint32_t VAL;       // 0x08
        volatile std::uint32_t CALIB;     // 0x0C
    };

    static_assert(offsetof(SysTickRegs, CTRL) == 0x00);
    static_assert(offsetof(SysTickRegs, LOAD) == 0x04);
    static_assert(offsetof(SysTickRegs, VAL) == 0x08);
    static_assert(offsetof(SysTickRegs, CALIB) == 0x0C);

    volatile SysTickRegs& systick(){
        return *reinterpret_cast<volatile SysTickRegs*>(0xE000E010);
    }
    
    volatile std::uint32_t millis_counter = 0; // Counter for milliseconds
    
    const std::uint32_t CTRL_ENABLE = 1u << 0;
    const std::uint32_t CTRL_TICKINT = 1u << 1;
    const std::uint32_t CTRL_CLKSOURCE = 1u << 2;
}

void systick_init(){
    systick().LOAD = 25000 - 1; // Set reload value for 1ms interval (assuming 25 MHz clock)
    systick().VAL = 0;           // clear current value
    systick().CTRL = CTRL_ENABLE | CTRL_TICKINT | CTRL_CLKSOURCE; // Enable SysTick, enable interrupt, use processor clock
}

std::uint32_t millis(){
    return millis_counter;
}

extern "C" void SysTick_Handler(){
    // This function is called by the SysTick interrupt.
    // It can be used to increment a millisecond counter or perform other periodic tasks.
    millis_counter++; // Increment the millisecond counter
}

