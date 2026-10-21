// nvic.hpp - minimal NVIC access (Cortex-M3)
#pragma once
#include <cstdint>

inline void nvic_enable_irq(unsigned irq){
    auto* iser = reinterpret_cast<volatile std::uint32_t*>(0xE000E100);   // ISER0; ISER1 is the next word
    iser[irq / 32] = 1u << (irq % 32);
}

// Priority is the full 8-bit register value: lower = more urgent. Only the
// implemented high-order bits are kept by the hardware.
inline void nvic_set_priority(unsigned irq, std::uint8_t priority){
    auto* ipr = reinterpret_cast<volatile std::uint8_t*>(0xE000E400);    // one byte per IRQ
    ipr[irq] = priority;
}
