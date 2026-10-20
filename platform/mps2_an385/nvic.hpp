#pragma once
#include <cstdint>
inline void nvic_enable_irq(unsigned irq){
    auto* iser = reinterpret_cast<volatile std::uint32_t*>(0xE000E100);   // ISER0; ISER1 is the next word
    iser[irq / 32] = 1u << (irq % 32);
}