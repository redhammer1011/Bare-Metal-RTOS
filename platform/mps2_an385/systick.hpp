#pragma once
#include <cstdint>

void systick_init();
std::uint32_t millis();     // wraps after ~49.7 days; compare with (now - start)
std::uint64_t millis64();   // never wraps in practice; costs a brief interrupt mask
