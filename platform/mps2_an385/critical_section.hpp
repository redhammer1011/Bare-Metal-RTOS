// critical_section.hpp - RAII guard that masks interrupts for one scope
//
//     {
//         CriticalSection cs;      // interrupts off
//         ...touch shared data...
//     }                            // previous state restored, however the scope exits
//
// Saves and restores PRIMASK instead of unconditionally re-enabling, so guards
// nest safely. Keep the guarded scope short: interrupts that arrive while it is
// held are delayed until it ends. NMI and HardFault are not masked.
#pragma once

#include <cstdint>

class CriticalSection {
public:
    CriticalSection(){
        __asm volatile("mrs %0, primask" : "=r"(saved_primask_));        // remember current state
        __asm volatile("cpsid i" ::: "memory");                          // mask interrupts
    }

    ~CriticalSection(){
        __asm volatile("msr primask, %0" :: "r"(saved_primask_) : "memory");  // restore it
    }

    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    std::uint32_t saved_primask_;
};
