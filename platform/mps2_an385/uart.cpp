// uart.cpp - polled driver for CMSDK APB UART0 (MPS2-AN385)
//
// Uses no globals, so it is safe to call from Reset_Handler before .data/.bss
// are initialized.

#include "uart.hpp"

#include <cstddef>
#include <cstdint>

namespace {
    struct UartRegs {
        volatile std::uint32_t DATA;      // 0x00
        volatile std::uint32_t STATE;     // 0x04
        volatile std::uint32_t CTRL;      // 0x08
        volatile std::uint32_t INTSTATUS; // 0x0C
        volatile std::uint32_t BAUDDIV;   // 0x10
    };

    static_assert(offsetof(UartRegs, DATA) == 0x00);
    static_assert(offsetof(UartRegs, STATE) == 0x04);
    static_assert(offsetof(UartRegs, CTRL) == 0x08);
    static_assert(offsetof(UartRegs, INTSTATUS) == 0x0C);
    static_assert(offsetof(UartRegs, BAUDDIV) == 0x10);

    constexpr std::uint32_t STATE_TX_FULL = 1u << 0;
    constexpr std::uint32_t CTRL_TX_EN    = 1u << 0;
    constexpr std::uint32_t CTRL_RX_EN    = 1u << 1;

    volatile UartRegs& uart0(){
        return *reinterpret_cast<volatile UartRegs*>(0x40004000);
    }
}

void uart_init(){
    uart0().BAUDDIV = 0x10;                     // minimum valid divisor
    uart0().CTRL = CTRL_TX_EN | CTRL_RX_EN;
}

void uart_putc(char c){
    while(uart0().STATE & STATE_TX_FULL){}      // wait while TX buffer is full
    uart0().DATA = c;
}

void uart_puts(const char* s){
    while(*s != '\0'){
        uart_putc(*s++);
    }
}
