// uart.cpp - CMSDK APB UART0 driver (MPS2-AN385): polled TX, interrupt-driven RX
//
// uart_init() touches no globals, so it is safe to call from Reset_Handler
// before .data/.bss are initialized. uart_enable_rx_interrupt() must be called
// from main(), after memory init, because the RX ISR writes uart_rx_buffer.

#include "uart.hpp"

#include <cstddef>
#include <cstdint>

#include "ring_buffer.hpp"

#include "nvic.hpp"

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
    constexpr std::uint32_t CTRL_RX_INTEN = 1u << 3;
    constexpr std::uint32_t INTSTATUS_RX  = 1u << 1;   // write 1 to clear

    volatile UartRegs& uart0(){
        return *reinterpret_cast<volatile UartRegs*>(0x40004000);
    }

    RingBuffer<char, 64> uart_rx_buffer;
    volatile std::uint32_t rx_dropped = 0;
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

void uart_put_u32(std::uint32_t value){
    char buffer[11]; // Maximum length for 32-bit unsigned integer in decimal + null terminator
    int index = 10;  // Start from the end of the buffer
    buffer[index] = '\0'; // Null terminator

    if(value == 0){
        buffer[--index] = '0';
    } else {
        while(value > 0 && index > 0){
            buffer[--index] = '0' + (value % 10);
            value /= 10;
        }
    }

    uart_puts(&buffer[index]);
}

void uart_enable_rx_interrupt(){
    uart0().CTRL |= CTRL_RX_INTEN;
    nvic_enable_irq(0); // Enable UART0 RX interrupt (IRQ 0)
}

std::uint32_t uart_rx_dropped_count(){
    return rx_dropped;
}

bool uart_getc(char& c){
    return uart_rx_buffer.pop(c);
}

extern "C" void UART0RX_Handler(){
    uart0().INTSTATUS = INTSTATUS_RX; // clear RX interrupt
    const char c = static_cast<char>(uart0().DATA);
    if(!uart_rx_buffer.push(c)){
        ++rx_dropped; // RX buffer overflow, increment dropped count
    }
}
