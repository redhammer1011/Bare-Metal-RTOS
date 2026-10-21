// uart.cpp - CMSDK APB UART0 driver (MPS2-AN385): polled TX, interrupt-driven RX
//
// uart_init() touches no globals, so it is safe to call from Reset_Handler
// before .data/.bss are initialized. uart_enable_rx_interrupt() must be called
// from main(), after memory init, because the RX ISR writes uart_rx_buffer.

#include "uart.hpp"

#include <cstdint>

#include "cmsdk_uart_regs.hpp"
#include "nvic.hpp"
#include "ring_buffer.hpp"

namespace {
    using namespace cmsdk_uart;

    volatile Regs& uart0(){
        return at(UART0_BASE);
    }

    RingBuffer<char, 64> uart_rx_buffer;
    volatile std::uint32_t rx_dropped = 0;
    UartRxCallback rx_callback = nullptr;   // written before the IRQ is enabled, then only read by the ISR
}

void uart_init(){
    uart0().BAUDDIV = BAUDDIV_MIN;
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

void uart_put_hex32(std::uint32_t value){
    constexpr char digits[] = "0123456789abcdef";
    uart_puts("0x");
    for(int shift = 28; shift >= 0; shift -= 4){   // most significant nibble first
        uart_putc(digits[(value >> shift) & 0xF]);
    }
}

void uart_set_rx_callback(UartRxCallback callback){
    rx_callback = callback;
}

void uart_enable_rx_interrupt(std::uint8_t nvic_priority){
    nvic_set_priority(UART0_RX_IRQ, nvic_priority);
    uart0().CTRL |= CTRL_RX_INTEN;
    nvic_enable_irq(UART0_RX_IRQ);
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
    if(rx_callback != nullptr){
        rx_callback();
    }
}
