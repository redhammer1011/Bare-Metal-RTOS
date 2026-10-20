// uart.hpp - CMSDK APB UART0 driver (MPS2-AN385): polled TX, interrupt-driven RX
#pragma once
#include <cstdint>

void uart_init();
void uart_putc(char c);
void uart_puts(const char* s);
void uart_put_u32(std::uint32_t value);
void uart_enable_rx_interrupt();
bool uart_getc(char& c); // returns true if a character was read, false if no character was available
std::uint32_t uart_rx_dropped_count(); // returns the number of characters dropped due to RX buffer overflow
