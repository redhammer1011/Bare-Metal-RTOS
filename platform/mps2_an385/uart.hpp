// uart.hpp - polled driver for CMSDK APB UART0 (MPS2-AN385)
#pragma once
#include <cstdint>

void uart_init();
void uart_putc(char c);
void uart_puts(const char* s);
void uart_put_u32(std::uint32_t value);
