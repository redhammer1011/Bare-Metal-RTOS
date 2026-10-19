// uart.hpp - polled driver for CMSDK APB UART0 (MPS2-AN385)
#pragma once

void uart_init();
void uart_putc(char c);
void uart_puts(const char* s);
