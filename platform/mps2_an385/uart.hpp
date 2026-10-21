// uart.hpp - CMSDK APB UART0 driver (MPS2-AN385): polled TX, interrupt-driven RX
#pragma once
#include <cstdint>

void uart_init();
void uart_putc(char c);
void uart_puts(const char* s);
void uart_put_u32(std::uint32_t value);
void uart_put_hex32(std::uint32_t value);   // "0x" + 8 hex digits, e.g. 0x0000081c
// Called from the RX interrupt after each byte is queued, so an RTOS task can
// be woken without this driver depending on the RTOS. Runs in interrupt
// context: keep it short. Set it before enabling the RX interrupt.
using UartRxCallback = void (*)();
void uart_set_rx_callback(UartRxCallback callback);

// nvic_priority: full 8-bit NVIC priority (lower = more urgent). If the
// callback calls FreeRTOS ...FromISR() APIs it must be numerically >=
// configMAX_SYSCALL_INTERRUPT_PRIORITY.
void uart_enable_rx_interrupt(std::uint8_t nvic_priority);
bool uart_getc(char& c); // returns true if a character was read, false if no character was available
std::uint32_t uart_rx_dropped_count(); // returns the number of characters dropped due to RX buffer overflow
