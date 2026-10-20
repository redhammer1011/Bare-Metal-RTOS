// main.cpp - application entry point, called by Reset_Handler

#include "uart.hpp"
#include "systick.hpp"

int main(){
    systick_init();
    uart_enable_rx_interrupt();
    uart_puts("Hello from main\n");
    std::uint32_t start = millis();
    while(true){
        // Main application loop
        char c;
        if(uart_getc(c)){
            uart_putc(c); // Echo received character
        }
        if(millis() - start >= 1000){
            start += 1000;
            uart_puts("Tick ");
            uart_put_u32(start / 1000); // Print the number of seconds since start
            uart_puts("\n");
        }
    }
}
