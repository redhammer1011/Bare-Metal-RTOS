// main.cpp - application entry point, called by Reset_Handler

#include "uart.hpp"

int main(){
    uart_puts("Hello from main\n");
    while(true){}
}
