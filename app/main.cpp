// main.cpp - application entry point, called by Reset_Handler
//
// Work is split into tasks run by a cooperative, time-triggered scheduler.
// Every task must return quickly: while one runs, no other task can.

#include "scheduler.hpp"
#include "systick.hpp"
#include "uart.hpp"

namespace {
    // Set to true to add a task that hogs the CPU for 30 ms every 100 ms.
    // Typed characters then echo in bursts: the cooperative scheduler can't
    // run echo_task until slow_task returns.
    constexpr bool kSlowTaskDemo = false;

    Scheduler<4> scheduler;

    // Echo every byte the UART RX interrupt has queued since the last run.
    void echo_task(){
        char c;
        while(uart_getc(c)){
            uart_putc(c);
        }
    }

    void tick_task(){
        static std::uint32_t seconds = 0;
        ++seconds;
        uart_puts("Tick ");
        uart_put_u32(seconds);
        uart_puts("\n");
    }

    // Deliberately violates the "return quickly" rule.
    void slow_task(){
        const std::uint32_t start = millis();
        while(millis() - start < 30){}
    }
}

int main(){
    systick_init();
    uart_enable_rx_interrupt();
    uart_puts("Hello from main\n");

    const std::uint32_t now = millis();
    scheduler.add(echo_task, 1, now);
    scheduler.add(tick_task, 1000, now, 1000);   // first tick one second from now
    if(kSlowTaskDemo){
        scheduler.add(slow_task, 100, now);
    }

    while(true){
        scheduler.run(millis());
    }
}
