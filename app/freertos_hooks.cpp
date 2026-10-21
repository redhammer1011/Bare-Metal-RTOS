// freertos_hooks.cpp - callbacks the FreeRTOS kernel requires from the application
//
// Both report over polled UART and halt: after a failed kernel assertion or a
// stack overflow, kernel state can't be trusted, so there is nothing safe to
// continue with.

#include "FreeRTOS.h"
#include "task.h"

#include "uart.hpp"

#include <cstdint>

// configASSERT() in FreeRTOSConfig.h calls this when a kernel check fails,
// e.g. an ISR calling a FromISR() API at a priority above
// configMAX_SYSCALL_INTERRUPT_PRIORITY.
extern "C" void freertos_assert_failed(const char* file, int line){
    taskDISABLE_INTERRUPTS();
    uart_puts("\n*** FreeRTOS assert failed: ");
    uart_puts(file);
    uart_putc(':');
    uart_put_u32(static_cast<std::uint32_t>(line));
    uart_puts("\n");
    for(;;){}
}

// configCHECK_FOR_STACK_OVERFLOW = 2: called during a context switch if the
// outgoing task's stack pointer is past its stack, or the guard bytes at the
// end of its stack were overwritten.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t task, char* task_name){
    (void)task;
    taskDISABLE_INTERRUPTS();
    uart_puts("\n*** Stack overflow in task: ");
    uart_puts(task_name);
    uart_puts("\n");
    for(;;){}
}
