// main.cpp - application entry point, called by Reset_Handler
//
// Work runs as FreeRTOS tasks with fixed priorities. The kernel always runs
// the highest-priority task that is ready. A task gives up the CPU by
// blocking (waiting for a delay, a notification, or a mutex), not by
// returning: each task function is an infinite loop.

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "systick.hpp"
#include "uart.hpp"

#include <cstdint>

namespace {
    // Set to true to add a low-priority task that hogs the CPU for 30 ms every
    // 100 ms. Unlike under the cooperative scheduler, echo stays responsive:
    // the kernel preempts slow_task as soon as echo_task becomes ready.
    constexpr bool kSlowTaskDemo = false;

    // FreeRTOS task priorities: higher number = more urgent. (NVIC interrupt
    // priorities are the opposite: lower number = more urgent.)
    constexpr UBaseType_t kEchoPriority = 3;
    constexpr UBaseType_t kTickPriority = 2;
    constexpr UBaseType_t kSlowPriority = 1;

    // UART RX interrupt priority. Its callback calls vTaskNotifyGiveFromISR(),
    // so it must be numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY (0xA0).
    constexpr std::uint8_t kUartRxIrqPriority = 0xC0;

    constexpr configSTACK_DEPTH_TYPE kStackWords = 256;   // 1 KB per task

    // Statically allocated stacks and task control blocks, placed in .bss.
    StackType_t  echo_stack[kStackWords];
    StaticTask_t echo_tcb;
    StackType_t  tick_stack[kStackWords];
    StaticTask_t tick_tcb;
    StackType_t  slow_stack[kStackWords];
    StaticTask_t slow_tcb;

    TaskHandle_t echo_handle = nullptr;

    // Serializes UART output so lines from different tasks don't interleave.
    // A FreeRTOS mutex (unlike a binary semaphore) has priority inheritance.
    StaticSemaphore_t uart_mutex_storage;
    SemaphoreHandle_t uart_mutex = nullptr;

    // RAII: holds the UART mutex for the lifetime of the object.
    class UartLock {
    public:
        UartLock(){ xSemaphoreTake(uart_mutex, portMAX_DELAY); }
        ~UartLock(){ xSemaphoreGive(uart_mutex); }
        UartLock(const UartLock&) = delete;
        UartLock& operator=(const UartLock&) = delete;
    };

    // Runs inside the UART RX interrupt after a byte is queued.
    void on_uart_rx(){
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(echo_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);   // switch on ISR exit if echo now outranks the interrupted task
    }

    void echo_task(void*){
        // Enable RX only once the scheduler is running: the ISR wakes this
        // task, and a context switch requested before vTaskStartScheduler()
        // would have no task to switch from.
        uart_set_rx_callback(on_uart_rx);
        uart_enable_rx_interrupt(kUartRxIrqPriority);

        for(;;){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);       // sleep until the RX ISR signals
            UartLock lock;
            char c;
            while(uart_getc(c)){
                uart_putc(c);
            }
        }
    }

    void tick_task(void*){
        std::uint32_t seconds = 0;
        TickType_t last_wake = xTaskGetTickCount();
        for(;;){
            xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));   // fixed 1 s timeline, no drift
            ++seconds;
            UartLock lock;
            uart_puts("Tick ");
            uart_put_u32(seconds);
            uart_puts("\n");
        }
    }

    // Deliberately hogs the CPU: a busy-wait instead of blocking.
    void slow_task(void*){
        TickType_t last_wake = xTaskGetTickCount();
        for(;;){
            const std::uint32_t start = millis();
            while(millis() - start < 30){}
            xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
        }
    }
}

int main(){
    uart_puts("Hello from main\n");   // still single-threaded: no lock needed yet

    uart_mutex  = xSemaphoreCreateMutexStatic(&uart_mutex_storage);
    echo_handle = xTaskCreateStatic(echo_task, "echo", kStackWords, nullptr, kEchoPriority, echo_stack, &echo_tcb);
    xTaskCreateStatic(tick_task, "tick", kStackWords, nullptr, kTickPriority, tick_stack, &tick_tcb);
    if(kSlowTaskDemo){
        xTaskCreateStatic(slow_task, "slow", kStackWords, nullptr, kSlowPriority, slow_stack, &slow_tcb);
    }

    vTaskStartScheduler();   // starts SysTick and the first task; never returns
    for(;;){}
}
