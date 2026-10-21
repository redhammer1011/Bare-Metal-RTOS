/* FreeRTOSConfig.h - kernel configuration for this firmware (Cortex-M3, MPS2-AN385)
 *
 * Included by the FreeRTOS kernel's C sources and by our C++ code, so it must
 * stay valid C. Every option the kernel reads is listed here; anything not set
 * takes the default from FreeRTOS.h.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ---------------------------------------------------------------- scheduling */
#define configUSE_PREEMPTION                     1   /* higher-priority ready task runs immediately */
#define configUSE_TIME_SLICING                   1   /* equal-priority tasks share the CPU tick by tick */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1   /* pick next task with the CLZ instruction (max 32 priorities) */
#define configMAX_PRIORITIES                     5   /* 0 = idle ... 4 = highest */
#define configIDLE_SHOULD_YIELD                  1

/* -------------------------------------------------------------- clock / tick */
#define configCPU_CLOCK_HZ                       25000000UL   /* MPS2-AN385 core clock */
#define configTICK_RATE_HZ                       1000         /* 1 tick = 1 ms */
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS

/* ------------------------------------------------------------------- memory */
/* Static allocation only: every task stack and kernel object is a global the
 * linker places in .bss, so RAM use is fixed at build time and there is no
 * heap to fragment or exhaust. The kernel supplies the idle task's memory. */
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         0
#define configKERNEL_PROVIDED_STATIC_MEMORY      1
#define configMINIMAL_STACK_SIZE                 128          /* in words (4 bytes each): idle task stack */
#define configMAX_TASK_NAME_LEN                  12

/* ----------------------------------------------------------------- features */
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            0
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_TIMERS                         0
#define configUSE_CO_ROUTINES                    0
#define configUSE_TRACE_FACILITY                 0
#define configGENERATE_RUN_TIME_STATS            0
#define configQUEUE_REGISTRY_SIZE                0

/* -------------------------------------------------------------------- hooks */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      1   /* vApplicationTickHook() drives millis() */
#define configCHECK_FOR_STACK_OVERFLOW           2   /* check stack pointer and a guard pattern on every switch */

/* --------------------------------------------------------- interrupt priority
 * Cortex-M priorities are "lower number = more urgent", stored in the top bits
 * of each 8-bit priority register. QEMU's NVIC implements all 8 bits (measured
 * by writing 0xFF to a priority register and reading it back); real Cortex-M3
 * parts usually implement 3 or 4.
 *
 *   0x00 ... 0x9F   above the kernel: never masked by FreeRTOS critical
 *                   sections, but must NOT call any FreeRTOS API
 *   0xA0 ... 0xFF   may call ...FromISR() APIs; masked inside kernel critical sections
 *   0xFF            the kernel's own SysTick and PendSV (least urgent)
 *
 * With 8 implemented bits the lowest bit is a sub-priority, so the syscall
 * threshold must be even (the port asserts this). */
#define configPRIO_BITS                          8
#define configKERNEL_INTERRUPT_PRIORITY          0xFF
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     0xA0

/* -------------------------------------------------------------- API to build */
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskSuspend                     1   /* lets portMAX_DELAY mean "wait forever" */
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetSchedulerState           1

/* ---------------------------------------------------------------- assertion */
#ifdef __cplusplus
extern "C" {
#endif
void freertos_assert_failed(const char* file, int line);
#ifdef __cplusplus
}
#endif
#define configASSERT(x)  do { if(!(x)) { freertos_assert_failed(__FILE__, __LINE__); } } while(0)

/* ---------------------------------------------------- exception handler names
 * The port implements the three exceptions FreeRTOS needs. Renaming them to the
 * names in our vector table makes them override startup.cpp's weak aliases. */
#define vPortSVCHandler      SVC_Handler       /* starts the first task */
#define xPortPendSVHandler   PendSV_Handler    /* context switch */
#define xPortSysTickHandler  SysTick_Handler   /* kernel tick */

#endif /* FREERTOS_CONFIG_H */
