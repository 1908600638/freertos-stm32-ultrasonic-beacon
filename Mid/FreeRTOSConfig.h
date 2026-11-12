#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 6 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 17 * 1024 ) )
#define configMAX_TASK_NAME_LEN		( 16 )
#define configUSE_TRACE_FACILITY	1
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1

#define configUSE_STATS_FORMATTING_FUNCTIONS	1
#define configCHECK_FOR_STACK_OVERFLOW	        2
#define configGENERATE_RUN_TIME_STATS	        1

/* DWT cycle counter: 72MHz, delta calc handles wrap */
#define portDWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define portDEMCR       (*(volatile uint32_t *)0xE000EDFC)
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  do { \
    portDEMCR |= (1UL << 24);                              \
    portDWT_CYCCNT = 0;                                    \
    *(volatile uint32_t *)0xE0001000 |= (1UL << 0);       \
} while(0)
#define portGET_RUN_TIME_COUNTER_VALUE()  portDWT_CYCCNT

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetSchedulerState       1

#define configKERNEL_INTERRUPT_PRIORITY 		255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler

#endif
