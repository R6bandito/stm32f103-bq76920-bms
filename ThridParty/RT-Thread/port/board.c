/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-24                  the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "sys.h"
#include "stm32f1xx_hal.h"

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
/*
 * Please modify RT_HEAP_SIZE if you enable RT_USING_HEAP
 * the RT_HEAP_SIZE max value = (sram size - ZI size), 1024 means 1024 bytes
 */
#define RT_HEAP_SIZE (15*1024)
static rt_uint8_t rt_heap[RT_HEAP_SIZE];

RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

void rt_os_tick_callback(void)
{
    rt_interrupt_enter();
    
    rt_tick_increase();

    rt_interrupt_leave();
}

/**
 * This function will initial your board.
 */
void rt_hw_board_init(void)
{
    HAL_StatusTypeDef res;
    DwtStatus dwt_status;
    SysClockStatus clock_status;
    HAL_Init();
    dwt_status = DWT_Init();
    if ( dwt_status != DWT_OK ) {
        /* TODO. */
    }

    res = SystemDebugUart_Init( BAUDRATE_115200 );
    if ( res != HAL_OK ) {

    }

    clock_status = SystemCoreClock_Set_72Mhz();
    if ( clock_status != SYS_CLOCK_OK ) {
        while(1);
    }
    HAL_UART_DeInit( GetConsoleHandle() );
    res = SystemDebugUart_Init( BAUDRATE_115200 );
    if ( res != HAL_OK ) {
        
    }
    /* HAL_Init configured the tick at HSI 8MHz. Refresh it after switching.
     * RT_TICK_PER_SECOND=1000: HCLK=72MHz, LOAD=71999, HAL tick remains 1ms.
     */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
#ifdef RT_USING_COMPONENTS_INIT
    /* Board hooks may allocate memory, so initialize the heap first. */
    rt_components_board_init();
#endif
}

#ifdef RT_USING_CONSOLE

static int uart_init(void)
{
#error "TODO 2: Enable the hardware uart and config baudrate."
    return 0;
}
INIT_BOARD_EXPORT(uart_init);

void rt_hw_console_output(const char *str)
{
#error "TODO 3: Output the string 'str' through the uart."
}

#endif

