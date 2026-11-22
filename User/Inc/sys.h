#ifndef __SYS_H__
#define __SYS_H__
#include "stm32f1xx.h"
#include "dwt.h"  /* Re-export the shared cycle-counter API. */

typedef enum {
    SYS_CLOCK_OK = 0,
    SYS_CLOCK_INVALID_STARTUP_STATE,
    SYS_CLOCK_COUNTER_UNAVAILABLE,
    SYS_CLOCK_FLASH_ERROR,
    SYS_CLOCK_HSE_TIMEOUT,
    SYS_CLOCK_PLL_TIMEOUT,
    SYS_CLOCK_SWITCH_TIMEOUT,
    SYS_CLOCK_HSI_OFF_TIMEOUT
} SysClockStatus;

typedef enum {
    BAUDRATE_4800 = 0,
    BAUDRATE_9600,
    BAUDRATE_115200,
    BAUDRATE_921600,
} BaudRate_t;

// 日志级别
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} LogLevel_t;

#define CONSOLE_UART_INSTANCE       (USART1)
#define CONSOLE_UART_IO_PORT        (GPIOA)
#define CONSOLE_UART_IO_RX          (GPIO_PIN_10)
#define CONSOLE_UART_IO_TX          (GPIO_PIN_9)
#define CONSOLE_UART_CLK_EN()       do { __HAL_RCC_USART1_CLK_ENABLE(); } while(0)
#define CONSOLE_UART_IO_PORT_EN()   do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while(0)

/* 在 main 中完成邮箱初始化后，线程才能提交日志。 */
HAL_StatusTypeDef SystemConsoleMailboxInit( void );
HAL_StatusTypeDef system_log_print( LogLevel_t level, const char *file, int line, const char *fmt, ... );
#define LOG_INFO(fmt, ...)  system_log_print(LOG_LEVEL_INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)  system_log_print(LOG_LEVEL_DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  system_log_print(LOG_LEVEL_WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)  system_log_print(LOG_LEVEL_ERROR,  __FILE__, __LINE__, fmt, ##__VA_ARGS__)
 
/* Startup only: HSI 8MHz, AHB /1, HSE/PLL/CSS off; external 8MHz crystal.
 * No HAL, SysTick or interrupt dependency. Does not configure SysTick.
 * Failures do not promise full rollback; caller must handle non-OK results.
 */
SysClockStatus SystemCoreClock_Set_72Mhz(void);

HAL_StatusTypeDef SystemDebugUart_Init( uint32_t baudrate );
UART_HandleTypeDef *GetConsoleHandle( void );
#endif
