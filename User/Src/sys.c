#include "sys.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

/* ARMCC standard-library retargeting; all definitions stay in this file. */
#if defined(__ARMCC_VERSION) && !defined(__MICROLIB)
#pragma import(__use_no_semihosting)
#include <rt_sys.h>
#include <string.h>

struct __FILE { int handle; };
FILE __stdin = { 0 };
FILE __stdout = { 1 };
FILE __stderr = { 2 };
const char __stdin_name[] = "STDIN";
const char __stdout_name[] = "STDOUT";
const char __stderr_name[] = "STDERR";

FILEHANDLE _sys_open(const char *name, int openmode)
{
    (void)openmode;
    if (name == NULL) return -1;
    if (strcmp(name, __stdin_name) == 0) return 0;
    if (strcmp(name, __stdout_name) == 0) return 1;
    if (strcmp(name, __stderr_name) == 0) return 2;
    return -1;
}

int _sys_close(FILEHANDLE fh)
{
    return (fh >= 0 && fh <= 2) ? 0 : -1;
}

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)
{
    unsigned i;
    FILE *stream;
    (void)mode;
    if (fh != 1 && fh != 2) return -1;
    stream = (fh == 1) ? &__stdout : &__stderr;
    for (i = 0; i < len; ++i) {
        if (fputc(buf[i], stream) == EOF) return (int)(len - i);
    }
    return 0; /* ARM ABI: number of bytes NOT written. */
}

int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode)
{
    (void)buf; (void)mode;
    return (fh == 0) ? (int)len : -1; /* No data read: EOF. */
}

int _sys_istty(FILEHANDLE fh)
{
    return (fh >= 0 && fh <= 2);
}

int _sys_seek(FILEHANDLE fh, long pos)
{
    (void)fh; (void)pos;
    return -1;
}

int _sys_ensure(FILEHANDLE fh)
{
    return (fh >= 0 && fh <= 2) ? 0 : -1;
}

long _sys_flen(FILEHANDLE fh)
{
    (void)fh;
    return -1L;
}

int _sys_tmpnam(char *name, int sig, unsigned maxlen)
{
    (void)sig;
    if (name != NULL && maxlen != 0U) name[0] = '\0';
    return 0; /* ARM ABI: zero means failure here. */
}

char *_sys_command_string(char *cmd, int len)
{
    if (cmd == NULL || len <= 0) return NULL;
    cmd[0] = '\0';
    return cmd;
}

void _ttywrch(int ch)
{
    (void)ch; /* Last-resort output not implemented. */
}

void _sys_exit(int returncode)
{
    (void)returncode;
    /* This ABI function must not return. No debugger operation is used. */
    for (;;) { __NOP(); }
}

int fgetc(FILE *stream)
{
    (void)stream;
    return EOF;
}

int ferror(FILE *stream)
{
    (void)stream;
    return 0;
}
#endif /* ARM standard library */

/* TODO: bounded UART output goes here. Do not call printf recursively or
 * depend on the masked SysTick for a transmit timeout. No output backend yet.
 */
int fputc(int ch, FILE *stream)
{
    (void)ch; (void)stream;
    return EOF;
}

#define STARTUP_HCLK_HZ     8000000UL
#define TARGET_HCLK_HZ     72000000UL
#define HSE_TIMEOUT_US       100000UL
#define PLL_TIMEOUT_US        10000UL
#define SWITCH_TIMEOUT_US     10000UL
#define HSI_OFF_TIMEOUT_US    10000UL


static UART_HandleTypeDef huart1;

static const char *s_log_level_str[] = {
    "DEBUG",
    "INFO",
    "WARN",
    "ERROR"
};
#define LOG_BUFFER_SIZE         (128u)

/* These overrides, if present, must also match the CMSIS system file. */
#if defined(HSE_VALUE)
typedef char Hse8MHzRequired[(HSE_VALUE == 8000000UL) ? 1 : -1];
#endif
#if defined(HSI_VALUE)
typedef char Hsi8MHzRequired[(HSI_VALUE == STARTUP_HCLK_HZ) ? 1 : -1];
#endif

/* CYCCNT counts CPU cycles even with PRIMASK set. A loop iteration is NOT
 * one cycle. The poll limit also bounds a wait if CYCCNT stops advancing.
 * All budgets used here are below one 32-bit counter wrap.
 */
static int Clock_WaitBits(volatile uint32_t *reg, uint32_t mask,
                          uint32_t expected, uint32_t hclk_hz,
                          uint32_t timeout_us)
{
    const uint32_t cycle_budget = (hclk_hz / 1000000UL) * timeout_us;
    const uint32_t start = DWT_GetCount();
    uint32_t poll_limit = cycle_budget;

    while ((*reg & mask) != expected) {
        if (DWT_Elapsed(start) >= cycle_budget ||
            poll_limit == 0UL) {
            return 0;
        }
        --poll_limit;
    }
    return 1;
}

HAL_StatusTypeDef system_log_print( LogLevel_t level, const char *file, int line, const char *fmt, ... ) {
    if ( level > LOG_LEVEL_ERROR || !file || !fmt )
        return HAL_ERROR;
    
    char buffer[LOG_BUFFER_SIZE] = { 0 };
    va_list args;
    va_start( args, fmt );

    int len = vsnprintf( buffer, LOG_BUFFER_SIZE, fmt, args );
    if ( len < 0 ) return HAL_ERROR;
    va_end( args );

    printf("[%s] (%s:%d): %s\r\n", s_log_level_str[level], file, line, buffer);
}

SysClockStatus SystemCoreClock_Set_72Mhz(void)
{
    SysClockStatus status = SYS_CLOCK_OK;

    /* Startup only: reject an active PLL, pending switch or divided AHB. */
    if ((RCC->CFGR & (RCC_CFGR_SW | RCC_CFGR_SWS | RCC_CFGR_HPRE)) != 0UL ||
        (RCC->CR & (RCC_CR_HSEON | RCC_CR_PLLON | RCC_CR_CSSON)) != 0UL ||
        (RCC->CR & (RCC_CR_HSION | RCC_CR_HSIRDY)) !=
                   (RCC_CR_HSION | RCC_CR_HSIRDY)) {
        /* 系统时钟设置调用发生在系统正常工况时期,不响应在运行中切频的调用. */
        printf( "[ERROR] Error Param. Files: %s, Lines: %d\n", __FILE__, __LINE__ );
        return SYS_CLOCK_INVALID_STARTUP_STATE;
    }

    /* Increase Flash latency BEFORE raising HCLK; AHB remains /1. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) |
                 FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2) {
        status = SYS_CLOCK_FLASH_ERROR;
        printf( "[ERROR] Flash latency set fail. Files: %s, Lines: %d\n", __FILE__, __LINE__ );
        goto finish;
    }

    /* HSE is off by precondition: crystal mode, not external bypass. */
    RCC->CR &= ~RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    if (!Clock_WaitBits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY,
                        STARTUP_HCLK_HZ, HSE_TIMEOUT_US)) {
        RCC->CR &= ~RCC_CR_HSEON;
        status = SYS_CLOCK_HSE_TIMEOUT;
        printf( "[ERROR] HSE response error. Files: %s, Lines: %d\n", __FILE__, __LINE__ );
        goto finish;
    }
    /* APB1 /2 BEFORE switching: PCLK1 must not exceed 36MHz.
     * Preserve ADC/USB selections and all peripheral clock gates.
     */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 |
                   RCC_CFGR_PPRE2 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                   RCC_CFGR_PLLMULL)) |
                   RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    if (!Clock_WaitBits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY,
                        STARTUP_HCLK_HZ, PLL_TIMEOUT_US)) {
        /* SYSCLK is still HSI. Do not add another unbounded shutdown wait. */
        RCC->CR &= ~RCC_CR_PLLON;
        printf( "[ERROR] PLL response error. Files: %s, Lines: %d\n", __FILE__, __LINE__ );
        status = SYS_CLOCK_PLL_TIMEOUT;
        goto finish;
    }

    /* 切频前等待所有字符发送完毕. 随后读DR清掉对应可能挂起的标志. */
    while( !(CONSOLE_UART_INSTANCE->SR & USART_SR_TC) ) {  }
    (void)CONSOLE_UART_INSTANCE->DR;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    /* HCLK changes here. A 72MHz budget is conservative: at 8MHz it permits
     * up to 9x the nominal duration. Preserve both sources on switch failure.
     */
    if (!Clock_WaitBits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL,
                        TARGET_HCLK_HZ, SWITCH_TIMEOUT_US)) {
        status = SYS_CLOCK_SWITCH_TIMEOUT;
        goto finish;
    }
    __DSB();
    __ISB();

    RCC->CR &= ~RCC_CR_HSION;
    if (!Clock_WaitBits(&RCC->CR, RCC_CR_HSIRDY, 0UL,
                        TARGET_HCLK_HZ, HSI_OFF_TIMEOUT_US)) {
        status = SYS_CLOCK_HSI_OFF_TIMEOUT;
    }

finish:
    /* CMSIS reads the actual source/prescalers without HAL or SysTick. */
    SystemCoreClockUpdate();
    /* Shared DWT counter remains running for other modules. */
    return status;
}


HAL_StatusTypeDef SystemDebugUart_Init( uint32_t baudrate ) {
    if ( baudrate > BAUDRATE_921600 ) 
        return HAL_ERROR;

    uint32_t baud;
    switch (baudrate)
    {
        case BAUDRATE_4800:
            baud = 4800u;
            break;

        case BAUDRATE_9600:
            baud = 9600u;
            break;

        case BAUDRATE_115200:
            baud = 115200u;
            break;

        case BAUDRATE_921600:
            baud = 921600u;
            break;
        
        default: baud = 115200u;   break;
    }

    CONSOLE_UART_IO_PORT_EN();
    GPIO_InitTypeDef init_structure;
    init_structure.Mode = GPIO_MODE_AF_PP;
    init_structure.Pin =  CONSOLE_UART_IO_TX;
    init_structure.Pull = GPIO_PULLUP;
    init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init( CONSOLE_UART_IO_PORT, &init_structure );

    init_structure.Pin = CONSOLE_UART_IO_RX;
    init_structure.Mode = GPIO_MODE_INPUT;
    init_structure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( CONSOLE_UART_IO_PORT, &init_structure );

    CONSOLE_UART_CLK_EN();
    huart1.Instance = CONSOLE_UART_INSTANCE;
    huart1.Init.BaudRate = baud;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;

    if ( HAL_UART_Init( &huart1 ) != HAL_OK ) {
        /* TODO. */
        return HAL_ERROR;
    }

    return HAL_OK;
}


UART_HandleTypeDef *GetConsoleHandle( void ) {
    return &huart1;
}
