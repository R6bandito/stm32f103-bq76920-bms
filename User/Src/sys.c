#include "sys.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "rtthread.h"

/* Reject any Arm C library dependency that needs a debugger host call. */
#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)

/* ARMCC startup/exit ABI. This terminal handler only executes locally. */
void _sys_exit(int returncode)
{
    (void)returncode;
    for (;;) {
        __NOP();
    }
}
#endif

#define STARTUP_HCLK_HZ     8000000UL
#define TARGET_HCLK_HZ     72000000UL
#define HSE_TIMEOUT_US       100000UL
#define PLL_TIMEOUT_US        10000UL
#define SWITCH_TIMEOUT_US     10000UL
#define HSI_OFF_TIMEOUT_US    10000UL
#define LOG_BUFFER_SIZE       128u
#define LOG_SLOT_COUNT        2u
#define LOG_MB_WAIT_MS        50u
#define LOG_TX_TIMEOUT_MS     50u

static UART_HandleTypeDef huart1;
static rt_mailbox_t mb_freeBox;
static rt_mailbox_t mb_messageReq;

static uint8_t TxBuff_1[LOG_BUFFER_SIZE];
static uint8_t TxBuff_2[LOG_BUFFER_SIZE];

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

HAL_StatusTypeDef SystemConsoleMailboxInit( void ) {
    rt_mailbox_t free_box;
    rt_mailbox_t message_req;

    if (mb_freeBox != RT_NULL && mb_messageReq != RT_NULL)
        return HAL_OK;

    free_box = rt_mb_create("freebox", LOG_SLOT_COUNT, RT_IPC_FLAG_PRIO);
    if (free_box == RT_NULL)
        return HAL_ERROR;

    message_req = rt_mb_create("messagebox", LOG_SLOT_COUNT, RT_IPC_FLAG_PRIO);
    if (message_req == RT_NULL) {
        rt_mb_delete(free_box);
        return HAL_ERROR;
    }

    if (rt_mb_send(free_box, 1u) != RT_EOK || rt_mb_send(free_box, 2u) != RT_EOK) {
        rt_mb_delete(message_req);
        rt_mb_delete(free_box);
        return HAL_ERROR;
    }

    mb_freeBox = free_box;
    mb_messageReq = message_req;
    return HAL_OK;
}

static void mailbox_release( rt_ubase_t index ) {
    if (index != 1u && index != 2u)
        return;
    rt_mb_send(mb_freeBox, index);
}

/* Blocking register TX for the dedicated USART1 worker. */
static HAL_StatusTypeDef ConsoleSend(const char *data, uint32_t len)
{
    USART_TypeDef *uart = CONSOLE_UART_INSTANCE;
    const uint32_t timeout = (SystemCoreClock / 1000u) * LOG_TX_TIMEOUT_MS;
    const uint32_t start = DWT_GetCount();
    uint32_t polls_left = timeout;
    uint32_t i;
    if ((RCC->APB2ENR & RCC_APB2ENR_USART1EN) == 0u ||      /* 串口未正确初始化. 返回. */
        (uart->CR1 & (USART_CR1_UE | USART_CR1_TE)) != (USART_CR1_UE | USART_CR1_TE)) {
        return HAL_ERROR;
    }

    for (i = 0u; i <= len; ++i) {
        const uint32_t flag = (i < len) ? USART_SR_TXE : USART_SR_TC;
        while ((uart->SR & flag) == 0u) {
            if (DWT_Elapsed(start) >= timeout || polls_left == 0u)
                return HAL_TIMEOUT;
            --polls_left;
        }
        if (i < len)
            uart->DR = (uint8_t)data[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef system_log_print( LogLevel_t level, const char *file, int line, const char *fmt, ... ) {
    static const char *const level_name[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    uint32_t len;
    int written;
    va_list args;
    rt_ubase_t box;
    rt_err_t stat;
    uint8_t *buffer;
    rt_uint32_t message;
    const uint32_t text_capacity = LOG_BUFFER_SIZE - 2u; /* Includes the text NUL. */

    if ((unsigned)level >= sizeof(level_name) / sizeof(level_name[0]) || !file || !fmt ||
        mb_freeBox == RT_NULL || mb_messageReq == RT_NULL)
        return HAL_ERROR;

    stat = rt_mb_recv(mb_freeBox, &box, rt_tick_from_millisecond(LOG_MB_WAIT_MS));
    if (stat != RT_EOK)
        return (stat == -RT_ETIMEOUT) ? HAL_TIMEOUT : HAL_ERROR;
    if (box == 1u)
        buffer = TxBuff_1;
    else if (box == 2u)
        buffer = TxBuff_2;
    else
        return HAL_ERROR;

    written = snprintf((char *)buffer, text_capacity, "[%s] (%s:%d): ", level_name[level], file, line);
    if (written < 0)
        goto release_slot;
    len = ((uint32_t)written < text_capacity) ? (uint32_t)written : text_capacity - 1u;

    va_start(args, fmt);
    written = vsnprintf((char *)(buffer + len), text_capacity - len, fmt, args);
    va_end(args);
    if (written < 0)
        goto release_slot;
    len += ((uint32_t)written < text_capacity - len) ? (uint32_t)written : text_capacity - len - 1u;
    buffer[len++] = '\r';
    buffer[len++] = '\n';
    buffer[len] = '\0';

    message = ((rt_uint32_t)(box << 16u) | len);
    stat = rt_mb_send(mb_messageReq, message);
    if (stat != RT_EOK)
        goto release_slot;
    return HAL_OK;

release_slot:
    mailbox_release(box);
    return HAL_ERROR;
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
        return SYS_CLOCK_INVALID_STARTUP_STATE;
    }

    /* Increase Flash latency BEFORE raising HCLK; AHB remains /1. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) |
                 FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2) {
        status = SYS_CLOCK_FLASH_ERROR;
        goto finish;
    }

    /* HSE is off by precondition: crystal mode, not external bypass. */
    RCC->CR &= ~RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    if (!Clock_WaitBits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY,
                        STARTUP_HCLK_HZ, HSE_TIMEOUT_US)) {
        RCC->CR &= ~RCC_CR_HSEON;
        status = SYS_CLOCK_HSE_TIMEOUT;
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
