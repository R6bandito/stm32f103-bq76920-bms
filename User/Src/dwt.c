#include "dwt.h"

DwtStatus DWT_Init(void)
{
    const uint32_t saved_trace = CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk;
    uint32_t saved_counter;
    uint32_t start;
    volatile uint32_t probe;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    saved_counter = DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0UL) {
        if (saved_trace == 0UL) {
            CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
        }
        return DWT_UNAVAILABLE;
    }

    /* CYCCNT is shared: enable it without resetting any caller's timestamp. */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    start = DWT_GetCount();
    for (probe = 0UL; probe < 32UL; ++probe) {
        /* 让DWT跑几个周期,看看计数是否正常增长. */
        __NOP();
    }
    if (DWT_Elapsed(start) != 0UL) {
        return DWT_OK;
    }

    if (saved_counter == 0UL) {
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    }
    if (saved_trace == 0UL) {
        CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    }
    return DWT_UNAVAILABLE;
}

uint32_t DWT_GetCount(void)
{
    return DWT->CYCCNT;
}

uint32_t DWT_Elapsed(uint32_t start)
{
    return (uint32_t)(DWT_GetCount() - start);
}
