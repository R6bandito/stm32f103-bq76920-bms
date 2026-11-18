#ifndef __DWT_H__
#define __DWT_H__

#include "stm32f1xx.h"

typedef enum {
    DWT_OK = 0,
    DWT_UNAVAILABLE
} DwtStatus;

/* Call during startup before concurrent use. Success leaves CYCCNT running.
 * Repeated calls never reset CYCCNT; failure restores the original enable bits.
 */
DwtStatus DWT_Init(void);

/* Requires successful DWT_Init(). Units: CPU cycles, not microseconds. */
uint32_t DWT_GetCount(void);

/* Unsigned difference from start to now. Valid for intervals < 2^32 cycles.
 * Clock changes affect conversion to time; debug halt/sleep may stop counting.
 */
uint32_t DWT_Elapsed(uint32_t start);

#endif /* __DWT_H__ */
