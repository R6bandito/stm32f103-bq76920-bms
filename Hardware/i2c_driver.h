#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__

#include "stm32f1xx_hal.h"
#include "dwt.h"

#define I2C_SDA_PORT        (GPIOB)
#define I2C_SDA_PIN         (GPIO_PIN_13)
#define I2C_SCL_PORT        (GPIOB)
#define I2C_SCL_PIN         (GPIO_PIN_14)
#define I2C_IDLE            (1u)
#define I2C_BUSY            (0u)

/* Open drain: UP releases the line; IDR reports its actual level. */
#define SDA_UP()            do { I2C_SDA_PORT->BSRR = (uint32_t)I2C_SDA_PIN; } while(0)
#define SDA_LOW()           do { I2C_SDA_PORT->BSRR = ((uint32_t)I2C_SDA_PIN << 16u); } while(0)
#define SCL_UP()            do { I2C_SCL_PORT->BSRR = (uint32_t)I2C_SCL_PIN; } while(0)
#define SCL_LOW()           do { I2C_SCL_PORT->BSRR = ((uint32_t)I2C_SCL_PIN << 16u); } while(0)
#define SCL_READ()          ((I2C_SCL_PORT->IDR & I2C_SCL_PIN) ? 1u : 0u)
#define SDA_READ()          ((I2C_SDA_PORT->IDR & I2C_SDA_PIN) ? 1u : 0u)

#define I2C_SDA_PORT_CLK_EN()   do { __HAL_RCC_GPIOB_CLK_ENABLE(); } while(0)
#define I2C_SCL_PORT_CLK_EN()   do { __HAL_RCC_GPIOB_CLK_ENABLE(); } while(0)

typedef enum {
    IIC_STATE_EPARAM = 0,
    IIC_STATE_BUSY,
    IIC_STATE_NORESPONSE,
    IIC_STATE_EOK,
    IIC_STATE_TIMEOUT,
    IIC_STATE_NOT_READY,
} I2C_State_t;


void HX_I2C_Init( void );
/* Single controller/caller, thread context only. Requires a running DWT and
 * stable SystemCoreClock. timeouts_ms is the total budget of ONE API call,
 * must be nonzero and fit in one 32-bit DWT interval. Failed transfers may add
 * up to 1ms of STOP cleanup with a working DWT. No bus-clear or arbitration.
 * Compare explicitly with IIC_STATE_EOK (the existing success value is not 0).
 */
/* START + 7-bit address (rw: 0=write, 1=read) + ACK. Success holds the bus;
 * another Start_Send before Stop produces a repeated START. Transfer errors
 * attempt STOP/release. Public API errors (including bad arguments) abort an
 * active transaction; before START they return without touching the bus.
 */
I2C_State_t I2C_Start_Send( uint32_t devAddr, uint8_t rw, uint32_t timeouts_ms );

/* After Start_Send succeeds in WRITE direction: eight data bits + one ACK. */
I2C_State_t I2C_Write_Byte( uint8_t byte, uint32_t timeouts_ms );

/* Complete a transaction. If none is active, only release/check idle. */
I2C_State_t I2C_Stop( uint32_t timeouts_ms );

/* Receive eight bits only; call Send_ACK_NACK for the ninth clock.
 * Requires an active read transaction; *out is updated only on success. */
I2C_State_t I2C_Read_Byte( uint8_t *out, uint32_t timeouts_ms );

/* Public ACK flag: ACK >= 1 sends ACK; ACK == 0 sends NACK.
 * This is opposite to the internal primitive's nack flag. Both successful
 * responses return IIC_STATE_EOK. An active transaction is required. */
I2C_State_t I2c_Send_ACK_NACK( uint8_t ACK, uint32_t timeout_ms );

#endif /* __I2C_DRIVER_H__ */

