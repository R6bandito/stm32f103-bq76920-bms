#include "bq76920_driver.h"
#include "i2c_driver.h"

#define BQ76920_CRC8_POLY  0x07u /* x^8 + x^2 + x + 1; x^8 is implicit. */

/* BQ76920 CRC-8: initial value 0, MSB first, no reflection/final XOR.
 * data must point to data_len readable bytes; NULL is allowed only for len=0.
 * Each call starts a NEW CRC calculation. The future register layer chooses
 * which address/register/data bytes belong to each CRC according to the BQ
 * protocol; this arithmetic helper does not add any protocol bytes itself.
 */
static uint8_t bq76920_crc8(const uint8_t *data, uint8_t data_len)
{
    uint8_t crc = 0u;
    uint8_t i;
    uint8_t bit;

    for (i = 0u; i < data_len; ++i) {
        crc ^= data[i];                 /* Merge the next byte into remainder. */
        for (bit = 0u; bit < 8u; ++bit) {
            if (crc & 0x80u) {
                /* High bit was 1: shift, then XOR the polynomial's low bits. */
                crc = (uint8_t)((crc << 1u) ^ BQ76920_CRC8_POLY);
            } else {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return crc;
}

BqStatus BQ76920_ReadRegs(uint8_t first_reg, uint8_t *data, uint8_t data_len, uint32_t timeout_ms) {
    if(data == NULL || data_len == 0)
        return BQ_EPARAM;
    uint8_t i;
    uint8_t tmp;
    uint8_t rcv_crc;
    uint8_t fix_crc;
    I2C_State_t state;

    state = I2C_Start_Send(BQ769200_ADDRESS_7, 0, timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    state = I2C_Write_Byte(first_reg, timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    state = I2C_Start_Send(BQ769200_ADDRESS_7, 1, timeout_ms);  /* 重复起始条件. */
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    
    for(i = 0; i < data_len; i++) {
        state = I2C_Read_Byte(&tmp, timeout_ms);
        if (state != IIC_STATE_EOK)
            return BQ_EI2C;
        state = I2c_Send_ACK_NACK(1, timeout_ms);
        if (state != IIC_STATE_EOK)
            return BQ_EI2C;
        state = I2C_Read_Byte(&rcv_crc, timeout_ms);
        if (state != IIC_STATE_EOK)
            return BQ_EI2C;
        
        if (i == 0u) {
            uint8_t crc_data[2] = {
                (uint8_t)((BQ769200_ADDRESS_7 << 1u) | 1u),
                tmp
            };
            fix_crc = bq76920_crc8(crc_data, 2u);
        } else {
            fix_crc = bq76920_crc8(&tmp, 1u);
        }

        /* 先校验，再决定CRC字节的第九拍；错误或最后一组都回NACK。 */
        if (fix_crc != rcv_crc || i == (data_len - 1))
            state = I2c_Send_ACK_NACK(0, timeout_ms);
        else 
            state = I2c_Send_ACK_NACK(1, timeout_ms);
        if (state != IIC_STATE_EOK)
            return BQ_EI2C;
        if (fix_crc != rcv_crc) {   /* I2C本身成功，CRC错误由BQ层主动收尾。 */
            state = I2C_Stop(timeout_ms);
            if (state != IIC_STATE_EOK)
                return BQ_EI2C;
            return BQ_ECRC;
        }
        data[i] = tmp;
    }
    state = I2C_Stop(timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    return BQ_EOK;
}

BqStatus BQ76920_ReadReg(uint8_t reg, uint8_t *value, uint32_t timeout_ms) {
    if (value == NULL)
        return BQ_EPARAM;
    return BQ76920_ReadRegs(reg, value, 1u, timeout_ms);
}

BqStatus BQ76920_WriteReg(uint8_t reg, uint8_t value, uint32_t timeout_ms) {
    I2C_State_t state;
    state = I2C_Start_Send(BQ769200_ADDRESS_7, 0, timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    state = I2C_Write_Byte(reg, timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    state = I2C_Write_Byte(value, timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;

    uint8_t crc_data[3] = {     /* 发送CRC. */
        (uint8_t)(BQ769200_ADDRESS_7 << 1u),
        reg,
        value
    };
    uint8_t crc = bq76920_crc8(crc_data, 3u);
    state = I2C_Write_Byte(crc, timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    
    state = I2C_Stop(timeout_ms);
    if (state != IIC_STATE_EOK)
        return BQ_EI2C;
    return BQ_EOK;
}
