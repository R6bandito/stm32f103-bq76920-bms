#include "AFE_Core.h"
#include "bq76920_driver.h"
#include "rtthread.h"

typedef struct {
    uint16_t adc_gain_uv;   /* 公共电压增益，365～396 µV/LSB。 */
    int8_t adc_offset_mv;   /* ADCOFFSET按8位补码解释，-128～127 mV，不能使用无符号类型。 */
    uint8_t calibration_valid; /* 软件标志，没有对应寄存器；读取、CRC与解码完成后置1。 */
} AFE_Param;
static AFE_Param AFEParamPack;

/* 内部自然对数ln(x)，调用者保证x为有限正数。
 * x = m * 2^power2，将m缩放到[1, 2)，于是ln(x)=ln(m)+power2*ln(2)。
 * z=(m-1)/(m+1)，ln(m)=2*(z+z^3/3+z^5/5+...)，固定取8项。
 * 归一化最多160轮，覆盖本平台32位float的有限正数范围。
 */
static float afe_ln(float x) {
    int power2 = 0;
    uint16_t i;
    float z;
    float z2;
    float term;
    float sum;

    for (i = 0u; i < 160u; ++i) {
        if (x >= 2.0f) {
            x *= 0.5f;
            ++power2;
        } else if (x < 1.0f) {
            x *= 2.0f;
            --power2;
        } else {
            break;
        }
    }

    z = (x - 1.0f) / (x + 1.0f);
    z2 = z * z;
    term = z;
    sum = term;
    for (i = 1u; i < 8u; ++i) {
        term *= z2;
        sum += term / (float)(2u * i + 1u);
    }
    return 2.0f * sum + (float)power2 * 0.69314718056f;
}

static AFEState_t afe_err_handle(BqStatus state) {
    AFEState_t afe_s;
    switch (state)
    {
        case BQ_EPARAM:
            afe_s = AFE_EPARAM;
            break;
        case BQ_EI2C:
            afe_s = AFE_EI2C;
            break;
        case BQ_ECRC:
            afe_s = AFE_ECRC;
            break;
        default:  afe_s = AFE_ENOTREADY;   break;
    }
    return afe_s;
}

AFEState_t AFE_Init(uint32_t timeout_ms) {
    AFEParamPack.calibration_valid = 0u; /* 重初始化失败时，旧参数不能继续被当作有效。 */
    HX_I2C_Init();
    AFE_TS1_PORT_CLK_EN();
    AFE_TS1_PIN_PORT->BSRR = (uint32_t)AFE_TS1_PIN << 16u;  /* 先预置将TS1拉到低. */
    GPIO_InitTypeDef initStructure;
    initStructure.Mode = GPIO_MODE_OUTPUT_PP;
    initStructure.Pin = AFE_TS1_PIN;
    initStructure.Pull = GPIO_PULLDOWN;
    initStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AFE_TS1_PIN_PORT, &initStructure);
    rt_thread_mdelay(1);    
    AFE_TS1_PIN_PORT->BSRR = (uint32_t)AFE_TS1_PIN;         /* 输出约5ms高电平. */
    rt_thread_mdelay(5);

    initStructure.Mode = GPIO_MODE_INPUT;  /* 脉冲结束. 切换为无上下拉输入. */
    initStructure.Pin = AFE_TS1_PIN;
    initStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AFE_TS1_PIN_PORT, &initStructure);
    rt_thread_mdelay(20);

    uint8_t rcv;
    uint8_t fault;
    BqStatus state;
    uint8_t gain = 0;
    int8_t offset = 0;
    AFE_Param param = {0};
    state = BQ76920_ReadReg(BQ76920_REG_SYS_STAT, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    fault = (rcv & BQ76920_SYS_STAT_FAULT_MASK);
    if (fault != 0) {
        /* 读出来的AFE状态出现错误. */
        if (fault & BQ76920_SYS_STAT_DEVICE_XREADY) {
            /* TODO. */
            return AFE_ENOTREADY;
        }
        else if (fault & BQ76920_SYS_STAT_OVRD_ALERT) {
            /* TODO. */
            return AFE_ENOTREADY;
        }
    }

    /* AFE可能没有随MCU一起复位，先明确关闭充放电与均衡。 */
    state = BQ76920_WriteReg(BQ76920_REG_SYS_CTRL2, 0x00u, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    state = BQ76920_ReadReg(BQ76920_REG_SYS_CTRL2, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    if ((rcv & 0xE3u) != 0x00u) /* DELAY_DIS、CC_EN、CC_ONESHOT、DSG/CHG。 */
        return AFE_ECONFIG;

    state = BQ76920_WriteReg(BQ76920_REG_CELLBAL1, 0x00u, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    state = BQ76920_ReadReg(BQ76920_REG_CELLBAL1, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    if ((rcv & 0x1Fu) != 0x00u) /* 仅检查电芯1～5的均衡使能位。 */
        return AFE_ECONFIG;

    state = BQ76920_WriteReg(BQ76920_REG_CC_CFG, 0x19u, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    rt_thread_mdelay(1);    /* 延时1ms时长供AFE响应. */
    state = BQ76920_ReadReg(BQ76920_REG_CC_CFG, &rcv, timeout_ms);  /* 回读验证. */
    if (state != BQ_EOK)
        return afe_err_handle(state);
    if (rcv != 0x19u) 
        return AFE_ECONFIG; /* 回读不匹配. */
    
    state = BQ76920_ReadReg(BQ76920_REG_ADCGAIN2, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    gain = (uint8_t)((rcv & 0xE0u) >> 5u); /* ADCGAIN2[7:5] → gain[2:0]。 */
    state = BQ76920_ReadReg(BQ76920_REG_ADCGAIN1, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    gain |= (uint8_t)((rcv & 0x0Cu) << 1u); /* ADCGAIN1[3:2] → gain[4:3]。 */

    state = BQ76920_ReadReg(BQ76920_REG_ADCOFFSET, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    offset = (int8_t)rcv; /* ADCOFFSET是有符号补码，例如0xFF表示-1mV。 */
    
    param.adc_gain_uv = (uint16_t)(365u + gain);
    param.adc_offset_mv = offset;

    /* 开启电压ADC并选择外部NTC；不执行SHIP关闭序列。 */
    state = BQ76920_WriteReg(BQ76920_REG_SYS_CTRL1, 0x18u, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    state = BQ76920_ReadReg(BQ76920_REG_SYS_CTRL1, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    if ((rcv & 0x1Bu) != 0x18u) /* 检查ADC_EN、TEMP_SEL及SHUT_A/B，排除只读bit7。 */
        return AFE_ECONFIG;

    /* 清掉旧的电流就绪标志，保留其他故障记录，再开启连续CC。 */
    state = BQ76920_WriteReg(BQ76920_REG_SYS_STAT,
                            BQ76920_SYS_STAT_CC_READY, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    state = BQ76920_WriteReg(BQ76920_REG_SYS_CTRL2, 0x40u, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    state = BQ76920_ReadReg(BQ76920_REG_SYS_CTRL2, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    if ((rcv & 0xE3u) != 0x40u) /* 连续CC开启，测试旁路/单次命令/MOS均关闭。 */
        return AFE_ECONFIG;

    /* 配置后再次检查内部异常/ALERT覆盖；不批量清除OV/UV/SCD/OCD。 */
    state = BQ76920_ReadReg(BQ76920_REG_SYS_STAT, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    if (rcv & (BQ76920_SYS_STAT_DEVICE_XREADY | BQ76920_SYS_STAT_OVRD_ALERT))
        return AFE_ENOTREADY;

    /* 全部配置验证成功再提交缓存；首轮数据就绪由后续采集接口判断。 */
    param.calibration_valid = 1u;
    AFEParamPack = param;
    return AFE_EOK;
}

AFEState_t AFE_GetStatus(uint8_t *status, uint32_t timeout_ms) {
    if (status == NULL)
        return AFE_EPARAM;
    BqStatus state;
    uint8_t rcv;
    state = BQ76920_ReadReg(BQ76920_REG_SYS_STAT, &rcv, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    *status = rcv;
    return AFE_EOK;
}

AFEState_t AFE_ReadOutputState(uint8_t *sys_ctrl2, uint8_t *cellbal1, uint32_t timeout_ms) {
    BqStatus state;
    uint8_t ctrl;
    uint8_t balance;
    if (sys_ctrl2 == NULL || cellbal1 == NULL)
        return AFE_EPARAM;

    state = BQ76920_ReadReg(BQ76920_REG_SYS_CTRL2, &ctrl, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    state = BQ76920_ReadReg(BQ76920_REG_CELLBAL1, &balance, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);

    *sys_ctrl2 = ctrl;
    *cellbal1 = balance;
    return AFE_EOK;
}

AFEState_t AFE_GetCellVoltage(CellIndex_t cell_index, uint16_t *voltage_mv, uint32_t timeout_ms) {
    if ((uint32_t)cell_index >= AFE_CELL_COUNT || voltage_mv == NULL)
        return AFE_EPARAM;
    if (!AFEParamPack.calibration_valid)
        return AFE_ENOTREADY;
    BqStatus state;
    uint8_t raw[2];
    uint16_t voltage_raw;
    int32_t mv;
    uint8_t first_reg = (uint8_t)(BQ76920_REG_VC1_HI + ((uint32_t)cell_index * 2u));
    state = BQ76920_ReadRegs(first_reg, raw, 2, timeout_ms);    
    if (state != BQ_EOK)
        return afe_err_handle(state);
    voltage_raw = (((uint16_t)(raw[0] & 0x3Fu) << 8u) | raw[1]);    /* 电压采样原生值拼接. */
    mv = ((int32_t)voltage_raw * AFEParamPack.adc_gain_uv) / 1000 + AFEParamPack.adc_offset_mv; /* 电压换算. */
    if (mv < 0)
        return AFE_ENOTREADY;   /* 防止负电压转为uint后溢出. */
    *voltage_mv = mv;
    return AFE_EOK;
}

AFEState_t AFE_GetCurrent(int32_t *current_ma, uint32_t timeout_ms) {
    if (current_ma == NULL) 
        return AFE_EPARAM;
    if (!AFEParamPack.calibration_valid)
        return AFE_ENOTREADY;
    BqStatus state;
    uint8_t first_reg = BQ76920_REG_CC_HI;
    uint8_t raw[2];
    int16_t current_raw;
    int32_t current;
    state = BQ76920_ReadRegs(first_reg, raw, 2, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    current_raw = (int16_t)(((uint16_t)raw[0] << 8u) | (uint16_t)raw[1]);
    current = ((int32_t)current_raw * 1688) / 1000;     /* 电流换算. */
    *current_ma = current;
    return AFE_EOK;
}

AFEState_t AFE_GetTemperature(int16_t *temperature_decic, uint32_t timeout_ms) {
    if (temperature_decic == NULL)
        return AFE_EPARAM;
    if (!AFEParamPack.calibration_valid)
        return AFE_ENOTREADY;
    BqStatus state;
    uint8_t first_reg = BQ76920_REG_TS1_HI;
    uint8_t raw[2];
    uint16_t temp_raw;
    float Vts;
    float Rntc;
    float TempC;
    float TempDecic;
    state = BQ76920_ReadRegs(first_reg, raw, 2, timeout_ms);
    if (state != BQ_EOK)
        return afe_err_handle(state);
    temp_raw = (uint16_t)(((uint16_t)(raw[0] & 0x3Fu) << 8u) | raw[1]);
    Vts = (float)temp_raw * AFE_EXT_NTC_GAIN;
    if (Vts <= 0.0f || Vts >= 3.3f) 
        return AFE_ENOTREADY;   /* 端点无效分压. */
    Rntc = (Vts * 10000.0f / (3.3f - Vts));
    TempC = 1.0f / (1.0f / 298.15f + afe_ln(Rntc / 10000.0f) / 3950.0f) - 273.15f;
    TempDecic = TempC * 10.0f;
    if (!(TempDecic >= -32768.0f && TempDecic <= 32767.0f))
        return AFE_ENOTREADY;
    *temperature_decic = (int16_t)(TempDecic + (TempDecic >= 0.0f ? 0.5f : -0.5f));
    return AFE_EOK;
}
