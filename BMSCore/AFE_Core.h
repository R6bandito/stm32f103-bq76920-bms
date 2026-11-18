#ifndef AFE_CORE_H
#define AFE_CORE_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AFE_CELL_COUNT          (5u)
#define AFE_EXT_NTC_GAIN        (0.000382f) /* 382uV/LSB */

#define AFE_TS1_PIN_PORT        (GPIOA)
#define AFE_TS1_PIN             (GPIO_PIN_15)
#define AFE_TS1_PORT_CLK_EN()   do { __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                    __HAL_RCC_AFIO_CLK_ENABLE();   \
                                    __HAL_AFIO_REMAP_SWJ_NOJTAG(); \
                                 } while(0) 

/* API执行结果，不是AFE运行模式；与底层BqStatus显式转换。 */
typedef enum AFEState_t {
    AFE_EOK = 0,
    AFE_EPARAM,       /* 参数无效 */
    AFE_EI2C,         /* 底层通信失败 */
    AFE_ECRC,         /* 接收数据CRC不匹配 */
    AFE_ETIMEOUT,     /* 等待启动/转换等操作超时 */
    AFE_ECONFIG,      /* 配置回读不匹配 */
    AFE_ENOTREADY,    /* 尚未初始化或数据未就绪 */
    AFE_ENOTIMPL      /* 当前接口仅为骨架 */
} AFEState_t;

typedef enum {
    CELL_INDEX_1 = 0,
    CELL_INDEX_2,
    CELL_INDEX_3,
    CELL_INDEX_4,
    CELL_INDEx_5
}CellIndex_t;

/* Init及基础Get接口已实现，实际采样精度与硬件通信仍需上板验证。
 * 在线程上下文调用；只有AFE_EOK时输出参数才有效。
 * timeout_ms用于底层单次I2C调用，不是整个Init的总时间预算。
 */

/* 唤醒、检查通信/状态、缓存校准参数、基础配置并回读验证。
 * 保持充放电MOS与均衡关闭，保护阈值/故障恢复策略后续实现。
 * AFE_EOK表示基础配置完成，不表示故障状态全清或首轮采样已完成。
 */
AFEState_t AFE_Init(uint32_t timeout_ms);

/* 读取原始SYS_STAT状态字节；不自动清除故障或CC_READY。 */
AFEState_t AFE_GetStatus(uint8_t *status, uint32_t timeout_ms);

/* 使用CELL_INDEX_1～CELL_INDEx_5枚举（数值0～4）；输出单位mV。 */
AFEState_t AFE_GetCellVoltage(CellIndex_t cell_index, uint16_t *voltage_mv,
                             uint32_t timeout_ms);

/* 输出有符号电流，单位mA；初期保留CC原始极性，不预设充/放电符号。 */
AFEState_t AFE_GetCurrent(int32_t *current_ma, uint32_t timeout_ms);

/* 输出TS1外部NTC温度，单位0.1℃：253表示25.3℃，-50表示-5.0℃。 */
AFEState_t AFE_GetTemperature(int16_t *temperature_decic, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* AFE_CORE_H */
