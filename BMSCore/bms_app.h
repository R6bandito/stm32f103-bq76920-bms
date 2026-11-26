#ifndef __BMS_APP_H__
#define __BMS_APP_H__

#include "stm32f1xx_hal.h"
#include "AFE_Core.h"

#define  BMS_SAMPLE_BASE_PERIOD_MS          (250u)
#define  BMS_VOLTAGE_SAMPLE_PERIOD_MS       (250u)
#define  BMS_STATUS_SAMPLE_PERIOD_MS        (250u)
#define  BMS_CURRENT_SAMPLE_PERIOD_MS       (2000u)
#define  BMS_TEMP_SAMPLE_PERIOD_MS          (3000u)
#define  BMS_CONSOLE_PERIOD_MS              (1000u)

#define BMS_FAULT_EVENT_GENERIC_UPLOAD          (0x01u)
#define BMS_FAULT_EVENT_INIT_FAIL               (0x02u)
#define BMS_FAULT_EVENT_VERIFY_OUTPUTS_RESULT   (0x03u)

#define FAULT_MASK_OCD                      (0x01u)
#define FAULT_MASK_SCD                      (0x01u << 1)
#define FAULT_MASK_OV                       (0x01u << 2)
#define FAULT_MASK_UV                       (0x01u << 3)
#define FAULT_MASK_DEVICE_XREADY            (0x01u << 4)
#define FAULT_MASK_OVRD_ALERT               (0x01u << 5)
#define FAULT_MASK_ALL                      (FAULT_MASK_OCD | FAULT_MASK_SCD | FAULT_MASK_OV | FAULT_MASK_UV \
                                                | FAULT_MASK_DEVICE_XREADY | FAULT_MASK_OVRD_ALERT)


/* 采样快照约定（适用于单体电压、电流、温度和 SYS_STAT）。
 * 启动时有效位全清，result 初始化为 AFE_ENOTREADY。
 * 每次尝试都更新该项 result。成功时更新值和 tick，并置有效位。
 * 读取失败时保留上次成功的值、tick 和有效位。未安排读取的项保持原样。
 * Bms_GetSnapShot() 复制后按各项有效期检查 (uint32_t)(now - tick)，只清副本中过期的有效位。
 * 电压掩码 bit0～4 对应第 1～5 节。SYS_STAT 保存原始字节，result 保存读取结果。
 */
typedef struct {
    AFEState_t afe_init_result;        /* 初始化完成后保留 AFE_Init 的原始返回码。 */
    uint8_t afe_sys_stat;
    uint8_t afe_sys_stat_isValid;
    AFEState_t afe_sys_stat_result;     /* 最近一次读取是否成功. */
    uint32_t afe_sys_stat_tick;         /* 最近一次成功读取的Tick. */
} BatteryPackStatInfo_t;

typedef struct {
    uint16_t voltage_cell_mv[AFE_CELL_COUNT];
    uint8_t voltage_valid_mask;
    AFEState_t voltage_result[AFE_CELL_COUNT];
    uint32_t voltage_cell_result_tick[AFE_CELL_COUNT];

    int32_t current_mA;
    uint8_t current_valid_flag;
    AFEState_t current_result;
    uint32_t current_result_tick;

    int16_t temperature_decic;
    uint8_t temp_valid_flag;
    AFEState_t temperature_result;
    uint32_t temperature_result_tick;

    BatteryPackStatInfo_t stat;
} BatteryPackInfo_t;


/* 由启动线程调用一次，创建命令队列并启动独占 AFE I2C 的采样线程。 */
HAL_StatusTypeDef Bms_AfeIoThreadCreate( void );
HAL_StatusTypeDef Bms_ConsoleThreadCreate( void );
HAL_StatusTypeDef Bms_FaultThreadCreate( void );

/* 异步请求回读控制位，模块分配请求号，结果由故障线程按号报告。
 * timeout_ms 为单次寄存器读取的超时（1～20 ms）。
 * HAL_OK 表示已入队并写入 request_id_out。HAL_BUSY 表示队列满，HAL_ERROR
 * 表示参数错误或服务未就绪。失败时 request_id_out 指向的值保持原样。
 * 请求号是可回绕的 8 位序号，调用方保存当前待回复的请求号。
 */
HAL_StatusTypeDef Bms_AfeRequestVerifyOutputs(uint32_t timeout_ms, uint8_t *request_id_out);

/* 在线程中调用。HAL_OK 表示已写回完成有效期检查的快照。
 * 建议为每个调用方使用独立的输出对象。共享输出时，由调用方同步各处的读写。
 */
HAL_StatusTypeDef Bms_GetSnapShot( BatteryPackInfo_t *snapshot );

#endif /* __BMS_APP_H__ */
