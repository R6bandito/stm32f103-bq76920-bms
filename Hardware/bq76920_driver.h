#ifndef BQ76920_DRIVER_H
#define BQ76920_DRIVER_H

#include <stdint.h>

typedef enum {
    BQ_EOK = 0,
    BQ_EPARAM,
    BQ_EI2C,
    BQ_ECRC
} BqStatus;

#define BQ769200_ADDRESS_7  (0x08u)

/* BQ76920常用寄存器表。
 * 每个地址对应1字节；R=只读，R/W=包含可读写位，W1C=对应位写1清除。
 * R/W不代表所有位都可写：保留位保持0，只读位不能作为配置回读比较项。
 * 这里只列BQ76920有效寄存器，不访问BQ76930/40专用或保留地址。
 * 名称                              地址       属性   用途
 */

/* 状态、控制与保护配置 */
#define BQ76920_REG_SYS_STAT          (0x00u)  /* W1C   CC就绪、内部故障、OV/UV/SCD/OCD等状态 */
#define BQ76920_REG_CELLBAL1          (0x01u)  /* R/W   第1～5节均衡使能，bit0～4对应电芯1～5 */
/* 0x02、0x03不是BQ76920的有效均衡寄存器。 */
#define BQ76920_REG_SYS_CTRL1         (0x04u)  /* R/W   ADC_EN、TEMP_SEL、SHIP序列；含只读LOAD_PRESENT */
#define BQ76920_REG_SYS_CTRL2         (0x05u)  /* R/W   CC连续/单次转换、CHG/DSG使能、测试延时控制 */
#define BQ76920_REG_PROTECT1          (0x06u)  /* R/W   RSNS量程、放电短路SCD阈值与延时 */
#define BQ76920_REG_PROTECT2          (0x07u)  /* R/W   放电过流OCD阈值与延时 */
#define BQ76920_REG_PROTECT3          (0x08u)  /* R/W   欠压UV、过压OV保护延时 */
#define BQ76920_REG_OV_TRIP           (0x09u)  /* R/W   单体过压阈值编码，不是直接写mV */
#define BQ76920_REG_UV_TRIP           (0x0Au)  /* R/W   单体欠压阈值编码，不是直接写mV */
#define BQ76920_REG_CC_CFG            (0x0Bu)  /* R/W   启动时按手册写0x19 */

/* 单体电压结果：每节14位无符号码，HI在前、LO在后；每对连续读2字节。 */
#define BQ76920_REG_VC1_HI            (0x0Cu)  /* R     第1节ADC[13:8] */
#define BQ76920_REG_VC1_LO            (0x0Du)  /* R     第1节ADC[7:0]  */
#define BQ76920_REG_VC2_HI            (0x0Eu)  /* R     第2节ADC[13:8] */
#define BQ76920_REG_VC2_LO            (0x0Fu)  /* R     第2节ADC[7:0]  */
#define BQ76920_REG_VC3_HI            (0x10u)  /* R     第3节ADC[13:8] */
#define BQ76920_REG_VC3_LO            (0x11u)  /* R     第3节ADC[7:0]  */
#define BQ76920_REG_VC4_HI            (0x12u)  /* R     第4节ADC[13:8] */
#define BQ76920_REG_VC4_LO            (0x13u)  /* R     第4节ADC[7:0]  */
#define BQ76920_REG_VC5_HI            (0x14u)  /* R     第5节ADC[13:8] */
#define BQ76920_REG_VC5_LO            (0x15u)  /* R     第5节ADC[7:0]  */

/* 总压、温度与电流结果；0x16～0x29、0x2E～0x31不是本型号的采样通道。 */
#define BQ76920_REG_BAT_HI            (0x2Au)  /* R     总压结果高字节，16位无符号；量化步长不同于VCn */
#define BQ76920_REG_BAT_LO            (0x2Bu)  /* R     总压结果低字节 */
#define BQ76920_REG_TS1_HI            (0x2Cu)  /* R     温度ADC[13:8]，测量对象由TEMP_SEL选择 */
#define BQ76920_REG_TS1_LO            (0x2Du)  /* R     温度ADC[7:0] */
#define BQ76920_REG_CC_HI             (0x32u)  /* R     电流CC高字节，16位有符号补码 */
#define BQ76920_REG_CC_LO             (0x33u)  /* R     电流CC低字节 */

/* 出厂电压校准参数：初始化读取并缓存，不向这些寄存器写配置。 */
#define BQ76920_REG_ADCGAIN1          (0x50u)  /* R     bit3:2保存GAIN编码[4:3] */
#define BQ76920_REG_ADCOFFSET         (0x51u)  /* R     8位有符号偏移，单位mV */
#define BQ76920_REG_ADCGAIN2          (0x59u)  /* R     bit7:5保存GAIN编码[2:0] */

/* ******************************************************************************************* */
/* SYS_STAT位掩码：bit6保留；以下有效位均为W1C，写1清除。 */
#define BQ76920_SYS_STAT_CC_READY     (1u << 7u)  /* 0x80：新电流结果就绪，不是故障 */
#define BQ76920_SYS_STAT_DEVICE_XREADY (1u << 5u) /* 0x20：内部故障，不是启动完成 */
#define BQ76920_SYS_STAT_OVRD_ALERT   (1u << 4u)  /* 0x10：ALERT外部高电平覆盖 */
#define BQ76920_SYS_STAT_UV           (1u << 3u)  /* 0x08：单体欠压 */
#define BQ76920_SYS_STAT_OV           (1u << 2u)  /* 0x04：单体过压 */
#define BQ76920_SYS_STAT_SCD          (1u << 1u)  /* 0x02：放电短路 */
#define BQ76920_SYS_STAT_OCD          (1u << 0u)  /* 0x01：放电过流 */

/* 聚合掩码用于检查/分类，不代表可无条件写回清除所有状态。 */
#define BQ76920_SYS_STAT_FAULT_MASK   (BQ76920_SYS_STAT_DEVICE_XREADY | \
                                        BQ76920_SYS_STAT_OVRD_ALERT | \
                                        BQ76920_SYS_STAT_UV | BQ76920_SYS_STAT_OV | \
                                        BQ76920_SYS_STAT_SCD | BQ76920_SYS_STAT_OCD)
#define BQ76920_SYS_STAT_VALID_MASK   (BQ76920_SYS_STAT_FAULT_MASK | \
                                        BQ76920_SYS_STAT_CC_READY)
/* ******************************************************************************************* */

extern void HX_I2C_Init( void );

/* ReadReg/ReadRegs/WriteReg include the BQ76920 CRC handling.
 * data_len counts register DATA bytes, excluding CRC bytes.
 * Callers receive only register data; write CRC is appended internally.
 * Callers must use the output only when the complete operation returns BQ_EOK.
 */
BqStatus BQ76920_ReadRegs(uint8_t first_reg, uint8_t *data,
                        uint8_t data_len, uint32_t timeout_ms);

BqStatus BQ76920_ReadReg(uint8_t reg, uint8_t *value,
                       uint32_t timeout_ms);

BqStatus BQ76920_WriteReg(uint8_t reg, uint8_t value,
                        uint32_t timeout_ms);

#endif /* BQ76920_DRIVER_H */
