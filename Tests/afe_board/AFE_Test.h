#ifndef AFE_TEST_H
#define AFE_TEST_H

#include "AFE_Core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AFE_TEST_SAMPLE_COUNT  (5u)

typedef enum {
    AFE_TEST_IDLE = 0,
    AFE_TEST_PARAMETERS,
    AFE_TEST_INITIALIZING,
    AFE_TEST_SAMPLING,
    AFE_TEST_FINISHED
} AFE_TestPhase_t;

typedef struct {
    uint32_t begin_tick;
    uint32_t end_tick; /* MCU收集时间，不是ADC同时采样时间。 */
    AFEState_t status_result;
    uint8_t sys_stat;
    AFEState_t cell_result[AFE_CELL_COUNT];
    uint16_t cell_mv[AFE_CELL_COUNT];
    uint32_t sum_mv;
    AFEState_t current_result;
    int32_t current_ma;
    AFEState_t temperature_result;
    int16_t temperature_decic;
} AFE_TestSample_t;

typedef struct {
    AFE_TestPhase_t phase;
    AFEState_t init_result;
    AFEState_t final_result;
    uint32_t passed_checks;
    uint32_t failed_checks;
    uint32_t samples_done;
    uint32_t uart_failures;
    uint8_t observed_faults;
    AFE_TestSample_t samples[AFE_TEST_SAMPLE_COUNT];
} AFE_TestReport_t;

/* 线程/main中独占调用，不放ISR；建议调用线程栈至少2KB，供格式化日志使用。
 * 内部调用AFE_Init，外部不必重复Init。跑5轮后返回，约11秒加通信时间。
 * 不启动其他AFE访问者，不接充电器/外部负载；保持MOS与均衡关闭。
 * timeout_ms沿用单次I2C预算；0返回AFE_EPARAM。
 * 日志直发现有debug UART，不依赖fputc；报告可由调试器查看。
 * 成功表示接口测试通过，不代替万用表/温度计核对和实际保护验证。
 */
AFEState_t AFE_Test_Run(uint32_t timeout_ms);

/* 地址固定；运行时观察phase、samples_done，结束后取完整报告。
 * 不保证运行中跨线程读取整份报告的一致性，也不支持并发/重入Run。
 */
const volatile AFE_TestReport_t *AFE_Test_GetReport(void);

#ifdef __cplusplus
}
#endif
#endif /* AFE_TEST_H */
