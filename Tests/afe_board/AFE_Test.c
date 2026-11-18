#include "AFE_Test.h"
#include "bq76920_driver.h"
#include "rtthread.h"
#include "sys.h"
#include <stdarg.h>
#include <stdio.h>

#define AFE_TEST_SETTLE_MS   (2200u) /* 温度约2s更新，兼顾电压/电流首轮转换。 */
#define AFE_TEST_INTERVAL_MS (2200u)

static volatile AFE_TestReport_t report;

/* 仅供本测试使用；不改工程的fputc或日志框架，不使用浮点printf。 */
static void afe_test_log(const char *fmt, ...) {
    char buffer[160];
    int length;
    va_list args;
    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (length < 0) {
        ++report.uart_failures;
        return;
    }
    if ((uint32_t)length >= sizeof(buffer))
        length = (int)sizeof(buffer) - 1;
    if (HAL_UART_Transmit(GetConsoleHandle(), (uint8_t *)buffer,
                          (uint16_t)length, 100u) != HAL_OK)
        ++report.uart_failures;
}

static void afe_test_check(AFEState_t actual, AFEState_t expected) {
    if (actual == expected) {
        ++report.passed_checks;
    } else {
        ++report.failed_checks;
        if (report.final_result == AFE_EOK)
            report.final_result = actual == AFE_EOK ? AFE_ECONFIG : actual;
    }
}

const volatile AFE_TestReport_t *AFE_Test_GetReport(void) {
    return &report;
}

AFEState_t AFE_Test_Run(uint32_t timeout_ms) {
    AFE_TestSample_t sample;
    AFEState_t state;
    uint32_t i;
    uint32_t cell;
    uint16_t dummy_mv = 1234u;
    uint8_t faults;
    uint8_t all_cells_ok;

    /* 原地清报告，避免在调用线程栈上创建整份报告的副本。 */
    for (i = 0u; i < sizeof(report); ++i)
        ((volatile uint8_t *)&report)[i] = 0u;
    report.init_result = AFE_ENOTREADY;
    if (timeout_ms == 0u) {
        report.final_result = AFE_EPARAM;
        report.phase = AFE_TEST_FINISHED;
        return AFE_EPARAM;
    }

    report.phase = AFE_TEST_PARAMETERS;
    afe_test_log("\r\n[AFE TEST] begin; MOS/balance OFF; units: mV,mA,0.1C\r\n");
    afe_test_check(AFE_GetStatus(NULL, timeout_ms), AFE_EPARAM);
    afe_test_check(AFE_GetCellVoltage(CELL_INDEX_1, NULL, timeout_ms), AFE_EPARAM);
    afe_test_check(AFE_GetCellVoltage((CellIndex_t)AFE_CELL_COUNT,
                                     &dummy_mv, timeout_ms), AFE_EPARAM);
    afe_test_check(AFE_GetCellVoltage((CellIndex_t)-1,
                                     &dummy_mv, timeout_ms), AFE_EPARAM);
    afe_test_check(AFE_GetCurrent(NULL, timeout_ms), AFE_EPARAM);
    afe_test_check(AFE_GetTemperature(NULL, timeout_ms), AFE_EPARAM);
    afe_test_check(dummy_mv == 1234u ? AFE_EOK : AFE_ECONFIG, AFE_EOK);
    afe_test_log("[AFE TEST] parameters: pass=%lu fail=%lu\r\n",
                 (unsigned long)report.passed_checks, (unsigned long)report.failed_checks);
    if (report.failed_checks != 0u) {
        report.phase = AFE_TEST_FINISHED;
        return report.final_result;
    }

    report.phase = AFE_TEST_INITIALIZING;
    state = AFE_Init(timeout_ms);
    report.init_result = state;
    afe_test_check(state, AFE_EOK);
    afe_test_log("[AFE TEST] Init=%u (0=OK)\r\n", (unsigned)state);
    if (state != AFE_EOK) {
        report.phase = AFE_TEST_FINISHED;
        return report.final_result;
    }

    rt_thread_mdelay(AFE_TEST_SETTLE_MS);
    report.phase = AFE_TEST_SAMPLING;
    for (i = 0u; i < AFE_TEST_SAMPLE_COUNT; ++i) {
        AFE_TestSample_t empty_sample = {0};
        sample = empty_sample;
        sample.begin_tick = (uint32_t)rt_tick_get();
        sample.status_result = AFE_GetStatus(&sample.sys_stat, timeout_ms);
        afe_test_check(sample.status_result, AFE_EOK);
        if (sample.status_result == AFE_EOK) {
            faults = sample.sys_stat & BQ76920_SYS_STAT_FAULT_MASK;
            report.observed_faults |= faults;
            afe_test_check(faults == 0u ? AFE_EOK : AFE_ENOTREADY, AFE_EOK);
            /* CC_READY是锁存事件；不清它，不把后续的1当作本轮新转换。 */
            if (i == 0u)
                afe_test_check((sample.sys_stat & BQ76920_SYS_STAT_CC_READY)
                               ? AFE_EOK : AFE_ENOTREADY, AFE_EOK);
        }

        all_cells_ok = 1u;
        for (cell = 0u; cell < AFE_CELL_COUNT; ++cell) {
            sample.cell_result[cell] = AFE_GetCellVoltage((CellIndex_t)cell,
                                          &sample.cell_mv[cell], timeout_ms);
            afe_test_check(sample.cell_result[cell], AFE_EOK);
            if (sample.cell_result[cell] == AFE_EOK)
                sample.sum_mv += sample.cell_mv[cell];
            else
                all_cells_ok = 0u;
        }
        sample.current_result = AFE_GetCurrent(&sample.current_ma, timeout_ms);
        afe_test_check(sample.current_result, AFE_EOK);
        sample.temperature_result = AFE_GetTemperature(&sample.temperature_decic, timeout_ms);
        afe_test_check(sample.temperature_result, AFE_EOK);
        sample.end_tick = (uint32_t)rt_tick_get();
        report.samples[i] = sample;
        report.samples_done = i + 1u;

        afe_test_log("[AFE TEST] #%lu stat=0x%02X result=%u\r\n",
                     (unsigned long)(i + 1u), (unsigned)sample.sys_stat,
                     (unsigned)sample.status_result);
        for (cell = 0u; cell < AFE_CELL_COUNT; ++cell)
            afe_test_log("  VC%lu=%u mV result=%u\r\n", (unsigned long)(cell + 1u),
                         (unsigned)sample.cell_mv[cell], (unsigned)sample.cell_result[cell]);
        afe_test_log("  sum=%lu mV valid=%u; I=%ld mA result=%u; T=%d (0.1C) result=%u\r\n",
                     (unsigned long)sample.sum_mv, (unsigned)all_cells_ok,
                     (long)sample.current_ma, (unsigned)sample.current_result,
                     (int)sample.temperature_decic, (unsigned)sample.temperature_result);
        /* 失败字段的数值无效；sum仅在五节全部成功时才是完整总和。 */
        if (i + 1u < AFE_TEST_SAMPLE_COUNT)
            rt_thread_mdelay(AFE_TEST_INTERVAL_MS);
    }

    afe_test_log("[AFE TEST] end: result=%u pass=%lu fail=%lu faults=0x%02X\r\n",
                 (unsigned)report.final_result, (unsigned long)report.passed_checks,
                 (unsigned long)report.failed_checks, (unsigned)report.observed_faults);
    report.phase = AFE_TEST_FINISHED;
    return report.final_result;
}
