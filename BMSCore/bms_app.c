#include "bms_app.h"
#include "rtthread.h"

static const rt_uint32_t bms_sample_thread_stack_size = 1024u;
static const rt_uint8_t bms_sample_thread_priority = 15u;
static const rt_uint32_t bms_sample_thread_tick = 10u;
static const uint32_t bms_afe_timeout_ms = 20u;
static rt_thread_t bms_sample_thread;

/* 有效期保留三轮读取的时间。SYS_STAT 与电流同一周期读取。 */
#define BMS_SAMPLE_MAX_AGE_PERIODS (3u)

#if (BMS_VOLTAGE_SAMPLE_PERIOD_MS == 0u) || (BMS_CURRENT_SAMPLE_PERIOD_MS == 0u) || \
    (BMS_TEMP_SAMPLE_PERIOD_MS == 0u) || \
    (BMS_VOLTAGE_SAMPLE_PERIOD_MS % BMS_SAMPLE_BASE_PERIOD_MS != 0u) || \
    (BMS_CURRENT_SAMPLE_PERIOD_MS % BMS_SAMPLE_BASE_PERIOD_MS != 0u) || \
    (BMS_TEMP_SAMPLE_PERIOD_MS % BMS_SAMPLE_BASE_PERIOD_MS != 0u)
#error "BMS sample periods must be positive multiples of the base period"
#endif

static BatteryPackInfo_t SnapShot = {
    .voltage_result = {AFE_ENOTREADY, AFE_ENOTREADY, AFE_ENOTREADY,
                       AFE_ENOTREADY, AFE_ENOTREADY},
    .current_result = AFE_ENOTREADY,
    .temperature_result = AFE_ENOTREADY,
    .stat = {
        .afe_init_result = AFE_ENOTREADY,
        .afe_sys_stat_result = AFE_ENOTREADY
    }
};

static void bms_publish_snapshot(const BatteryPackInfo_t *snapshot) {
    /* 单核上只保护线程之间的复制。I2C 始终在调度锁外执行。 */
    rt_enter_critical();
    SnapShot = *snapshot;
    rt_exit_critical();
}

static void Bms_Sample_Thread( void *parameter ) {
    (void)parameter;
    BatteryPackInfo_t sample = SnapShot;
    const rt_tick_t base_ticks = rt_tick_from_millisecond(BMS_SAMPLE_BASE_PERIOD_MS);
    rt_tick_t last_wakeup;
    uint8_t voltage_count = 0u;
    uint8_t current_count = 0u;
    uint8_t temp_count = 0u;
    sample.stat.afe_init_result = AFE_Init(bms_afe_timeout_ms);
    bms_publish_snapshot(&sample);
    if (sample.stat.afe_init_result != AFE_EOK) {
        /* 保留失败结果供上层读取，采样线程进入故障待机。 */
        for (;;) {
            rt_thread_mdelay(1000);
        }
    }

    last_wakeup = rt_tick_get();
    for (;;) {
        uint8_t publish = 0u;
        uint32_t cell;

        rt_thread_delay_until(&last_wakeup, base_ticks);

        if (++voltage_count >= BMS_VOLTAGE_SAMPLE_PERIOD_MS / BMS_SAMPLE_BASE_PERIOD_MS) {
            /* 单体电压每300ms采样一次. */
            voltage_count = 0u;
            for (cell = 0u; cell < AFE_CELL_COUNT; ++cell) {
                uint16_t mv;
                sample.voltage_result[cell] = AFE_GetCellVoltage((CellIndex_t)cell,
                                                                  &mv, bms_afe_timeout_ms);
                if (sample.voltage_result[cell] == AFE_EOK) {
                    sample.voltage_cell_mv[cell] = mv;
                    sample.voltage_cell_result_tick[cell] = rt_tick_get();
                    sample.voltage_valid_mask |= (uint8_t)(1u << cell);
                }
            }
            publish = 1u;
        }

        if (++current_count >= BMS_CURRENT_SAMPLE_PERIOD_MS / BMS_SAMPLE_BASE_PERIOD_MS) {
            /* 每300ms采样一次电流与STAT状态寄存器. */
            uint8_t sys_stat;
            int32_t ma;
            current_count = 0u;
            sample.stat.afe_sys_stat_result = AFE_GetStatus(&sys_stat, bms_afe_timeout_ms);
            if (sample.stat.afe_sys_stat_result == AFE_EOK) {
                sample.stat.afe_sys_stat = sys_stat;
                sample.stat.afe_sys_stat_tick = rt_tick_get();
                sample.stat.afe_sys_stat_isValid = 1u;
            }
            sample.current_result = AFE_GetCurrent(&ma, bms_afe_timeout_ms);
            if (sample.current_result == AFE_EOK) {
                sample.current_mA = ma;
                sample.current_result_tick = rt_tick_get();
                sample.current_valid_flag = 1u;
            }
            publish = 1u;
        }

        if (++temp_count >= BMS_TEMP_SAMPLE_PERIOD_MS / BMS_SAMPLE_BASE_PERIOD_MS) {
            /* 每3s采样一次温度. */
            int16_t decic;
            temp_count = 0u;
            sample.temperature_result = AFE_GetTemperature(&decic, bms_afe_timeout_ms);
            if (sample.temperature_result == AFE_EOK) {
                sample.temperature_decic = decic;
                sample.temperature_result_tick = rt_tick_get();
                sample.temp_valid_flag = 1u;
            }
            publish = 1u;
        }

        if (publish) {
            bms_publish_snapshot(&sample);
        }
    }
}

HAL_StatusTypeDef Bms_SampleTaskCreate( void ) {
    if (bms_sample_thread != RT_NULL)   /* 线程启动方法.不允许重入. */
        return HAL_BUSY;
    bms_sample_thread = rt_thread_create("bms_sample", Bms_Sample_Thread, NULL, 
                            bms_sample_thread_stack_size, bms_sample_thread_priority, bms_sample_thread_tick);
    if (bms_sample_thread == RT_NULL) 
        return HAL_ERROR;
    if (rt_thread_startup(bms_sample_thread) != RT_EOK) {
        rt_thread_delete(bms_sample_thread);
        bms_sample_thread = RT_NULL;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Bms_GetSnapShot( BatteryPackInfo_t *snapshot ) {
    BatteryPackInfo_t local_snapshot;
    rt_tick_t now;
    uint32_t cell;
    if (snapshot == RT_NULL)
        return HAL_ERROR;
    const rt_tick_t voltage_max_age = rt_tick_from_millisecond(BMS_VOLTAGE_SAMPLE_PERIOD_MS * BMS_SAMPLE_MAX_AGE_PERIODS);
    const rt_tick_t current_max_age = rt_tick_from_millisecond(BMS_CURRENT_SAMPLE_PERIOD_MS * BMS_SAMPLE_MAX_AGE_PERIODS);
    const rt_tick_t temp_max_age = rt_tick_from_millisecond(BMS_TEMP_SAMPLE_PERIOD_MS * BMS_SAMPLE_MAX_AGE_PERIODS);

    rt_enter_critical();
    local_snapshot = SnapShot;
    rt_exit_critical();

    now = rt_tick_get();
    for (cell = 0u; cell < AFE_CELL_COUNT; ++cell) {
        if ((rt_tick_t)(now - local_snapshot.voltage_cell_result_tick[cell]) >= voltage_max_age) {
            local_snapshot.voltage_valid_mask &= (uint8_t)~(1u << cell);
        }
    }
    if ((rt_tick_t)(now - local_snapshot.current_result_tick) >= current_max_age)
        local_snapshot.current_valid_flag = 0u;
    if ((rt_tick_t)(now - local_snapshot.stat.afe_sys_stat_tick) >= current_max_age)
        local_snapshot.stat.afe_sys_stat_isValid = 0u;
    if ((rt_tick_t)(now - local_snapshot.temperature_result_tick) >= temp_max_age)
        local_snapshot.temp_valid_flag = 0u;

    /* 单核线程之间一次性写回已完成过期检查的快照。 */
    rt_enter_critical();
    *snapshot = local_snapshot;
    rt_exit_critical();

    return HAL_OK;
}




