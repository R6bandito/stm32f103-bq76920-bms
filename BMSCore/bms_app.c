#include "bms_app.h"
#include "bq76920_driver.h"
#include "rtthread.h"
#include "sys.h"

#define BMS_AFE_COMMAND_QUEUE_DEPTH          (4u)
#define BMS_FAULT_REQUEST_QUEUE_DEPTH        (4u)
#define BMS_FAULT_REPLY_TIMEOUT_MS            (2000u)
#define BMS_FAULT_MAX_RETRIES                 (5u)
#define BMS_AFE_FET_ENABLE_MASK              (0x03u)
#define BMS_AFE_CELLBAL_ENABLE_MASK          (0x1Fu)

typedef enum {
    BMS_AFE_CMD_VERIFY_OUTPUTS = 1
} BmsAfeCommandType_t;

typedef union {
    struct {
        uint32_t timeout_ms;
    } verify_outputs;
} BmsAfeCommandArgs_t;

typedef struct {    /* IO线程控制命令结构. */
    BmsAfeCommandType_t type;
    uint8_t request_id;
    BmsAfeCommandArgs_t args;
} BmsAfeCommand_t;

static const rt_uint32_t bms_afe_io_thread_stack_size = 1024u;
static const rt_uint8_t bms_afe_io_thread_priority = 15u;
static const rt_uint32_t bms_afe_io_thread_tick = 10u;
static const uint32_t bms_afe_timeout_ms = 20u;
static rt_thread_t bms_afe_io_thread;
static rt_mq_t bms_afe_command_queue;
static uint8_t bms_afe_last_request_id;

static const rt_uint32_t bms_console_thread_stack_size = 512u;
static const rt_uint8_t bms_console_thread_priority = 15u;
static const rt_uint32_t bms_console_thread_tick = 10u;
static rt_thread_t bms_console_thread;

static const rt_uint32_t bms_fault_thread_stack_size = 512u;
static const rt_uint8_t bms_fault_thread_priority = 12u;
static const rt_uint32_t bms_fault_thread_tick = 10u;
static rt_thread_t bms_fault_thread;
static rt_mailbox_t fault_mailbox;

typedef struct {
    uint8_t isXReady_Triggerd;  /* 只有在Triggerd标志置位情况下, Tick才有效. */
    uint32_t XReady_Triggerd_Tick;

    uint8_t isOCD_Triggerd;
    uint32_t OCD_Triggerd_Tick;

    uint8_t isSCD_Triggerd;
    uint32_t SCD_Triggerd_Tick;

    uint8_t isOV_Triggerd;
    uint32_t OV_Triggerd_Tick;

    uint8_t isUV_Triggerd;
    uint32_t UV_Triggerd_Tick;

    uint8_t isOVRD_ALERT_Triggerd;
    uint32_t OVRD_ALERT_Triggerd_Tick;
} GlobalError_t;
static GlobalError_t ErrorPacket;

typedef struct {
    BmsAfeCommandType_t type;
    BmsAfeCommandArgs_t args;
} BmsFaultRequest_t;

typedef struct {
    uint8_t request_id;
    uint8_t retry_count;
    rt_bool_t pending;
    rt_tick_t submitted_tick;
} BmsFaultTransaction_t;

/* 仅由故障线程访问，队首保留到事务结束。 */
static BmsFaultRequest_t bms_fault_requests[BMS_FAULT_REQUEST_QUEUE_DEPTH];
static uint8_t bms_fault_request_read;
static uint8_t bms_fault_request_write;
static uint8_t bms_fault_request_count;
static BmsFaultTransaction_t bms_fault_transaction;

/* 每项数据的有效期为其读取周期的三倍。 */
#define BMS_SAMPLE_MAX_AGE_PERIODS (3u)

#if (BMS_VOLTAGE_SAMPLE_PERIOD_MS == 0u) || (BMS_CURRENT_SAMPLE_PERIOD_MS == 0u) || \
    (BMS_STATUS_SAMPLE_PERIOD_MS == 0u) || (BMS_TEMP_SAMPLE_PERIOD_MS == 0u) || \
    (BMS_VOLTAGE_SAMPLE_PERIOD_MS % BMS_SAMPLE_BASE_PERIOD_MS != 0u) || \
    (BMS_STATUS_SAMPLE_PERIOD_MS % BMS_SAMPLE_BASE_PERIOD_MS != 0u) || \
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

HAL_StatusTypeDef Bms_AfeRequestVerifyOutputs(uint32_t timeout_ms, uint8_t *request_id_out) {
    rt_err_t result;
    BmsAfeCommand_t command = {
        .type = BMS_AFE_CMD_VERIFY_OUTPUTS,
        .args.verify_outputs = { .timeout_ms = timeout_ms }
    };
    if (request_id_out == RT_NULL || timeout_ms == 0u || timeout_ms > bms_afe_timeout_ms ||
            bms_afe_command_queue == RT_NULL || bms_fault_thread == RT_NULL)
        return HAL_ERROR;

    /* 取号和无阻塞入队一起完成，8 位序号按入队顺序递增。 */
    rt_enter_critical();
    command.request_id = (uint8_t)(bms_afe_last_request_id + 1u);
    result = rt_mq_send(bms_afe_command_queue, &command, sizeof(command));
    if (result == RT_EOK) {
        bms_afe_last_request_id = command.request_id;
        *request_id_out = command.request_id;
    }
    rt_exit_critical();

    if (result == RT_EOK)
        return HAL_OK;
    return (result == -RT_EFULL) ? HAL_BUSY : HAL_ERROR;
}

static void bms_afe_process_command(rt_mq_t command_queue, rt_int32_t wait_ticks) {
    BmsAfeCommand_t command;
    if (rt_mq_recv(command_queue, &command, sizeof(command), wait_ticks) != RT_EOK)
        return;
    switch (command.type) {
        case BMS_AFE_CMD_VERIFY_OUTPUTS: {
            uint8_t sys_ctrl2;
            uint8_t cellbal1;
            rt_ubase_t event;
            const AFEState_t result = AFE_ReadOutputState(&sys_ctrl2, &cellbal1, command.args.verify_outputs.timeout_ms);

            /* 类型 [31:28]，请求号 [23:16]，返回码 [15:8]。
             * 低字节的 bit 0/1 是 CHG/DSG，bit 2～6 是均衡位。
             */
            event = ((rt_ubase_t)BMS_FAULT_EVENT_VERIFY_OUTPUTS_RESULT << 28u) |
                    ((rt_ubase_t)command.request_id << 16u) |
                    ((rt_ubase_t)result << 8u);
            if (result == AFE_EOK) {
                event |= (sys_ctrl2 & BMS_AFE_FET_ENABLE_MASK) |
                         ((cellbal1 & BMS_AFE_CELLBAL_ENABLE_MASK) << 2u);
            }
            if (rt_mb_send(fault_mailbox, event) != RT_EOK)
                LOG_WARN("AFE output readback reply lost. id=%u", (unsigned)command.request_id);
            break;
        }
        default:    break;
    }
}

static void Bms_AfeIo_Thread( void *parameter ) {
    rt_mq_t command_queue = (rt_mq_t)parameter;
    BatteryPackInfo_t sample = SnapShot;
    const rt_tick_t base_ticks = rt_tick_from_millisecond(BMS_SAMPLE_BASE_PERIOD_MS);
    rt_tick_t last_wakeup;
    uint8_t voltage_count = 0u;
    uint8_t status_count = 0u;
    uint8_t current_count = 0u;
    uint8_t temp_count = 0u;
    sample.stat.afe_init_result = AFE_Init(bms_afe_timeout_ms);
    bms_publish_snapshot(&sample);
    if (sample.stat.afe_init_result != AFE_EOK) {
        /* 保留失败结果供上层读取，采样线程进入故障待机。 */
        if (fault_mailbox != RT_NULL) {
            const rt_ubase_t init_event = ((rt_ubase_t)BMS_FAULT_EVENT_INIT_FAIL << 28u) |
                                         ((rt_ubase_t)sample.stat.afe_init_result & 0x00FFFFFFu);
            (void)rt_mb_send(fault_mailbox, init_event);
        }
        for (;;) {
            bms_afe_process_command(command_queue, RT_WAITING_FOREVER);
        }
    }

    last_wakeup = rt_tick_get();
    for (;;) {
        uint8_t publish = 0u;
        rt_ubase_t fault_thread_event = 0u;
        uint32_t cell;
        rt_thread_delay_until(&last_wakeup, base_ticks);

        if (++voltage_count >= BMS_VOLTAGE_SAMPLE_PERIOD_MS / BMS_SAMPLE_BASE_PERIOD_MS) {
            /* 到达单体电压采样周期。 */
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

        if (++status_count >= BMS_STATUS_SAMPLE_PERIOD_MS / BMS_SAMPLE_BASE_PERIOD_MS) {
            uint8_t sys_stat;
            status_count = 0u;
            sample.stat.afe_sys_stat_result = AFE_GetStatus(&sys_stat, bms_afe_timeout_ms);
            if (sample.stat.afe_sys_stat_result == AFE_EOK) {
                sample.stat.afe_sys_stat = sys_stat;
                sample.stat.afe_sys_stat_tick = rt_tick_get();
                sample.stat.afe_sys_stat_isValid = 1u;
                fault_thread_event = ((rt_ubase_t)BMS_FAULT_EVENT_GENERIC_UPLOAD << 28u) |
                                     (rt_ubase_t)sys_stat;
            }
            publish = 1u;
        }

        if (++current_count >= BMS_CURRENT_SAMPLE_PERIOD_MS / BMS_SAMPLE_BASE_PERIOD_MS) {
            int32_t ma;
            current_count = 0u;
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
            if (fault_mailbox != RT_NULL && fault_thread_event != 0u) {
                (void)rt_mb_send(fault_mailbox, fault_thread_event);
            }
        }
        bms_afe_process_command(command_queue, 0);      /* 处理用户上层的命令. */
    }
}

HAL_StatusTypeDef Bms_AfeIoThreadCreate( void ) {
    rt_mq_t command_queue;
    rt_thread_t thread;
    if (bms_afe_io_thread != RT_NULL)   /* 线程正常运行时不允许重入. */
        return HAL_BUSY;
    command_queue = rt_mq_create("afe_cmd", sizeof(BmsAfeCommand_t),
                                    BMS_AFE_COMMAND_QUEUE_DEPTH, RT_IPC_FLAG_FIFO);
    if (command_queue == RT_NULL)
        return HAL_ERROR;

    thread = rt_thread_create("afe_io", Bms_AfeIo_Thread, command_queue,
                                bms_afe_io_thread_stack_size, bms_afe_io_thread_priority, bms_afe_io_thread_tick);
    if (thread == RT_NULL) {
        rt_mq_delete(command_queue);
        return HAL_ERROR;
    }
    if (rt_thread_startup(thread) != RT_EOK) {
        rt_thread_delete(thread);
        rt_mq_delete(command_queue);
        return HAL_ERROR;
    }
    bms_afe_io_thread = thread;
    bms_afe_command_queue = command_queue;
    return HAL_OK;
}

static void Bms_Console_Thread( void *parameter ) {
    (void)parameter;
    HAL_StatusTypeDef res;
    BatteryPackInfo_t bmsinfo;
    const rt_tick_t base_ticks = rt_tick_from_millisecond(BMS_CONSOLE_PERIOD_MS);
    rt_tick_t last_wake = rt_tick_get();
    LOG_INFO("console ready.");

    for (;;) {
        rt_tick_t now;
        uint8_t i;

        rt_thread_delay_until(&last_wake, base_ticks);
        res = Bms_GetSnapShot(&bmsinfo);
        if (res != HAL_OK)
            goto next_round;
        now = rt_tick_get();

        if (bmsinfo.stat.afe_sys_stat_isValid) {
            res = LOG_INFO("AFE init=%d STAT=0x%02X stat_rc=%d", (int)bmsinfo.stat.afe_init_result,
                           (unsigned)bmsinfo.stat.afe_sys_stat, (int)bmsinfo.stat.afe_sys_stat_result);
        } else {
            res = LOG_INFO("AFE init=%d STAT=-- stat_rc=%d", (int)bmsinfo.stat.afe_init_result,
                           (int)bmsinfo.stat.afe_sys_stat_result);
        }
        if (res != HAL_OK)
            goto next_round;    /* 打印超时. 直接跳过该轮取下一轮快照. */

        for (i = 0u; i < AFE_CELL_COUNT; ++i) {
            const unsigned cell_number = (unsigned)i + 1u;
            if ((bmsinfo.voltage_valid_mask & (1u << i)) == 0u) {
                res = LOG_INFO("CELL%u: --", cell_number);    /* 采样多次失败、数据失效. */
            } else if (bmsinfo.voltage_result[i] != AFE_EOK) {
                const rt_tick_t age = (rt_tick_t)(now - bmsinfo.voltage_cell_result_tick[i]);
                res = LOG_INFO("CELL%u: %u mV cached rc=%d age=%lu ticks", cell_number,     /* 最近一次采样失败，打印缓存旧数据信息. */
                               (unsigned)bmsinfo.voltage_cell_mv[i], (int)bmsinfo.voltage_result[i],
                               (unsigned long)age);
            } else {
                res = LOG_INFO("CELL%u: %u mV", cell_number, (unsigned)bmsinfo.voltage_cell_mv[i]); /* 采样正常. 正常打印最新采样数据. */
            }
            if (res != HAL_OK)
                goto next_round;
        }

        if (!bmsinfo.current_valid_flag) {
            res = LOG_INFO("Current: --");
        } else if (bmsinfo.current_result != AFE_EOK) {
            const rt_tick_t age = (rt_tick_t)(now - bmsinfo.current_result_tick);
            res = LOG_INFO("Current: %ld mA cached rc=%d age=%lu ticks", (long)bmsinfo.current_mA,
                           (int)bmsinfo.current_result, (unsigned long)age);
        } else {
            res = LOG_INFO("Current: %ld mA", (long)bmsinfo.current_mA);
        }
        if (res != HAL_OK)
            goto next_round;

        if (!bmsinfo.temp_valid_flag) {
            res = LOG_INFO("Temp: --");
        } else {
            const int decic = bmsinfo.temperature_decic;
            const char *sign = decic < 0 ? "-" : "";  /* 温度正负号. */
            const unsigned magnitude = (unsigned)(decic < 0 ? -decic : decic);
            if (bmsinfo.temperature_result != AFE_EOK) {
                const rt_tick_t age = (rt_tick_t)(now - bmsinfo.temperature_result_tick);
                res = LOG_INFO("Temp: %s%u.%u C cached rc=%d age=%lu ticks", sign, magnitude / 10u,
                               magnitude % 10u, (int)bmsinfo.temperature_result, (unsigned long)age);
            } else {
                res = LOG_INFO("Temp: %s%u.%u C", sign, magnitude / 10u, magnitude % 10u);
            }
        }
        if (res != HAL_OK)
            goto next_round;

next_round:
        /* 邮箱满或日志不可用时，跳过本轮剩余输出。 */
        continue;
    }
}

HAL_StatusTypeDef Bms_ConsoleThreadCreate( void ) {
    rt_err_t stat;
    bms_console_thread = rt_thread_create("console", Bms_Console_Thread, NULL,
                                            bms_console_thread_stack_size, bms_console_thread_priority, bms_console_thread_tick);
    if (bms_console_thread == RT_NULL)
        return HAL_ERROR;
    stat = rt_thread_startup(bms_console_thread);
    if (stat != RT_EOK) {
        rt_thread_delete(bms_console_thread);
        bms_console_thread = RT_NULL;
        return HAL_ERROR;
    }
    return HAL_OK;
}

static rt_uint32_t bms_evaluate(uint8_t sys_stat) {
    rt_uint32_t mask = 0u;

    if (sys_stat & BQ76920_SYS_STAT_OCD)
        mask |= FAULT_MASK_OCD;
    if (sys_stat & BQ76920_SYS_STAT_SCD)
        mask |= FAULT_MASK_SCD;
    if (sys_stat & BQ76920_SYS_STAT_OV)
        mask |= FAULT_MASK_OV;
    if (sys_stat & BQ76920_SYS_STAT_UV)
        mask |= FAULT_MASK_UV;
    if (sys_stat & BQ76920_SYS_STAT_DEVICE_XREADY)
        mask |= FAULT_MASK_DEVICE_XREADY;
    if (sys_stat & BQ76920_SYS_STAT_OVRD_ALERT)
        mask |= FAULT_MASK_OVRD_ALERT;

    return mask;
}

static rt_bool_t bms_fault_enqueue_request(const BmsFaultRequest_t *request) {
    if (bms_fault_request_count == BMS_FAULT_REQUEST_QUEUE_DEPTH)
        return RT_FALSE;
    bms_fault_requests[bms_fault_request_write] = *request;
    bms_fault_request_write = (bms_fault_request_write + 1u) % BMS_FAULT_REQUEST_QUEUE_DEPTH;
    ++bms_fault_request_count;
    return RT_TRUE;
}

static void bms_fault_finish_request(void) {
    bms_fault_request_read = (bms_fault_request_read + 1u) % BMS_FAULT_REQUEST_QUEUE_DEPTH;
    --bms_fault_request_count;
    bms_fault_transaction = (BmsFaultTransaction_t){0};
}

static void bms_fault_service_requests(void) {
    HAL_StatusTypeDef result;
    const BmsFaultRequest_t *request;
    if (bms_fault_transaction.pending) {        /* 如果pending已经置位，且Elapsed过期.说明对端回复超时(FaultThread未收到正确响应.) */
        const rt_tick_t elapsed = rt_tick_get() - bms_fault_transaction.submitted_tick;
        if (elapsed < (rt_tick_t)rt_tick_from_millisecond(BMS_FAULT_REPLY_TIMEOUT_MS))
            return;
        LOG_ERROR("AFE fault request reply timed out. id=%u", (unsigned)bms_fault_transaction.request_id);
        bms_fault_finish_request();
    }
    if (bms_fault_request_count == 0u)  /* 队列已空. 直接返回. */
        return;

    request = &bms_fault_requests[bms_fault_request_read];
    switch (request->type) {
        case BMS_AFE_CMD_VERIFY_OUTPUTS:
            result = Bms_AfeRequestVerifyOutputs(request->args.verify_outputs.timeout_ms, &bms_fault_transaction.request_id);
            break;
        default:
            result = HAL_ERROR;
            break;
    }
    if (result == HAL_OK) {
        bms_fault_transaction.submitted_tick = rt_tick_get();
        bms_fault_transaction.pending = RT_TRUE;
    } else if (result != HAL_BUSY) {
        LOG_ERROR("AFE fault request enqueue failed. type=%u rc=%u", (unsigned)request->type, (unsigned)result);
        bms_fault_finish_request();
    }
}

/* 故障处理入口，具体动作待接入。 */
static void bms_error_ocd_handler(void) {}
static void bms_error_scd_handler(void) {}
static void bms_error_ov_handler(void) {}

static void bms_error_uv_handler(void) {}

static void bms_error_ovrd_alert_handler(void) {
    ErrorPacket.isOVRD_ALERT_Triggerd = 1;
    ErrorPacket.OVRD_ALERT_Triggerd_Tick = rt_tick_get();
}

static void bms_error_device_xready_handler(void) {
    const BmsFaultRequest_t request = {
        .type = BMS_AFE_CMD_VERIFY_OUTPUTS,
        .args.verify_outputs = { .timeout_ms = bms_afe_timeout_ms }
    };
    ErrorPacket.isXReady_Triggerd = 1u;
    ErrorPacket.XReady_Triggerd_Tick = rt_tick_get();
    if (!bms_fault_enqueue_request(&request))
        LOG_ERROR("Fault request queue full. XREADY readback request skipped.");
}

static void Bms_Fault_Thread( void *parameter ) {
    (void)parameter;
    static const struct {
        rt_uint32_t mask;
        void (*handler)(void);
    } handlers[] = {
        { FAULT_MASK_OCD,           bms_error_ocd_handler },
        { FAULT_MASK_SCD,           bms_error_scd_handler },
        { FAULT_MASK_OV,            bms_error_ov_handler },
        { FAULT_MASK_UV,            bms_error_uv_handler },
        { FAULT_MASK_DEVICE_XREADY, bms_error_device_xready_handler },
        { FAULT_MASK_OVRD_ALERT,    bms_error_ovrd_alert_handler }
    };
    const rt_int32_t timeout = rt_tick_from_millisecond(750);
    rt_uint32_t previous_faults = 0u;
    rt_bool_t stat_unavailable = RT_FALSE;

    for (;;) {
        rt_ubase_t recv_event;
        rt_err_t recv_result;
        bms_fault_service_requests();   /* 检查本地FIFO是否有待处理命令(发给IO线程) */
        recv_result = rt_mb_recv(fault_mailbox, &recv_event, timeout);
        if (recv_result == -RT_ETIMEOUT) {
            BatteryPackInfo_t batterySnap;
            if (Bms_GetSnapShot(&batterySnap) != HAL_OK) {
                LOG_ERROR("STAT timeout: snapshot read failed.");
                continue;
            }
            if (!batterySnap.stat.afe_sys_stat_isValid) {
                /* AFE 尚未初始化成功时，由初始化路径报告。 */
                if (batterySnap.stat.afe_init_result == AFE_EOK && !stat_unavailable) {
                    stat_unavailable = RT_TRUE;
                    LOG_ERROR("SYS_STAT unavailable. Last read result: %u.", (unsigned)batterySnap.stat.afe_sys_stat_result);
                }
                continue;
            }
            if (batterySnap.stat.afe_sys_stat_result == AFE_EOK)
                stat_unavailable = RT_FALSE;
            recv_event = ((rt_ubase_t)BMS_FAULT_EVENT_GENERIC_UPLOAD << 28u) |      /* 用有效快照伪造一份事件复用下述检测逻辑。 */
                         (rt_ubase_t)batterySnap.stat.afe_sys_stat;
        } else if (recv_result != RT_EOK) {
            LOG_ERROR("Fault mailbox receive failed: %d.", (int)recv_result);
            rt_thread_mdelay(100u);
            continue;
        }

        switch ((recv_event >> 28u) & 0x0Fu) {
            case BMS_FAULT_EVENT_GENERIC_UPLOAD: {
                const rt_uint32_t fault_mask = bms_evaluate((uint8_t)(recv_event & 0xFFu));
                const rt_uint32_t new_faults = fault_mask & ~previous_faults;
                rt_size_t i;
                if (recv_result == RT_EOK)
                    stat_unavailable = RT_FALSE;
                previous_faults = fault_mask;
                for (i = 0u; i < sizeof(handlers) / sizeof(handlers[0]); ++i) {
                    if (new_faults & handlers[i].mask)
                        handlers[i].handler();      /* 根据错误标志位置位情况.依次分发处理逻辑. */
                }
                break;
            }
            case BMS_FAULT_EVENT_INIT_FAIL: {
                const AFEState_t ret = (AFEState_t)(recv_event & 0x00FFFFFFu);
                LOG_ERROR("BMS Init Failed! Error Code: %x.", (unsigned)ret);
                break;
            }
            case BMS_FAULT_EVENT_VERIFY_OUTPUTS_RESULT: {
                const unsigned request_id = (unsigned)((recv_event >> 16u) & 0xFFu);
                const unsigned result = (unsigned)((recv_event >> 8u) & 0xFFu);
                const unsigned fet_enable = (unsigned)(recv_event & BMS_AFE_FET_ENABLE_MASK);
                const unsigned balance_enable = (unsigned)((recv_event >> 2u) & BMS_AFE_CELLBAL_ENABLE_MASK);

                if (bms_fault_transaction.pending && request_id == bms_fault_transaction.request_id &&
                    bms_fault_requests[bms_fault_request_read].type == BMS_AFE_CMD_VERIFY_OUTPUTS) {
                    /* 单笔事务Id与命令类型能够对应上.说明收到的回复确实是当前这笔事务的. */
                    if (result != AFE_EOK && bms_fault_transaction.retry_count < BMS_FAULT_MAX_RETRIES) {  /* 远端未正确执行命令. */
                        bms_fault_transaction.pending = RT_FALSE;
                        ++bms_fault_transaction.retry_count;
                        break;
                    }
                    bms_fault_finish_request(); /* 重试次数耗尽. 清掉请求. */
                }

                if (result != AFE_EOK)
                    LOG_ERROR("AFE output readback failed. id=%u rc=%u", request_id, result);
                else if (fet_enable != 0u || balance_enable != 0u)
                    LOG_WARN("AFE outputs active. id=%u fet=0x%X balance=0x%X",
                             request_id, fet_enable, balance_enable);
                else
                    LOG_INFO("AFE outputs off. id=%u", request_id);
                break;
            }
            default:  LOG_WARN("Unknown Stat Code Detected.");   break;
        }
    }
}

HAL_StatusTypeDef Bms_FaultThreadCreate( void ) {
    rt_err_t stat;
    fault_mailbox = rt_mb_create("fault_mb", 4u, RT_IPC_FLAG_FIFO);
    if (fault_mailbox == RT_NULL)
        return HAL_ERROR;
    bms_fault_thread = rt_thread_create("fault", Bms_Fault_Thread, NULL,
                                            bms_fault_thread_stack_size, bms_fault_thread_priority, bms_fault_thread_tick);
    if (bms_fault_thread == RT_NULL)
        return HAL_ERROR;
    stat = rt_thread_startup(bms_fault_thread);
    if (stat != RT_EOK) {
        rt_thread_delete(bms_fault_thread);
        bms_fault_thread = RT_NULL;
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
    const rt_tick_t status_max_age = rt_tick_from_millisecond(BMS_STATUS_SAMPLE_PERIOD_MS * BMS_SAMPLE_MAX_AGE_PERIODS);
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
    if ((rt_tick_t)(now - local_snapshot.stat.afe_sys_stat_tick) >= status_max_age)
        local_snapshot.stat.afe_sys_stat_isValid = 0u;
    if ((rt_tick_t)(now - local_snapshot.temperature_result_tick) >= temp_max_age)
        local_snapshot.temp_valid_flag = 0u;

    /* 单核线程之间一次性写回已完成过期检查的快照。 */
    rt_enter_critical();
    *snapshot = local_snapshot;
    rt_exit_critical();

    return HAL_OK;
}


