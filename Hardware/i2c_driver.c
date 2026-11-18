#include "i2c_driver.h"
#include "sys.h"

#define I2C_HALF_PERIOD_US       5u
#define I2C_CLEANUP_TIMEOUT_MS   1u

typedef struct {
    uint32_t start;
    uint32_t cycles;
} I2C_Deadline;

static uint8_t i2c_initialized;
static uint8_t i2c_started;
static I2C_State_t i2c_abort(I2C_State_t cause);

static void i2c_release_lines(void)
{
    SDA_UP();
    SCL_UP();
    i2c_started = 0u;
}

static I2C_State_t i2c_entry_error(I2C_State_t state)
{
    /* Public API error: an active transaction is cleaned up inside the driver.
     * Before START, return the error without touching the bus. */
    if (i2c_started)
        return i2c_abort(state);
    return state;
}

static I2C_State_t i2c_deadline_init(I2C_Deadline *deadline, uint32_t timeout_ms)
{
    uint64_t cycles;
    uint32_t probe_start;
    volatile uint32_t probe;

    if (timeout_ms == 0u) return IIC_STATE_EPARAM;
    if (!i2c_initialized || SystemCoreClock == 0u) return IIC_STATE_NOT_READY;

    /* Cast BEFORE multiplication; a uint32_t cannot detect its own overflow. */
    cycles = ((uint64_t)SystemCoreClock * timeout_ms + 999u) / 1000u;
    if (cycles == 0u || cycles >= UINT32_MAX) return IIC_STATE_EPARAM;

    /* Read-only probe: never reset or reinitialize the shared DWT counter. */
    probe_start = DWT_GetCount();
    for (probe = 0u; probe < 32u; ++probe) __NOP();
    if (DWT_Elapsed(probe_start) == 0u) return IIC_STATE_NOT_READY;

    deadline->cycles = (uint32_t)cycles;
    deadline->start = DWT_GetCount();
    return IIC_STATE_EOK;
}

static uint8_t i2c_expired(const I2C_Deadline *deadline)
{
    return DWT_Elapsed(deadline->start) >= deadline->cycles;
}

static I2C_State_t wait_us(uint32_t us, const I2C_Deadline *deadline)
{
    uint64_t wide_cycles = ((uint64_t)SystemCoreClock * us + 999999u) / 1000000u;
    uint32_t cycles;
    uint32_t start;
    uint32_t guard;

    if (wide_cycles == 0u || wide_cycles >= UINT32_MAX) return IIC_STATE_EPARAM;
    cycles = (uint32_t)wide_cycles;
    start = DWT_GetCount();
    guard = cycles;
    while (DWT_Elapsed(start) < cycles) {
        /* A finite fail-safe if DWT stops after the probe, not a calibrated timer. */
        if (i2c_expired(deadline) || guard-- == 0u) return IIC_STATE_TIMEOUT;
    }
    return i2c_expired(deadline) ? IIC_STATE_TIMEOUT : IIC_STATE_EOK;
}

static I2C_State_t i2c_wait_idle(const I2C_Deadline *deadline)
{
    uint32_t guard = deadline->cycles;
    while (SCL_READ() == 0u || SDA_READ() == 0u) {
        if (i2c_expired(deadline) || guard-- == 0u) return IIC_STATE_BUSY;
    }
    return i2c_expired(deadline) ? IIC_STATE_BUSY : IIC_STATE_EOK;
}

static I2C_State_t i2c_scl_high(const I2C_Deadline *deadline)
{
    uint32_t guard = deadline->cycles;
    SCL_UP();
    while (SCL_READ() == 0u) {
        if (i2c_expired(deadline) || guard-- == 0u) return IIC_STATE_TIMEOUT;
    }
    return i2c_expired(deadline) ? IIC_STATE_TIMEOUT : IIC_STATE_EOK;
}

static I2C_State_t i2c_send_start(const I2C_Deadline *deadline)
{
    I2C_State_t state;

    if (!i2c_started) {
        /* New transaction: both lines must be idle, not only SCL. */
        SDA_UP();
        SCL_UP();
        state = i2c_wait_idle(deadline);
        if (state != IIC_STATE_EOK) 
            return state;

        state = wait_us(I2C_HALF_PERIOD_US, deadline); /* tBUF */
        if (state != IIC_STATE_EOK) 
            return state;

        if (!SDA_READ() || !SCL_READ()) 
            return IIC_STATE_BUSY;
    } else {
        /* Repeated START: SCL is held low by US; skip the idle test. */
        SCL_LOW();
        SDA_UP();
        state = wait_us(I2C_HALF_PERIOD_US, deadline);
        if (state != IIC_STATE_EOK) return state;
        if (!SDA_READ()) return IIC_STATE_BUSY;
        state = i2c_scl_high(deadline);
        if (state != IIC_STATE_EOK) return state;
        state = wait_us(I2C_HALF_PERIOD_US, deadline); /* tSU;STA */
        if (state != IIC_STATE_EOK) return state;
    }

    SDA_LOW();                         /* SDA falling while SCL is high. */
    i2c_started = 1u;
    state = wait_us(I2C_HALF_PERIOD_US, deadline); /* tHD;STA */
    SCL_LOW();
    if (state != IIC_STATE_EOK) return state;
    return wait_us(I2C_HALF_PERIOD_US, deadline);
}

static I2C_State_t i2c_send_a_byte(uint8_t byte, const I2C_Deadline *deadline)
{
    uint8_t i;
    I2C_State_t state;

    /* ONLY eight bits; ACK owns the ninth clock. */
    SCL_LOW();
    for (i = 0u; i < 8u; ++i) {
        if ((byte >> (7u - i)) & 1u) 
            SDA_UP();
        else 
            SDA_LOW();
        state = wait_us(I2C_HALF_PERIOD_US, deadline);  /* 数据建立时间. */
        if (state != IIC_STATE_EOK) return state;

        state = i2c_scl_high(deadline);
        if (state != IIC_STATE_EOK) return state;

        state = wait_us(I2C_HALF_PERIOD_US, deadline);
        SCL_LOW();
        if (state != IIC_STATE_EOK) return state;
    }
    return IIC_STATE_EOK;
}

static I2C_State_t i2c_wait_ack(const I2C_Deadline *deadline)
{
    uint8_t ack;
    I2C_State_t state;

    SCL_LOW();
    SDA_UP();                          /* Receiver owns SDA during ACK. */
    state = wait_us(I2C_HALF_PERIOD_US, deadline);
    if (state != IIC_STATE_EOK) return state;

    state = i2c_scl_high(deadline);     /* 送出第九个时钟. */
    if (state != IIC_STATE_EOK) return state;

    state = wait_us(I2C_HALF_PERIOD_US, deadline);
    if (state != IIC_STATE_EOK) {
        SCL_LOW();
        return state;
    }

    ack = (uint8_t)SDA_READ();      /* 建立时间稳定后采样Ack. */
    SCL_LOW();                         /* Finish ninth clock for ACK AND NACK. */
    state = wait_us(I2C_HALF_PERIOD_US, deadline);
    if (state != IIC_STATE_EOK) return state;
    return ack ? IIC_STATE_NORESPONSE : IIC_STATE_EOK;
}

static I2C_State_t i2c_send_stop(const I2C_Deadline *deadline)
{
    I2C_State_t state;

    SCL_LOW();
    SDA_LOW();
    state = wait_us(I2C_HALF_PERIOD_US, deadline);
    if (state == IIC_STATE_EOK) {
        state = i2c_scl_high(deadline);
        if (state == IIC_STATE_EOK) {
            state = wait_us(I2C_HALF_PERIOD_US, deadline);
            if (state == IIC_STATE_EOK) {
                SDA_UP();                      /* SDA rising while SCL is high. */
                state = i2c_wait_idle(deadline);
            }
        }
    }

    if (state == IIC_STATE_EOK) 
        state = wait_us(I2C_HALF_PERIOD_US, deadline);
    /* Release on failure too; this cannot promise a physical STOP while a
     * target holds a line low. The next transaction must check actual idle. */
    i2c_release_lines();
    return state;
}

/* 第九拍：nack=0发送ACK，nack=1发送NACK；结束时SCL低、SDA由主机释放。 */
static I2C_State_t i2c_send_ack_nack( uint8_t nack, const I2C_Deadline *deadline ) {
    if ( !deadline || nack > 1 )
        return IIC_STATE_EPARAM;
    I2C_State_t state;
    SCL_LOW();
    state = wait_us(I2C_HALF_PERIOD_US, deadline);  /* 确保SCL低. */
    if (state != IIC_STATE_EOK) {
        SDA_UP();
        return state;
    }
    if (nack) 
        SDA_UP();   /* NACK. */
    else 
        SDA_LOW();  /* ACK. */
    state = wait_us(I2C_HALF_PERIOD_US, deadline);
    if (state != IIC_STATE_EOK) {
        SDA_UP();
        return state;
    }

    state = i2c_scl_high(deadline);     /* 送出第九个时钟. */
    if (state != IIC_STATE_EOK) {
        SCL_LOW();
        SDA_UP();
        return state;
    }
    state = wait_us(I2C_HALF_PERIOD_US, deadline);  /* 留出时间供从机读取. */
    SCL_LOW();      /* 成功或超时都结束高电平阶段. */
    if (state != IIC_STATE_EOK) {
        SDA_UP();
        return state;
    }

    state = wait_us(I2C_HALF_PERIOD_US, deadline);  /* SCL下降后保持应答，再交出SDA. */
    SDA_UP();
    return state;
}

/* 只接收八位，成功才更新*out；第九拍另由i2c_send_ack_nack完成。 */
static I2C_State_t i2c_read_a_byte( const I2C_Deadline *deadline, uint8_t *out ) {
    if (!deadline || !out)
        return IIC_STATE_EPARAM;
    I2C_State_t state;
    uint8_t byte = 0;
    SCL_LOW();
    SDA_UP();
    state = wait_us(I2C_HALF_PERIOD_US, deadline);
    if (state != IIC_STATE_EOK)
        return state;
    for(uint8_t i = 0; i < 8; i++) {
        state = i2c_scl_high(deadline);  /* 实际变高之后，才开始保持和采样. */
        if (state != IIC_STATE_EOK) {
            SCL_LOW();
            return state;
        }
        state = wait_us(I2C_HALF_PERIOD_US, deadline);
        if (state != IIC_STATE_EOK) {
            SCL_LOW();
            return state;
        }
        byte |= (SDA_READ() << (7u - i));
        SCL_LOW();
        state = wait_us(I2C_HALF_PERIOD_US, deadline);
        if (state != IIC_STATE_EOK)
            return state;
    }
    *out = byte;
    return IIC_STATE_EOK;
}

static I2C_State_t i2c_abort(I2C_State_t cause)
{
    I2C_Deadline cleanup;
    I2C_State_t state;

    if (i2c_started) {
        state = i2c_deadline_init(&cleanup, I2C_CLEANUP_TIMEOUT_MS);
        if (state == IIC_STATE_EOK) {
            state = i2c_send_stop(&cleanup);
            if (state != IIC_STATE_EOK) cause = state;
        }
    }
    i2c_release_lines();
    return cause;
}

void HX_I2C_Init(void)
{
    GPIO_InitTypeDef init_structure = {0};
    I2C_SDA_PORT_CLK_EN();
    I2C_SCL_PORT_CLK_EN();

    /* Preload ODR before switching to output: avoid a low pulse at init. */
    SDA_UP();
    SCL_UP();
    init_structure.Mode = GPIO_MODE_OUTPUT_OD;
    init_structure.Pull = GPIO_NOPULL;  /* External pull-ups on this board. */
    init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    init_structure.Pin = I2C_SCL_PIN;
    HAL_GPIO_Init(I2C_SCL_PORT, &init_structure);
    init_structure.Pin = I2C_SDA_PIN;
    HAL_GPIO_Init(I2C_SDA_PORT, &init_structure);

    i2c_started = 0u;
    i2c_initialized = 1u;
    LOG_INFO("I2C Init Success.");
}

I2C_State_t I2C_Start_Send(uint32_t devAddr, uint8_t rw, uint32_t timeouts_ms)
{
    I2C_Deadline deadline;
    I2C_State_t state;

    /* Ordinary 7-bit targets only; no general call or reserved addresses. */
    if (devAddr < 0x08u || devAddr > 0x77u || rw > 1u) 
        return i2c_entry_error(IIC_STATE_EPARAM);
    state = i2c_deadline_init(&deadline, timeouts_ms);
    if (state != IIC_STATE_EOK) 
        return i2c_entry_error(state);

    state = i2c_send_start(&deadline);
    if (state == IIC_STATE_EOK) {
        state = i2c_send_a_byte((uint8_t)((devAddr << 1u) | rw), &deadline);
    }
    if (state == IIC_STATE_EOK) 
        state = i2c_wait_ack(&deadline);    /* 地址送出成功. 等待对端ACK. */
    return state == IIC_STATE_EOK ? state : i2c_abort(state);
}

I2C_State_t I2C_Write_Byte(uint8_t byte, uint32_t timeouts_ms)
{
    I2C_Deadline deadline;
    I2C_State_t state;

    state = i2c_deadline_init(&deadline, timeouts_ms);
    if (state != IIC_STATE_EOK) 
        return i2c_entry_error(state);

    if (!i2c_started) 
        return IIC_STATE_EPARAM;  /* 通信未启动. 不应发出信号波形. 拦截本次操作. */

    state = i2c_send_a_byte(byte, &deadline);
    if (state == IIC_STATE_EOK) 
        state = i2c_wait_ack(&deadline);    /* 一字节发送完毕. 等待ACK. */
    return state == IIC_STATE_EOK ? state : i2c_abort(state);
}

I2C_State_t I2C_Stop(uint32_t timeouts_ms)
{
    I2C_Deadline deadline;
    I2C_State_t state = i2c_deadline_init(&deadline, timeouts_ms);
    if (state != IIC_STATE_EOK) 
        return i2c_entry_error(state);
    if (i2c_started) 
        return i2c_send_stop(&deadline);

    i2c_release_lines();
    return i2c_wait_idle(&deadline);
}

I2C_State_t I2C_Read_Byte(uint8_t *out, uint32_t timeouts_ms) {
    if (out == NULL)
        return i2c_entry_error(IIC_STATE_EPARAM);
    I2C_Deadline deadline;
    I2C_State_t state;
    state = i2c_deadline_init(&deadline, timeouts_ms);
    if (state != IIC_STATE_EOK)
        return i2c_entry_error(state);
    if (!i2c_started)
        return IIC_STATE_EPARAM;  /* 未START时不产生读取波形. */
    
    uint8_t recv = 0;
    state = i2c_read_a_byte(&deadline, &recv);
    if (state != IIC_STATE_EOK) 
        return i2c_abort(state);    /* 读取错误：尝试STOP并释放本机引脚. */
    *out = recv;
    return IIC_STATE_EOK;
}

I2C_State_t I2c_Send_ACK_NACK( uint8_t ACK, uint32_t timeout_ms ) {
    uint8_t nack;
    if (ACK >= 1) 
        nack = 0;
    else 
        nack = 1;   
    I2C_Deadline deadline;
    I2C_State_t state;
    state = i2c_deadline_init(&deadline, timeout_ms);
    if (state != IIC_STATE_EOK)
        return i2c_entry_error(state);
    if (!i2c_started)
        return IIC_STATE_EPARAM;  /* 未START时不产生第九拍. */
    state = i2c_send_ack_nack(nack, &deadline);
    if (state != IIC_STATE_EOK)
        return i2c_abort(state);
    return IIC_STATE_EOK;
}
