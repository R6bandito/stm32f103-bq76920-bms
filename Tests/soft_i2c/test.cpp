// Host-side protocol model. The actual C driver is included unchanged; mock
// BSRR/IDR register objects observe edges and emulate a slave's ninth-clock ACK.
#include <cassert>
#include <cstdio>
#include <vector>
#include "stubs/stm32f1xx_hal.h"

uint32_t SystemCoreClock = 72000000u;
GPIO_TypeDef mock_gpio_b;

struct Bus {
    uint64_t now = 0;
    uint64_t release_clock_at = 0;
    uint32_t reads = 0;
    uint32_t freeze_after = 0;
    uint32_t stretch_cycles = 0;
    bool counter_running = true;
    bool master_scl = true, master_sda = true;
    bool scl = true, sda = true;
    bool stuck_scl = false, stuck_sda = false;
    bool slave_ack = false, active = false;
    bool read_mode = false, slave_data_low = false;
    bool freeze_on_next_rise = false;
    size_t tx_index = 0;
    std::vector<uint8_t> tx_bytes;
    int nack_byte = -1;
    int jam_after_byte = -1;
    unsigned bit = 0, starts = 0, stops = 0, clocks = 0, gpio_inits = 0;
    uint8_t assembling = 0;
    std::vector<uint8_t> bytes;
    std::vector<bool> acks;

    void begin_read(std::vector<uint8_t> data) {
        assert(!scl && !slave_ack && !data.empty());
        tx_bytes = data; tx_index = 0; bit = 0;
        read_mode = true;
        slave_data_low = !(tx_bytes[0] & 0x80u);
        refresh();
    }

    void refresh() {
        bool new_scl = master_scl && !stuck_scl && now >= release_clock_at;
        bool new_sda = master_sda && !stuck_sda && !slave_ack && !slave_data_low;
        if (scl && new_scl && sda != new_sda) {
            if (!new_sda) {
                starts++; active = true; bit = 0; assembling = 0;
            } else {
                stops++; active = false; bit = 0; assembling = 0;
            }
        }
        if (!scl && new_scl && active) {
            clocks++;
            if (freeze_on_next_rise) { counter_running = false; freeze_on_next_rise = false; }
            if (read_mode && bit < 8) {
                assert(master_sda); // The master must release all eight data bits.
                bit++;
            } else if (bit < 8) {
                assembling = uint8_t((assembling << 1) | (new_sda ? 1u : 0u));
                if (++bit == 8) bytes.push_back(assembling);
            } else {
                acks.push_back(!new_sda);
                bit = 9;
            }
        }
        if (scl && !new_scl && active) {
            if (read_mode) {
                if (bit == 9) {
                    tx_index++;
                    bit = 0;
                    if (!acks.back() || tx_index >= tx_bytes.size()) read_mode = false;
                }
                slave_data_low = read_mode && bit < 8 && !(tx_bytes[tx_index] & (1u << (7u - bit)));
            } else if (bit == 8) {
                slave_ack = int(bytes.size() - 1) != nack_byte;
                if (int(bytes.size() - 1) == jam_after_byte) stuck_scl = true;
            } else if (bit == 9) {
                slave_ack = false; bit = 0; assembling = 0;
            }
            new_sda = master_sda && !stuck_sda && !slave_ack && !slave_data_low;
        }
        scl = new_scl;
        sda = new_sda;
    }

    void write(uint32_t value) {
        refresh();
        bool previous_master_scl = master_scl;
        if (value & GPIO_PIN_13) master_sda = true;
        if (value & (GPIO_PIN_13 << 16)) master_sda = false;
        if (value & GPIO_PIN_14) master_scl = true;
        if (value & (GPIO_PIN_14 << 16)) master_scl = false;
        if (!previous_master_scl && master_scl) release_clock_at = now + stretch_cycles;
        refresh();
    }
} bus;

void MockSetReset::operator=(uint32_t value) { bus.write(value); }
MockInput::operator uint32_t() const {
    bus.refresh();
    return (bus.scl ? GPIO_PIN_14 : 0u) | (bus.sda ? GPIO_PIN_13 : 0u);
}
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *init) {
    assert(bus.master_sda && bus.master_scl); // ODR was preloaded before output.
    assert(init->Mode == GPIO_MODE_OUTPUT_OD && init->Pull == GPIO_NOPULL);
    assert(init->Speed == GPIO_SPEED_FREQ_LOW);
    bus.gpio_inits++;
}
uint32_t DWT_GetCount(void) {
    bus.reads++;
    if (bus.counter_running && (!bus.freeze_after || bus.reads < bus.freeze_after)) bus.now += 24u;
    bus.refresh();
    return uint32_t(bus.now);
}
uint32_t DWT_Elapsed(uint32_t start) { return uint32_t(DWT_GetCount() - start); }

#include "../../Hardware/i2c_driver.c"

static void reset_bus() {
    bus = Bus{};
    HX_I2C_Init();
    assert(bus.gpio_inits == 2);
    assert(bus.master_sda && bus.master_scl);
}

int main() {
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_NOT_READY);

    reset_bus();
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert((bus.bytes == std::vector<uint8_t>{0x10}));
    assert(bus.clocks == 9 && bus.acks.size() == 1 && !bus.scl);
    assert(I2C_Write_Byte(0xA5, 1) == IIC_STATE_EOK);
    assert(I2C_Write_Byte(0x81, 1) == IIC_STATE_EOK);
    assert((bus.bytes == std::vector<uint8_t>{0x10, 0xA5, 0x81}));
    assert(bus.clocks == 27);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    assert(bus.starts == 1 && bus.stops == 1 && bus.sda && bus.scl);
    std::puts("PASS: init preload, MSB byte order, exactly nine clocks per byte, STOP");

    reset_bus();
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    assert(bus.starts == 2 && bus.stops == 0);
    assert((bus.bytes == std::vector<uint8_t>{0x10, 0x11}));
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    assert(bus.starts == 3 && bus.stops == 2);
    std::puts("PASS: repeated START and fresh START after STOP");

    reset_bus();
    bus.nack_byte = 0;
    I2C_Deadline deadline;
    assert(i2c_deadline_init(&deadline, 1) == IIC_STATE_EOK);
    assert(i2c_send_start(&deadline) == IIC_STATE_EOK);
    assert(i2c_send_a_byte(0x10, &deadline) == IIC_STATE_EOK);
    assert(i2c_wait_ack(&deadline) == IIC_STATE_NORESPONSE);
    assert(!bus.scl && bus.clocks == 9);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    reset_bus();
    bus.nack_byte = 0;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_NORESPONSE);
    assert(bus.stops == 1 && bus.master_scl && bus.master_sda);
    bus.nack_byte = -1;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    std::puts("PASS: NACK finishes ninth clock; public error cleanup sends STOP");

    reset_bus();
    bus.stuck_sda = true; bus.sda = false;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_BUSY);
    assert(bus.starts == 0 && bus.stops == 0);
    assert(bus.master_sda && bus.master_scl);
    reset_bus();
    bus.stuck_scl = true; bus.scl = false;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_BUSY);
    assert(bus.starts == 0);
    std::puts("PASS: initial idle checks BOTH lines and does not clock a busy bus");

    reset_bus();
    bus.stretch_cycles = 72u * 8u;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert((bus.bytes == std::vector<uint8_t>{0x10}));
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    reset_bus();
    bus.jam_after_byte = 0;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_TIMEOUT);
    assert(bus.master_scl && bus.master_sda && !i2c_started);
    assert(!bus.scl); // Released ODR does not falsely imply physical STOP/idle.
    std::puts("PASS: actual SCL high wait, stretching, stuck-clock timeout and release");

    reset_bus();
    bus.stretch_cycles = 72u * 150u;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_TIMEOUT);
    assert(!i2c_started && bus.master_scl && bus.master_sda);
    reset_bus();
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    bus.stuck_sda = true; bus.sda = false;
    assert(I2C_Stop(1) == IIC_STATE_BUSY);
    assert(!i2c_started && bus.master_scl && bus.master_sda);
    std::puts("PASS: shared API deadline and STOP does not falsely report stuck SDA idle");

    reset_bus();
    assert(I2C_Write_Byte(0x12, 1) == IIC_STATE_EPARAM);
    assert(I2C_Start_Send(0x08, 0, 0) == IIC_STATE_EPARAM);
    assert(I2C_Start_Send(0x08, 0, UINT32_MAX) == IIC_STATE_EPARAM);
    assert(I2C_Start_Send(0x80, 0, 1) == IIC_STATE_EPARAM);
    assert(I2C_Start_Send(0x08, 2, 1) == IIC_STATE_EPARAM);
    assert(bus.starts == 0);
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert(I2C_Start_Send(0x08, 2, 1) == IIC_STATE_EPARAM);
    assert(!i2c_started && bus.stops == 1);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    std::puts("PASS: bad public arguments abort an active transaction, not an idle bus");

    reset_bus();
    bus.counter_running = false;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_NOT_READY);
    reset_bus();
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    bus.counter_running = false;
    assert(I2C_Stop(1) == IIC_STATE_NOT_READY);
    assert(!i2c_started && bus.master_scl && bus.master_sda);
    reset_bus();
    bus.freeze_after = 50;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_TIMEOUT);
    assert(bus.master_scl && bus.master_sda);
    reset_bus();
    bus.now = uint64_t(UINT32_MAX) - 500;
    assert(I2C_Start_Send(0x08, 0, 1) == IIC_STATE_EOK);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    std::puts("PASS: unavailable/stalled DWT bounded exits and counter wraparound");

    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    bus.begin_read({0xA5, 0x3C});
    bus.stretch_cycles = 72u * 8u;
    assert(i2c_deadline_init(&deadline, 2) == IIC_STATE_EOK);
    unsigned before = bus.clocks;
    uint8_t out = 0;
    assert(i2c_read_a_byte(&deadline, &out) == IIC_STATE_EOK && out == 0xA5);
    assert(bus.clocks == before + 8 && !bus.scl);
    assert(i2c_send_ack_nack(0, &deadline) == IIC_STATE_EOK);
    assert(bus.clocks == before + 9 && bus.acks.back() && !bus.scl && bus.master_sda);
    assert(i2c_read_a_byte(&deadline, &out) == IIC_STATE_EOK && out == 0x3C);
    assert(i2c_send_ack_nack(1, &deadline) == IIC_STATE_EOK);
    assert(bus.clocks == before + 18 && !bus.acks.back() && !bus.scl && bus.master_sda);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    std::puts("PASS: read MSB bytes with SCL stretching; master ACK then final NACK");

    for (uint8_t value : {0x00, 0xFF, 0x81}) {
        reset_bus();
        assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
        bus.begin_read({value});
        assert(i2c_deadline_init(&deadline, 1) == IIC_STATE_EOK);
        assert(i2c_read_a_byte(&deadline, &out) == IIC_STATE_EOK && out == value);
        assert(i2c_send_ack_nack(1, &deadline) == IIC_STATE_EOK);
        assert(I2C_Stop(1) == IIC_STATE_EOK);
    }
    std::puts("PASS: all-zero/all-one and asymmetric received bytes");

    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    bus.begin_read({0xA5});
    assert(i2c_deadline_init(&deadline, 1) == IIC_STATE_EOK);
    out = 0xCD;
    before = bus.clocks;
    assert(i2c_read_a_byte(nullptr, &out) == IIC_STATE_EPARAM);
    assert(i2c_read_a_byte(&deadline, nullptr) == IIC_STATE_EPARAM);
    assert(i2c_send_ack_nack(2, &deadline) == IIC_STATE_EPARAM);
    assert(i2c_send_ack_nack(0, nullptr) == IIC_STATE_EPARAM);
    assert(bus.clocks == before);
    bus.stuck_scl = true;
    assert(i2c_read_a_byte(&deadline, &out) == IIC_STATE_TIMEOUT);
    assert(out == 0xCD && !bus.master_scl && bus.master_sda);
    std::puts("PASS: bad read parameters, stuck clock, unchanged output on failure");

    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    bus.begin_read({0xA5, 0x81});
    assert(i2c_deadline_init(&deadline, 1) == IIC_STATE_EOK);
    assert(i2c_read_a_byte(&deadline, &out) == IIC_STATE_EOK);
    bus.freeze_on_next_rise = true;
    assert(i2c_send_ack_nack(0, &deadline) == IIC_STATE_TIMEOUT);
    assert(!bus.master_scl && bus.master_sda);
    std::puts("PASS: ACK high-phase failure lowers SCL and releases SDA before return");

    reset_bus();
    out = 0xCD;
    before = bus.clocks;
    assert(I2C_Read_Byte(&out, 1) == IIC_STATE_EPARAM);
    assert(I2c_Send_ACK_NACK(1, 1) == IIC_STATE_EPARAM);
    assert(bus.clocks == before && bus.starts == 0 && bus.stops == 0 && out == 0xCD);
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    bus.begin_read({0xA5, 0x3C});
    assert(I2C_Read_Byte(&out, 1) == IIC_STATE_EOK && out == 0xA5);
    assert(I2c_Send_ACK_NACK(2, 1) == IIC_STATE_EOK); // >=1, as requested.
    assert(bus.acks.back() && i2c_started && !bus.scl && bus.master_sda);
    assert(I2C_Read_Byte(&out, 1) == IIC_STATE_EOK && out == 0x3C);
    assert(I2c_Send_ACK_NACK(0, 1) == IIC_STATE_EOK);
    assert(!bus.acks.back() && i2c_started);
    assert(I2C_Stop(1) == IIC_STATE_EOK);
    std::puts("PASS: public read/response wrappers; nonzero ACK, zero NACK, no START guard");

    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    assert(I2C_Read_Byte(nullptr, 1) == IIC_STATE_EPARAM);
    assert(!i2c_started && bus.stops == 1 && bus.master_sda && bus.master_scl);
    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    assert(I2C_Read_Byte(&out, 0) == IIC_STATE_EPARAM);
    assert(!i2c_started && bus.stops == 1);
    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    assert(I2c_Send_ACK_NACK(1, 0) == IIC_STATE_EPARAM);
    assert(!i2c_started && bus.stops == 1);
    reset_bus();
    assert(I2C_Start_Send(0x08, 1, 1) == IIC_STATE_EOK);
    bus.begin_read({0xA5});
    bus.stuck_scl = true;
    out = 0xCD;
    assert(I2C_Read_Byte(&out, 1) == IIC_STATE_TIMEOUT);
    assert(!i2c_started && bus.master_sda && bus.master_scl && out == 0xCD);
    std::puts("PASS: public read/response errors perform driver-owned cleanup");
    std::puts("All software-I2C host protocol checks passed (not a hardware measurement).");
}
