#include <assert.h>
#include <stddef.h>
#include <stdio.h>

/* Test the actual static implementation without exporting it in production. */
#include "../../Hardware/bq76920_driver.c"

/* Independent modulo-2 long division: append 8 zeros, divide by 0x107. */
static uint8_t reference_crc(const uint8_t *data, uint8_t len)
{
    uint8_t bits[80] = {0};
    unsigned int i, j;
    uint8_t remainder = 0;
    assert(len <= 9);
    for (i = 0; i < (unsigned int)len * 8u; ++i)
        bits[i] = (uint8_t)((data[i / 8u] >> (7u - i % 8u)) & 1u);
    for (i = 0; i < (unsigned int)len * 8u; ++i) {
        if (bits[i]) {
            for (j = 0; j < 9u; ++j)
                bits[i + j] ^= (uint8_t)((0x107u >> (8u - j)) & 1u);
        }
    }
    for (i = (unsigned int)len * 8u; i < (unsigned int)len * 8u + 8u; ++i)
        remainder = (uint8_t)((remainder << 1u) | bits[i]);
    return remainder;
}

int main(void)
{
    const uint8_t check[] = "123456789";
    uint8_t input[3];
    unsigned int value, reg;

    assert(bq76920_crc8(NULL, 0) == 0);
    assert(bq76920_crc8(check, 9) == 0xF4);
    input[0] = 0x80;
    assert(bq76920_crc8(input, 1) == 0x89);
    for (value = 0; value < 256u; ++value) {
        input[0] = (uint8_t)value;
        assert(bq76920_crc8(input, 1) == reference_crc(input, 1));
        input[0] = 0x11; input[1] = (uint8_t)value;
        assert(bq76920_crc8(input, 2) == reference_crc(input, 2));
        for (reg = 0; reg < 256u; ++reg) {
            input[0] = 0x10; input[1] = (uint8_t)reg; input[2] = (uint8_t)value;
            assert(bq76920_crc8(input, 3) == reference_crc(input, 3));
        }
    }
    puts("PASS: empty input, check vector 0xF4, teaching example 0x80 -> 0x89");
    puts("PASS: modulo-2 division matches all one-byte inputs and BQ read/write CRC inputs");
    return 0;
}
