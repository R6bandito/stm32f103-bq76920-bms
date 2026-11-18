#ifndef SOFT_I2C_TEST_DWT_H
#define SOFT_I2C_TEST_DWT_H
#include <cstdint>
uint32_t DWT_GetCount(void);
uint32_t DWT_Elapsed(uint32_t start);
#endif
