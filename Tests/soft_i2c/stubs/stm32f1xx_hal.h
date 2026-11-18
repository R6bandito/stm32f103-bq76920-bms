#ifndef SOFT_I2C_TEST_HAL_H
#define SOFT_I2C_TEST_HAL_H
#include <cstdint>

struct MockSetReset { void operator=(uint32_t value); };
struct MockInput { operator uint32_t() const; };
struct GPIO_TypeDef { MockSetReset BSRR; MockInput IDR; };
extern GPIO_TypeDef mock_gpio_b;
#define GPIOB (&mock_gpio_b)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_MODE_OUTPUT_OD 1u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 2u
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __NOP() ((void)0)
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
#endif
