/*
 * encoder.c - 四轮霍尔编码器 AB 相 4 倍频解码
 *
 * 车轮编号: A=左前，B=左后，C=右后，D=右前。
 */
#include "encoder.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define ENCODER_A_DIRECTION_INVERTED  0 /* A 左前编码器：置 1 时反转计数符号。 */
#define ENCODER_B_DIRECTION_INVERTED  0 /* B 左后编码器：置 1 时反转计数符号。 */
#define ENCODER_C_DIRECTION_INVERTED  0 /* C 右后编码器：置 1 时反转计数符号。 */
#define ENCODER_D_DIRECTION_INVERTED  0 /* D 右前编码器：置 1 时反转计数符号。 */

typedef struct {
    GPIO_Regs *a_port;
    uint32_t a_pin;
    GPIO_Regs *b_port;
    uint32_t b_pin;
    volatile int32_t count;
    uint8_t state;
    int8_t direction;
} EncoderChannel_t;

static EncoderChannel_t sEncoders[ENCODER_COUNT] = {
    { GPIO_AB_ALA_PORT, GPIO_AB_ALA_PIN, GPIO_AB_ALB_PORT, GPIO_AB_ALB_PIN,
      0, 0, ENCODER_A_DIRECTION_INVERTED ? -1 : 1 },
    { GPIO_AB_BRA_PORT, GPIO_AB_BRA_PIN, GPIO_AB_BRB_PORT, GPIO_AB_BRB_PIN,
      0, 0, ENCODER_B_DIRECTION_INVERTED ? -1 : 1 },
    { GPIO_AB_CLA_PORT, GPIO_AB_CLA_PIN, GPIO_AB_CLB_PORT, GPIO_AB_CLB_PIN,
      0, 0, ENCODER_C_DIRECTION_INVERTED ? -1 : 1 },
    { GPIO_AB_DRA_PORT, GPIO_AB_DRA_PIN, GPIO_AB_DRB_PORT, GPIO_AB_DRB_PIN,
      0, 0, ENCODER_D_DIRECTION_INVERTED ? -1 : 1 }
};

/* 索引为 (旧 AB << 2) | 新 AB，state=(B << 1) | A。 */
static const int8_t sDecode4x[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static uint8_t encoder_read_state(const EncoderChannel_t *encoder,
                                  uint32_t pins_a, uint32_t pins_b)
{
    uint8_t state = 0;
    uint32_t a_pins = (encoder->a_port == GPIOA) ? pins_a : pins_b;
    uint32_t b_pins = (encoder->b_port == GPIOA) ? pins_a : pins_b;
    if ((a_pins & encoder->a_pin) != 0U) state |= 1U;
    if ((b_pins & encoder->b_pin) != 0U) state |= 2U;
    return state;
}

static void encoder_process(EncoderChannel_t *encoder,
                            uint32_t status_a, uint32_t status_b,
                            uint32_t pins_a, uint32_t pins_b)
{
    bool changed = false;

    if (encoder->a_port == GPIOA) changed |= (status_a & encoder->a_pin) != 0U;
    else                          changed |= (status_b & encoder->a_pin) != 0U;
    if (encoder->b_port == GPIOA) changed |= (status_a & encoder->b_pin) != 0U;
    else                          changed |= (status_b & encoder->b_pin) != 0U;
    if (!changed) return;

    uint8_t current = encoder_read_state(encoder, pins_a, pins_b);
    uint8_t index = (uint8_t)((encoder->state << 2) | current);
    encoder->count += (int32_t)(sDecode4x[index] * encoder->direction);
    encoder->state = current;
}

void Encoder_Init(void)
{
    uint32_t i;
    const uint32_t mask_a = GPIO_AB_ALA_PIN | GPIO_AB_ALB_PIN |
                            GPIO_AB_BRA_PIN | GPIO_AB_BRB_PIN |
                            GPIO_AB_CLA_PIN | GPIO_AB_DRB_PIN;
    const uint32_t mask_b = GPIO_AB_CLB_PIN | GPIO_AB_DRA_PIN;
    uint32_t pins_a = DL_GPIO_readPins(GPIOA, mask_a);
    uint32_t pins_b = DL_GPIO_readPins(GPIOB, mask_b);

    for (i = 0; i < ENCODER_COUNT; i++) {
        sEncoders[i].state = encoder_read_state(&sEncoders[i],
                                                 pins_a, pins_b);
    }

    DL_GPIO_clearInterruptStatus(GPIOA, mask_a);
    DL_GPIO_clearInterruptStatus(GPIOB, mask_b);
    DL_GPIO_enableInterrupt(GPIOA, mask_a);
    DL_GPIO_enableInterrupt(GPIOB, mask_b);
    NVIC_SetPriority(GPIOA_INT_IRQn, 1);
    NVIC_SetPriority(GPIOB_INT_IRQn, 1);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Encoder_GetCounts(int32_t counts[ENCODER_COUNT])
{
    uint32_t i;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < ENCODER_COUNT; i++) counts[i] = sEncoders[i].count;
    if (primask == 0U) __enable_irq();
}

void Encoder_Reset(void)
{
    uint32_t i;
    const uint32_t mask_a = GPIO_AB_ALA_PIN | GPIO_AB_ALB_PIN |
                            GPIO_AB_BRA_PIN | GPIO_AB_BRB_PIN |
                            GPIO_AB_CLA_PIN | GPIO_AB_DRB_PIN;
    const uint32_t mask_b = GPIO_AB_CLB_PIN | GPIO_AB_DRA_PIN;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t pins_a = DL_GPIO_readPins(GPIOA, mask_a);
    uint32_t pins_b = DL_GPIO_readPins(GPIOB, mask_b);
    for (i = 0; i < ENCODER_COUNT; i++) {
        sEncoders[i].count = 0;
        sEncoders[i].state = encoder_read_state(&sEncoders[i],
                                                 pins_a, pins_b);
    }
    if (primask == 0U) __enable_irq();
}

void GROUP1_IRQHandler(void)
{
    uint32_t i;
    const uint32_t mask_a = GPIO_AB_ALA_PIN | GPIO_AB_ALB_PIN |
                            GPIO_AB_BRA_PIN | GPIO_AB_BRB_PIN |
                            GPIO_AB_CLA_PIN | GPIO_AB_DRB_PIN;
    const uint32_t mask_b = GPIO_AB_CLB_PIN | GPIO_AB_DRA_PIN;
    uint32_t status_a = DL_GPIO_getEnabledInterruptStatus(GPIOA, mask_a);
    uint32_t status_b = DL_GPIO_getEnabledInterruptStatus(GPIOB, mask_b);
    uint32_t pins_a = DL_GPIO_readPins(GPIOA, mask_a);
    uint32_t pins_b = DL_GPIO_readPins(GPIOB, mask_b);

    if (status_a != 0U) DL_GPIO_clearInterruptStatus(GPIOA, status_a);
    if (status_b != 0U) DL_GPIO_clearInterruptStatus(GPIOB, status_b);
    for (i = 0; i < ENCODER_COUNT; i++) {
        encoder_process(&sEncoders[i], status_a, status_b, pins_a, pins_b);
    }
}
