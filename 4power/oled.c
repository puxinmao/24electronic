/*
 * oled.c - OLED SSD1306 I2C 驱动 (硬件 I2C1, 地址 0x3C)
 *
 * 改编自 wit-oled-hardware-spi 参考项目
 */
#include "oled.h"
#include "oledfont.h"
#include "ti_msp_dl_config.h"

#define I2C_WAIT_LOOP_LIMIT  (CPUCLK_FREQ / 500U)

/* ---------- 简单延时（使用 SysTick 风格） ---------- */
static bool g_oled_available = true;

static void oled_delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000);
    }
}

static bool oled_wait_status(uint32_t mask, bool asserted)
{
    uint32_t limit = I2C_WAIT_LOOP_LIMIT;

    while (limit-- > 0U) {
        bool set = (DL_I2C_getControllerStatus(I2C_INST) & mask) != 0U;
        if (set == asserted) return true;
    }

    return false;
}

static bool oled_wait_tx_done(void)
{
    uint32_t limit = I2C_WAIT_LOOP_LIMIT;

    while (limit-- > 0U) {
        if (DL_I2C_getRawInterruptStatus(I2C_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            return true;
        }
    }

    return false;
}

static void oled_disable(void)
{
    DL_I2C_resetControllerTransfer(I2C_INST);
    DL_I2C_reset(I2C_INST);
    g_oled_available = false;
}

/* ---------- I2C 总线恢复（SDA 被拉低时用） ---------- */

static void i2c_disable_to_gpio(void)
{
    DL_I2C_reset(I2C_INST);
    DL_GPIO_initDigitalOutput(GPIO_I2C_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_IOMUX_SDA,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_I2C_SCL_PORT, GPIO_I2C_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_SCL_PORT, GPIO_I2C_SCL_PIN);
}

static void i2c_enable_from_gpio(void)
{
    DL_I2C_reset(I2C_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_IOMUX_SDA,
        GPIO_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_IOMUX_SCL,
        GPIO_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_IOMUX_SCL);
    DL_I2C_enablePower(I2C_INST);
    SYSCFG_DL_I2C_init();
    DL_I2C_enableController(I2C_INST);
}

static void oled_i2c_sda_unlock(void)
{
    uint8_t cnt = 0;
    i2c_disable_to_gpio();
    do {
        DL_GPIO_clearPins(GPIO_I2C_SCL_PORT, GPIO_I2C_SCL_PIN);
        oled_delay_ms(1);
        DL_GPIO_setPins(GPIO_I2C_SCL_PORT, GPIO_I2C_SCL_PIN);
        oled_delay_ms(1);
        if (DL_GPIO_readPins(GPIO_I2C_SDA_PORT, GPIO_I2C_SDA_PIN))
            break;
    } while (++cnt < 100);
    i2c_enable_from_gpio();
}

/* ---------- 底层写字节 ---------- */

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t buf[2];

    if (!g_oled_available) return;

    buf[0] = (mode == OLED_CMD) ? 0x00 : 0x40;
    buf[1] = dat;

    if (!oled_wait_status(DL_I2C_CONTROLLER_STATUS_IDLE, true)) {
        oled_disable();
        return;
    }

    DL_I2C_fillControllerTXFIFO(I2C_INST, buf, 2);
    DL_I2C_clearInterruptStatus(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    DL_I2C_startControllerTransfer(I2C_INST, 0x3C,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    if (!oled_wait_tx_done()) {
        oled_disable();
    }
}

/* ---------- 显示函数 ---------- */

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte(x & 0x0f, OLED_CMD);
}

void OLED_ColorTurn(uint8_t i)
{
    OLED_WR_Byte((i == 0) ? 0xA6 : 0xA7, OLED_CMD);
}

void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    } else {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void OLED_Display_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++) OLED_WR_Byte(0, OLED_DATA);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c, sizex = sizey / 2;
    uint16_t i, size1;

    if (sizey == 8) size1 = 6;
    else size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);

    c = chr - ' ';
    OLED_Set_Pos(x, y);
    for (i = 0; i < size1; i++) {
        if (i % sizex == 0 && sizey != 8) OLED_Set_Pos(x, y++);
        if (sizey == 8)
            OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);
        else if (sizey == 16)
            OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);
        else return;
    }
}

static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t, temp, m = 0, enshow = 0;
    if (sizey == 8) m = 2;
    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                continue;
            } else enshow = 1;
        }
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey)
{
    uint8_t j = 0;
    while (chr[j] != '\0') {
        OLED_ShowChar(x, y, chr[j++], sizey);
        if (sizey == 8) x += 6;
        else x += sizey / 2;
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)
{
    uint16_t i, size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    for (i = 0; i < size1; i++) {
        if (i % sizey == 0) OLED_Set_Pos(x, y++);
        if (sizey == 16)
            OLED_WR_Byte(Hzk[no][i], OLED_DATA);
        else return;
    }
}

void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[])
{
    uint16_t j = 0;
    uint8_t i, m;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (i = 0; i < sizey; i++) {
        OLED_Set_Pos(x, i + y);
        for (m = 0; m < sizex; m++) {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

/* ---------- 初始化 ---------- */

void OLED_Init(void)
{
    g_oled_available = true;
    DL_I2C_enableController(I2C_INST);

    if (DL_I2C_getSDAStatus(I2C_INST) == DL_I2C_CONTROLLER_SDA_LOW)
        oled_i2c_sda_unlock();

    oled_delay_ms(200);

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3f, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xd5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);
}
