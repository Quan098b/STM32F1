#include "tcs34725.h"
#include "usart.h"
#include "i2c.h"
#include <string.h>

uint8_t _tcs34725Initialised[2] = {0, 0};

static uint8_t getI2CIndex(I2C_TypeDef *I2Cx)
{
    return (I2Cx == I2C1) ? 0U : 1U;
}

static void uart_puts_fast(const char *s)
{
    USART_Send_bytes(s, (uint16_t)strlen(s));
}

void Delay_ms(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 8000U; j++)
            __NOP();
}

int write8(I2C_TypeDef *I2Cx, uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (uint8_t)(value & 0xFFU);
    return I2C_writeTwoByte(I2Cx, txBuffer, TCS34725_ADDRESS);
}

int read8(I2C_TypeDef *I2Cx, uint8_t reg, uint8_t *value)
{
    uint8_t cmd = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    if (!I2C_writeByte(I2Cx, cmd, TCS34725_ADDRESS)) return 0;
    if (!I2C_readByte(I2Cx, TCS34725_ADDRESS, value)) return 0;
    return 1;
}

int read16(I2C_TypeDef *I2Cx, uint8_t reg, uint16_t *ret)
{
    uint8_t cmd = (uint8_t)(TCS34725_COMMAND_BIT | reg);
    uint8_t rxBuffer[2];
    if (!I2C_writeByte(I2Cx, cmd, TCS34725_ADDRESS)) return 0;
    if (!I2C_readTwoByte(I2Cx, TCS34725_ADDRESS, rxBuffer)) return 0;
    *ret = ((uint16_t)rxBuffer[1] << 8) | rxBuffer[0];
    return 1;
}

int enable(I2C_TypeDef *I2Cx)
{
    if (!write8(I2Cx, TCS34725_ENABLE, TCS34725_ENABLE_PON)) return 0;
    Delay_ms(3);
    if (!write8(I2Cx, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) return 0;
    Delay_ms(24);
    return 1;
}

int disable(I2C_TypeDef *I2Cx)
{
    uint8_t reg;
    if (!read8(I2Cx, TCS34725_ENABLE, &reg)) return 0;
    if (!write8(I2Cx, TCS34725_ENABLE, reg & (uint8_t)~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN))) return 0;
    return 1;
}

int setIntegrationTime(I2C_TypeDef *I2Cx, uint8_t it)
{
    return write8(I2Cx, TCS34725_ATIME, it);
}

int setGain(I2C_TypeDef *I2Cx, uint8_t gain)
{
    return write8(I2Cx, TCS34725_CONTROL, gain);
}

void tcs3272_init(I2C_TypeDef *I2Cx)
{
    uint8_t readValue = 0;
    uint8_t idx = getI2CIndex(I2Cx);

    _tcs34725Initialised[idx] = 0;

    if (!read8(I2Cx, TCS34725_ID, &readValue)) {
        uart_puts_fast("TCS ERR\r\n");
        return;
    }

    if ((readValue != 0x44U) && (readValue != 0x4DU)) {
        uart_puts_fast("TCS ID ERR\r\n");
        return;
    }

    if (!setIntegrationTime(I2Cx, TCS34725_INTEGRATIONTIME_101MS)) {
        uart_puts_fast("TCS AT ERR\r\n");
        return;
    }

    Delay_ms(3);

    if (!setGain(I2Cx, TCS34725_GAIN_4X)) {
        uart_puts_fast("TCS G ERR\r\n");
        return;
    }

    if (!enable(I2Cx)) {
        uart_puts_fast("TCS EN ERR\r\n");
        return;
    }

    _tcs34725Initialised[idx] = 1;
    uart_puts_fast("TCS OK\r\n");
}

void getRawData(I2C_TypeDef *I2Cx, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    uint8_t idx = getI2CIndex(I2Cx);

    if (_tcs34725Initialised[idx] != 1U) {
        *c = 0; *r = 0; *g = 0; *b = 0;
        return;
    }

    if (!read16(I2Cx, TCS34725_CDATAL, c) ||
        !read16(I2Cx, TCS34725_RDATAL, r) ||
        !read16(I2Cx, TCS34725_GDATAL, g) ||
        !read16(I2Cx, TCS34725_BDATAL, b)) {
        *c = 0; *r = 0; *g = 0; *b = 0;
    }
}

void getRGB(I2C_TypeDef *I2Cx, int *R, int *G, int *B, uint16_t *c)
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(I2Cx, &rawRed, &rawGreen, &rawBlue, &rawClear);

    *c = rawClear;
    if (rawClear == 0U) {
        *R = 0; *G = 0; *B = 0;
        return;
    }

    *R = ((int)rawRed * 255) / rawClear;
    *G = ((int)rawGreen * 255) / rawClear;
    *B = ((int)rawBlue * 255) / rawClear;
}
