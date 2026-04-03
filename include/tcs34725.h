#ifndef __TCS34725__H__
#define __TCS34725__H__

#include "stm32f10x.h"
#include "i2c.h"

#define TCS34725_ADDRESS            (0x29 << 1)
#define TCS34725_COMMAND_BIT        (0x80)
#define TCS34725_ENABLE             (0x00)
#define TCS34725_ENABLE_AEN         (0x02)
#define TCS34725_ENABLE_PON         (0x01)
#define TCS34725_ATIME              (0x01)
#define TCS34725_CONTROL            (0x0F)
#define TCS34725_ID                 (0x12)

#define TCS34725_CDATAL             (0x14)
#define TCS34725_CDATAH             (0x15)
#define TCS34725_RDATAL             (0x16)
#define TCS34725_RDATAH             (0x17)
#define TCS34725_GDATAL             (0x18)
#define TCS34725_GDATAH             (0x19)
#define TCS34725_BDATAL             (0x1A)
#define TCS34725_BDATAH             (0x1B)

#define TCS34725_INTEGRATIONTIME_24MS   0xF6
#define TCS34725_INTEGRATIONTIME_101MS  0xD5
#define TCS34725_GAIN_4X                0x01

void Delay_ms(uint32_t ms);

int write8(I2C_TypeDef *I2Cx, uint8_t reg, uint32_t value);
int read8(I2C_TypeDef *I2Cx, uint8_t reg, uint8_t *value);
int read16(I2C_TypeDef *I2Cx, uint8_t reg, uint16_t *ret);
int enable(I2C_TypeDef *I2Cx);
int disable(I2C_TypeDef *I2Cx);
int setIntegrationTime(I2C_TypeDef *I2Cx, uint8_t it);
int setGain(I2C_TypeDef *I2Cx, uint8_t gain);

void tcs3272_init(I2C_TypeDef *I2Cx);
void getRawData(I2C_TypeDef *I2Cx, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(I2C_TypeDef *I2Cx, int *R, int *G, int *B, uint16_t *c);

#endif /* __TCS34725__H__ */