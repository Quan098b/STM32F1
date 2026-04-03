#ifndef __I2C__H__
#define __I2C__H__

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/* Generic I2C initialization */
void I2C_Peripheral_Init(I2C_TypeDef* I2Cx);

/* Generic transmit and receive functions */
int I2C_Transmit(I2C_TypeDef* I2Cx, uint8_t slaveAddr, uint8_t *pData, uint16_t size);
int I2C_Receive(I2C_TypeDef* I2Cx, uint8_t slaveAddr, uint8_t *pData, uint16_t size);

/* Wrapper functions required by TCS34725 */
int I2C_writeByte(I2C_TypeDef* I2Cx, uint8_t data, uint8_t address);
int I2C_readByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t *data);
int I2C_writeTwoByte(I2C_TypeDef* I2Cx, uint8_t* data, uint8_t address);
int I2C_readTwoByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data);

#endif /* __I2C__H__ */