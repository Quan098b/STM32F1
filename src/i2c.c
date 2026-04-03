#include "i2c.h"

/*
 * i2c.c
 * - Khoi tao I2C1 / I2C2
 * - Co timeout de tranh treo vo han
 * - Co bus recovery + peripheral reset khi gap loi
 * - Ham transmit / receive tra ve:
 *      1 = thanh cong
 *      0 = that bai / timeout
 */

#define I2C_TIMEOUT  ((uint32_t)100000UL)

static void short_delay(volatile uint32_t count)
{
    while (count--) { __NOP(); }
}

static void i2c_gpio_to_od_output(uint16_t scl_pin, uint16_t sda_pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = scl_pin | sda_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, scl_pin | sda_pin);
}

static void i2c_gpio_to_af_od(uint16_t scl_pin, uint16_t sda_pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = scl_pin | sda_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void i2c_get_pins(I2C_TypeDef* I2Cx, uint16_t *scl_pin, uint16_t *sda_pin)
{
    if (I2Cx == I2C1)
    {
        *scl_pin = GPIO_Pin_6;
        *sda_pin = GPIO_Pin_7;
    }
    else
    {
        *scl_pin = GPIO_Pin_10;
        *sda_pin = GPIO_Pin_11;
    }
}

static void i2c_clear_error_flags(I2C_TypeDef* I2Cx)
{
    I2Cx->SR1 &= (uint16_t)~(I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR | I2C_SR1_TIMEOUT);
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;
}

static void i2c_bus_recover(I2C_TypeDef* I2Cx)
{
    uint16_t scl_pin, sda_pin;
    uint8_t i;

    i2c_get_pins(I2Cx, &scl_pin, &sda_pin);

    I2C_Cmd(I2Cx, DISABLE);
    i2c_gpio_to_od_output(scl_pin, sda_pin);

    GPIO_SetBits(GPIOB, sda_pin);
    GPIO_SetBits(GPIOB, scl_pin);
    short_delay(200);

    for (i = 0; i < 9; i++)
    {
        GPIO_SetBits(GPIOB, scl_pin);
        short_delay(200);
        GPIO_ResetBits(GPIOB, scl_pin);
        short_delay(200);
    }

    GPIO_SetBits(GPIOB, sda_pin);
    short_delay(200);
    GPIO_SetBits(GPIOB, scl_pin);
    short_delay(200);

    GPIO_ResetBits(GPIOB, sda_pin);
    short_delay(200);
    GPIO_ResetBits(GPIOB, scl_pin);
    short_delay(200);

    GPIO_SetBits(GPIOB, scl_pin);
    short_delay(200);
    GPIO_SetBits(GPIOB, sda_pin);
    short_delay(200);

    i2c_gpio_to_af_od(scl_pin, sda_pin);
}

static void i2c_peripheral_recover(I2C_TypeDef* I2Cx)
{
    i2c_clear_error_flags(I2Cx);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
    short_delay(200);
    I2C_SoftwareResetCmd(I2Cx, DISABLE);
    I2C_Peripheral_Init(I2Cx);
}

static void i2c_force_recover(I2C_TypeDef* I2Cx)
{
    i2c_bus_recover(I2Cx);
    i2c_peripheral_recover(I2Cx);
}

static int wait_flag_set(I2C_TypeDef* I2Cx, uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, flag))
    {
        if (I2Cx->SR1 & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR | I2C_SR1_TIMEOUT))
        {
            i2c_force_recover(I2Cx);
            return 0;
        }

        if (--timeout == 0)
        {
            i2c_force_recover(I2Cx);
            return 0;
        }
    }
    return 1;
}

static int wait_flag_reset(I2C_TypeDef* I2Cx, uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, flag))
    {
        if (I2Cx->SR1 & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR | I2C_SR1_TIMEOUT))
        {
            i2c_force_recover(I2Cx);
            return 0;
        }

        if (--timeout == 0)
        {
            i2c_force_recover(I2Cx);
            return 0;
        }
    }
    return 1;
}

static int wait_event(I2C_TypeDef* I2Cx, uint32_t event)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, event))
    {
        if (I2Cx->SR1 & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR | I2C_SR1_TIMEOUT))
        {
            i2c_force_recover(I2Cx);
            return 0;
        }

        if (--timeout == 0)
        {
            i2c_force_recover(I2Cx);
            return 0;
        }
    }
    return 1;
}

/* Initialize I2C based on selected peripheral */
void I2C_Peripheral_Init(I2C_TypeDef* I2Cx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    if (I2Cx == I2C1)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    }
    else
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    }

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_DeInit(I2Cx);

    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);
    i2c_clear_error_flags(I2Cx);
}

int I2C_Transmit(I2C_TypeDef* I2Cx, uint8_t slaveAddr, uint8_t *pData, uint16_t size)
{
    uint16_t i;

    if (!wait_flag_reset(I2Cx, I2C_FLAG_BUSY))
        return 0;

    I2C_GenerateSTART(I2Cx, ENABLE);
    if (!wait_event(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        return 0;

    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
    if (!wait_event(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        return 0;

    for (i = 0; i < size; i++)
    {
        I2C_SendData(I2Cx, pData[i]);
        if (!wait_event(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
            return 0;
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 1;
}

int I2C_Receive(I2C_TypeDef* I2Cx, uint8_t slaveAddr, uint8_t *pData, uint16_t size)
{
    uint16_t i;

    if (size == 0)
        return 0;

    if (!wait_flag_reset(I2Cx, I2C_FLAG_BUSY))
        return 0;

    I2C_GenerateSTART(I2Cx, ENABLE);
    if (!wait_event(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        return 0;

    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);
    if (!wait_event(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        return 0;

    for (i = 0; i < size; i++)
    {
        if (i == (size - 1))
        {
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
        }

        if (!wait_flag_set(I2Cx, I2C_FLAG_RXNE))
        {
            I2C_AcknowledgeConfig(I2Cx, ENABLE);
            I2C_GenerateSTOP(I2Cx, ENABLE);
            return 0;
        }

        pData[i] = I2C_ReceiveData(I2Cx);
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    return 1;
}

/* Wrapper functions for TCS34725 compatibility */
int I2C_writeByte(I2C_TypeDef* I2Cx, uint8_t data, uint8_t address)
{
    return I2C_Transmit(I2Cx, address, &data, 1);
}

int I2C_writeTwoByte(I2C_TypeDef* I2Cx, uint8_t* data, uint8_t address)
{
    return I2C_Transmit(I2Cx, address, data, 2);
}

int I2C_readByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t *data)
{
    return I2C_Receive(I2Cx, address, data, 1);
}

int I2C_readTwoByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data)
{
    return I2C_Receive(I2Cx, address, data, 2);
}
