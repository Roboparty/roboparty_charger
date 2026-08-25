
// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 zz020


#include "main.h"

#include "i2c_soft.h"


void I2C_SOFT_INIT()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);

	/*Configure GPIO pin : I2C_SCL_PIN */
	GPIO_InitStruct.Pin = I2C_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);

	/*Configure GPIO pin : I2C_SDA_PIN */
	GPIO_InitStruct.Pin = I2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
}

// Delay function (approximate at 72MHz, with optimization enabled)
void I2C_Delay_us(uint32_t us)
{
    uint32_t i;
    for (i = 0; i < us * 8; i++)
    {
        __NOP();
    }
}

// I2C start condition
void I2C_Start(void)
{
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_Delay_us(5);
    I2C_SDA_LOW();
    I2C_Delay_us(5);
    I2C_SCL_LOW();
}

// I2C stop condition
void I2C_Stop(void)
{
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_Delay_us(5);
    I2C_SDA_HIGH();
    I2C_Delay_us(5);
}

// Send one byte
void I2C_SendByte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            I2C_SDA_HIGH();
        else
            I2C_SDA_LOW();
        data <<= 1;
        I2C_Delay_us(2);
        I2C_SCL_HIGH();
        I2C_Delay_us(5);
        I2C_SCL_LOW();
        I2C_Delay_us(2);
    }
}

// Read one byte
uint8_t I2C_ReadByte(void)
{
    uint8_t i, data = 0;
    I2C_SDA_HIGH();
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        I2C_SCL_HIGH();
        I2C_Delay_us(2);
        if (I2C_SDA_READ())
            data |= 0x01;
        I2C_SCL_LOW();
        I2C_Delay_us(2);
    }
    return data;
}

// Wait for ACK, return 0:ACK, 1:NACK
uint8_t I2C_WaitAck(void)
{
    uint8_t ack;
    I2C_SDA_HIGH();
    I2C_Delay_us(2);
    I2C_SCL_HIGH();
    I2C_Delay_us(2);
    ack = I2C_SDA_READ();
    I2C_SCL_LOW();
    I2C_Delay_us(2);
    return ack;
}

// Send ACK
void I2C_SendAck(void)
{
    I2C_SDA_LOW();
    I2C_Delay_us(2);
    I2C_SCL_HIGH();
    I2C_Delay_us(5);
    I2C_SCL_LOW();
    I2C_SDA_HIGH();
}

// Send NACK
void I2C_SendNoAck(void)
{
    I2C_SDA_HIGH();
    I2C_Delay_us(2);
    I2C_SCL_HIGH();
    I2C_Delay_us(5);
    I2C_SCL_LOW();
    I2C_SDA_LOW();
}

// Write register (for TMP100 etc.)
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 2; }
    I2C_SendByte(data);
    if (I2C_WaitAck()) { I2C_Stop(); return 3; }
    I2C_Stop();
    return 0;
}

// Read register
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 2; }
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x01);
    if (I2C_WaitAck()) { I2C_Stop(); return 3; }
    *data = I2C_ReadByte();
    I2C_SendNoAck();
    I2C_Stop();
    return 0;
}

// Write data (continuous)
uint8_t I2C_WriteData(uint8_t dev_addr, uint8_t *data, uint16_t len)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    for (uint16_t i = 0; i < len; i++)
    {
        I2C_SendByte(data[i]);
        if (I2C_WaitAck()) { I2C_Stop(); return 2; }
    }
    I2C_Stop();
    return 0;
}

// Read data (continuous)
uint8_t I2C_ReadData(uint8_t dev_addr, uint8_t *data, uint16_t len)
{
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x01);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = I2C_ReadByte();
        if (i == len - 1)
            I2C_SendNoAck();
        else
            I2C_SendAck();
    }
    I2C_Stop();
    return 0;
}

// Write data with register address
uint8_t I2C_WriteDataWithReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 2; }
    for (uint16_t i = 0; i < len; i++)
    {
        I2C_SendByte(data[i]);
        if (I2C_WaitAck()) { I2C_Stop(); return 3; }
    }
    I2C_Stop();
    return 0;
}

// Read data with register address
uint8_t I2C_ReadDataWithReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    I2C_Start();
    I2C_SendByte(dev_addr << 1);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }
    I2C_SendByte(reg_addr);
    if (I2C_WaitAck()) { I2C_Stop(); return 2; }
    I2C_Start();
    I2C_SendByte((dev_addr << 1) | 0x01);
    if (I2C_WaitAck()) { I2C_Stop(); return 3; }
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = I2C_ReadByte();
        if (i == len - 1)
            I2C_SendNoAck();
        else
            I2C_SendAck();
    }
    I2C_Stop();
    return 0;
}
