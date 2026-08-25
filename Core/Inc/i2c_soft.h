
#ifndef __I2C_SOFT_H
#define __I2C_SOFT_H

#include "gpio.h"

// Device I2C bus (TMP100 etc.) - PB8/PB9
#define I2C_SCL_PORT       GPIOB
#define I2C_SCL_PIN        GPIO_PIN_8
#define I2C_SDA_PORT       GPIOB
#define I2C_SDA_PIN        GPIO_PIN_9

// Bit-bang macros
#define I2C_SCL_HIGH()     HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_LOW()      HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_HIGH()     HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_LOW()      HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define I2C_SDA_READ()     HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)

void I2C_SOFT_INIT();

void I2C_Delay_us(uint32_t us);

// I2C basic operations
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t data);
uint8_t I2C_ReadByte(void);
uint8_t I2C_WaitAck(void);
void I2C_SendAck(void);
void I2C_SendNoAck(void);

// Register/data operations
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
uint8_t I2C_WriteData(uint8_t dev_addr, uint8_t *data, uint16_t len);
uint8_t I2C_ReadData(uint8_t dev_addr, uint8_t *data, uint16_t len);
uint8_t I2C_WriteDataWithReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t I2C_ReadDataWithReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif
