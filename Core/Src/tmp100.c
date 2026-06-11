
// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 zz020


#include "main.h"

#include "tmp100.h"
#include "i2c_soft.h"

float TMP100_GetTemperature(void)
{
    uint8_t temp_reg[2];
    // ??????(TMP100 ?????? 0x00 ????)
    if (I2C_ReadDataWithReg(TMP100_ADDR, 0x00, temp_reg, 2))
        return -273.15f;  // ??????????
    int16_t raw = (temp_reg[0] << 8) | temp_reg[1];
	
    raw >>= 4;  // TMP100 ? 12 ????,? 12 ???
	//printf("temp raw = 0x%X", raw);
	
    return raw * 0.0625f;
}
