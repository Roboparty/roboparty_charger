

// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 zz020


#include "main.h"

#include "parse_command.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

void Flash_ErasePage(uint32_t PageAddress)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit = {0};
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = PageAddress;
    eraseInit.NbPages     = 1;

    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&eraseInit, &PageError);

    HAL_FLASH_Lock();
}


void Flash_WriteData(uint32_t Address, uint8_t *data, uint16_t len)
{
    HAL_FLASH_Unlock();
    for (uint16_t i = 0; i < len; i += 2)
    {
        uint16_t word = (uint16_t)data[i] | ((i+1 < len) ? (data[i+1] << 8) : 0);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + i, word);
    }
    HAL_FLASH_Lock();
}

void SaveConfigToFlash(ConfigData *cfg)
{
    Flash_ErasePage(CFG_FLASH_ADDR);
    Flash_WriteData(CFG_FLASH_ADDR, (uint8_t*)cfg, CFG_SAVE_BYTES);//sizeof(ConfigData)
}

int LoadConfigFromFlash(ConfigData *cfg)
{
    ConfigData *p = (ConfigData*)CFG_FLASH_ADDR;
    if (p->magic == CFG_MAGIC_NUMBER)
    {
        memcpy(cfg, p, CFG_SAVE_BYTES);//sizeof(ConfigData)
		return 0;
    }
    else
    {
        // ??,?????
        cfg->magic = CFG_MAGIC_NUMBER;
        cfg->ictrl = 0;
        cfg->r = 0;
        cfg->g = 0;
        cfg->b = 0;
        cfg->chg_en = 0;
        cfg->fan = 49;
		return 1;
    }
}

ConfigData g_config;   // ????

void UpdateConfigFromCommand(const char *cmd_name, int value)
{
    if (strcmp(cmd_name, "ICTRL") == 0) {
        if (value >= 0 && value <= 100) g_config.ictrl = value;
    } else if (strcmp(cmd_name, "R") == 0) {
        if (value >= 0 && value <= 100) g_config.r = value;
    } else if (strcmp(cmd_name, "G") == 0) {
        if (value >= 0 && value <= 100) g_config.g = value;
    } else if (strcmp(cmd_name, "B") == 0) {
        if (value >= 0 && value <= 100) g_config.b = value;
    } else if (strcmp(cmd_name, "CHG_EN") == 0) {
        if (value == 0 || value == 1) g_config.chg_en = value;
    } else if (strcmp(cmd_name, "FAN") == 0) {
        if (value >= 0 && value <= 100) g_config.fan = value;
    } 
}

void ApplyConfig(void)
{
    /* FAN PWM control via TIM1 CH1 (PA8) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (g_config.fan * 99) / 100);

    /* Charge enable control */
    HAL_GPIO_WritePin(Charge_EN_GPIO_Port, Charge_EN_Pin,
                      g_config.chg_en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void parse_command(const uint8_t *cmd, uint16_t len)
{
    const uint8_t *p = cmd;
    const uint8_t *end = cmd + len;
    const uint8_t *start_cmd, *colon, *semicolon;
    char cmd_name[64];
    char data_str[64];
    int value;

    while (p < end)
    {
        while (p < end && (*p == ' ' || *p == '\t')) p++;
        if (p >= end) break;

        start_cmd = p;

        colon = memchr(p, ':', end - p);
        if (colon == NULL) break;

        semicolon = memchr(colon, ';', end - colon);
        if (semicolon == NULL) semicolon = end;

        size_t name_len = colon - start_cmd;
        if (name_len >= sizeof(cmd_name)) name_len = sizeof(cmd_name) - 1;
        memcpy(cmd_name, start_cmd, name_len);
        cmd_name[name_len] = '\0';

        const uint8_t *data_start = colon + 1;
        size_t data_len = semicolon - data_start;
        if (data_len >= sizeof(data_str)) data_len = sizeof(data_str) - 1;
        memcpy(data_str, data_start, data_len);
        data_str[data_len] = '\0';

        int valid = 0;
        if (strcmp(cmd_name, "ICTRL") == 0 ||
            strcmp(cmd_name, "R") == 0 ||
            strcmp(cmd_name, "G") == 0 ||
            strcmp(cmd_name, "B") == 0 ||
            strcmp(cmd_name, "FAN") == 0 ||
            strcmp(cmd_name, "CHG_EN") == 0)
        {
            value = atoi(data_str);
            UpdateConfigFromCommand(cmd_name, value);
            ApplyConfig();
            valid = 1;
        }
        else if (strcmp(cmd_name, "SAVE") == 0)
        {
            value = atoi(data_str);
            if (value == 1)
            {
                SaveConfigToFlash(&g_config);
                valid = 1;  //
            }
        }
        else if (strcmp(cmd_name, "CHRG_INFO") == 0)
        {
            printf("CHRG_SN:%.16s;", g_config.str_chrg_sn);
            printf("SW_DATE:%.12s;", g_config.str_sw_date);
            printf("R:%d;G:%d;B:%d;FAN:%d;", g_config.r, g_config.g, g_config.b, g_config.fan);
            printf("BAT_INST:%d;", BAT_INST);
            printf("CHRG_V:%.1fV;", CHRG_V);
            printf("CHRG_I:%.1fA;", CHRG_I);
            printf("BAT_V:%.1fV;", BAT_V);
            printf("ICTRL:%d;", g_config.ictrl);
            printf("CHG_EN:%d;", g_config.chg_en);
            if(BAT_INST)
            {
                printf("BMS_SOC:%s;", strBmsSoc);
                printf("BMS_SOH:%s;", strBmsSoh);
            }
        }
        else if (strcmp(cmd_name, "SW_DATE") == 0) 
        {
            memcpy(g_config.str_sw_date, data_str, 12);
        }
        else if (strcmp(cmd_name, "CHRG_SN") == 0) 
        {
            memcpy(g_config.str_chrg_sn, data_str, 16);
        }
        else if (strcmp(cmd_name, "BAT_INST") == 0)
        {
            printf("BAT_INST:%d", BAT_INST);
        }
        else if (strcmp(cmd_name, "BAT_SOC") == 0)
        {
            printf("BMS_SOC:%s;", strBmsSoc);
        }
        else if (strcmp(cmd_name, "BAT_SOH") == 0)
        {
            printf("BMS_SOH:%s;", strBmsSoh);
        }

        // ???????
        if (valid)
        {
            size_t sub_len = semicolon - start_cmd;
            char sub_buf[64];
            if (sub_len < sizeof(sub_buf))
            {
                memcpy(sub_buf, start_cmd, sub_len);
                sub_buf[sub_len] = '\0';
//                HAL_UART_Transmit(&huart3, (uint8_t*)sub_buf, sub_len, 1000);
//                printf("%s;", sub_buf);
            }
        }
		HAL_Delay(100);

        // ????
        p = semicolon;
        if (p < end && *p == ';') p++;
    }
}

