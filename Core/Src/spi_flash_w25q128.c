

// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 zz020




/*--------------------------------------------------------------------------------
 * ??: spi_flash_w25q128.c
 * ??: W25Q128JVSQ SPI Flash ?????? (??CS + ?DMA)
 *       ?? SPI2,???????? DMA (HAL_SPI_TransmitReceive_DMA)
 *--------------------------------------------------------------------------------*/
#include "spi_flash_w25q128.h"
#include "spi.h"           // ?? hspi2 ??
#include "gpio.h"          // ?? GPIO ???
#include <string.h>
#include <stdlib.h>        // ?? malloc/free

extern SPI_HandleTypeDef hspi2;

/* ??DMA???? (??) */
static void SPI_WaitDMAComplete(void)
{
    // ??SPI DMA????
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    // ???????DMA???
    if (hspi2.hdmatx != NULL) {
        while (hspi2.hdmatx->State != HAL_DMA_STATE_READY);
    }
    if (hspi2.hdmarx != NULL) {
        while (hspi2.hdmarx->State != HAL_DMA_STATE_READY);
    }
}

/* ???????DMA???? (CS??????) */
static void SPI_Transact(uint8_t *txBuf, uint8_t *rxBuf, uint16_t size)
{
    W25Q_CS_LOW();
    HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, size);
    SPI_WaitDMAComplete();
    W25Q_CS_HIGH();
}

/* ????? (???) */
static void SPI_SendCommand(uint8_t cmd)
{
    uint8_t tx = cmd, rx;
    SPI_Transact(&tx, &rx, 1);
}

/* ??????1 (??8??) */
static uint8_t SPI_ReadStatus(void)
{
    uint8_t tx[2] = {W25Q_CMD_RDSR1, 0xFF};
    uint8_t rx[2];
    SPI_Transact(tx, rx, 2);
    return rx[1];
}

/* ???: ??CS????,?????? */
void W25Q128_Init(void)
{
    // ??CS???????? (???gpio.c?????)
    W25Q_CS_HIGH();
    HAL_Delay(10);
    // ????????
    SPI_SendCommand(W25Q_CMD_RDPD);
    HAL_Delay(1);
}

/* ??JEDEC ID (3??) */
void W25Q128_ReadID(uint8_t *id)
{
    uint8_t tx[4] = {W25Q_CMD_RDID, 0xFF, 0xFF, 0xFF};
    uint8_t rx[4];
    SPI_Transact(tx, rx, 4);
    id[0] = rx[1];
    id[1] = rx[2];
    id[2] = rx[3];
}

/* ???????? */
void W25Q128_WaitBusy(void)
{
    uint8_t status;
    do {
        status = SPI_ReadStatus();
    } while (status & W25Q_SR1_BUSY);
}

/* ??? */
void W25Q128_WriteEnable(void)
{
    SPI_SendCommand(W25Q_CMD_WREN);
}

/* ???? (4KB) */
void W25Q128_EraseSector(uint32_t sector_addr)
{
    W25Q128_WriteEnable();
    uint8_t tx[4] = {
        W25Q_CMD_SE,
        (uint8_t)(sector_addr >> 16),
        (uint8_t)(sector_addr >> 8),
        (uint8_t)sector_addr
    };
    uint8_t rx[4];
    SPI_Transact(tx, rx, 4);
    W25Q128_WaitBusy();
}

/* 32KB??? */
void W25Q128_EraseBlock32K(uint32_t block_addr)
{
    W25Q128_WriteEnable();
    uint8_t tx[4] = {
        W25Q_CMD_BE32,
        (uint8_t)(block_addr >> 16),
        (uint8_t)(block_addr >> 8),
        (uint8_t)block_addr
    };
    uint8_t rx[4];
    SPI_Transact(tx, rx, 4);
    W25Q128_WaitBusy();
}

/* 64KB??? */
void W25Q128_EraseBlock64K(uint32_t block_addr)
{
    W25Q128_WriteEnable();
    uint8_t tx[4] = {
        W25Q_CMD_BE64,
        (uint8_t)(block_addr >> 16),
        (uint8_t)(block_addr >> 8),
        (uint8_t)block_addr
    };
    uint8_t rx[4];
    SPI_Transact(tx, rx, 4);
    W25Q128_WaitBusy();
}

/* ???? */
void W25Q128_EraseChip(void)
{
    W25Q128_WriteEnable();
    SPI_SendCommand(W25Q_CMD_CE);
    W25Q128_WaitBusy();
}

/* ??? (??256??) */
void W25Q128_PageProgram(uint32_t page_addr, uint8_t *data, uint16_t len)
{
    if (len == 0 || len > 256) return;
    W25Q128_WriteEnable();
    // ???????: ??(1) + ??(3) + ??(len)
    uint16_t total_len = 4 + len;
    uint8_t *txBuf = (uint8_t*)malloc(total_len);
    if (txBuf == NULL) return;
    txBuf[0] = W25Q_CMD_PP;
    txBuf[1] = (uint8_t)(page_addr >> 16);
    txBuf[2] = (uint8_t)(page_addr >> 8);
    txBuf[3] = (uint8_t)page_addr;
    memcpy(txBuf + 4, data, len);
    uint8_t *rxBuf = (uint8_t*)malloc(total_len);
    if (rxBuf == NULL) {
        free(txBuf);
        return;
    }
    SPI_Transact(txBuf, rxBuf, total_len);
    free(txBuf);
    free(rxBuf);
    W25Q128_WaitBusy();
}

/* ?????? */
void W25Q128_Read(uint32_t addr, uint8_t *rxBuf, uint16_t len)
{
    if (len == 0) return;
    uint16_t total_len = 4 + len;
    uint8_t *txBuf = (uint8_t*)malloc(total_len);
    if (txBuf == NULL) return;
    txBuf[0] = W25Q_CMD_READ;
    txBuf[1] = (uint8_t)(addr >> 16);
    txBuf[2] = (uint8_t)(addr >> 8);
    txBuf[3] = (uint8_t)addr;
    memset(txBuf + 4, 0xFF, len);   // ??0xFF????
    uint8_t *rxFull = (uint8_t*)malloc(total_len);
    if (rxFull == NULL) {
        free(txBuf);
        return;
    }
    SPI_Transact(txBuf, rxFull, total_len);
    memcpy(rxBuf, rxFull + 4, len);
    free(txBuf);
    free(rxFull);
}

void spi_flash_simple_test()
{
    uint8_t id[3];
    uint8_t writeBuf[] = "Hello W25Q128!";
    uint8_t readBuf[32];

    W25Q128_Init();
    W25Q128_ReadID(id);
    printf("ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);

    W25Q128_EraseSector(0);          // ????0
    W25Q128_PageProgram(0, writeBuf, sizeof(writeBuf));
    W25Q128_Read(0, readBuf, sizeof(readBuf));
    printf("Read: %s\r\n", readBuf);
}
