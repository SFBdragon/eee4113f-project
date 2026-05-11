/* SD_IO.c
 * Raw HAL SDMMC read/write — no FatFS dependency.
 * Handles uptime persistence to a fixed SD sector over DMA.
 */

#include "SD_IO.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── Config ─────────────────────────────────────────────────── */
#define UPTIME_SECTOR   100     // fixed sector, away from FAT structures
#define SD_TIMEOUT      5000    // ms

/* ── Private state ──────────────────────────────────────────── */
extern SD_HandleTypeDef hsd1;

// DMA-safe aligned buffers — never pass stack buffers to DMA
static uint8_t txBuf[512] __attribute__((aligned(4)));
static uint8_t rxBuf[512] __attribute__((aligned(4)));

static volatile uint8_t SD_TxDone = 0;
static volatile uint8_t SD_RxDone = 0;

// UART handle injected via SD_IO_Init — no direct dependency on huart3
static UART_HandleTypeDef *_uart = NULL;

static uint8_t flushBuf[512] __attribute__((aligned(4)));  


/* ── Internal helpers ───────────────────────────────────────── */
static void SD_Log(const char *msg) {
    if (_uart) {
        HAL_UART_Transmit(_uart, (uint8_t*)msg, strlen(msg), 200);
    }
}

static int SD_WaitCardReady(void) {
    uint32_t t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > SD_TIMEOUT) return -1;
    }
    return 0;
}

/* ── DMA Callbacks ──────────────────────────────────────────── */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    SD_TxDone = 1;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
    SD_RxDone = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
    SD_TxDone = 0;
    SD_RxDone = 0;
    SD_Log("SD DMA Error\r\n");
}

/* ── Public API ─────────────────────────────────────────────── */

/**
 * @brief  Inject UART handle for logging. Call once in main() before
 *         any SD_ReadUptime / SD_WriteUptime calls.
 * @param  uartHandle  pointer to your huart (e.g. &huart3), or NULL to disable logging
 */
void SD_IO_Init(UART_HandleTypeDef *uartHandle) {
    _uart = uartHandle;
    SD_Log("SD_IO ready\r\n");
}

/**
 * @brief  Write uptime value to fixed SD sector.
 * @param  uptime  seconds of total uptime
 * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
 */
/* ── Write uptime to SD ─────────────────────────────────────── */

HAL_StatusTypeDef SD_WriteUptime(uint32_t uptime) {
    char msg[64];

    memset(txBuf, 0, sizeof(txBuf));
    snprintf((char*)txBuf, sizeof(txBuf), "UPTIME=%lu\r\n", (unsigned long)uptime);

    SD_Log("SD Write: starting...\r\n");

    // Primary write
    if (HAL_SD_WriteBlocks(&hsd1, txBuf, UPTIME_SECTOR, 1, 5000) != HAL_OK) {
        SD_Log("SD Write: failed\r\n");
        return HAL_ERROR;
    }

    uint32_t t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 5000) return HAL_TIMEOUT;
    }

    // Force flush — write a dummy sector immediately after
    // This pushes sector 100 out of card cache into NAND
    memset(flushBuf, 0xFF, sizeof(flushBuf));
    HAL_SD_WriteBlocks(&hsd1, flushBuf, UPTIME_SECTOR + 1, 1, 5000);
    t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 5000) return HAL_TIMEOUT;
    }

    // Read back sector 100 to verify it persisted
    memset(rxBuf, 0, sizeof(rxBuf));
    HAL_SD_ReadBlocks(&hsd1, rxBuf, UPTIME_SECTOR, 1, 5000);
    t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 5000) return HAL_TIMEOUT;
    }

    // Confirm data is readable back
    if (strstr((char*)rxBuf, "UPTIME=")) {
        snprintf(msg, sizeof(msg), "Uptime saved+verified: %lu sec\r\n", (unsigned long)uptime);
        SD_Log(msg);
        return HAL_OK;
    }

    SD_Log("SD Write: verify FAILED — data not persisting!\r\n");
    return HAL_ERROR;
}

/**
 * @brief  Read uptime value from fixed SD sector.
 * @retval Restored uptime in seconds, or 0 if not found / error
 */
/* ── Read uptime from SD ────────────────────────────────────── */
uint32_t SD_ReadUptime(void) {
    char msg[64];
    memset(rxBuf, 0, sizeof(rxBuf));

    SD_Log("SD Read: starting...\r\n");

    // BLOCKING — same as raw test, no callbacks needed
    if (HAL_SD_ReadBlocks(&hsd1, rxBuf, UPTIME_SECTOR, 1, 5000) != HAL_OK) {
        SD_Log("SD Read: failed\r\n");
        return 0;
    }

    // Wait for card ready
    uint32_t t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 5000) {
            SD_Log("SD Read: card not ready\r\n");
            return 0;
        }
    }

    // Parse UPTIME=12345
    char *ptr = strstr((char*)rxBuf, "UPTIME=");
    if (ptr) {
        uint32_t uptime = (uint32_t)strtoul(ptr + 7, NULL, 10);
        snprintf(msg, sizeof(msg), "Uptime restored: %lu sec\r\n", (unsigned long)uptime);
        SD_Log(msg);
        return uptime;
    }

    SD_Log("SD Read: no uptime found, starting from 0\r\n");
    return 0;
}