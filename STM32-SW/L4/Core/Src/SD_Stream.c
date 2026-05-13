#include "SD_IO.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "SD_Stream.h"

extern SD_HandleTypeDef hsd1;

static uint32_t current_sector = DATA_START_SECTOR;
static UART_HandleTypeDef *_uart = NULL;

static void Stream_Log(const char *msg) {
    if (_uart) HAL_UART_Transmit(_uart, (uint8_t*)msg, strlen(msg), 200);
}

void SD_Stream_Init(UART_HandleTypeDef *uartHandle) {
    _uart          = uartHandle;
    current_sector = DATA_START_SECTOR;
    Stream_Log("SD_Stream ready\r\n");
}

// Write a single packed block directly to SD
HAL_StatusTypeDef SD_Stream_WriteBlock(uint8_t *block) {
    char dbg[80];

    HAL_StatusTypeDef result = HAL_SD_WriteBlocks(
        &hsd1,
        block,
        current_sector,
        1,       // 1 block at a time (512 bytes)
        5000
    );

    if (result != HAL_OK) {
        Stream_Log("SD: write failed\r\n");
        return result;
    }

    uint32_t t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 5000) {
            Stream_Log("SD: card timeout\r\n");
            return HAL_TIMEOUT;
        }
    }

    snprintf(dbg, sizeof(dbg), "SD: wrote sector %lu\r\n", (unsigned long)current_sector);
    Stream_Log(dbg);

    current_sector++;
    return HAL_OK;
}

uint32_t SD_Stream_GetCurrentSector(void) {
    return current_sector;
}

void SD_Stream_ReadDebug(uint32_t start_sector, uint32_t num_blocks) {
    static uint8_t read_buf[512] __attribute__((aligned(4)));
    char msg[64];

    Stream_Log("--- SD READ START ---\r\n");

    for (uint32_t i = 0; i < num_blocks; i++) {
        HAL_StatusTypeDef res = HAL_SD_ReadBlocks(
            &hsd1, read_buf, start_sector + i, 1, 2000);

        if (res != HAL_OK) { Stream_Log("Read error\r\n"); break; }

        uint32_t t = HAL_GetTick();
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            if (HAL_GetTick() - t > 2000) { Stream_Log("Timeout\r\n"); return; }
        }

        for (int row = 0; row < 512; row += 16) {
            int len = snprintf(msg, sizeof(msg),
                "%04lX: %02X %02X %02X %02X %02X %02X %02X %02X "
                       "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned long)row,
                read_buf[row+0],  read_buf[row+1],  read_buf[row+2],  read_buf[row+3],
                read_buf[row+4],  read_buf[row+5],  read_buf[row+6],  read_buf[row+7],
                read_buf[row+8],  read_buf[row+9],  read_buf[row+10], read_buf[row+11],
                read_buf[row+12], read_buf[row+13], read_buf[row+14], read_buf[row+15]);
            HAL_UART_Transmit(_uart, (uint8_t*)msg, len, 500);
        }
    }
    Stream_Log("--- SD READ END ---\r\n");
}