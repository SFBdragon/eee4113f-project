/* SD_Stream.c
 * Ring buffer → SD card pipeline.
 * Receives chunks from UART DMA callbacks, writes to SD in 4KB bursts.
 */

#include "SD_IO.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "SD_Stream.h"


/* ── External handles ───────────────────────────────────────── */
extern SD_HandleTypeDef hsd1;

/* ── Ring Buffer ────────────────────────────────────────────── */
static uint8_t ring_buffer[RX_BUF_SIZE];
static volatile uint32_t rb_head = 0;   // written by ISR context
static volatile uint32_t rb_tail = 0;   // read  by main loop

/* ── DMA transfer buffer ────────────────────────────────────── */
// Must be aligned for SD DMA — never use stack buffers
static uint8_t sd_write_buf[SD_BURST_SIZE] __attribute__((aligned(4)));

/* ── SD sector tracking ─────────────────────────────────────── */
static uint32_t current_sector = DATA_START_SECTOR;

/* ── UART for logging ───────────────────────────────────────── */
static UART_HandleTypeDef *_uart = NULL;

static void Stream_Log(const char *msg) {
    if (_uart) HAL_UART_Transmit(_uart, (uint8_t*)msg, strlen(msg), 200);
}

/* ── Ring Buffer helpers ────────────────────────────────────── */

// How many bytes are available to read
static uint32_t RB_Available(void) {
    return (rb_head - rb_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

// How many bytes of free space remain
static uint32_t RB_Free(void) {
    return RX_BUF_SIZE - RB_Available() - 1;
}

// Write bytes into ring buffer — called from ISR context, must be fast
static void RB_Write(uint8_t *src, uint16_t len) {
    if (len > RB_Free()) {
        Stream_Log("RB: OVERFLOW — data lost\r\n");
        return;
    }

    for (uint16_t i = 0; i < len; i++) {
        ring_buffer[rb_head] = src[i];
        rb_head = (rb_head + 1) % RX_BUF_SIZE;
    }
}

// Read a contiguous block out of ring buffer into dst
static void RB_Read(uint8_t *dst, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        dst[i] = ring_buffer[rb_tail];
        rb_tail = (rb_tail + 1) % RX_BUF_SIZE;
    }
}

/* ── Public API ─────────────────────────────────────────────── */

void SD_Stream_Init(UART_HandleTypeDef *uartHandle) {
    _uart          = uartHandle;
    rb_head        = 0;
    rb_tail        = 0;
    current_sector = DATA_START_SECTOR;
    Stream_Log("SD_Stream ready\r\n");
}

// Called from HAL_UART_RXEVENT_HT or HAL_UART_RXEVENT_IDLE
// src  = rxBuffer (start of first half)
// len  = number of valid bytes in this chunk
void SD_Stream_WriteHalf(uint8_t *src, uint16_t len) {
    Stream_Log("SD_STREAM_HALF_CALL\r\n");
    RB_Write(src, len);
}

// Called from HAL_UART_RXEVENT_TC
// src  = rxBuffer + RX_BUF_SIZE/2 (start of second half)
// len  = RX_BUF_SIZE/2
void SD_Stream_WriteSecondHalf(uint8_t *src, uint16_t len) {
    Stream_Log("SD_STREAM_SECOND_HALF_CALL\r\n");
    RB_Write(src, len);
}

// Call this from main loop — drains ring buffer to SD in 4KB bursts
void SD_Stream_Flush(void) {
    uint32_t bytes_waiting = RB_Available();  // fix: was never assigned

    char dbg[80];
    snprintf(dbg, sizeof(dbg), "Flush called | RB available: %lu | Need: %lu\r\n",
        (unsigned long)bytes_waiting,
        (unsigned long)SD_BURST_SIZE);
    //Stream_Log(dbg);

    while (RB_Available() >= SD_BURST_SIZE)
    {
        bytes_waiting = RB_Available();  // update each iteration

        // Log first 4 bytes of what we're about to write
        snprintf(dbg, sizeof(dbg), "Flush: reading %lu bytes from RB, first bytes: %02X %02X %02X %02X\r\n",
            (unsigned long)SD_BURST_SIZE,
            sd_write_buf[0], sd_write_buf[1], sd_write_buf[2], sd_write_buf[3]);
        Stream_Log(dbg);

        // Pull out of ring buffer
        RB_Read(sd_write_buf, SD_BURST_SIZE);

        // Log what we actually read
        snprintf(dbg, sizeof(dbg), "Flush: writing to sector %lu | first bytes: %02X %02X %02X %02X\r\n",
            (unsigned long)current_sector,
            sd_write_buf[0], sd_write_buf[1], sd_write_buf[2], sd_write_buf[3]);
        Stream_Log(dbg);

        HAL_StatusTypeDef result = HAL_SD_WriteBlocks(
            &hsd1,
            sd_write_buf,
            current_sector,
            SD_BURST_BLOCKS,
            5000
        );

        if (result != HAL_OK) {
            Stream_Log("SD Stream: write failed\r\n");
            return;
        }

        uint32_t t = HAL_GetTick();
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            if (HAL_GetTick() - t > 5000) {
                Stream_Log("SD Stream: card timeout\r\n");
                return;
            }
        }

        snprintf(dbg, sizeof(dbg),
            "SD: Wrote to sector %lu | Buffer remaining: %lu bytes\r\n",
            (unsigned long)current_sector,
            (unsigned long)bytes_waiting);
        Stream_Log(dbg);

        current_sector += SD_BURST_BLOCKS;
    }

    // Log why we exited the loop
    snprintf(dbg, sizeof(dbg), "Flush exit | RB available: %lu | Need: %lu\r\n",
        (unsigned long)RB_Available(),
        (unsigned long)SD_BURST_SIZE);
    //Stream_Log(dbg);
}
/**
 * @brief Reads data back from SD card starting from a specific sector
 * @param start_sector The sector to begin reading from (e.g., DATA_START_SECTOR)
 * @param num_blocks   How many 512-byte blocks to read
 */


void SD_Stream_ReadDebug(uint32_t start_sector, uint32_t num_blocks) {
    static uint8_t read_buf[512] __attribute__((aligned(4))); // static = not on stack
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

        // Dump all 512 bytes as hex, 16 per line
        for (int row = 0; row < 512; row += 16) {
            int len = snprintf(msg, sizeof(msg),
                "%04lX: %02X %02X %02X %02X %02X %02X %02X %02X "
                       "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned long)(row),
                read_buf[row+0],  read_buf[row+1],  read_buf[row+2],  read_buf[row+3],
                read_buf[row+4],  read_buf[row+5],  read_buf[row+6],  read_buf[row+7],
                read_buf[row+8],  read_buf[row+9],  read_buf[row+10], read_buf[row+11],
                read_buf[row+12], read_buf[row+13], read_buf[row+14], read_buf[row+15]);
            HAL_UART_Transmit(_uart, (uint8_t*)msg, len, 500);
        }
    }
    Stream_Log("--- SD READ END ---\r\n");
}


uint32_t SD_Stream_GetCurrentSector(void) {
    return current_sector;
}