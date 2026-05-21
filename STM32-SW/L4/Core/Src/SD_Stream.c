#include "SD_IO.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "SD_Stream.h"
#define TEST_BUFFER_SIZE 16384  // 32 sectors (16KB) - larger buffers = better speed
#define STRESS_TEST_TOTAL_SIZE (100ULL * 1024 * 1024)
#define TOTAL_TEST_SIZE ( 1024*1024*1024)//(100ULL * 1024 * 1024)// 1MB total test


#include <string.h> // for memcmp

extern SD_HandleTypeDef hsd1;

uint64_t read_base_id = 0;
uint64_t prot_base_id = 0;
uint64_t write_head_id = 0;
uint32_t policy_overwrite = 1;

uint32_t id_to_sector(uint16_t id) {
    return DATA_START_SECTOR + (id * (SD_BLOCK_SIZE / SD_SECTOR_SIZE)) % FAKE_NUM_BLOCKS;
}

static UART_HandleTypeDef *_uart = NULL;

static uint8_t write_buf[512] __attribute__((aligned(4)));
static uint8_t read_buf[512] __attribute__((aligned(4)));

uint32_t SD_Stream_BlockCount() {
    return FAKE_NUM_BLOCKS;
    // if (block_count != 0) {
    //     return block_count;
    // }

    // HAL_SD_CardInfoTypeDef cardInfo;
    
    // if (HAL_SD_GetCardInfo(&hsd1, &cardInfo) == HAL_OK) {
    //     // LogBlockNbr holds the total number of logical blocks on your SD card
    //     block_count = cardInfo.LogBlockNbr * (SD_BLOCK_SIZE / SD_SECTOR_SIZE);
    //     return block_count;
    // }
    
    // // Fallback default in case the SD card is not initialized/present
    // return 0;
}

static void Stream_Log(const char *msg) {
    if (_uart) HAL_UART_Transmit(_uart, (uint8_t*)msg, strlen(msg), 200);
}

void SD_Stream_Init(UART_HandleTypeDef *uartHandle) {
    _uart          = uartHandle;
    read_base_id = 0;
    prot_base_id = 0;
    write_head_id = 0;
    Stream_Log("SD_Stream ready\r\n");
}

uint32_t SD_Stream_GetCurrentSector(void) {
    return id_to_sector(write_head_id);
}

// Write a single packed block directly to SD
HAL_StatusTypeDef SD_Stream_WriteBlock(uint8_t *block) {
    char dbg[80];

    if (write_head_id >= prot_base_id + FAKE_NUM_BLOCKS && !policy_overwrite) {
        Stream_Log("SD: discarded data\r\n");
        return HAL_OK;
    }

    uint32_t sector = id_to_sector(write_head_id);

    HAL_StatusTypeDef result = HAL_SD_WriteBlocks(
        &hsd1,
        block,
        sector,
        SD_BLOCK_SIZE / SD_SECTOR_SIZE,
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

    snprintf(dbg, sizeof(dbg), "SD: wrote block %lu\r\n", (unsigned long)write_head_id);
    Stream_Log(dbg);

    // Next block ID
    write_head_id += 1;

    // there can't be more than FAKE_NUM_BLOCKS readable
    if (read_base_id < write_head_id - FAKE_NUM_BLOCKS) {
        read_base_id = write_head_id - FAKE_NUM_BLOCKS;
    }

    // if a block is no longer readable, it's no longer protected
    if (prot_base_id < read_base_id) {
        prot_base_id = read_base_id;
    }

    return HAL_OK;
}

uint64_t SD_Stream_GetReadBase(void) {
    return read_base_id;
}
uint64_t SD_Stream_GetProtBase(void) {
    return prot_base_id;
}
uint64_t SD_Stream_GetWriteHead(void) {
    return write_head_id;
}
void SD_Stream_SetProtHead(uint64_t from_block) {
    prot_base_id = from_block;
}
void SD_Stream_SetOverwritePolicy(uint32_t policy) {
    policy_overwrite = policy;
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


void SD_Stream_ReadDebug_Last_Line(uint32_t start_sector, uint32_t num_blocks) {
    static uint8_t read_buf[512] __attribute__((aligned(4)));
    char msg[128];

    Stream_Log("--- SD READ START ---\r\n");

    for (uint32_t i = 0; i < num_blocks; i++) {
        HAL_StatusTypeDef res = HAL_SD_ReadBlocks(
            &hsd1, read_buf, start_sector + i, 1, 2000);

        if (res != HAL_OK) { Stream_Log("Read error\r\n"); break; }

        uint32_t t = HAL_GetTick();
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            if (HAL_GetTick() - t > 2000) { Stream_Log("Timeout\r\n"); return; }
        }

        // Only print last 16 bytes of the block (offset 0x1F0)
        int row = 512 - 16;
        int len = snprintf(msg, sizeof(msg),
            "Sector %lu [last 16]: %02X %02X %02X %02X %02X %02X %02X %02X "
                          "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            (unsigned long)(start_sector + i),
            read_buf[row+0],  read_buf[row+1],  read_buf[row+2],  read_buf[row+3],
            read_buf[row+4],  read_buf[row+5],  read_buf[row+6],  read_buf[row+7],
            read_buf[row+8],  read_buf[row+9],  read_buf[row+10], read_buf[row+11],
            read_buf[row+12], read_buf[row+13], read_buf[row+14], read_buf[row+15]);
        HAL_UART_Transmit(_uart, (uint8_t*)msg, len, 500);
        Stream_Log("\r\n");
    }
    Stream_Log("--- SD READ END ---\r\n");
}

HAL_StatusTypeDef SD_Stream_ReadSectors(uint32_t sector, uint32_t count, uint8_t* buffer) {
    HAL_StatusTypeDef res = HAL_SD_ReadBlocks(&hsd1, buffer, sector, count, 2000);
    if (res != HAL_OK) {
        Stream_Log("ReadSectors: failed\r\n");
        return res;
    }

    uint32_t t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 2000) {
            Stream_Log("ReadSectors: timeout\r\n");
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}


void SD_Stream_ReadAndForward(uint32_t start_sector, uint32_t num_blocks) {
    static uint8_t read_buf[512] __attribute__((aligned(4)));
    char msg[80];

    Stream_Log("--- SD READ AND FORWARD START ---\r\n");

    for (uint32_t i = 0; i < num_blocks; i++) {
        HAL_StatusTypeDef res = HAL_SD_ReadBlocks(
            &hsd1, read_buf, start_sector + i, 1, 2000);

        if (res != HAL_OK) { Stream_Log("Read error\r\n"); break; }

        uint32_t t = HAL_GetTick();
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            if (HAL_GetTick() - t > 2000) { Stream_Log("Timeout\r\n"); return; }
        }

        // Print last 16 bytes as hex (debug)
        int row = 512 - 16;
        int len = snprintf(msg, sizeof(msg),
            "Sector %lu [last 16]: %02X %02X %02X %02X %02X %02X %02X %02X "
                          "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            (unsigned long)(start_sector + i),
            read_buf[row+0],  read_buf[row+1],  read_buf[row+2],  read_buf[row+3],
            read_buf[row+4],  read_buf[row+5],  read_buf[row+6],  read_buf[row+7],
            read_buf[row+8],  read_buf[row+9],  read_buf[row+10], read_buf[row+11],
            read_buf[row+12], read_buf[row+13], read_buf[row+14], read_buf[row+15]);
        HAL_UART_Transmit(_uart, (uint8_t*)msg, len, 500);
        Stream_Log("\r\n");

        // Forward raw 512 bytes over USART3
        HAL_UART_Transmit(&huart3, read_buf, 512, 5000);
    }

    Stream_Log("--- SD READ AND FORWARD END ---\r\n");
}

void SD_BruteSpeedTest(void) {
    static uint8_t test_buf[TEST_BUFFER_SIZE] __attribute__((aligned(4)));
    char log[128];
    
    // Fill buffer with dummy pattern
    for(int i=0; i<TEST_BUFFER_SIZE; i++) test_buf[i] = (uint8_t)(i % 256);

    uint32_t start_tick, end_tick, elapsed;
    uint32_t num_iterations = TOTAL_TEST_SIZE / TEST_BUFFER_SIZE;
    
    Stream_Log("\r\n--- SD BRUTE SPEED TEST ---\r\n");

    // --- WRITE TEST ---
    start_tick = HAL_GetTick();
    for (uint32_t i = 0; i < num_iterations; i++) {
        HAL_StatusTypeDef res = HAL_SD_WriteBlocks(&hsd1, test_buf, 
                                 DATA_START_SECTOR + (i * (TEST_BUFFER_SIZE/512)), 
                                 TEST_BUFFER_SIZE/512, 10000);
        
        if (res != HAL_OK) { Stream_Log("Write Error\r\n"); return; }
        
        // Wait for card to be ready
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);
    }
    end_tick = HAL_GetTick();
    elapsed = end_tick - start_tick;
    
    float write_speed = (float)TOTAL_TEST_SIZE / (float)elapsed; // KB/s
    snprintf(log, sizeof(log), "Write: %lu bytes in %lu ms (%.2f KB/s)\r\n", 
             (unsigned long)TOTAL_TEST_SIZE, elapsed, write_speed);
    Stream_Log(log);

    // --- READ TEST ---
    start_tick = HAL_GetTick();
    for (uint32_t i = 0; i < num_iterations; i++) {
        HAL_StatusTypeDef res = HAL_SD_ReadBlocks(&hsd1, test_buf, 
                                 DATA_START_SECTOR + (i * (TEST_BUFFER_SIZE/512)), 
                                 TEST_BUFFER_SIZE/512, 10000);
        
        if (res != HAL_OK) { Stream_Log("Read Error\r\n"); return; }
        
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);
    }
    end_tick = HAL_GetTick();
    elapsed = end_tick - start_tick;

    float read_speed = (float)TOTAL_TEST_SIZE / (float)elapsed; // KB/s
    snprintf(log, sizeof(log), "Read: %lu bytes in %lu ms (%.2f KB/s)\r\n", 
             (unsigned long)TOTAL_TEST_SIZE, elapsed, read_speed);
    Stream_Log(log);
}

void SD_MassAccuracyTest(uint32_t iterations) {
    char msg[64];
    uint32_t errors = 0;

    for (uint32_t i = 0; i < iterations; i++) {
        // 1. Generate a pattern that changes every iteration
        // Using (i + j) ensures every block is unique
        for (int j = 0; j < 512; j++) {
            write_buf[j] = (uint8_t)(i + j); 
        }

        // 2. Write to a new sector each time to test the whole card
        uint32_t target_sector = DATA_START_SECTOR + i;
        HAL_SD_WriteBlocks(&hsd1, write_buf, target_sector, 1, 1000);
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);

        // 3. Clear and Read back
        memset(read_buf, 0, 512);
        HAL_SD_ReadBlocks(&hsd1, read_buf, target_sector, 1, 1000);
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);

        // 4. Compare
        if (memcmp(write_buf, read_buf, 512) != 0) {
            errors++;
            snprintf(msg, sizeof(msg), "Error at sector %lu!\r\n", target_sector);
            Stream_Log(msg);
        }

        // Heartbeat log every 100 blocks
        if (i % 100 == 0) {
            snprintf(msg, sizeof(msg), "Tested %lu blocks...\r\n", i);
            Stream_Log(msg);
        }
    }

    snprintf(msg, sizeof(msg), "TEST FINISHED. Total Errors: %lu\r\n", errors);
    Stream_Log(msg);
}