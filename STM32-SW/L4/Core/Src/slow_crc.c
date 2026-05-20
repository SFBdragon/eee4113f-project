#include <stdint.h>

int16_t crc16(const uint8_t *data, uintptr_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uintptr_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0xA7D3);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}
