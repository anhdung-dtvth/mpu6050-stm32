/**
 * @file    frame.c
 * @brief   Binary sample frame packing (see frame.h for the layout)
 */
#include "frame.h"

uint16_t Frame_Crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;

    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

void Frame_Pack(uint8_t out[FRAME_LEN], uint8_t cfg, uint8_t seq, uint32_t t_us,
                const MPU6050_Raw_t *raw)
{
    out[0] = FRAME_SYNC0;
    out[1] = FRAME_SYNC1;
    out[2] = cfg;
    out[3] = seq;
    put_u16(&out[4], (uint16_t)(t_us & 0xFFFFU));
    put_u16(&out[6], (uint16_t)(t_us >> 16));
    put_u16(&out[8],  (uint16_t)raw->ax);
    put_u16(&out[10], (uint16_t)raw->ay);
    put_u16(&out[12], (uint16_t)raw->az);
    put_u16(&out[14], (uint16_t)raw->temp);
    put_u16(&out[16], (uint16_t)raw->gx);
    put_u16(&out[18], (uint16_t)raw->gy);
    put_u16(&out[20], (uint16_t)raw->gz);
    put_u16(&out[22], Frame_Crc16(&out[2], 20U));
}
