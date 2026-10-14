/**
 * @file    frame.h
 * @brief   Binary sample frame sent to the PC (little endian, 24 bytes)
 *
 *  offset size field
 *   0     2    sync   0xAA 0x55
 *   2     1    cfg    bits0-1 accel range (0..3 = 2/4/8/16 g), bits2-3 gyro range (0..3 = 250/500/1000/2000 dps)
 *   3     1    seq    increments by 1 per frame (wraps at 256) -> PC detects lost frames
 *   4     4    t_us   sample timestamp in microseconds (wraps every ~71 min)
 *   8    14    data   ax ay az temp gx gy gz (int16, calibration offsets already subtracted)
 *  22     2    crc    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 2..21
 */
#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include "mpu6050.h"

#define FRAME_SYNC0  0xAAU
#define FRAME_SYNC1  0x55U
#define FRAME_LEN    24U

uint16_t Frame_Crc16(const uint8_t *data, uint32_t len);

/** Pack one sample into @p out (FRAME_LEN bytes). */
void Frame_Pack(uint8_t out[FRAME_LEN], uint8_t cfg, uint8_t seq, uint32_t t_us,
                const MPU6050_Raw_t *raw);

#endif /* FRAME_H */
