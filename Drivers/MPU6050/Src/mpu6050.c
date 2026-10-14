/**
 * @file    mpu6050.c
 * @brief   MPU-6050 driver over I2C, STM32 HAL
 */
#include "mpu6050.h"

#define MPU6050_TEMP_SCALE      340.0f
#define MPU6050_TEMP_OFFSET_C   36.53f
#define MPU6050_CAL_MAX_SAMPLES 1024U

static MPU6050_Status hal_to_status(HAL_StatusTypeDef st)
{
    switch (st) {
    case HAL_OK:      return MPU6050_OK;
    case HAL_TIMEOUT: return MPU6050_ERR_TIMEOUT;
    default:          return MPU6050_ERR_I2C;
    }
}

static MPU6050_Status reg_read(MPU6050_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return hal_to_status(HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT,
                                          buf, len, MPU6050_I2C_TIMEOUT_MS));
}

static MPU6050_Status reg_write(MPU6050_t *dev, uint8_t reg, uint8_t val)
{
    return hal_to_status(HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT,
                                           &val, 1U, MPU6050_I2C_TIMEOUT_MS));
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

MPU6050_Status MPU6050_Init(MPU6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t ad0,
                            const MPU6050_Config_t *cfg)
{
    static const MPU6050_Config_t def = MPU6050_CONFIG_DEFAULT;
    static const float accel_lsb[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
    static const float gyro_lsb[]  = { 131.0f, 65.5f, 32.8f, 16.4f };
    MPU6050_Status st;
    uint8_t id;

    if (dev == NULL || hi2c == NULL) {
        return MPU6050_ERR_PARAM;
    }
    if (cfg == NULL) {
        cfg = &def;
    }
    if ((unsigned)cfg->accel_range > MPU6050_ACCEL_16G ||
        (unsigned)cfg->gyro_range  > MPU6050_GYRO_2000DPS ||
        (unsigned)cfg->dlpf        > MPU6050_DLPF_5HZ) {
        return MPU6050_ERR_PARAM;
    }

    dev->hi2c = hi2c;
    dev->addr = (uint8_t)((ad0 ? MPU6050_I2C_ADDR_AD0_HIGH : MPU6050_I2C_ADDR_AD0_LOW) << 1);
    for (int i = 0; i < 3; i++) {
        dev->accel_off[i] = 0;
        dev->gyro_off[i]  = 0;
    }

    st = hal_to_status(HAL_I2C_IsDeviceReady(hi2c, dev->addr, 3U, MPU6050_I2C_TIMEOUT_MS));
    if (st != MPU6050_OK) return st;

    st = reg_read(dev, MPU6050_REG_WHO_AM_I, &id, 1U);
    if (st != MPU6050_OK) return st;
    if (id != MPU6050_WHO_AM_I_VALUE) return MPU6050_ERR_ID;

    /* Reset all registers, then wait for the reset bit to self-clear. */
    st = reg_write(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (st != MPU6050_OK) return st;
    HAL_Delay(100);

    uint8_t pwr = MPU6050_PWR1_DEVICE_RESET;
    uint32_t t0 = HAL_GetTick();
    while (pwr & MPU6050_PWR1_DEVICE_RESET) {
        st = reg_read(dev, MPU6050_REG_PWR_MGMT_1, &pwr, 1U);
        if (st != MPU6050_OK) return st;
        if ((HAL_GetTick() - t0) > MPU6050_I2C_TIMEOUT_MS) return MPU6050_ERR_TIMEOUT;
    }

    /* Wake up (clear SLEEP), use the gyro PLL as clock source. */
    st = reg_write(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_XG);
    if (st != MPU6050_OK) return st;
    HAL_Delay(10);

    st = reg_write(dev, MPU6050_REG_SMPLRT_DIV, cfg->sample_rate_div);
    if (st != MPU6050_OK) return st;
    st = reg_write(dev, MPU6050_REG_CONFIG, (uint8_t)cfg->dlpf);
    if (st != MPU6050_OK) return st;
    st = reg_write(dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)((unsigned)cfg->gyro_range << 3));
    if (st != MPU6050_OK) return st;
    st = reg_write(dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)((unsigned)cfg->accel_range << 3));
    if (st != MPU6050_OK) return st;

    dev->accel_lsb_per_g  = accel_lsb[cfg->accel_range];
    dev->gyro_lsb_per_dps = gyro_lsb[cfg->gyro_range];
    return MPU6050_OK;
}

MPU6050_Status MPU6050_ReadRaw(MPU6050_t *dev, MPU6050_Raw_t *raw)
{
    uint8_t b[MPU6050_DATA_LEN];
    MPU6050_Status st;

    if (dev == NULL || raw == NULL) {
        return MPU6050_ERR_PARAM;
    }
    st = reg_read(dev, MPU6050_REG_ACCEL_XOUT_H, b, MPU6050_DATA_LEN);
    if (st != MPU6050_OK) return st;

    raw->ax   = be16(&b[0]);
    raw->ay   = be16(&b[2]);
    raw->az   = be16(&b[4]);
    raw->temp = be16(&b[6]);
    raw->gx   = be16(&b[8]);
    raw->gy   = be16(&b[10]);
    raw->gz   = be16(&b[12]);
    return MPU6050_OK;
}

static int16_t sat16(int32_t v)
{
    return (int16_t)(v > INT16_MAX ? INT16_MAX : (v < INT16_MIN ? INT16_MIN : v));
}

MPU6050_Status MPU6050_ReadRawCalibrated(MPU6050_t *dev, MPU6050_Raw_t *raw)
{
    MPU6050_Status st = MPU6050_ReadRaw(dev, raw);
    if (st != MPU6050_OK) return st;

    raw->ax = sat16((int32_t)raw->ax - dev->accel_off[0]);
    raw->ay = sat16((int32_t)raw->ay - dev->accel_off[1]);
    raw->az = sat16((int32_t)raw->az - dev->accel_off[2]);
    raw->gx = sat16((int32_t)raw->gx - dev->gyro_off[0]);
    raw->gy = sat16((int32_t)raw->gy - dev->gyro_off[1]);
    raw->gz = sat16((int32_t)raw->gz - dev->gyro_off[2]);
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Read(MPU6050_t *dev, MPU6050_Data_t *data)
{
    MPU6050_Raw_t r;
    MPU6050_Status st;

    if (data == NULL) {
        return MPU6050_ERR_PARAM;
    }
    st = MPU6050_ReadRaw(dev, &r);
    if (st != MPU6050_OK) return st;

    data->ax = (float)((int32_t)r.ax - dev->accel_off[0]) / dev->accel_lsb_per_g;
    data->ay = (float)((int32_t)r.ay - dev->accel_off[1]) / dev->accel_lsb_per_g;
    data->az = (float)((int32_t)r.az - dev->accel_off[2]) / dev->accel_lsb_per_g;
    data->gx = (float)((int32_t)r.gx - dev->gyro_off[0]) / dev->gyro_lsb_per_dps;
    data->gy = (float)((int32_t)r.gy - dev->gyro_off[1]) / dev->gyro_lsb_per_dps;
    data->gz = (float)((int32_t)r.gz - dev->gyro_off[2]) / dev->gyro_lsb_per_dps;
    data->temp_c = (float)r.temp / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET_C;
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Calibrate(MPU6050_t *dev, uint16_t samples)
{
    int32_t sum[6] = { 0 };
    MPU6050_Raw_t r;
    MPU6050_Status st;

    if (dev == NULL || samples == 0U || samples > MPU6050_CAL_MAX_SAMPLES) {
        return MPU6050_ERR_PARAM;
    }

    for (uint16_t i = 0; i < samples; i++) {
        st = MPU6050_ReadRaw(dev, &r);
        if (st != MPU6050_OK) return st;
        sum[0] += r.ax; sum[1] += r.ay; sum[2] += r.az;
        sum[3] += r.gx; sum[4] += r.gy; sum[5] += r.gz;
        HAL_Delay(2);   /* >= one sample period at the default 1 kHz internal rate */
    }

    dev->accel_off[0] = (int16_t)(sum[0] / samples);
    dev->accel_off[1] = (int16_t)(sum[1] / samples);
    dev->accel_off[2] = (int16_t)(sum[2] / samples - (int32_t)dev->accel_lsb_per_g); /* remove +1 g */
    dev->gyro_off[0]  = (int16_t)(sum[3] / samples);
    dev->gyro_off[1]  = (int16_t)(sum[4] / samples);
    dev->gyro_off[2]  = (int16_t)(sum[5] / samples);
    return MPU6050_OK;
}

MPU6050_Status MPU6050_RecoverBus(MPU6050_t *dev)
{
    if (dev == NULL || dev->hi2c == NULL) {
        return MPU6050_ERR_PARAM;
    }
    if (HAL_I2C_DeInit(dev->hi2c) != HAL_OK) return MPU6050_ERR_I2C;
    if (HAL_I2C_Init(dev->hi2c)   != HAL_OK) return MPU6050_ERR_I2C;
    return MPU6050_OK;
}
