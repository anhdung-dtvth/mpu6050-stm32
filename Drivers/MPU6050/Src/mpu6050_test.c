#include "mpu6050.h"
#include "mpu6050_regs.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

#define MPU6050_TEMP_SCALE      340.0f
#define MPU6050_TEMP_OFFSET_C   36.53f
#define MPU6050_CAL_MAX_SAMPLES 1024U

static MPU6050_Status hal_to_status(HAL_StatusTypeDef st) {
    switch(st) {
        case HAL_OK: return MPU6050_OK;
        case HAL_TIMEOUT: return MPU6050_ERR_TIMEOUT;
        default: return MPU6050_ERR_I2C;
    }
}

static MPU6050_Status read_reg(MPU6050_t *dev, uint8_t reg, uint8_t *buf, uint16_t len) {
    return hal_to_status(HAL_I2C_Mem_Read(dev->hi2c, dev->addr, 
        reg, I2C_MEMADD_SIZE_8BIT, 
        buf, len, MPU6050_I2C_TIMEOUT_MS));
}

static MPU6050_Status write_reg(MPU6050_t *dev, uint8_t reg, uint8_t val) {
    return hal_to_status(HAL_I2C_Mem_Write(dev->hi2c, dev->addr, 
        reg, I2C_MEMADD_SIZE_8BIT, 
        &val, 1U, MPU6050_I2C_TIMEOUT_MS));
}

static int16_t be16(const uint8_t *p) {
    // Ghep 2 byte thanh thanh mot so nguyen
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

MPU6050_Status MPU6050_Init(MPU6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t ad0, const MPU6050_Config_t *cfg) {
    static const MPU6050_Config_t def = MPU6050_CONFIG_DEFAULT;
    // He so quy doi:
    static const float accel_lsb[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    static const float gyro_lsb[] = {131.0f, 65.5f, 32.8f, 16.4f};
    MPU6050_Status st;
    uint8_t id;
    // Kiem tra tham so
    if (dev == NULL || hi2c == NULL) {
        return MPU6050_ERR_PARAM;
    }
    if (cfg == NULL) {
        cfg = &def;
    }
    if ((unsigned)cfg->accel_range  > MPU6050_ACCEL_16G ||
        (unsigned)cfg->gyro_range   > MPU6050_GYRO_2000DPS ||
        (unsigned)cfg->dlpf         > MPU6050_DLPF_5HZ
    ) {
        return MPU6050_ERR_PARAM;
    }

    // Luu thong tin thiet bi
    dev->hi2c = hi2c;
        // dia chi 7 bit tuy chan AD0 -> 8bit vi HAL yeu cau 8bit
    dev->addr = (uint8_t)((ad0 ? MPU6050_I2C_ADDR_AD0_HIGH : MPU6050_I2C_ADDR_AD0_LOW));
        // xoa cac offset calibrate ve 0
    for(int i = 0; i < 3; i++) {
        dev->accel_off[i] = 0;
        dev->gyro_off[i] = 0;
    }

    // Kiem tra cam bien
    st = hal_to_status(HAL_I2C_IsDeviceReady(hi2c, dev->addr, 3U, MPU6050_I2C_TIMEOUT_MS));
    if (st != MPU6050_OK) return st;
    st = read_reg(dev, MPU6050_REG_WHO_AM_I, &id, 1U);
    if (st != MPU6050_OK) return st;
    if (id != MPU6050_WHO_AM_I_VALUE) return MPU6050_ERR_ID;

    // reset chip
    st = write_reg(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (st != MPU6050_OK) return st;
    uint8_t pwr = MPU6050_REG_PWR_MGMT_1;
    uint32_t t0 = HAL_GetTick();
    while (pwr & MPU6050_REG_PWR_MGMT_1) {
        st = read_reg(dev, MPU6050_REG_PWR_MGMT_1, &pwr, 1U);
        if (st != MPU6050_OK) return st;
        if ((HAL_GetTick() - t0) > MPU6050_I2C_TIMEOUT_MS)  return MPU6050_ERR_TIMEOUT;
    }

    // danh thuc va chon clock sau khi chip reset
    st = write_reg(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_XG);
    if (st != MPU6050_OK) return st;

    // Ghi cau hinh do
    st = reg_write(dev, MPU6050_REG_SMPLRT_DIV, cfg->sample_rate_div);
    if (st != MPU6050_OK) return st;
    st = reg_write(dev, MPU6050_REG_CONFIG, (uint8_t)cfg->dlpf);
    if (st != MPU6050_OK) return st;
    st = reg_write(dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)((unsigned)cfg->gyro_range << 3));
    if (st != MPU6050_OK) return st;
    st = reg_write(dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)((unsigned)cfg->accel_range << 3));
    if (st != MPU6050_OK) return st;
    
    // luu he so quy doi
    dev->accel_lsb_per_g = accel_lsb[cfg->accel_range];
    dev->gyro_lsb_per_dps = gyro_lsb[cfg->gyro_range];
    return MPU6050_OK;
}