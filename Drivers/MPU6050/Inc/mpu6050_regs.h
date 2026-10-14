/**
 * @file    mpu6050_regs.h
 * @brief   MPU-6050 register map (subset used by the driver)
 */
#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

/* 7-bit I2C address; AD0 pin low = 0x68, high = 0x69 */
#define MPU6050_I2C_ADDR_AD0_LOW    0x68U
#define MPU6050_I2C_ADDR_AD0_HIGH   0x69U

/* Registers */
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_INT_ENABLE      0x38U
#define MPU6050_REG_INT_STATUS      0x3AU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU   /* start of the 14-byte burst block */
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_WHO_AM_I        0x75U

/* PWR_MGMT_1 bits */
#define MPU6050_PWR1_DEVICE_RESET   0x80U
#define MPU6050_PWR1_SLEEP          0x40U
#define MPU6050_PWR1_CLKSEL_PLL_XG  0x01U   /* PLL with X-axis gyro reference */

/* WHO_AM_I value (upper 6 bits of the address, independent of AD0) */
#define MPU6050_WHO_AM_I_VALUE      0x68U

/* Length of ACCEL_XOUT_H .. GYRO_ZOUT_L */
#define MPU6050_DATA_LEN            14U

#endif /* MPU6050_REGS_H */
