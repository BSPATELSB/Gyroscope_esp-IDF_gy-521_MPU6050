#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

// I2C Master Initialization
esp_err_t i2c_master_init(void);

// MPU6050 Initialization
void mpu6050_init(void);

// Function to read accelerometer values from MPU6050
void mpu6050_read_accel(void);

// Function to write a byte to a specific MPU6050 register
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data);

// Function to read a byte from a specific MPU6050 register
esp_err_t mpu6050_read_byte(uint8_t reg_addr, uint8_t *data);

#endif // MPU6050_H

