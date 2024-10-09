#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 22        // Set the GPIO for SCL
#define I2C_MASTER_SDA_IO 21        // Set the GPIO for SDA
#define I2C_MASTER_FREQ_HZ 100000   // I2C frequency
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number

#define MPU6050_ADDR 0x68           // MPU6050 I2C address
#define WHO_AM_I_REG 0x75           // Who am I register
#define PWR_MGMT_1_REG 0x6B         // Power management register
#define ACCEL_XOUT_H_REG 0x3B       // Accelerometer data register

static const char *TAG = "MPU6050";

esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    esp_err_t res = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (res == ESP_OK) {
        res = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    }
    return res;
}

esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t mpu6050_read_byte(uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Repeated start for reading
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

void mpu6050_init() {
    // Wake up the MPU6050
    mpu6050_write_byte(PWR_MGMT_1_REG, 0x00);
    ESP_LOGI(TAG, "MPU6050 initialized");
}

void mpu6050_read_accel() {
    uint8_t data[6];
    for (int i = 0; i < 6; i++) {
        mpu6050_read_byte(ACCEL_XOUT_H_REG + i, &data[i]);
    }
    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];

    ESP_LOGI(TAG, "Accel X: %d, Accel Y: %d, Accel Z: %d", accel_x, accel_y, accel_z);
}

