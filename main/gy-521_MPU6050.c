#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"

void app_main(void) {
    esp_err_t res = i2c_master_init();
    if (res == ESP_OK) {
        ESP_LOGI("APP_MAIN", "I2C initialized successfully");
    } else {
        ESP_LOGE("APP_MAIN", "I2C initialization failed");
        return;
    }

    mpu6050_init();

    while (1) {
        mpu6050_read_accel();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Read data every second
    }
}

