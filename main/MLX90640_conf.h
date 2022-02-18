#pragma once
#include "driver/gpio.h"
#define MLX90640_USE_ESP32_I2C   1

#if MLX90640_USE_ESP32_I2C
    #define MLX90640_ESP_I2C_SCL_GPIO_PIN	GPIO_NUM_12
    #define MLX90640_ESP_I2C_SDA_GPIO_PIN	GPIO_NUM_13
    #define MLX90640_ESP_I2C_INTERFACE	    I2C_NUM_0
#endif