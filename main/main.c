#include "sensor_api.h"
#include <stdio.h>
#include "driver/i2c.h"
#include "MLX90640_I2C_Driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void mlx_init_task()
{
    // Initialize the MLX90640 sensor.
    if (mlx90640_api.init() != ESP_OK) {
        ESP_LOGE(MLX90640_TAG, "Failed to initialize MLX90640!");
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(mlx_init_task, "MLX90640_init", 4096, NULL, 5, NULL);

    mlx90640_data_t mlx_data;

    //while(true) {
        // Read from MLX90640.
    if (mlx90640_api.read(&mlx_data) != ESP_OK) {
        ESP_LOGE(MLX90640_TAG, "Failed to read from DHT22!");
    } else {
        ESP_LOGI(MLX90640_TAG, "Read succsessful!");
        /* for (int i = 0; i < 768; i++) {
            ESP_LOGI(MLX90640_TAG,"Grayscale value: %d", mlx_data.grayscale_val_array[i]);
        } */
    }
/*         vTaskDelay(1000 / portTICK_PERIOD_MS);
    } */
    

}