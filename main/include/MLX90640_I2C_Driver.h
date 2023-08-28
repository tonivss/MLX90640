/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _MLX90640_I2C_Driver_H_
#define _MLX90640_I2C_Driver_H_

#include <stdint.h>
#include "MLX90640_API.h"
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_api.h"

extern TaskHandle_t MLX90640_task_handle;               // Task handle for MLX90640 task


//#define SDA_PIN 21
//#define SCL_PIN 22
#define I2C_BUS_FREQUENCY_HZ 1000000
#define ACK_CHECK_EN 0x1            
#define ACK_CHECK_DIS 0x0          
#define ACK_VAL 0x0               
#define NACK_VAL 0x1 
#define I2C_MASTER_TIMEOUT_MS 1000

extern const char *MLX90640_TAG;            /*!< Tag name of the module */
extern bool mlx_isConnected;                            /*!< Flag to indicate if the MLX90640 is connected */



#define I2C_MASTER_FREQ_HZ          1000000     /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000
//#define MLX90640_I2C_ADDR           0x33        /*!< Slave address of the MPU9250 sensor */
#define ACK_CHECK_EN                0x1         /*!< I2C master will check ack from slave*/

// data structure for mlx90640
typedef struct {
    uint8_t grayscale_val_array[768];
} mlx90640_data_t;

extern sensor_api_t mlx90640_api;

typedef struct
{
    float min;
    float max;
} mm;

extern mm minmax(const float* input,size_t input_size);
extern bool MLX90640_isConnected(void);
extern void MLX90640_getFrames(void);
extern void MLX90640_config(void);
extern void MLX90640_init(void);
extern void MLX90640_I2CInit(void);
extern int MLX90640_I2CGeneralReset(void);
extern int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
extern int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
extern void MLX90640_I2CFreqSet(int freq);

// own functions
extern void temperatureToGrayscale(float *temperatureData, int len, float minTemp, float maxTemp, uint8_t *grayscaleData);
extern char *convertGrayscaleArrayToString(uint8_t *grayscaleArray, int len);
extern char *convertFloatArrayToString(float *arr, int len);
#endif