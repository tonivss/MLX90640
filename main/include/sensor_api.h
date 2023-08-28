#ifndef _SENSOR_API_H_
#define _SENSOR_API_H_

#include "esp_err.h"

typedef esp_err_t (*sensor_init_func_t)(void);
typedef esp_err_t (*sensor_read_func_t)(void *data);
typedef esp_err_t (*sensor_write_func_t)(const void *data);
typedef esp_err_t (*sensor_deinit_func_t)(void);
typedef char* (*sensor_mqtt_topic_func_t)(void);
typedef char* (*sensor_mqtt_payload_func_t)(const void *data);
typedef void (*sensor_mqtt_message_handler_func_t)(const char *topic, const char *payload);

typedef struct {
    sensor_init_func_t init;
    sensor_read_func_t read;
    sensor_write_func_t write;
    sensor_deinit_func_t deinit;
    sensor_mqtt_topic_func_t get_mqtt_topic;
    sensor_mqtt_payload_func_t get_mqtt_payload;
    sensor_mqtt_message_handler_func_t handle_mqtt_message;
    size_t data_size;
} sensor_api_t;

#endif // SENSOR_API_H
