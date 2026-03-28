#pragma once

#include "esp_brookesia.hpp"
#include "lvgl.h"
#include "driver/gpio.h"
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "esp_event.h"
// Sensor type definition (modified according to actual hardware)
#define SENSOR_TYPE 1        // 1=DHT11，2=DHT22
#define DHT_GPIO GPIO_NUM_20 // DHT11 data pin, modify according to actual wiring

class SensorApp : public ESP_Brookesia_PhoneApp
{
private:
    lv_obj_t *_screen;
    lv_obj_t *_app_cont;
    lv_obj_t *_main_cont;
    lv_obj_t *_temp_panel;
    lv_obj_t *_hum_panel;
    lv_obj_t *_lbl_temp_val;
    lv_obj_t *_lbl_temp_unit;
    lv_obj_t *_lbl_hum_val;
    lv_obj_t *_lbl_hum_unit;
    lv_obj_t *_lbl_status;
    lv_timer_t *_ui_timer;
    TaskHandle_t _dht_task_handle;
    bool _dht_task_running;
    TaskHandle_t _mqtt_task_handle;
    bool _mqtt_task_running;
    QueueHandle_t _sensor_queue;

    // MQTT相关
    esp_mqtt_client_handle_t _mqtt_client;
    bool _mqtt_connected;

    // 传感器数据
    float _temperature;
    float _humidity;
    bool _data_changed;
    bool _sensor_ready;

    // 防抖标记
    bool _last_sensor_state;
    float _last_temp;
    float _last_hum;

    // 私有函数
    void _create_ui();
    void _update_ui();
    static void _ui_timer_cb(lv_timer_t *timer);
    bool _dht_read(float *temp, float *hum);
    void _dht_read_task();
    static void _dht_task_wrapper(void *arg);

    static void _mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    bool _mqtt_init();
    void _mqtt_publish_task();
    static void _mqtt_task_wrapper(void *arg);
    // // MQTT事件回调
    // static void _mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    
    // // MQTT初始化
    // bool _mqtt_init();

    // // 发布传感器数据（你CPP里的真实函数名）
    // void _mqtt_publish_sensor_data();

public:
    static const char *TAG;
    SensorApp();
    ~SensorApp() override;
    bool init(void) override;
    bool close(void) override;
    bool run(void) override;
    bool back(void) override;
};
