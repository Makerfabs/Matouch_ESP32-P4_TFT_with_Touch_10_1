#include "SensorApp.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <cmath>
#include "mqtt_client.h"
#include "esp_event.h"
#include "freertos/queue.h"

#define MQTT_BROKER_URI "mqtt://broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32p4_sensor"
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_TOPIC_TEMP "temperature"
#define MQTT_TOPIC_HUM "humidity"

#define TEMP_THRESHOLD 0.1f
#define HUM_THRESHOLD 0.1f
#define FAIL_RETRY_COUNT 3
#define DHT_TASK_PRIORITY 2
#define MQTT_TASK_PRIORITY 3

// 队列：传感器数据 -> MQTT 任务（解耦核心）
typedef struct
{
    float temperature;
    float humidity;
    bool sensor_ready;
} sensor_data_t;

const char *SensorApp::TAG = "SensorApp";

SensorApp::SensorApp() : ESP_Brookesia_PhoneApp("DHT11", nullptr, true),
                         _screen(nullptr),
                         _app_cont(nullptr),
                         _main_cont(nullptr),
                         _temp_panel(nullptr),
                         _hum_panel(nullptr),
                         _lbl_temp_val(nullptr),
                         _lbl_temp_unit(nullptr),
                         _lbl_hum_val(nullptr),
                         _lbl_hum_unit(nullptr),
                         _lbl_status(nullptr),
                         _ui_timer(nullptr),
                         _dht_task_handle(nullptr),
                         _dht_task_running(false),
                         _mqtt_task_handle(nullptr),
                         _mqtt_task_running(false),
                         _sensor_queue(nullptr),
                         _mqtt_client(nullptr),
                         _mqtt_connected(false),
                         _temperature(0.0f),
                         _humidity(0.0f),
                         _data_changed(false),
                         _sensor_ready(false),
                         _last_sensor_state(false),
                         _last_temp(-999.0f),
                         _last_hum(-999.0f)
{
}

SensorApp::~SensorApp()
{
    close();
}

bool SensorApp::init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DHT_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(DHT_GPIO, 1);

    _sensor_queue = xQueueCreate(1, sizeof(sensor_data_t));
    return true;
}

bool SensorApp::close(void)
{

    _data_changed = false;

    if (_ui_timer)
    {
        lv_timer_del(_ui_timer);
        _ui_timer = nullptr;
    }

    _dht_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(300));

    if (_app_cont != nullptr)
    {
        lv_obj_del(_app_cont);
        _app_cont = nullptr;
        _main_cont = nullptr;
        _temp_panel = nullptr;
        _hum_panel = nullptr;
        _lbl_temp_val = nullptr;
        _lbl_temp_unit = nullptr;
        _lbl_hum_val = nullptr;
        _lbl_hum_unit = nullptr;
        _lbl_status = nullptr;
    }

    gpio_set_level(DHT_GPIO, 1);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    return true;
}

bool SensorApp::run(void)
{
    if (!_app_cont)
    {
        _create_ui();
    }

    if (!_ui_timer)
    {
        _ui_timer = lv_timer_create(_ui_timer_cb, 1000, this);
    }

    if (_dht_task_handle == nullptr)
    {
        _dht_task_running = true;
        xTaskCreate(_dht_task_wrapper, "dht_task", 4096, this, DHT_TASK_PRIORITY, &_dht_task_handle);
    }

    if (_mqtt_task_handle == nullptr)
    {
        _mqtt_task_running = true;
        xTaskCreate(_mqtt_task_wrapper, "mqtt_task", 8192, this, MQTT_TASK_PRIORITY, &_mqtt_task_handle);
    }

    return true;
}

bool SensorApp::back(void)
{
    close();
    notifyCoreClosed();
    return true;
}

void SensorApp::_create_ui()
{
    _screen = lv_scr_act();

    if (_app_cont == nullptr)
    {
        _app_cont = lv_obj_create(_screen);
        lv_obj_set_size(_app_cont, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_opa(_app_cont, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(_app_cont, lv_color_hex(0x101010), 0);
        lv_obj_set_style_border_width(_app_cont, 0, 0);
        lv_obj_set_style_radius(_app_cont, 0, 0);
        lv_obj_align(_app_cont, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_anim_time(_app_cont, 0, 0);
        lv_obj_clear_flag(_app_cont, LV_OBJ_FLAG_CLICKABLE);
    }

    lv_obj_t *title = lv_label_create(_app_cont);
    lv_label_set_text(title, "DHT11");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_anim_time(title, 0, 0);

    _main_cont = lv_obj_create(_app_cont);
    lv_obj_set_size(_main_cont, LV_PCT(90), LV_PCT(70));
    lv_obj_align(_main_cont, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_bg_opa(_main_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(_main_cont, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(_main_cont, 0, 0);
    lv_obj_set_style_radius(_main_cont, 20, 0);
    lv_obj_set_style_pad_all(_main_cont, 20, 0);
    lv_obj_set_style_pad_gap(_main_cont, 20, 0);
    lv_obj_set_flex_flow(_main_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(_main_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);
    lv_obj_set_style_anim_time(_main_cont, 0, 0);

    _temp_panel = lv_obj_create(_main_cont);
    lv_obj_set_size(_temp_panel, LV_PCT(100), LV_PCT(45));
    lv_obj_set_style_bg_opa(_temp_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(_temp_panel, lv_color_hex(0x2a4a6a), 0);
    lv_obj_set_style_border_width(_temp_panel, 0, 0);
    lv_obj_set_style_radius(_temp_panel, 15, 0);
    lv_obj_set_flex_flow(_temp_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(_temp_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(_temp_panel, 15, 0);
    lv_obj_set_style_anim_time(_temp_panel, 0, 0);

    lv_obj_t *temp_label = lv_label_create(_temp_panel);
    lv_label_set_text(temp_label, "Temperature");
    lv_obj_set_style_text_color(temp_label, lv_color_hex(0xccccff), 0);
    lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_anim_time(temp_label, 0, 0);

    lv_obj_t *temp_value_cont = lv_obj_create(_temp_panel);
    lv_obj_set_size(temp_value_cont, LV_PCT(100), 60);
    lv_obj_set_style_bg_opa(temp_value_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(temp_value_cont, 0, 0);
    lv_obj_set_flex_flow(temp_value_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(temp_value_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_anim_time(temp_value_cont, 0, 0);

    _lbl_temp_val = lv_label_create(temp_value_cont);
    lv_label_set_text(_lbl_temp_val, "--.-");
    lv_obj_set_style_text_font(_lbl_temp_val, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(_lbl_temp_val, lv_color_hex(0xff6b6b), 0);
    lv_obj_set_width(_lbl_temp_val, 80);
    lv_obj_set_style_text_align(_lbl_temp_val, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(_lbl_temp_val, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_anim_time(_lbl_temp_val, 0, 0);
    lv_obj_clear_flag(_lbl_temp_val, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(_lbl_temp_val, LV_OBJ_FLAG_SCROLLABLE);

    _lbl_temp_unit = lv_label_create(temp_value_cont);
    lv_label_set_text(_lbl_temp_unit, "°C");
    lv_obj_set_style_text_font(_lbl_temp_unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(_lbl_temp_unit, lv_color_hex(0xcccccc), 0);
    lv_obj_set_style_pad_left(_lbl_temp_unit, 5, 0);
    lv_obj_set_style_anim_time(_lbl_temp_unit, 0, 0);

    _hum_panel = lv_obj_create(_main_cont);
    lv_obj_set_size(_hum_panel, LV_PCT(100), LV_PCT(45));
    lv_obj_set_style_bg_opa(_hum_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(_hum_panel, lv_color_hex(0x2a6a4a), 0);
    lv_obj_set_style_border_width(_hum_panel, 0, 0);
    lv_obj_set_style_radius(_hum_panel, 15, 0);
    lv_obj_set_flex_flow(_hum_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(_hum_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(_hum_panel, 15, 0);
    lv_obj_set_style_anim_time(_hum_panel, 0, 0);

    lv_obj_t *hum_label = lv_label_create(_hum_panel);
    lv_label_set_text(hum_label, "Humidity");
    lv_obj_set_style_text_color(hum_label, lv_color_hex(0xccffcc), 0);
    lv_obj_set_style_text_font(hum_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_anim_time(hum_label, 0, 0);

    lv_obj_t *hum_value_cont = lv_obj_create(_hum_panel);
    lv_obj_set_size(hum_value_cont, LV_PCT(100), 60);
    lv_obj_set_style_bg_opa(hum_value_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(hum_value_cont, 0, 0);
    lv_obj_set_flex_flow(hum_value_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hum_value_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_anim_time(hum_value_cont, 0, 0);

    _lbl_hum_val = lv_label_create(hum_value_cont);
    lv_label_set_text(_lbl_hum_val, "--.-");
    lv_obj_set_style_text_font(_lbl_hum_val, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(_lbl_hum_val, lv_color_hex(0x6bcfff), 0);
    lv_obj_set_width(_lbl_hum_val, 80);
    lv_obj_set_style_text_align(_lbl_hum_val, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(_lbl_hum_val, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_anim_time(_lbl_hum_val, 0, 0);
    lv_obj_clear_flag(_lbl_hum_val, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(_lbl_hum_val, LV_OBJ_FLAG_SCROLLABLE);

    _lbl_hum_unit = lv_label_create(hum_value_cont);
    lv_label_set_text(_lbl_hum_unit, "%");
    lv_obj_set_style_text_font(_lbl_hum_unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(_lbl_hum_unit, lv_color_hex(0xcccccc), 0);
    lv_obj_set_style_pad_left(_lbl_hum_unit, 5, 0);
    lv_obj_set_style_anim_time(_lbl_hum_unit, 0, 0);

    _lbl_status = lv_label_create(_app_cont);
    lv_label_set_text(_lbl_status, "...");
    lv_obj_set_style_text_color(_lbl_status, lv_color_hex(0xaaaaaa), 0);
    lv_obj_set_style_text_font(_lbl_status, &lv_font_montserrat_14, 0);
    lv_obj_align(_lbl_status, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_anim_time(_lbl_status, 0, 0);

    _last_sensor_state = false;
    _last_temp = -999.0f;
    _last_hum = -999.0f;
    _data_changed = true;
    _update_ui();
}

void SensorApp::_update_ui()
{
    if (!_data_changed || !_app_cont)
        return;

    if (_sensor_ready)
    {
        if (fabs(_temperature - _last_temp) > TEMP_THRESHOLD || _last_temp == -999.0f)
        {
            char temp_str[16];
            snprintf(temp_str, sizeof(temp_str), "%.1f", _temperature);
            lv_label_set_text(_lbl_temp_val, temp_str);
            lv_obj_invalidate(_lbl_temp_val);
            _last_temp = _temperature;
        }

        if (fabs(_humidity - _last_hum) > HUM_THRESHOLD || _last_hum == -999.0f)
        {
            char hum_str[16];
            snprintf(hum_str, sizeof(hum_str), "%.1f", _humidity);
            lv_label_set_text(_lbl_hum_val, hum_str);
            lv_obj_invalidate(_lbl_hum_val);
            _last_hum = _humidity;
        }
    }

    if (_sensor_ready != _last_sensor_state)
    {
        lv_label_set_text(_lbl_status, _sensor_ready ? "OK" : "...");
        lv_obj_set_style_text_color(_lbl_status, _sensor_ready ? lv_color_hex(0x00ff00) : lv_color_hex(0xffaa00), 0);
        lv_obj_invalidate(_lbl_status);
        _last_sensor_state = _sensor_ready;
    }
}

void SensorApp::_ui_timer_cb(lv_timer_t *timer)
{
    SensorApp *app = static_cast<SensorApp *>(timer->user_data);
    if (app && app->_data_changed)
    {
        app->_update_ui();
    }
}

bool SensorApp::_dht_read(float *temp, float *hum)
{
    uint8_t data[5] = {0};
    uint8_t checksum;
    int timeout;

    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(50000);
    gpio_set_level(DHT_GPIO, 0);
    esp_rom_delay_us(20000);
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(40);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1)
    {
        esp_rom_delay_us(1);
        if (timeout++ > 100)
            return false;
    }
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 0)
    {
        esp_rom_delay_us(1);
        if (timeout++ > 100)
            return false;
    }
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1)
    {
        esp_rom_delay_us(1);
        if (timeout++ > 100)
            return false;
    }

    for (int i = 0; i < 40; i++)
    {
        timeout = 0;
        while (gpio_get_level(DHT_GPIO) == 0)
        {
            esp_rom_delay_us(1);
            if (timeout++ > 200)
                return false;
        }
        esp_rom_delay_us(30);
        if (gpio_get_level(DHT_GPIO) == 1)
        {
            data[i / 8] |= (1 << (7 - i % 8));
            timeout = 0;
            while (gpio_get_level(DHT_GPIO) == 1)
            {
                esp_rom_delay_us(1);
                if (timeout++ > 200)
                    return false;
            }
        }
    }

    checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4])
        return false;

    if (SENSOR_TYPE == 1)
    {
        *hum = data[0];
        *temp = data[2];
    }
    else
    {
        *hum = (data[0] << 8 | data[1]) / 10.0f;
        int16_t t = (data[2] << 8 | data[3]);
        *temp = (t & 0x8000) ? -(t & 0x7FFF) / 10.0f : t / 10.0f;
    }
    return true;
}

void SensorApp::_dht_read_task()
{
    float t, h;
    int fail_count = 0;
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (_dht_task_running)
    {
        if (_dht_read(&t, &h))
        {
            if (fabs(t - _temperature) > TEMP_THRESHOLD || fabs(h - _humidity) > HUM_THRESHOLD || !_sensor_ready)
            {
                _temperature = t;
                _humidity = h;
                _sensor_ready = true;
                _data_changed = true;

                sensor_data_t data;
                data.temperature = t;
                data.humidity = h;
                data.sensor_ready = true;
                xQueueOverwrite(_sensor_queue, &data);
            }
            fail_count = 0;
        }
        else
        {
            fail_count++;
            if (fail_count >= FAIL_RETRY_COUNT)
            {
                _sensor_ready = false;
                _data_changed = true;
                fail_count = 0;

                sensor_data_t data;
                data.sensor_ready = false;
                xQueueOverwrite(_sensor_queue, &data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    _dht_task_handle = nullptr;
    vTaskDelete(NULL);
}

void SensorApp::_dht_task_wrapper(void *arg)
{
    SensorApp *app = static_cast<SensorApp *>(arg);
    if (app)
        app->_dht_read_task();
}

void SensorApp::_mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    if (!handler_args)
        return;
    SensorApp *app = static_cast<SensorApp *>(handler_args);

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        app->_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        app->_mqtt_connected = false;
        break;
    default:
        break;
    }
}

bool SensorApp::_mqtt_init()
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;
    mqtt_cfg.broker.address.port = MQTT_PORT;
    mqtt_cfg.credentials.client_id = MQTT_CLIENT_ID;
    mqtt_cfg.credentials.username = MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password = MQTT_PASSWORD;

    _mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!_mqtt_client)
        return false;

    esp_mqtt_client_register_event(_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, _mqtt_event_handler, this);
    esp_mqtt_client_start(_mqtt_client);
    return true;
}

void SensorApp::_mqtt_publish_task()
{

    _mqtt_init();
    sensor_data_t data;

    while (_mqtt_task_running)
    {

        xQueuePeek(_sensor_queue, &data, pdMS_TO_TICKS(10));

        if (_mqtt_connected && data.sensor_ready)
        {
            char temp_msg[32];
            snprintf(temp_msg, sizeof(temp_msg), "%.1f", data.temperature);
            int msg_id = esp_mqtt_client_publish(_mqtt_client, MQTT_TOPIC_TEMP, temp_msg, 0, 1, 0);
            ESP_LOGI(TAG, "Published temperature: %s to %s, msg_id=%d", temp_msg, MQTT_TOPIC_TEMP, msg_id);

            char hum_msg[32];
            snprintf(hum_msg, sizeof(hum_msg), "%.1f", data.humidity);
            msg_id = esp_mqtt_client_publish(_mqtt_client, MQTT_TOPIC_HUM, hum_msg, 0, 1, 0);
            ESP_LOGI(TAG, "Published humidity: %s to %s, msg_id=%d", hum_msg, MQTT_TOPIC_HUM, msg_id);
            
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    if (_mqtt_client != nullptr)
    {
        esp_mqtt_client_stop(_mqtt_client);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_mqtt_client_destroy(_mqtt_client);
        _mqtt_client = nullptr;
        _mqtt_connected = false;
    }

    _mqtt_task_handle = nullptr;
    vTaskDelete(NULL);
}

void SensorApp::_mqtt_task_wrapper(void *arg)
{
    SensorApp *app = static_cast<SensorApp *>(arg);
    if (app)
        app->_mqtt_publish_task();
}