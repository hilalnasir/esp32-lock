#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#define SERVO_GPIO      26    
#define IR_GPIO         14      

#define SERVO_LOCKED    500     
#define SERVO_UNLOCKED  2500    

#define RMT_CLK_HZ      1000000 
#define IR_NEC_BITS     32      // NEC protocol
#define IR_UNLOCK_CODE  0xFFA25D // Remote code for unlocking the lock

static const char *TAG = "ESP32-LOCK";

#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

#define AWS_ENDPOINT "asczbrce6nb95-ats.iot.us-east-2.amazonaws.com"
#define CLIENT_ID "esp32LockClient"
#define MQTT_PUB_TOPIC "esp32/lock/state"
#define MQTT_SUB_TOPIC "esp32/lock/control"

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[]   asm("_binary_aws_root_ca_pem_end");
extern const uint8_t client_crt_start[] asm("_binary_client_crt_start");
extern const uint8_t client_crt_end[]   asm("_binary_client_crt_end");
extern const uint8_t client_key_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_end[]   asm("_binary_client_key_end");

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static esp_mqtt_client_handle_t mqtt_client = NULL;

static void publish_state(bool unlocked);

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        esp_mqtt_client_subscribe(event->client, MQTT_SUB_TOPIC, 1);
        break;
    case MQTT_EVENT_DATA:
        if (strncmp(event->topic, MQTT_SUB_TOPIC, event->topic_len) == 0) {
            if (strncmp(event->data, "UNLOCK", event->data_len) == 0) {
                publish_state(true);
            } else if (strncmp(event->data, "LOCK", event->data_len) == 0) {
                publish_state(false);
            }
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtts://"AWS_ENDPOINT,
        .client_id = CLIENT_ID,
        .cert_pem = (const char *)aws_root_ca_pem_start,
        .client_cert_pem = (const char *)client_crt_start,
        .client_key_pem = (const char *)client_key_start,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void publish_state(bool unlocked)
{
    if (mqtt_client) {
        const char *msg = unlocked ? "UNLOCKED" : "LOCKED";
        esp_mqtt_client_publish(mqtt_client, MQTT_PUB_TOPIC, msg, 0, 1, 0);
    }
}

void set_servo_us(uint32_t microseconds) {
    uint32_t max_duty = (1 << 14) - 1; 
    uint32_t period = 20000; 
    uint32_t duty = (microseconds * max_duty) / period;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void servo_init() {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50, 
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_conf);
    set_servo_us(SERVO_LOCKED); // Default = locked
}

void ir_task(void *pvParameter) {
    // Set up RMT receiver
    rmt_rx_channel_config_t rx_conf = {
        .gpio_num = IR_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_CLK_HZ,
        .mem_block_symbols = 64,
        .flags = {
            .invert_in = false
        }
    };
    rmt_channel_handle_t rx_chan = NULL;
    rmt_new_rx_channel(&rx_conf, &rx_chan);

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = NULL
    };
    rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL);

    rmt_receive_config_t rcv_config = {
        .signal_range_min_ns = 2000,   // Reduced from 9000 to be within the 3187ns limit
        .signal_range_max_ns = 10000000,
    };

    rmt_symbol_word_t rx_symbols[64];
    set_servo_us(SERVO_LOCKED);
    bool unlocked = false;

    ESP_LOGI(TAG, "IR task running. Point remote and press button...");

    while (1) {
        rmt_enable(rx_chan);  // Enable the channel before receiving
        esp_err_t res = rmt_receive(rx_chan, rx_symbols, sizeof(rx_symbols), &rcv_config);
        if (res == ESP_OK) {
            // Parse NEC protocol manually
            uint32_t data = 0;
            
            // Basic NEC protocol parsing
            if (rx_symbols[0].level0 == 0 && rx_symbols[0].duration0 > 8000) {
                // Start bit detected
                uint32_t bits = 0;
                ESP_LOGI(TAG, "Start bit detected, duration: %d", rx_symbols[0].duration0);
                
                for (int i = 1; i < 33; i++) {
                    if (rx_symbols[i].level0 == 1 && rx_symbols[i].duration0 > 1500) {
                        bits |= (1 << (31 - (i-1)));
                        ESP_LOGI(TAG, "Bit %d: 1 (duration: %d)", i, rx_symbols[i].duration0);
                    } else {
                        ESP_LOGI(TAG, "Bit %d: 0 (duration: %d)", i, rx_symbols[i].duration0);
                    }
                }
                data = bits;
                ESP_LOGI(TAG, "Received IR code: 0x%08X", data);
                if (data == IR_UNLOCK_CODE) {
                    unlocked = !unlocked;
                    if (unlocked) {
                        set_servo_us(SERVO_UNLOCKED);
                        ESP_LOGI(TAG, "Lock state changed to: UNLOCKED");
                        publish_state(true);
                    } else {
                        set_servo_us(SERVO_LOCKED);
                        ESP_LOGI(TAG, "Lock state changed to: LOCKED");
                        publish_state(false);
                    }
                } else {
                    ESP_LOGI(TAG, "Unknown IR code received");
                }
            } else {
                ESP_LOGI(TAG, "Invalid start bit: level=%d, duration=%d", 
                         rx_symbols[0].level0, rx_symbols[0].duration0);
            }
        } else {
            ESP_LOGE(TAG, "RMT receive error: %d", res);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    mqtt_init();

    servo_init();
    xTaskCreate(ir_task, "ir_task", 4096, NULL, 5, NULL);
}
