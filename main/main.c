#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"
#include "esp_log.h"
#include <string.h>

#define SERVO_GPIO      26    
#define IR_GPIO         14      

#define SERVO_LOCKED    500     
#define SERVO_UNLOCKED  2500    

#define RMT_CLK_HZ      1000000 
#define IR_NEC_BITS     32      // NEC protocol
#define IR_UNLOCK_CODE  0xFFA25D // Remote code for unlocking the lock

static const char *TAG = "ESP32-LOCK";

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
                    } else {
                        set_servo_us(SERVO_LOCKED);
                        ESP_LOGI(TAG, "Lock state changed to: LOCKED");
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
    servo_init();
    xTaskCreate(ir_task, "ir_task", 4096, NULL, 5, NULL);
}
