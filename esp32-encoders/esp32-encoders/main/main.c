#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "string.h"
#include "esp_vfs_dev.h"
#include <stdio.h>
#include <stdlib.h>

#define ENCODER1_PIN GPIO_NUM_34
#define ENCODER2_PIN GPIO_NUM_35

#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3
#define UART_PORT_NUM UART_NUM_0

#define TICKS_PER_REV 700.0
#define WHEEL_RADIUS 0.10
#define WHEEL_BASE 0.42

const float V_MAX = 1.0f; // [m/s] — ustal doświadczalnie

// Silnik lewy
#define ENA GPIO_NUM_12
#define IN1 GPIO_NUM_14
#define IN2 GPIO_NUM_27
// Silnik prawy
#define ENB GPIO_NUM_26
#define IN3 GPIO_NUM_25
#define IN4 GPIO_NUM_33

#define MAX_PWM 255

static volatile int32_t encoder1_count = 0;
static volatile int32_t encoder2_count = 0;

float target_v = 0.0f;
float target_w = 0.0f;

void IRAM_ATTR encoder1_isr_handler(void* arg) {
    encoder1_count++;
}
void IRAM_ATTR encoder2_isr_handler(void* arg) {
    encoder2_count++;
}

void encoder_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<ENCODER1_PIN) | (1ULL<<ENCODER2_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER1_PIN, encoder1_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER2_PIN, encoder2_isr_handler, NULL);
}

void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void motor_driver_init() {
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t channel1 = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config_t channel2 = {
        .channel = LEDC_CHANNEL_1,
        .duty = 0,
        .gpio_num = ENB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&channel1);
    ledc_channel_config(&channel2);
}

void set_motor_pwm(int pwm_l, int pwm_r) {
    // LEFT MOTOR
    if (pwm_l >= 0) {
        gpio_set_level(IN1, 1);
        gpio_set_level(IN2, 0);
    } else {
        gpio_set_level(IN1, 0);
        gpio_set_level(IN2, 1);
        pwm_l = -pwm_l;
    }

    // RIGHT MOTOR
    if (pwm_r >= 0) {
        gpio_set_level(IN3, 1);
        gpio_set_level(IN4, 0);
    } else {
        gpio_set_level(IN3, 0);
        gpio_set_level(IN4, 1);
        pwm_r = -pwm_r;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_l);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm_r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void uart_receive_task(void *arg) {
    uint8_t data[64];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            float vx = 0, wz = 0;
            if (sscanf((char*)data, "Vx:%f,Wz:%f", &vx, &wz) == 2) {
                target_v = vx;
                target_w = wz;
            }
        }
    }
}

void app_main(void) {
    // Inicjalizacja
    encoder_init();
    uart_init();
    motor_driver_init();

    // Uruchom zadanie odbioru UART
    xTaskCreate(uart_receive_task, "uart_receive_task", 2048, NULL, 5, NULL);

    // Zmienna do obliczeń odometrii
    int32_t prev_e1 = 0, prev_e2 = 0;
    const int sample_period_ms = 40;

    while (1) {
        // Odczyt enkoderów
        int32_t curr_e1 = encoder1_count;
        int32_t curr_e2 = encoder2_count;
        int32_t delta_e1 = curr_e1 - prev_e1;
        int32_t delta_e2 = curr_e2 - prev_e2;
        prev_e1 = curr_e1;
        prev_e2 = curr_e2;

        // Oblicz prędkości kół
        float dl = (2.0f * 3.1415f * WHEEL_RADIUS * delta_e1) / TICKS_PER_REV;
        float dr = (2.0f * 3.1415f * WHEEL_RADIUS * delta_e2) / TICKS_PER_REV;
        float vl = dl / (sample_period_ms / 1000.0f);
        float vr = dr / (sample_period_ms / 1000.0f);

        // Wyślij odometrię do ROS2 (np. VL:0.12 VR:0.12\n)
        char msg[64];
        snprintf(msg, sizeof(msg), "VL:%.2f VR:%.2f\n", vl, vr);
        uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));

        // Prędkości docelowe z ROS2
        float v = target_v;
        float w = target_w;

        // Konwersja na prędkości kół (skid steering)
        // float v_left = v * 2000 - (WHEEL_BASE / 2.0f) * w * 100;
        // float v_right = v * 2000 + (WHEEL_BASE / 2.0f) * w * 100;

        float v_left = 180;
        float v_right = -180; 

        // Normalizacja prędkości kół do PWM (zakładamy vmax ≈ 1.0 m/s)
        set_motor_pwm(v_left, v_right);

        // Opóźnienie pętli
        vTaskDelay(pdMS_TO_TICKS(sample_period_ms));
    }
}
