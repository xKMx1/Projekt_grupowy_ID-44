#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define ENCODER1_PIN GPIO_NUM_32   // Kanał A enkodera 1
#define ENCODER2_PIN GPIO_NUM_35   // Kanał A enkodera 2

static volatile int32_t encoder1_count = 0;
static volatile int32_t encoder2_count = 0;

#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3
#define UART_PORT_NUM UART_NUM_0

#define TICKS_PER_REV 700.0    // impulsów na obrót
#define WHEEL_RADIUS 0.10        // promień koła w metrach
#define WHEEL_BASE 0.42          // odległość między kołami w metrach

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
        
void app_main(void) {
    encoder_init();
    uart_init();

    int32_t prev_e1 = 0, prev_e2 = 0;
    const int sample_period_ms = 100;

    while (1) {
        int32_t curr_e1 = encoder1_count;
        int32_t curr_e2 = encoder2_count;

        int32_t delta_e1 = curr_e1 - prev_e1;
        int32_t delta_e2 = curr_e2 - prev_e2;

        prev_e1 = curr_e1;
        prev_e2 = curr_e2;

        // przelicz impulsy na metry
        float dl = (2.0f * 3.1415f * WHEEL_RADIUS * delta_e1) / TICKS_PER_REV;
        float dr = (2.0f * 3.1415f * WHEEL_RADIUS * delta_e2) / TICKS_PER_REV;

        // opcjonalnie: prędkość liniowa każdego koła
        float vl = dl / (sample_period_ms / 1000.0f);
        float vr = dr / (sample_period_ms / 1000.0f);

        char msg[64];
        snprintf(msg, sizeof(msg), "VL:%.3f VR:%.3f\n", vl, vr);
        uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));

        vTaskDelay(pdMS_TO_TICKS(sample_period_ms));
    }
}
