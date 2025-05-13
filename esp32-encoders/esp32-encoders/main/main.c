#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define ENCODER1_PIN GPIO_NUM_4   // Kanał A enkodera 1
#define ENCODER2_PIN GPIO_NUM_5   // Kanał A enkodera 2

static volatile int32_t encoder1_count = 0;
static volatile int32_t encoder2_count = 0;

#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3
#define UART_PORT_NUM UART_NUM_0

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

    while (1) {
        int32_t e1 = encoder1_count;
        int32_t e2 = encoder2_count;

        // Wyślij dane jako tekst, np. "E1:123 E2:456\n"
        char msg[64];
        snprintf(msg, sizeof(msg), "E1:%ld E2:%ld\n", e1, e2);
        uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}
