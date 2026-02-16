

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "nuos_nvs.h"
#include "common.h"
#include "frame_enc_dec.h"

// handle of the timer
static esp_timer_handle_t periodic_timer;

static const char *TAG                                          = "ESP_ZB_TO_UART";

#define UART_PORT_NUM                                           UART_NUM_1
#define UART_TX_PIN                                             5
#define UART_RX_PIN                                             4
#define UART_BAUD_RATE                                          115200
#define UART_BUF_SIZE                                           (2048)

#define UART_MAX_PAYLOAD                                        256

typedef struct {
    uint16_t len;
    uint8_t  data[UART_MAX_PAYLOAD];
} uart_rx_msg_t;


QueueHandle_t uart_rx_queue = NULL;

void uart_receiver_task(void *arg) {
    static uint8_t rx_buf[UART_BUF_SIZE] = {0}; 
    static uart_rx_msg_t msg;
    while(1) {
        memset(&msg, 0, sizeof(msg));   // clear all fields
        memset(rx_buf, 0, sizeof(rx_buf));
        // Read ONE byte at a time to feed into XBee parser
        int len = uart_read_bytes(UART_PORT_NUM, rx_buf, sizeof(rx_buf), 10 / portTICK_PERIOD_MS);
        if(len > 1) {
            // rx_buf[len] = '\0'; // Null-terminate safely 
            // Echo back received data
            //printf("\n\nReceived encrypted data from Gateway: \n");
            // process_decrypt_uart_receiver_data(rx_buf, len);
            
            
            msg.len = len;
            memcpy(msg.data, rx_buf, len);

            if (xQueueSend(uart_rx_queue, &msg, 0) != pdPASS) {
                ESP_LOGW("UART", "RX queue full, dropping packet");
            }
            
        }
       
    }
}

uart_rx_msg_t msg;
void uart_process_task(void *arg)
{
   
    while (1) {
        if (xQueueReceive(uart_rx_queue, &msg, portMAX_DELAY) == pdPASS) {
            printf("\n\n===> New encrypted data received from Gateway (%d bytes)\n", msg.len);
            process_decrypt_uart_receiver_data((const uint8_t*)msg.data, msg.len);
        }
    }
}

const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_XTAL, // or UART_SCLK_APB if preferred
};

extern "C" void uart_init(void)
{

    // 1) configure params
    esp_err_t err = uart_param_config(UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", err);
        return;
    }

    // 2) set pins
    err = uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
        return;
    }

    // 3) install driver
    err = uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
        return;
    }
    // optional: small delay while bringing hardware up (for debugging only)
    vTaskDelay(pdMS_TO_TICKS(50));
    xTaskCreate(uart_receiver_task, "uart_receiver_task", 4096, NULL, TASK_PRIORITY_UART_RXD, NULL);

    vTaskDelay(pdMS_TO_TICKS(50));
    uart_rx_queue = xQueueCreate(
        10,                    // queue depth
        sizeof(uart_rx_msg_t)  // item size
    );

    if (uart_rx_queue == NULL) {
        ESP_LOGE("UART", "Failed to create UART RX queue");
        return;
    }
    xTaskCreate(uart_process_task,  "uart_proc", 4096, NULL, TASK_PRIORITY_UART_PROCESS,  NULL);
    init_uart_queue();
}    

extern "C" int uart_putc(uint8_t byte) {
    return uart_write_bytes(UART_PORT_NUM, (const char*)&byte, 1);
}

extern "C" int uart_put_all(const uint8_t *byte, const uint8_t len) {
    uart_write_bytes(UART_PORT_NUM, (const char*)byte, len);  
    if (uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100)) != ESP_OK) {
        ESP_LOGW(TAG, "UART TX busy / timeout");
        return -1;
    } 
    return len; 
}
extern "C" int uart_send_bytes_to_gateway(uint8_t *byte, uint8_t len) {

    return uart_write_bytes(UART_PORT_NUM, (const char*)byte, len);   
}

/**
 * @brief Timer callback function (runs every 10 seconds)
 */
static void timer_callback(void *arg)
{
    esp_zb_ieee_addr_t *ieee_addrarg = (esp_zb_ieee_addr_t *)arg;
    if (ieee_addrarg) {
        ESP_LOGI(TAG, "Timer callback invoked for IEEE Address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (*ieee_addrarg)[0], (*ieee_addrarg)[1], (*ieee_addrarg)[2], (*ieee_addrarg)[3],
                 (*ieee_addrarg)[4], (*ieee_addrarg)[5], (*ieee_addrarg)[6], (*ieee_addrarg)[7]);
        send_module_unconfigured_over_uart(*ieee_addrarg);
    }
    
    ESP_LOGI(TAG, "Timer fired!");
}

/**
 * @brief Start a periodic timer with 10 seconds interval
 */

extern "C" void start_timer_10s(esp_zb_ieee_addr_t ieee_addr)
{
    
    if (periodic_timer != NULL) {
        ESP_LOGW(TAG, "Timer already running!");
        return;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = &ieee_addr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "my_10s_timer"
    };
    // Create timer
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    // Start periodic timer: 10 sec = 10,000,000 microseconds
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000 * 1000));
    ESP_LOGI(TAG, "10-second periodic timer started ...");
}

/**
 * @brief Stop and delete the periodic timer
 */
extern "C" void stop_timer_10s(void)
{
    if (periodic_timer == NULL) {
        ESP_LOGW(TAG, "Timer not running!");
        return;
    }
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_ERROR_CHECK(esp_timer_delete(periodic_timer));
    periodic_timer = NULL;
    ESP_LOGI(TAG, "10-second periodic timer stopped");
}