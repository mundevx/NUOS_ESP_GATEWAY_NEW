

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_zigbee_type.h"
#include "nuos_nvs.h"
#include "cJSON.h"
#include "esp_zigbee_core.h"
#include "driver/uart.h"
#include "common.h"

#include "esp_timer.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mbedtls/base64.h"
#include "mbedtls/aes.h"


#include "XBee.h"
#include "frame_enc_dec.h"



// handle of the timer
static esp_timer_handle_t periodic_timer;

static const char *TAG                                          = "ESP_ZB_TO_UART";



#define UART_PORT_NUM                                           UART_NUM_1
#define UART_TX_PIN                                             5
#define UART_RX_PIN                                             4
#define UART_BAUD_RATE                                          115200
#define UART_BUF_SIZE                                           (1024)


void stop_timer_10s(void);
void start_timer_10s(void);


//void send_module_unconfigured_over_uart();


void uart_receiver_task(void *arg) {
    static uint8_t data[UART_BUF_SIZE] = {0}; 
    while(1) {
         vTaskDelay(10 / portTICK_PERIOD_MS);
        // Read ONE byte at a time to feed into XBee parser
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if(len > 1) {
            data[len] = '\0'; // Null-terminate safely 
            // Echo back received data
            printf("\n\nReceived encrypted data from Gateway: \n");
            for(int i = 0; i < len; i++) {
                // Process each byte
                printf(" %02X", data[i]);
            }
            printf("\n");

            process_decrypt_uart_receiver_data( data, len);

            // // Store byte in XBee's buffer
            // memcpy(xbee.serial_read_data, (const uint8_t*)data, len);
            // xbee.serial_read_data[len] = '\0'; // Null terminate for single byte
            // xbee.serial_read_data_counter = 0;
            // xbee.serial_data_available = true;
        
            // // Process the byte through XBee parser
            // xbee.readPacket();
            // // Check if we have a complete response
            // if(xbee.getResponse().isAvailable()) {
            //     printf("XBee Response Available. API ID: 0x%02X\n", xbee.getResponse().getApiId());
            //     if(xbee.getResponse().getApiId() == ZB_TX_REQUEST) {
            //         ZBRxResponse rx;
            //         xbee.getResponse().getZBRxResponse(rx);

            //         printf("Rx_len: %d\n", rx.getDataLength()-1);
            //         // Process received data
            //         char received_data[rx.getDataLength()-1 + 1];
            //         for(int i = 0; i < rx.getDataLength()-1; i++) {
            //             received_data[i] = rx.getData(1+i);
            //             printf(" %02x", received_data[i]);
            //         }
            //         printf("\n");
            //         received_data[rx.getDataLength()-1] = '\0';

            //         //char encrypted_buffer[256];
            //         char* decrypted = Fn_DECRYPT_DATA(received_data);
            //         if (decrypted) {
            //             printf( "decrypted:%s\n", decrypted);
            //             esp_zb_ieee_addr_t zigbee_addr;// = {0x00, 0x13 , 0xa2 , 0x00 , 0x42, 0x1b, 0x37, 0x9c};
            //             convert_xbee_to_esp_zigbee_addr_memcpy(rx.getRemoteAddress64(), zigbee_addr);

            //             //send_xbee_data_over_uart(decrypted, strlen(decrypted), zigbee_addr, data[4]);
            //             process_uart_data((const char*)decrypted, strlen(decrypted), zigbee_addr, data, len);
            //             free(decrypted);
            //         }
            //     }
            // } else {
            //     printf("No complete XBee response available yet.\n");
            // }
            
        }
       
    }
}


static portMUX_TYPE uart_mutex = portMUX_INITIALIZER_UNLOCKED;

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
    xTaskCreate(uart_receiver_task, "uart_receiver_task", 4096, NULL, 20, NULL);
    //send_serial("NUOS Zigbee Gateway!!");
    vTaskDelay(pdMS_TO_TICKS(50));
}    

extern "C" int uart_putc(uint8_t byte) {
    return uart_write_bytes(UART_PORT_NUM, (const char*)&byte, 1);
}

extern "C" int uart_put_all(uint8_t *byte, uint8_t len) {
    printf("\nSending encrypted data to Gateway: \n");
    for(int i = 0; i < len; i++) {
        printf(" %02x", byte[i]);
    }
    printf("\n");
    
    uart_write_bytes(UART_PORT_NUM, (const char*)byte, len);
    return uart_write_bytes(UART_PORT_NUM, "\n", 1); 
}
extern "C" int uart_send_bytes_to_gateway(uint8_t *byte, uint8_t len) {

    return uart_write_bytes(UART_PORT_NUM, (const char*)byte, len);   
}

/**
 * @brief Timer callback function (runs every 10 seconds)
 */
static void timer_callback(void *arg)
{
    //send_module_unconfigured_over_uart();
    ESP_LOGI(TAG, "Timer fired!");
}

/**
 * @brief Start a periodic timer with 10 seconds interval
 */

void start_timer_10s(void)
{
    if (periodic_timer != NULL) {
        ESP_LOGW(TAG, "Timer already running!");
        return;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
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
void stop_timer_10s(void)
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