/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee switch driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_driver.h"
#include "nuos_uart_protocol.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
/**
 * @brief:
 * This example code shows how to configure light switch with attribute as well as button switch handler.
 *
 * @note:
   Currently only support toggle switch functionality is available
 *
 * @note:
 * For other possible switch functions (on/off,level up/down,step up/down). User need to implement and create them by themselves
 */

static QueueHandle_t gpio_evt_queue = NULL;
/* button function pair, should be defined in switch example source file */
static switch_func_pair_t *switch_func_pair;
/* call back function pointer */
static esp_switch_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
static const char *TAG = "ESP_ZB_SWITCH";

static void switch_driver_gpios_intr_enabled(bool enabled);

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    //switch_driver_gpios_intr_enabled(false);
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t *)arg, NULL);
}

/**
 * @brief Enable GPIO (switches refer to) isr
 *
 * @param enabled      enable isr if true.
 */
static void switch_driver_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i) {
        if (enabled) {
            gpio_intr_enable((switch_func_pair + i)->pin);
        } else {
            gpio_intr_disable((switch_func_pair + i)->pin);
        }
    }
}

bool button_read_flag[4] 										= {false, false, false, false};
uint32_t switch_pressed_counts_to_enter_commissioning 			= 0;
bool two_switch_pressed_flag                            		= false;
extern const gpio_num_t gpio_touch_btn_pins[2];

#define  TOTAL_BUTTONS   										2
#define MAX_TIME_TO_START_COMMISSIONING_ON_2_BUTTONS_PRESSED  	5 //5 secs
extern void turn_on_commissioning_led_switch();

void initTwoSwitchPressedPins(){
	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
            button_read_flag[i] = false;
        }
	}
    switch_pressed_counts_to_enter_commissioning = 0;
}

uint32_t IdentifyTwoSwitchPressed(){
	for(int i=0; i<TOTAL_BUTTONS; i++){
		bool value = gpio_get_level(gpio_touch_btn_pins[i]);
		if(value == 0) {
			if(button_read_flag[i] == false){
				button_read_flag[i] = true;
				switch_pressed_counts_to_enter_commissioning++;
			}
		}
	}
	return switch_pressed_counts_to_enter_commissioning;
}

void actionOnTwoSwitchPressed(int16_t timeout){

	bool first_button_pressed = false;
	bool second_button_pressed = false;
	bool third_button_pressed = false;
	bool fourth_button_pressed = false;

	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
           if(i==0){
			  first_button_pressed = true;
		   }else if(i==1){
			  second_button_pressed = true;
		   }else if(i==2){
			  third_button_pressed = true;
		   }else if(i==3){
			  fourth_button_pressed = true;
		   }
        }
	}
	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
            button_read_flag[i] = false;
        }
	}
	if(timeout >= MAX_TIME_TO_START_COMMISSIONING_ON_2_BUTTONS_PRESSED){
		if(first_button_pressed && second_button_pressed){
			printf("set_commissioning\n");
			esp_zb_nvram_erase_at_start(true);
			turn_on_commissioning_led_switch();
		}

	}
}

/**
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */
static void switch_driver_button_detected(void *arg)
{
    gpio_num_t io_num = GPIO_NUM_NC;
    switch_func_pair_t button_func_pair;
    switch_state_t switch_state = SWITCH_IDLE;
    bool evt_flag = false;
    uint32_t press_duration = 0;
    uint32_t last_press_time = 0;
    uint16_t total_press_in_secs = 0;
    bool single_button_provisioning_triggered = false;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
            io_num = button_func_pair.pin;
            evt_flag = true;
            switch_state = SWITCH_IDLE;
            two_switch_pressed_flag = false;
            single_button_provisioning_triggered = false;
            initTwoSwitchPressedPins();
        }

        while (evt_flag) {
            bool value = gpio_get_level(io_num);
            uint32_t current_time = esp_timer_get_time() / 1000U;

            switch (switch_state) {
            case SWITCH_IDLE:
                if (value == GPIO_INPUT_LEVEL_ON) {
                    switch_state = SWITCH_PRESS_DETECTED;
                    last_press_time = current_time;
                    press_duration = 0;
                    total_press_in_secs = 0;
                    single_button_provisioning_triggered = false;
                }
                break;

            case SWITCH_PRESS_DETECTED:
                if (value == GPIO_INPUT_LEVEL_ON) {
                    press_duration = current_time - last_press_time;

                    /* Preserve existing two-button Zigbee factory reset /
                     * commissioning behavior. */
                    if (!two_switch_pressed_flag && IdentifyTwoSwitchPressed() >= 2) {
                        two_switch_pressed_flag = true;
                        printf("two switch pressed!!\n");
                    }

                    if (two_switch_pressed_flag &&
                        press_duration >= ZIGBEE_FORMATION_LONG_PRESS_MS &&
                        !single_button_provisioning_triggered) {
                        printf("BOTH BUTTONS LONG PRESS 5s -> START ZIGBEE FORMATION\n");
                        turn_on_commissioning_led_switch();
                        nuos_start_commissioning(120);
                        single_button_provisioning_triggered = true;
                    }

                    /* Single-button provisioning mode selection.
                     * Button 1 = Wi-Fi, Button 2 = BLE. */
                    if (!two_switch_pressed_flag &&
                        !single_button_provisioning_triggered &&
                        press_duration >= SINGLE_BUTTON_PROVISIONING_LONG_PRESS_MS) {

                        if (io_num == gpio_touch_btn_pins[0]) {
                            printf("BUTTON 1 LONG PRESS 5s -> WIFI PROVISIONING\n");
                            uart_send_gateway_provisioning_event("wifi");
                            single_button_provisioning_triggered = true;
                        } else if (io_num == gpio_touch_btn_pins[1]) {
                            printf("BUTTON 2 LONG PRESS 5s -> BLE COMMISSIONING (GPIO=%d)\n", (int)io_num);
                            uart_send_gateway_provisioning_event("ble");
                            single_button_provisioning_triggered = true;
                        }
                    }

                    vTaskDelay(pdMS_TO_TICKS(20));
                } else {
                    switch_state = SWITCH_RELEASE_DETECTED;
                }
                break;

            case SWITCH_RELEASE_DETECTED:
                printf("SWITCH_RELEASE_DETECTED\n");
                switch_state = SWITCH_IDLE;

                /* A provisioning long press consumes the release. */
                if (!single_button_provisioning_triggered) {
                    (*func_ptr)(&button_func_pair);
                }
                break;

            default:
                switch_state = SWITCH_IDLE;
                break;
            }

            if (switch_state == SWITCH_IDLE) {
                actionOnTwoSwitchPressed(total_press_in_secs);
                switch_driver_gpios_intr_enabled(true);
                evt_flag = false;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static bool switch_driver_gpio_init(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    gpio_config_t io_conf = {};
    switch_func_pair = button_func_pair;
    switch_num = button_num;
    uint64_t pin_bit_mask = 0;

    /* set up button func pair pin mask */
    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << (button_func_pair + i)->pin);
    }
    /* interrupt of falling edge */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);

    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(switch_func_pair_t));
    if ( gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", 4096, NULL, 10, NULL);
    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *) (button_func_pair + i));
    }

    return true;
}

bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    if (!switch_driver_gpio_init(button_func_pair, button_num)) {
        return false;
    }
    func_ptr = cb;
    return true;
}
