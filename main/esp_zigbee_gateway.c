/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Gateway Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

 #define _DECLARE_MAIN_

#include <fcntl.h>
#include "string.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_eventfd.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_zigbee_type.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_rcp_update.h"
#include "esp_coexist.h"
#include "esp_zigbee_gateway.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"

#include "common.h"

#include <stdio.h>

#include "freertos/timers.h"
#include <cJSON.h>

#if CONFIG_OPENTHREAD_SPINEL_ONLY
#include "esp_radio_spinel.h"
#endif
#include "switch_driver.h"
#include "nuos_nvs.h"
#include "http_server.h"
#include "zigbee_2_uart.h"
#include "frame_enc_dec.h"
#include "freertos/semphr.h"
#include <ctype.h>

// Declare the semaphore handle globally
SemaphoreHandle_t task_done_semaphore;

typedef struct {
    node_module_t module;
    uint8_t index;
} node_module_evt_t;
static QueueHandle_t send_to_uart_evt_queue = NULL;

static const char *TAG = "ESP_ZB_GATEWAY";

void nuos_zb_read_attributes_from_clusters(uint16_t cluster_id, uint8_t dst_ep, uint16_t short_addr);

/* Production configuration app data */
typedef struct app_production_config_s {
    uint16_t version;
    uint16_t manuf_code;
    char manuf_name[16];
} app_production_config_t;


#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))
typedef struct zbstring_s {
    uint8_t len;
    char data[];
} ESP_ZB_PACKED_STRUCT
zbstring_t;


device_status_handle_t device_info[2];

node_module_t device_array[MAX_DEVICES];
uint16_t devices_count = 0;


const gpio_num_t gpio_touch_led_pins[2]                    	= { GPIO_NUM_27, GPIO_NUM_11};
const gpio_num_t gpio_load_pins[2]                        	= { GPIO_NUM_2, GPIO_NUM_5};
const gpio_num_t gpio_touch_btn_pins[2]                 	= { GPIO_NUM_0, GPIO_NUM_1};


static switch_func_pair_t button_func_pair[] = {
    {GPIO_NUM_0, SWITCH_ONOFF_TOGGLE_CONTROL},
	{GPIO_NUM_1, SWITCH_ONOFF_TOGGLE_CONTROL}
};

void set_load_state(uint8_t index, bool state){
	gpio_set_level(gpio_touch_led_pins[index], state);
	gpio_set_level(gpio_load_pins[index], state);
}

typedef struct config_struct_s{
	uint16_t motion_sensor_inactivity_detection_time;
	uint16_t door_remain_open_detection_time;
	uint16_t zigbee_nw_open_commissioning_time;
}config_t;

void start_motion_detection_timer();
void delete_motion_detection_timer();

void delete_door_open_detection_timer();
void start_door_open_detection_timer();

#define CHECK_TIME_FOR_PIR_DETECTION     			30 //20 sec
#define CHECK_TIME_FOR_DOOR_OPEN_DETECTION     		60 //20 sec

#define CHECK_TIME_FOR_ZB_COMMISSIONING  			120 //120 sec


esp_timer_handle_t my_timer_2						= NULL;
bool timer_door_open_initialized_ok 				= false;
bool timer_started_flag_2 							= false;
volatile bool start_getting_door_open_detetced_flag = false;
volatile uint16_t door_open_detetced_counts 		= 0;

esp_timer_handle_t my_timer							= NULL;
bool timer_initialized_ok 							= false;
bool timer_started_flag 							= false;
volatile bool start_getting_motion_detetced_flag 	= false;
volatile uint16_t motion_detetced_counts 			= 0;

bool time_zb_commissioning_started_flag 			= false;
uint16_t zb_commissioning_counts 					= 0;

TaskHandle_t commissioningTaskHandle;

static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx);

void write_custom_cluster_attr(uint16_t cluster_id, uint8_t custom_cmd_id, uint16_t dst_short_addr, uint8_t dst_ep, uint8_t custom_value);

#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof(arr[0]))

static QueueHandle_t nvs_save_queue = NULL;

typedef struct {
    uint8_t node_i;
} nvs_save_evt_t;


extern void nuos_zb_scenes_store_scene_request(uint16_t group_id, uint8_t scene_id, 
                                        uint8_t src_ep, uint8_t dst_ep, 
                                        uint16_t dst_addr_short);

extern void nuos_zb_scene_recall_scene_request(uint16_t group_id, uint8_t scene_id, 
										uint8_t src_ep, 
                                        uint8_t dst_ep, uint16_t dst_addr_short, 
										esp_zb_zcl_address_mode_t address_mode);



void init_variables() {

	timer_door_open_initialized_ok 					= false;
	timer_started_flag_2 							= false;
	start_getting_door_open_detetced_flag 			= false;
	door_open_detetced_counts 						= 0;

	timer_initialized_ok 							= false;
	timer_started_flag 								= false;
	start_getting_motion_detetced_flag 				= false;
	motion_detetced_counts 							= 0;

	time_zb_commissioning_started_flag 				= false;
	zb_commissioning_counts 						= 0;
}

void nvs_save_device_task(void *arg)
{
    nvs_save_evt_t evt;

    while (1) {
        if (xQueueReceive(nvs_save_queue, &evt, portMAX_DELAY)) {
            //printf("Saving device %d to NVS\n", evt.node_i);
            save_or_update_light_bulb_device_to_nvs(
                &device_array[evt.node_i]
            );
        }
    }
}

void nuos_zb_init_hardware(){

    for(int index=0; index<MAX_SWITCHES_TO_CONTROL; index++){
        gpio_reset_pin(gpio_touch_led_pins[index]);
        // /* Set the GPIO as a push/pull output */
        gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[index], 0);

        gpio_reset_pin(gpio_load_pins[index]);
        /* Set the GPIO as a push/pull output */
        gpio_set_direction(gpio_load_pins[index], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_load_pins[index], 0);

    }
    init_variables();
	uart_init();
	nvs_save_queue = xQueueCreate(10, sizeof(nvs_save_evt_t));
    assert(nvs_save_queue);

	xTaskCreate(
        nvs_save_device_task,
        "nvs_save_dtask",
        4096,
        NULL,
        TASK_PRIORITY_NVS_SAVE,
        NULL
    );
	//send_serial("Welcome Gateway!!\n");
}


void set_all_led_state(bool state){
	for(int index=0; index<MAX_SWITCHES_TO_CONTROL; index++){
		gpio_set_level(gpio_touch_led_pins[index], state);
	}
}

void set_light_switch_state(uint8_t index, bool state){
	//turn-off switch
	set_load_state(index, device_info[index].state);
	delete_motion_detection_timer();
}

void turn_off_light_switch(){
	//turn-off switch
	device_info[0].state = false;
	set_load_state(0, 0);
	delete_motion_detection_timer();
}

void turn_on_light_switch(){
	//turn-on switch
	device_info[0].state = true;
	set_load_state(0, 1);
	delete_motion_detection_timer();
}

void turn_off_commissioning_led_switch(){
    if(time_zb_commissioning_started_flag){
    	time_zb_commissioning_started_flag = false;
    	if(commissioningTaskHandle != NULL){
			vTaskDelete(NULL);
			commissioningTaskHandle = NULL;
		}
    	esp_zb_bdb_open_network(ZIGBEE_NW_REMAIN_OPEN_IN_SECS);
		//esp_zb_bdb_close_network();
    }
}

void turn_on_commissioning_led_switch(){
	if(!time_zb_commissioning_started_flag){
		time_zb_commissioning_started_flag = true;
		zb_commissioning_counts = 0;
		esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
		xTaskCreate(set_commissioning_task, "commissioning_task", 2098, NULL, TASK_PRIORITY_ZB_COMMISSIONING, &commissioningTaskHandle);
	}
}




bool is_fan_control(uint8_t nodeindex, uint8_t epindex){
	if(device_array[nodeindex].endpoint[epindex].cluster_counts > 0){
		for(int cl=0; cl<device_array[nodeindex].endpoint[epindex].cluster_counts; cl++){
			if(device_array[nodeindex].endpoint[epindex].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
				return false;
			}else if(device_array[nodeindex].endpoint[epindex].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
				return true;
			}
		}
	}
	return false;
}

bool is_thermostat_control(uint8_t nodeindex, uint8_t epindex){
	if(device_array[nodeindex].endpoint[epindex].cluster_counts > 0){
		for(int cl=0; cl<device_array[nodeindex].endpoint[epindex].cluster_counts; cl++){
			if(device_array[nodeindex].endpoint[epindex].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
				return true;
			}else if(device_array[nodeindex].endpoint[epindex].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
				return false;
			}
		}
	}
	return false;
}

// Task function to set the fan speed
void set_commissioning_task(void* arg) {
	bool state = false;
	while(1) {
		vTaskDelay(pdMS_TO_TICKS(500));
		if(time_zb_commissioning_started_flag){
			if(zb_commissioning_counts++ > 2*CHECK_TIME_FOR_ZB_COMMISSIONING){
				printf("Finish set_commissioning_task\n");
				for(int i=0; i<MAX_SWITCHES_TO_CONTROL; i++){
					set_light_switch_state(i, device_info[i].state);
				}
				break;
			}else{
				state = !state; //toggle all leds state
				set_all_led_state(state);
			}
		}
	}
	turn_off_commissioning_led_switch();
 }

static void esp_app_zb_attribute_handler(uint16_t short_addr, uint8_t end_point, uint16_t cluster_id, const esp_zb_zcl_attribute_t* attribute){
	
	switch(cluster_id){
		case ESP_ZB_ZCL_CLUSTER_ID_BASIC:
	        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID &&
	            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
	            attribute->data.value) {
	            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
	            char *string = (char*)malloc(zbstr->len + 1);
	            memcpy(string, zbstr->data, zbstr->len);
	            string[zbstr->len] = '\0';
	            ESP_LOGI(TAG, "Peer Manufacturer is \"%s\"", string);
	            free(string);
	        }
	        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID &&
	            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
	            attribute->data.value) {
	            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
	            char *string = (char*)malloc(zbstr->len + 1);
	            memcpy(string, zbstr->data, zbstr->len);
	            string[zbstr->len] = '\0';
	            ESP_LOGI(TAG, "Peer Model is \"%s\"", string);
	            free(string);
	        }
			break;

		case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
		    for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
								//
								if(end_point == device_array[k].endpoint[j].ep){
									//printf("CLUSTER_ID_1:0x%X\n", device_array[k].endpoint[j].cluster_id[cl]);
									if ((attribute->id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID || attribute->id == 0x4003) && attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {

										if(attribute->id == 0x0000){
											device_array[k].endpoint[j].device_t.state = attribute->data.value ? *(bool *)attribute->data.value : device_array[k].endpoint[j].device_t.state;
										}else if(attribute->id == 0x4003){
											nuos_update_onoff_cluster(k, j, device_array[k].endpoint[j].device_t.level);
										}	

										device_array[k].endpoint[j].dev_id = device_array[k].endpoint[j].ep;
										uint8_t val = 0;
									
										if(device_array[k].endpoint[j].dev_type == 0 || device_array[k].endpoint[j].dev_type == 256){
											if(!device_array[k].endpoint[j].device_t.state){
												device_array[k].endpoint[j].device_t.level = 0;
											}else {
												device_array[k].endpoint[j].device_t.level = 255;
											}
										}
										if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
											send_module_unconfigured_over_uart(device_array[k].ieee_addr);
										}else{
											if(get_max_endpoint_in_active_seq() == 1){
												set_res_active_seq_response_received(false);
												send_module_ctrl_over_uart(&device_array[k], 1, j);
											}else{
												if(is_res_active_seq_response_completed()){
													set_res_active_seq_response_received(false);
													uint8_t k = find_index();
													send_module_ctrl_over_uart(&device_array[k], 0, 0);	
													printf("-----------Hurray Complete!!----------\n");
												}	
											}

											if(device_array[k].endpoint[j].scene_add_command_flag){
												device_array[k].endpoint[j].scene_add_command_flag = false;
                                                printf("group_id:%d  scene_id:%d\n", scene_nodes[scene_idx].group_id, scene_nodes[scene_idx].scene_id);
												nuos_zb_scenes_store_scene_request(
													scene_nodes[scene_idx].group_id, 
													scene_nodes[scene_idx].scene_id, 1, 
													device_array[k].endpoint[j].ep, 
													device_array[k].short_addr);
											}
										}

										if(lock_nvs_save){
											lock_nvs_save = false;
										}else{
											save_or_update_light_bulb_device_to_nvs(&device_array[k]);
										}										
										break;
									}
								}
							}else{
								printf("CLUSTER_ID_2:0x%X\n", device_array[k].endpoint[j].cluster_id[cl]);
							}
						}
		    		}
		    	}
		    }
			break;
		case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
		    for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
								if(end_point == device_array[k].endpoint[j].ep){
									if (attribute->id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
										attribute->data.value) {
										if(device_array[k].endpoint[j].device_t.state){

											uint8_t val = device_array[k].endpoint[j].device_t.level = attribute->data.value ? *(uint8_t *)attribute->data.value : device_array[k].endpoint[j].device_t.level;
											if(val == 0){
												device_array[k].endpoint[j].device_t.state = false;
											} else { 
												device_array[k].endpoint[j].device_t.state = true;
											}
                                            device_array[k].endpoint[j].dev_id = device_array[k].endpoint[j].ep;

											if(device_array[k].endpoint[j].dev_type == 769){
												//for dimmable light
												if(val > 0) device_array[k].endpoint[j].device_t.level = val;
												if(val == 191) device_array[k].endpoint[j].device_t.level = 142; 
												else if(val == 128) device_array[k].endpoint[j].device_t.level = 102; 
												else if(val == 64) device_array[k].endpoint[j].device_t.level = 80;
												else if(val >= 254) device_array[k].endpoint[j].device_t.level = 255;													
											}

											send_module_ctrl_over_uart(&device_array[k], 1, j);
											// printf("X_LEVEL:%d\n", device_array[k].endpoint[j].device_t.level);
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}
											break;
										}
									}
								}
							}
						}
		    		}
		    	}
		    }
			break;
		case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
								if(end_point == device_array[k].endpoint[j].ep){
									if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
										attribute->data.value) {
											device_array[k].endpoint[j].device_t.value = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											printf("X_COLOR:%ld\n", device_array[k].endpoint[j].device_t.value);
											device_array[k].endpoint[j].dev_id = device_array[k].endpoint[j].ep;
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}
											//send_uart_command(&device_array[k], j, "ctrl", device_array[k].endpoint[j].device_t.level);
											break;
									}
								}
							}
						}

		    		}
		    	}
		    }
			break;
		case ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
								if(end_point == device_array[k].endpoint[j].ep){
									if (attribute->id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM &&
										attribute->data.value) {
											device_array[k].endpoint[j].device_t.level = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											printf("X_FAN_SPEED:%d\n", device_array[k].endpoint[j].device_t.level);
											device_array[k].endpoint[j].dev_id = device_array[k].endpoint[j].ep;
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}
											//send_uart_command(&device_array[k], j, "ctrl", device_array[k].endpoint[j].device_t.level);
											break;
									}
								}
							}
						}
		    		}
		    	}
		    }
			break;

		case ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID:
		    for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
								if(end_point == device_array[k].endpoint[j].ep){
							        printf("ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID\n");
								}
							}else{
								printf("CLUSTER_ID:0x%X\n", device_array[k].endpoint[j].cluster_id[cl]);
							}
						}
		    		}
		    	}
		    }		
			break;	
        case ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID:
       		printf("ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT  custom cluster attr\n");
			//0xFC31
		break;
        case ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE:
       		printf("ESP_ZB_HA_IAS_ZONE_ID  motion cluster attr\n");
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE){
								if(end_point == device_array[k].endpoint[j].ep){
									printf("attribute->id:%0x04X attribute->data.type:%d \n", attribute->id, attribute->data.type);


									device_array[k].endpoint[j].device_t.level = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
                                    device_array[k].endpoint[j].device_t.state = true;
									if(device_array[k].endpoint[j].device_t.level==0){
										device_array[k].endpoint[j].device_t.state = false;
									}
										if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
											send_module_motion_sensor_unconfigured_over_uart(device_array[k].ieee_addr);
										}else{
											send_module_ctrl_over_uart(&device_array[k], 1, 0);
										}
								}
							}
						}

		    		}
		    	}
		    }
		break;        
		case ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT:
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int	 cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
								printf("ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT  ep:%d  %d\n", end_point, device_array[k].endpoint[j].ep);
								if(end_point == device_array[k].endpoint[j].ep){
									printf("end_point:%d\n", end_point);
									if (attribute->id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM &&
										attribute->data.value) {
											printf("ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID\n");
											uint8_t mode = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											if(mode == 0){ 
												device_array[k].endpoint[j].device_t.state = false;
												device_array[k].endpoint[j].device_t.level = 0;
											} else { 
												device_array[k].endpoint[j].device_t.state = true;
												device_array[k].endpoint[j].device_t.level = 255;
											}

											device_array[k].endpoint[j].dev_id = device_array[k].endpoint[j].ep;
											// if(device_array[k].endpoint[j].device_t.level <18 || device_array[k].endpoint[j].device_t.level >30){
											// 	 //default temp
											// 	device_array[k].endpoint[j].device_t.level = 24;
											// }
						                    //device_array[k].endpoint[0].device_t.level = device_array[k].endpoint[j].device_t.level; 
											printf("X_AC_STATE:%d  AC_TEMP:%d\n", device_array[k].endpoint[j].device_t.state, device_array[k].endpoint[j].device_t.level);
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}
											uint8_t val = device_array[k].endpoint[j].device_t.level;

											if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
												send_module_ir_unconfigured_over_uart(device_array[k].ieee_addr);
											}else{
												send_module_ctrl_over_uart(&device_array[k], 0, j);
											}
											break;
									}else if (attribute->id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16 &&
										attribute->data.value) {
											device_array[k].endpoint[j].device_t.level = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.level;
											printf("X_AC_TEMP:%d\n", device_array[k].endpoint[j].device_t.level);
											//device_array[k].endpoint[1].device_t.level = device_array[k].endpoint[j].device_t.level;
											if(device_array[k].endpoint[j].device_t.level <18 || device_array[k].endpoint[j].device_t.level >30){
												 //default temp
												device_array[k].endpoint[j].device_t.level = 24;
											}
											
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}
											if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
												send_module_ir_unconfigured_over_uart(device_array[k].ieee_addr);
											}else{
												send_module_ctrl_over_uart(&device_array[k], 1, j);
											}
											break;
									}
								}
							}
						}

		    		}
		    	}
		    }
			break;

		case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF_SWITCH_CONFIG:

			break;

		case ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING: //ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						printf("cluster_counts:%d\n", device_array[k].endpoint[j].cluster_counts);
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING){
								if(end_point == device_array[k].endpoint[j].ep){
									printf("attr:0x%x, type:%d\n", attribute->id, attribute->data.type);
									if (attribute->id == ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
										attribute->data.value) {

											uint8_t value = attribute->data.value ? *(uint8_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											printf("WINDOW COMMAND:%d \n", value);

											if(value == 0){
												device_array[k].endpoint[0].device_t.state = true; //open 
												device_array[k].endpoint[0].device_t.value = 255;
												device_array[k].endpoint[1].device_t.state = true;
												device_array[k].endpoint[1].device_t.value = 255;
											}else if(value == 1){
												device_array[k].endpoint[0].device_t.state = false; //closed
												device_array[k].endpoint[0].device_t.value = 0;
												device_array[k].endpoint[1].device_t.state = true;
												device_array[k].endpoint[1].device_t.value = 255;
											}else{
												device_array[k].endpoint[1].device_t.state = false; //stop
												device_array[k].endpoint[1].device_t.value = 0;
											}
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}

											if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
												send_module_unconfigured_over_uart(device_array[k].ieee_addr);
											}else{
												send_module_ctrl_over_uart(&device_array[k], 1, 0);
												vTaskDelay(pdMS_TO_TICKS(100));
												send_module_ctrl_over_uart(&device_array[k], 2, 1);
											}
											break;
									}
								}
							}
						}

		    		}
		    	}
		    }		
			break;

		case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT){
								if(end_point == device_array[k].endpoint[j].ep){
									if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16 &&
										attribute->data.value) {
											device_array[k].endpoint[j].device_t.value = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											device_array[k].endpoint[j].device_t.value = device_array[k].endpoint[j].device_t.value/100;
											printf("X_TEMP:%ld\n", device_array[k].endpoint[j].device_t.value);
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}

											if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
												send_module_temp_humidity_unconfigured_over_uart(device_array[k].ieee_addr);
												
											}else{
												send_module_ctrl_over_uart(&device_array[k], 1, 0);
											}
											break;
									}
								}
							}
						}

		    		}
		    	}
		    }
			break;

		case ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT:
		    printf("ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT\n");
			for(int k=0; k<devices_count; k++){
				printf("ep_counts:%d \n", device_array[k].ep_counts);
		    	for(int j=0; j<device_array[k].ep_counts; j++){
					printf("short_addr:0x%02X  0x%02X\n", short_addr, device_array[k].short_addr);
		    		if(short_addr == device_array[k].short_addr){
						
						printf("cluster_counts:%d \n", device_array[k].endpoint[j].cluster_counts);

						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							printf("cluster_id:0x%02X \n", device_array[k].endpoint[j].cluster_id[cl]);
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT){
								printf("end_point:%d, ep:%d \n", end_point, device_array[k].endpoint[j].ep);
								if(end_point == device_array[k].endpoint[j].ep){
 
									printf("attr_id:%d  attr_type:%d \n", attribute->id, attribute->data.type);
									
									if (attribute->id == ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
										attribute->data.value) {
											device_array[k].endpoint[j].device_t.value = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											device_array[k].endpoint[j].device_t.value = device_array[k].endpoint[j].device_t.value/100;
											printf("LUX_VAL:%ld\n", device_array[k].endpoint[j].device_t.value);
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}

											if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
												send_module_lux_sensor_unconfigured_over_uart(device_array[k].ieee_addr);
												
											}else{
												send_module_ctrl_over_uart(&device_array[k], 1, 0);
											}
											break;
									}
								}
							}
						}

		    		}
		    	}
		    }
			break;

		case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
			for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT){
								if(end_point == device_array[k].endpoint[j].ep){
									if (attribute->id == ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID &&
										attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
										attribute->data.value) {
											device_array[k].endpoint[j].device_t.value = attribute->data.value ? *(uint16_t *)attribute->data.value : device_array[k].endpoint[j].device_t.value;
											device_array[k].endpoint[j].device_t.value = device_array[k].endpoint[j].device_t.value / 100;
											printf("X_HUMID:%ld\n", device_array[k].endpoint[j].device_t.value);
											if(lock_nvs_save){
												lock_nvs_save = false;
											}else{
												save_or_update_light_bulb_device_to_nvs(&device_array[k]);
											}

											if(device_array[k].room_id == 0 || device_array[k].room_id == 255 || device_array[k].module_id == 0 || device_array[k].module_id == 255){
												send_module_temp_humidity_unconfigured_over_uart(device_array[k].ieee_addr);
												
											}else{
												send_module_ctrl_over_uart(&device_array[k], 2, 0);
											}
											break;
									}
								}
							}
						}

		    		}
		    	}
		    }
			break;			
		default: break;
	}
}






static void esp_app_zb_attribute_report_handler_2(uint16_t short_addr, uint8_t end_point, uint16_t cluster_id, const esp_zb_zcl_attribute_t* attribute){
	
	switch(cluster_id){
		case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
		    for(int k=0; k<devices_count; k++){
		    	for(int j=0; j<device_array[k].ep_counts; j++){
		    		if(short_addr == device_array[k].short_addr){
						for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
							if(device_array[k].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
								if(end_point == device_array[k].endpoint[j].ep){
									if ((attribute->id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID || attribute->id == 0x4003) && attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
										
										device_array[k].endpoint[j].device_t.state = attribute->data.value ? *(bool *)attribute->data.value : device_array[k].endpoint[j].device_t.state;	
										device_array[k].endpoint[j].dev_id = device_array[k].endpoint[j].ep;
									
										if(device_array[k].endpoint[j].dev_type == 0 || device_array[k].endpoint[j].dev_type == 256){
											if(!device_array[k].endpoint[j].device_t.state){
												device_array[k].endpoint[j].device_t.level = 0;
											}else {
												device_array[k].endpoint[j].device_t.level = 255;
											}
										}
																				
										break;
									}
								}
							}
						}
		    		}
		    	}
		    }
			break;			
		default: break;
	}
}



static uint8_t w_cmd_id[1] = {0};


void nuos_send_window_covering_lift_percent(uint8_t node_i, uint8_t ep_i, uint8_t percent)
{
    esp_zb_zcl_window_covering_cluster_send_cmd_req_t cmd_req = {0};

    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short =
        device_array[node_i].short_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint =
        device_array[node_i].endpoint[ep_i].ep;
    cmd_req.zcl_basic_cmd.src_endpoint =
        ESP_ZB_GATEWAY_ENDPOINT;

    cmd_req.cmd_id =
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID;

    cmd_req.value = &percent;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_window_covering_cluster_send_cmd_req(&cmd_req);
    esp_zb_lock_release();
}

void nuos_send_window_covering_command(uint8_t node_i, uint8_t ep_i, uint8_t command_id){

    w_cmd_id[0] = command_id?100:0;
	
	esp_zb_zcl_window_covering_cluster_send_cmd_req_t cmd_req;
	cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
	cmd_req.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
	cmd_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
	cmd_req.value = w_cmd_id;
	cmd_req.cmd_id = command_id;
    esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_window_covering_cluster_send_cmd_req(&cmd_req);
	esp_zb_lock_release();

	printf("Updated window covering cluster  command_id:%d\n", command_id);
    lock_nvs_save = true;
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };
    xQueueSend(nvs_save_queue, &evt, 0);

	nuos_send_window_covering_lift_percent(node_i, 0, w_cmd_id[0]);
}

void nuos_update_curtain_cluster(uint8_t node_i, uint8_t ep_i, uint8_t curtain_state){
	esp_zb_zcl_on_off_cmd_t cmd_req;
	device_array[node_i].endpoint[ep_i].device_t.level = curtain_state;
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
	cmd_req.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
	cmd_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
	cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	cmd_req.on_off_cmd_id = curtain_state;
	if(!curtain_state){
		device_array[node_i].endpoint[ep_i].device_t.state = false;
	}else{
		device_array[node_i].endpoint[ep_i].device_t.state = true;
	}
	esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_on_off_cmd_req(&cmd_req);
	esp_zb_lock_release();
	//printf("Updated on-off cluster\n");
    lock_nvs_save = true; 
	//save_or_update_light_bulb_device_to_nvs(&device_array[node_i]);
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };

    xQueueSend(nvs_save_queue, &evt, 0);
}

void send_report(int index, uint16_t cluster_id, uint16_t attr_id, uint8_t ep_i){
 
	esp_zb_zcl_report_attr_cmd_t report_cmd_2 = {
		.zcl_basic_cmd = {
			.dst_addr_u.addr_short = device_array[index].short_addr,
			.dst_endpoint = device_array[index].endpoint[ep_i].ep,
			.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT,
		},
		.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
		.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		.clusterID = cluster_id,
		.attributeID = attr_id,
	};
	esp_zb_lock_acquire(portMAX_DELAY);
	esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
	esp_zb_lock_release();
	if(err != ESP_OK){
		ESP_LOGE(TAG, "Failed to send report command: %d", err);
		esp_zb_lock_acquire(portMAX_DELAY);
		err = esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
		esp_zb_lock_release();
		if(err != ESP_OK){
			ESP_LOGE(TAG, "Failed to send report command again: %d", err);
			esp_zb_lock_acquire(portMAX_DELAY);
			err = esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
			esp_zb_lock_release();
		}
	}
}

void nuos_update_scene_add_cmd(uint8_t ep_i, uint16_t short_addr, uint8_t scene_id, uint16_t group_id){
	// Create octet string: first byte is length, then payload
	uint8_t quad_data[4];
	quad_data[0] = ep_i; // length byte
	quad_data[1] = scene_id;	
	quad_data[2] = (group_id >> 8) & 0xFF;
	quad_data[3] = group_id & 0xFF;

	esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
		.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		.cluster_id = 0xEF00,
		.custom_cmd_id = 0xe1,
		.data = {
			.value = quad_data,
			.size = 4,
			.type = ESP_ZB_ZCL_ATTR_TYPE_U32,
		},
		.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
		.profile_id = 0x0104,
		.zcl_basic_cmd.dst_addr_u.addr_short = short_addr,
		.zcl_basic_cmd.dst_endpoint = ep_i,
		.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT,
		.manuf_specific = 0,
		.manuf_code = 0,
		.dis_defalut_resp = 1,
	};
	esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
}

void nuos_update_onoff_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level){

	esp_zb_zcl_on_off_cmd_t cmd_req;
	bool state = level ? true : false;
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
	cmd_req.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
	cmd_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
	cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	bool current_state = device_array[node_i].endpoint[ep_i].device_t.state;

	if(device_array[node_i].zb_app_id == ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID){
		if(current_state == 0) device_array[node_i].endpoint[ep_i].device_t.level = 0;
		else device_array[node_i].endpoint[ep_i].device_t.level = level;
	}
	printf("device_state:%d app_state:%d\n", current_state, state);
	if (current_state != state) {
		cmd_req.on_off_cmd_id = state ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
		device_array[node_i].endpoint[ep_i].device_t.state = state;
		
		device_array[node_i].endpoint[ep_i].device_t.last_state = current_state;
		device_array[node_i].endpoint[ep_i].device_t.state = state;

		esp_zb_lock_acquire(portMAX_DELAY);
		esp_zb_zcl_on_off_cmd_req(&cmd_req);
        esp_zb_lock_release();
		if(device_array[node_i].zb_app_id == ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID){
			if(!state)  nuos_update_level_cluster(node_i, ep_i, 0);
			else nuos_update_level_cluster(node_i, ep_i, level);
		}
		
		printf("------>esp command sent for 0x%4x of ep:%d\n", device_array[node_i].short_addr, 
			device_array[node_i].endpoint[ep_i].ep);
		esp_zb_cmd_sent_flag = false;
		lock_nvs_save = true; 
		/* 🔁 Send NVS save request to another task */
		nvs_save_evt_t evt = {
			.node_i = node_i
		};
		xQueueSend(nvs_save_queue, &evt, 0);
	}else{
		printf("------->Sending report only!!\n");
		send_report(node_i, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, ep_i);
		send_module_ctrl_over_uart(&device_array[node_i], 1, ep_i);		
	}
}

void nuos_update_level_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level){
	device_array[node_i].endpoint[ep_i].device_t.level = level;
	esp_zb_zcl_move_to_level_cmd_t cmd_level;
	cmd_level.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
	cmd_level.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr; //
	cmd_level.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
	cmd_level.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	printf("Level to set:%d\n", device_array[node_i].endpoint[ep_i].device_t.level);
	cmd_level.level = device_array[node_i].endpoint[ep_i].device_t.level;
	cmd_level.transition_time = 0;
	esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_level_move_to_level_with_onoff_cmd_req(&cmd_level);
	esp_zb_lock_release();
	esp_zb_cmd_sent_flag = true;
    lock_nvs_save = true; 
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };
    xQueueSend(nvs_save_queue, &evt, 0);
}



void nuos_update_color_temperature(uint8_t node_i, uint8_t ep_i, uint16_t temp){
    esp_zb_zcl_color_move_to_color_temperature_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
    cmd_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;  
    cmd_req.color_temperature = temp;
    cmd_req.transition_time = 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_color_move_to_color_temperature_cmd_req(&cmd_req);
    esp_zb_lock_release();
	esp_zb_cmd_sent_flag = true;
    lock_nvs_save = true; 
	device_array[node_i].endpoint[ep_i].device_t.value = temp;		
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };
    xQueueSend(nvs_save_queue, &evt, 0);
}


void nuos_update_color_hue(uint8_t node_i, uint8_t ep_i, uint16_t hue){

	esp_zb_color_move_to_hue_saturation_cmd_t cmd2;
    cmd2.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
    cmd2.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    cmd2.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
    cmd2.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;  
    cmd2.hue = (hue * 254) / 360;
	cmd2.saturation = 254;
    //cmd2.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd2.transition_time = 1;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_color_move_to_hue_and_saturation_cmd_req(&cmd2);
    esp_zb_lock_release(); 
	lock_nvs_save = true; 
	device_array[node_i].endpoint[ep_i].device_t.value = (hue * 254) / 360;  //take nother parameetr as hue and saturation
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };
    xQueueSend(nvs_save_queue, &evt, 0);	
}

void zigbee_cluster_fan_control_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t speed)
{
    static esp_zb_zcl_attribute_t fan_mode_attr;
    static esp_zb_zcl_write_attr_cmd_t write_cmd;

	device_array[node_i].endpoint[ep_i].device_t.level = speed;

    fan_mode_attr.id = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID; // Fan Mode attribute ID
    fan_mode_attr.data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM;  // 8-bit enum type
    fan_mode_attr.data.size = sizeof(uint8_t);             // Size of a single byte (8-bit enum)
    fan_mode_attr.data.value = &speed;            // Pointer to the fan mode value (e.g., 0x02 for Auto Mode)

    // Configure the write command
    write_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    write_cmd.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
    write_cmd.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
    write_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    
    write_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL; // Fan Control Cluster
    write_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV; // Writing to the Server
    write_cmd.manuf_specific = 0; // Not manufacturer-specific
    write_cmd.manuf_code = 0; // No manufacturer code
    write_cmd.dis_defalut_resp = 1; // Disable default response
    write_cmd.attr_number = 1; // Writing a single attribute
    write_cmd.attr_field = &fan_mode_attr; // Assign attribute field
    // Send the Write Attribute request
    esp_zb_zcl_write_attr_cmd_req(&write_cmd);
    lock_nvs_save = true; 
	//save_or_update_light_bulb_device_to_nvs(&device_array[node_i]);	
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };

    xQueueSend(nvs_save_queue, &evt, 0);
}

void zigbee_cluster_thermostat_state_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t state)
{
    uint8_t ac_mode = 0;
	if(state == 0){	
		ac_mode = 0;
		device_array[node_i].endpoint[ep_i].device_t.state = false;			
	}else{
		ac_mode = 3;
		device_array[node_i].endpoint[ep_i].device_t.state = true;			
	}

    // 1. Write AC Mode
    esp_zb_zcl_attribute_t ac_mode_attr;
    ac_mode_attr.id = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID;
    ac_mode_attr.data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM;
    ac_mode_attr.data.size = sizeof(uint8_t);
    ac_mode_attr.data.value = &ac_mode;

    esp_zb_zcl_write_attr_cmd_t write_cmd;
    write_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    write_cmd.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
    write_cmd.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
    write_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    write_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    write_cmd.manuf_specific = 0;
    write_cmd.manuf_code = 0;
    write_cmd.dis_defalut_resp = 1;
   
    write_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
	 write_cmd.attr_number = 1;
    write_cmd.attr_field = &ac_mode_attr;

    esp_zb_zcl_write_attr_cmd_req(&write_cmd);

	lock_nvs_save = true; 
	//save_or_update_light_bulb_device_to_nvs(&device_array[node_i]);	
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };

    xQueueSend(nvs_save_queue, &evt, 0);
}

void zigbee_cluster_thermostat_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t temp)
{
    uint8_t ac_mode = 0;
    device_array[node_i].endpoint[ep_i].device_t.state = false;
    if(temp > 0) {
        ac_mode = 3;
        device_array[node_i].endpoint[ep_i].device_t.state = true;
		device_array[node_i].endpoint[ep_i].device_t.level = temp;
    }

    // 1. Write AC Mode
    esp_zb_zcl_attribute_t ac_mode_attr;
    ac_mode_attr.id = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID;
    ac_mode_attr.data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM;
    ac_mode_attr.data.size = sizeof(uint8_t);
    ac_mode_attr.data.value = &ac_mode;

    esp_zb_zcl_write_attr_cmd_t write_cmd;
    write_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    write_cmd.zcl_basic_cmd.dst_addr_u.addr_short = device_array[node_i].short_addr;
    write_cmd.zcl_basic_cmd.dst_endpoint = device_array[node_i].endpoint[ep_i].ep;
    write_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    write_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    write_cmd.manuf_specific = 0;
    write_cmd.manuf_code = 0;
    write_cmd.dis_defalut_resp = 1;
    write_cmd.attr_number = 1;
    write_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
    write_cmd.attr_field = &ac_mode_attr;

    esp_zb_zcl_write_attr_cmd_req(&write_cmd);

    vTaskDelay(pdMS_TO_TICKS(300)); // Increase delay if needed

    // 2. Write Cooling Setpoint
    int16_t temp_value = temp * 100; // 25°C -> 2500
    esp_zb_zcl_attribute_t temp_attr;
    temp_attr.id = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID;
    temp_attr.data.type = ESP_ZB_ZCL_ATTR_TYPE_S16;
    temp_attr.data.size = sizeof(int16_t);
    temp_attr.data.value = &temp_value;

    write_cmd.attr_field = &temp_attr;
    esp_zb_zcl_write_attr_cmd_req(&write_cmd);
	lock_nvs_save = true; 
	//save_or_update_light_bulb_device_to_nvs(&device_array[node_i]);	
	/* 🔁 Send NVS save request to another task */
    nvs_save_evt_t evt = {
        .node_i = node_i
    };

    xQueueSend(nvs_save_queue, &evt, 0);
}

//////////////////////////////////////////////////

void nuos_update_all_commissioned_zigbee_attribute(){

    for(int k=0; k<devices_count; k++){
    	for(int j=0; j<device_array[k].ep_counts; j++){
    		printf("dev_type:%d ep:%d\n", device_array[k].endpoint->dev_type, device_array[k].endpoint[j].ep);
			for(int cl=0; cl<device_array[k].endpoint[j].cluster_counts; cl++){
				switch(device_array[k].endpoint[j].cluster_id[cl]){
					case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
						esp_zb_zcl_on_off_cmd_t cmd_req;
						cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = device_array[k].short_addr; //
						cmd_req.zcl_basic_cmd.dst_endpoint = device_array[k].endpoint[j].ep;
						cmd_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;//device_array[k].endpoint[j].ep;
						cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
						cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
						//esp_zb_lock_acquire(portMAX_DELAY);
						esp_zb_zcl_on_off_cmd_req(&cmd_req);
						//esp_zb_lock_release();
						break;
					case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
						device_array[k].endpoint[j].device_t.level = 200;
						esp_zb_zcl_move_to_level_cmd_t cmd_level;
						cmd_level.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
						cmd_level.zcl_basic_cmd.dst_addr_u.addr_short = device_array[k].short_addr; //
						cmd_level.zcl_basic_cmd.dst_endpoint = device_array[k].endpoint[j].ep;
						cmd_level.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
						cmd_level.level = device_array[k].endpoint[j].device_t.level;
						cmd_level.transition_time = 0xffff;
	//		            esp_zb_lock_acquire(portMAX_DELAY);
						esp_zb_zcl_level_move_to_level_with_onoff_cmd_req(&cmd_level);
	//		            esp_zb_lock_release();
						break;
					case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
						esp_zb_zcl_color_move_to_color_cmd_t cmd_color;
						cmd_color.color_x = device_array[k].endpoint[j].device_t.value & 0xffff;
						cmd_color.color_y = (device_array[k].endpoint[j].device_t.value >> 16) & 0xffff;
						cmd_color.transition_time = 0;
						cmd_color.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
						cmd_color.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
	//		            esp_zb_lock_acquire(portMAX_DELAY);
						esp_zb_zcl_color_move_to_color_cmd_req(&cmd_color);
	//		            esp_zb_lock_release();
						break;
					case ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
						printf("ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL\n");
						if(device_array[k].endpoint[j].device_t.level > 0) device_array[k].endpoint[j].device_t.level = 0;
						else device_array[k].endpoint[j].device_t.level = 2;
						//write_fan_control_fan_mode(device_array[k].short_addr, device_array[k].endpoint[j].ep, device_array[k].endpoint[j].device_t.level);

						zigbee_cluster_fan_control_attr_update(k, j, device_array[k].endpoint[j].device_t.level);
						break;
					default: break;
				}
			}
    	}
    }
}

typedef struct {
	uint8_t bytes[17]; // 136 bits = 17 bytes
} uint136_t;



uint8_t array_data[17] = {0,1,0,1,0,0,0,10,0,0,0,10,0,0,0,0,0};	
uint8_t seq_num = 1;									
static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
    	if(button_func_pair->pin == gpio_touch_btn_pins[1]){
			
            array_data[1] = seq_num++;
            // Create octet string: first byte is length, then payload
            uint8_t octet_string[18];
            octet_string[0] = 17; // length byte
            memcpy(&octet_string[1], array_data, 17);

			esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
				.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
				.cluster_id = 0xEF00,
				.custom_cmd_id = 0xfc,
				.data = {
					.value = octet_string,
					.size = 18,
					.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
				},
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
				.profile_id = 0x0104,
				.zcl_basic_cmd.dst_addr_u.addr_short = 0x709b,
				.zcl_basic_cmd.dst_endpoint = 1,
				.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT,
				.manuf_specific = 0,
				.manuf_code=0,
				.dis_defalut_resp = 1,
			};
			esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 
    	}else if(button_func_pair->pin == gpio_touch_btn_pins[0]){
			nuos_zb_scene_recall_scene_request(10, 10, 1, 1, 1, ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT);
    	}    
	}
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    
    esp_app_zb_attribute_handler(message->src_address.u.short_addr, message->src_endpoint, message->cluster, &message->attribute);

	return ESP_OK;
}

typedef struct zdo_info_ctx_s {
    uint8_t endpoint;
    uint16_t short_addr;
} zdo_info_user_ctx_t;


void configure_ias_zone_status_reporting(uint16_t dst_short_addr, uint8_t dst_endpoint, uint8_t src_endpoint)
{
    esp_zb_zcl_config_report_cmd_t report_cmd;
	bool report_change = 0;
	report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
	report_cmd.zcl_basic_cmd.dst_endpoint = dst_endpoint;
	report_cmd.zcl_basic_cmd.src_endpoint = src_endpoint;
	report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE;


		esp_zb_zcl_config_report_record_t records[] = {
		{
			.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
			.attributeID = ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATUS_ID,
			.attrType = ESP_ZB_ZCL_ATTR_TYPE_16BITMAP,
			.min_interval = 1,
			.max_interval = 10,
			.reportable_change = &report_change,
		},
	};
	report_cmd.record_number = ARRAY_LENGTH(records);
	report_cmd.record_field = records;
    // Send the configure reporting command
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
}

typedef struct {
    node_module_t* devices_stt;
	int node_index;
    int ep_index;
    int current_cluster;
} bind_state_t;

void send_next_bind(bind_state_t* state);

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
	bind_state_t* state = (bind_state_t*)user_ctx;
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bind response from address(0x%x), endpoint(%d) with status(%d)", 
			state->devices_stt->short_addr, 
			state->devices_stt->endpoint[state->ep_index].ep,
        	zdo_status);

	   device_array[state->node_index].endpoint[state->ep_index].is_bind[state->current_cluster] = true;

	   state->devices_stt->endpoint[state->ep_index].is_bind[state->current_cluster] = true;		
       /* configure report attribute command */
       esp_zb_zcl_config_report_cmd_t report_cmd; 
       bool report_change = 1;
       report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = state->devices_stt->short_addr;
       report_cmd.zcl_basic_cmd.dst_endpoint = state->devices_stt->endpoint[state->ep_index].ep;
       report_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
       report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
       report_cmd.clusterID = state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster];
       
	   printf("clusterID: 0x%x \n", state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster]);
	   if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
			esp_zb_zcl_config_report_record_t records[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
				.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, 
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_BOOL, 
				.min_interval = 1, 
				.max_interval = 0, 
				.reportable_change = &report_change}};
			report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records;
			esp_zb_zcl_config_report_cmd_req(&report_cmd);
	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
			esp_zb_zcl_config_report_record_t records1[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
				.attributeID = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, 
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_U8, 
				.min_interval = 1, 
				.max_interval = 0, 
				.reportable_change = &report_change}};

			report_cmd.record_number = sizeof(records1) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records1;		
			esp_zb_zcl_config_report_cmd_req(&report_cmd);
			 
	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){

	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING){
			esp_zb_zcl_config_report_record_t records[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
				.attributeID = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, 
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_U8, 
				.min_interval = 1, 
				.max_interval = 0, 
				.reportable_change = &report_change}};
			report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records;
			esp_zb_zcl_config_report_cmd_req(&report_cmd);			
	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
			esp_zb_zcl_config_report_record_t records[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
				.attributeID = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID, 
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, 
				.min_interval = 1, 
				.max_interval = 0, 
				.reportable_change = &report_change}};
			report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records;
			esp_zb_zcl_config_report_cmd_req(&report_cmd);		
	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
			esp_zb_zcl_config_report_record_t records[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
				.attributeID = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, 
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_S16, 
				.min_interval = 1, 
				.max_interval = 0, 
				.reportable_change = &report_change}};
			report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records;
			esp_zb_zcl_config_report_cmd_req(&report_cmd);				
	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE){

	
			esp_zb_zcl_write_attr_cmd_t cie_write_req;
			cie_write_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
			memcpy(cie_write_req.zcl_basic_cmd.dst_addr_u.addr_long, state->devices_stt->ieee_addr,
					sizeof(esp_zb_ieee_addr_t));
			cie_write_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
			cie_write_req.zcl_basic_cmd.dst_endpoint = state->devices_stt->endpoint[state->ep_index].ep;
			cie_write_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE;

			// Get gateway address
			esp_zb_ieee_addr_t addr = {0};
			esp_zb_get_long_address(addr);

			// // Write the CIE Address (Gateway's IEEE Address)
			esp_zb_zcl_attribute_t cie_attribute;
			cie_attribute.id = ESP_ZB_ZCL_ATTR_IAS_ZONE_IAS_CIE_ADDRESS_ID;
			cie_attribute.data.type = ESP_ZB_ZCL_ATTR_TYPE_IEEE_ADDR;
			cie_attribute.data.size = sizeof(esp_zb_ieee_addr_t);
			cie_attribute.data.value = addr;

			cie_write_req.attr_field = &cie_attribute;
			cie_write_req.attr_number = 1;

			//esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zcl_write_attr_cmd_req(&cie_write_req);
			//esp_zb_lock_release();

			//printf("\n=========================================\n");


			esp_zb_zcl_ias_zone_enroll_response_cmd_t cmd_resp;
			cmd_resp.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
			memcpy(cmd_resp.zcl_basic_cmd.dst_addr_u.addr_long, state->devices_stt->ieee_addr,
					sizeof(esp_zb_ieee_addr_t));
			cmd_resp.zcl_basic_cmd.dst_endpoint = state->devices_stt->endpoint[state->ep_index].ep;
			cmd_resp.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
			cmd_resp.zone_id = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION;
			cmd_resp.enroll_rsp_code = ESP_ZB_ZCL_IAS_ZONE_ENROLL_RESPONSE_CODE_SUCCESS;
			//esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zcl_ias_zone_enroll_cmd_resp(&cmd_resp);//ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION
			//esp_zb_lock_release();

			//printf("\n=========================================\n");

			esp_zb_zcl_config_report_cmd_t report_cmd;
			report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = state->devices_stt->short_addr;
			report_cmd.zcl_basic_cmd.dst_endpoint = state->devices_stt->endpoint[state->ep_index].ep;
			report_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
			report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

			report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE;
			uint8_t min_change = 1;
			esp_zb_zcl_config_report_record_t records_window[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, // Request direction
				.attributeID = ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATUS_ID,
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_16BITMAP,        // Attribute variable type
				.min_interval = 1, // Min report time
				.max_interval = 10, // Max report time
				.reportable_change = &min_change,
			}};
			report_cmd.record_number =
				sizeof(records_window) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records_window;
			//esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zcl_config_report_cmd_req(&report_cmd);
			//esp_zb_lock_release();		

			configure_ias_zone_status_reporting(state->devices_stt->short_addr, 
				state->devices_stt->endpoint[state->ep_index].ep, 
				ESP_ZB_GATEWAY_ENDPOINT);
	   }else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT){
			printf("REPORT ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT\n");
			esp_zb_zcl_config_report_record_t records_window0[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, // Request direction
				.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,        // Attribute variable type
				.min_interval = 1, // Min report time
				.max_interval = 10, // Max report time
				.reportable_change = &report_change,
			}};
			report_cmd.record_number =
				sizeof(records_window0) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records_window0;

			//esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zcl_config_report_cmd_req(&report_cmd);
			//esp_zb_lock_release();	
		}else if(state->devices_stt->endpoint[state->ep_index].cluster_id[state->current_cluster] == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT){
			printf("REPORT ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT\n");
			esp_zb_zcl_config_report_record_t records_window01[] = {{
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, // Request direction
				.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
				.attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,        // Attribute variable type
				.min_interval = 1, // Min report time
				.max_interval = 10, // Max report time
				.reportable_change = &report_change,
			}};
			report_cmd.record_number =
				sizeof(records_window01) / sizeof(esp_zb_zcl_config_report_record_t);
			report_cmd.record_field = records_window01;
			//esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zcl_config_report_cmd_req(&report_cmd);
			//esp_zb_lock_release();				
		}
    }
	state->current_cluster++;
    send_next_bind(state);
}

static void ieee_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx){
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
           
		if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
			node_module_t* device_ctx = ((node_module_t*)user_ctx);
            esp_zb_zdo_ieee_addr_rsp_t *ieee_resp = (esp_zb_zdo_ieee_addr_rsp_t *)resp;
			memcpy(&(device_ctx->ieee_addr), ieee_resp->ieee_addr, sizeof(esp_zb_ieee_addr_t));
			ESP_LOGI(TAG, "IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
					ieee_resp->ieee_addr[7], ieee_resp->ieee_addr[6], ieee_resp->ieee_addr[5], ieee_resp->ieee_addr[4],
					ieee_resp->ieee_addr[3], ieee_resp->ieee_addr[2], ieee_resp->ieee_addr[1], ieee_resp->ieee_addr[0]);

			printf("------SHORT_ADDR:0x%x\n", device_ctx->short_addr);
			/* find the active endpoint */
			esp_zb_zdo_active_ep_req_param_t active_ep_req;
			active_ep_req.addr_of_interest = device_ctx->short_addr;
			esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, user_ctx);
			esp_zb_lock_release();
		}		
		
    }
}


void device_leave_callback(esp_zb_zdp_status_t zdo_status, void *user_ctx){
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        uint16_t short_addr = *(uint16_t*)user_ctx;
		printf("Device(0x%x) removed successfully!!\n", short_addr);  
    }
}

void nuos_zb_leave_device(node_module_t * module_remove ) {
	uint8_t nodeindex	= 0;
	uint8_t epindex		= 0;
	
	if(module_remove == NULL){
		printf("module_removed is NULL\n");
	}else{
		printf("find_device_by_dev_id: 0x%x\n", module_remove->short_addr);
		// Not a direct SDK API, but the typical process is:
		esp_zb_zdo_mgmt_leave_req_param_t leave_req = {0};
		leave_req.dst_nwk_addr = module_remove->short_addr;
		memcpy(leave_req.device_address, module_remove->ieee_addr, sizeof(esp_zb_ieee_addr_t));
		leave_req.remove_children = 0;
		leave_req.rejoin = 0;
		
		printf("deleted_short_addr:0x%x\n", module_remove->short_addr);
		remove_zigbee_device_from_nvs(module_remove->short_addr);
		vTaskDelay(pdMS_TO_TICKS(100));
		esp_zb_zdo_device_leave_req(&leave_req, device_leave_callback, &leave_req.dst_nwk_addr);
		vTaskDelay(pdMS_TO_TICKS(100));
    	memset(device_array, 0, sizeof(device_array));
		devices_count = 0;
		read_all_devices_from_nvs(NULL);
	}
}


// Read attribute helper
void poll_sensor_read_attributes(uint8_t sensor_ep, uint16_t sensor_short_addr)
{
    // Temperature
    esp_zb_zcl_read_attr_cmd_t temp_read = {0};
    temp_read.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

	temp_read.zcl_basic_cmd.dst_addr_u.addr_short = sensor_short_addr;
    temp_read.zcl_basic_cmd.dst_endpoint = sensor_ep;
    temp_read.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    temp_read.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    temp_read.manuf_specific = 0;
    temp_read.direction = 0; // To server
    temp_read.dis_defalut_resp = 1;
    temp_read.manuf_code = 0;
    uint16_t temp_attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;  
    temp_read.attr_field  = &temp_attr_id;
    esp_zb_zcl_read_attr_cmd_req(&temp_read);

    // Humidity
    esp_zb_zcl_read_attr_cmd_t hum_read = {0};
    hum_read.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	hum_read.zcl_basic_cmd.dst_addr_u.addr_short = sensor_short_addr;
    hum_read.zcl_basic_cmd.dst_endpoint = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    hum_read.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    hum_read.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    hum_read.manuf_specific = 0;
    hum_read.direction = 0; // To server
    hum_read.dis_defalut_resp = 1;
    hum_read.manuf_code = 0;
    uint16_t hum_attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
    hum_read.attr_field = &hum_attr_id;
    esp_zb_zcl_read_attr_cmd_req(&hum_read);
}

// #define POLL_INTERVAL_MS 6000 // Poll every 60 seconds
bool start_polling = false;
// void periodic_polling_timer_callback(void* arg)
// {
// 	if(start_polling){
// 		printf("\n=========================================\n");
// 		poll_sensor_read_attributes(1, 0x0d4e);
// 	}
    
// }

// void start_periodic_polling()
// {
//     const esp_timer_create_args_t periodic_timer_args = {
//         .callback = &periodic_polling_timer_callback,
//         .name = "poll_sensor_timer"
//     };
//     esp_timer_handle_t periodic_timer;
//     esp_timer_create(&periodic_timer_args, &periodic_timer);
//     esp_timer_start_periodic(periodic_timer, POLL_INTERVAL_MS * 1000); // microseconds
// }

void send_next_bind(bind_state_t* state) {
    int cl = state->current_cluster;
    node_module_t* devices_stt = state->devices_stt;
    int ep_index = state->ep_index;

    if (cl >= devices_stt->endpoint[ep_index].cluster_counts) {
        // All clusters done
        // if (devices_stt->endpoint[ep_index].dev_type == 770) {
        //     start_polling = true;
        // }
        free(state);
        return;
    }
    if(!devices_stt->endpoint[ep_index].is_bind[cl]){
		esp_zb_zdo_bind_req_param_t *bind_req = calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));


		bind_req->req_dst_addr = devices_stt->short_addr;

        /* populate the src information of the binding */
        memcpy(bind_req->src_address, devices_stt->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req->src_endp = devices_stt->endpoint[ep_index].ep;
        bind_req->cluster_id = devices_stt->endpoint[ep_index].cluster_id[cl];

        /* populate the dst information of the binding */
        bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req->dst_address_u.addr_long);
        bind_req->dst_endp = ESP_ZB_GATEWAY_ENDPOINT;

        ESP_LOGI(TAG, "Request device to bind us:0x%x ep:%d", bind_req->cluster_id, devices_stt->endpoint[ep_index].ep);
	
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, state);

		///////////////////////////////////////////////////////////////////////////////
		// ... fill bind_req fields as before ...
		if(bind_req != NULL){
			bind_req->cluster_id = devices_stt->endpoint[ep_index].cluster_id[cl];
			bind_req->dst_endp = devices_stt->endpoint[ep_index].ep;
			// ... other fields ...
			esp_zb_ieee_address_by_short(devices_stt->short_addr, devices_stt->ieee_addr);
			esp_zb_get_long_address(bind_req->src_address);
			bind_req->src_endp = ESP_ZB_GATEWAY_ENDPOINT;
			bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
			memcpy(bind_req->dst_address_u.addr_long, devices_stt->ieee_addr, sizeof(esp_zb_ieee_addr_t));
			bind_req->req_dst_addr = esp_zb_get_short_address(); /* TODO: Send bind request to self */
			// Pass state as user_ctx so callback knows where we are
			esp_zb_zdo_device_bind_req(bind_req, NULL, NULL);
			// Do NOT free bind_req here; free it in the callback after use
		}
	}
}

void start_zigbee_cluster_ep_binding(node_module_t* devices_stt, int ep_index) {
    bind_state_t* state = calloc(1, sizeof(bind_state_t));

	find_device_by_dev_id(devices_stt->room_id, devices_stt->module_id, devices_stt->endpoint[ep_index].ep, (uint8_t*)&state->node_index, (uint8_t*)&state->ep_index); 
    state->devices_stt = devices_stt;
    state->ep_index = ep_index;
    state->current_cluster = 0;
    // Start the first binding
    send_next_bind(state);
}


void add_number_to_string(const char* input_str, int decimal_val, char* output_result) {
    char result[24];
    // Format: "Scene Switch 2"
    snprintf(result, sizeof(result), "%s %d", input_str, decimal_val);

	strcpy(output_result, result);
    // Now result contains "Scene Switch 2"
    printf("%s\n", output_result);
}

void nuos_zb_read_attributes_from_clusters(uint16_t cluster_id, uint8_t dst_ep, uint16_t short_addr){
	
	// uint16_t attr_list[] = {
	// 	ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONETYPE_ID,
	// };

	// esp_zb_zcl_read_attr_cmd_t read_cmd = {
	// 	.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
	// 	.clusterID = cluster_id,
	// 	.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT,
	// 	.zcl_basic_cmd.dst_endpoint = dst_ep,
	// 	.zcl_basic_cmd.dst_addr_u.addr_short = short_addr,
	// 	.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
	// 	.manuf_specific = 0,
	// 	.dis_defalut_resp = 0,
	// 	.manuf_code = 0,
	// 	.attr_number  = sizeof(attr_list) / sizeof(uint16_t),
	// 	.attr_field   = attr_list,
	// };

	// esp_zb_zcl_read_attr_cmd_req(&read_cmd);




	esp_zb_zcl_read_attr_cmd_t read_req;
	read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	read_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
	read_req.zcl_basic_cmd.dst_endpoint = dst_ep;
	read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
	read_req.clusterID = cluster_id;

	uint16_t attributes[] = {
		ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONETYPE_ID,
	};
	read_req.attr_number = ARRAY_LENTH(attributes);
	read_req.attr_field = attributes;
	esp_zb_zcl_read_attr_cmd_req(&read_req);

	printf("esp_zb_zcl_read_attr_cmd_t\n");

}


static void simple_desc_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    	node_module_t* devices_stt = ((node_module_t*)user_ctx);
        uint8_t cluster_counts = 0;  
		uint8_t ep_index = 0;
        // Dereference the pointer to get the value
        ESP_LOGI(TAG, "Simple desc response: status(%d), device_id(%d), app_version(%d), profile_id(0x%x), endpoint_ID(%d)", zdo_status,
                 simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);

        for(int j=0; j<devices_stt->ep_counts; j++){
        	if(devices_stt->endpoint[j].ep == simple_desc->endpoint){
        		ep_index = j;
        		break;
        	}
        }
		devices_stt->endpoint[ep_index].dev_type = simple_desc->app_device_id;

        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            ESP_LOGI(TAG, "Cluster ID list: 0x%x", *(simple_desc->app_cluster_list + i));
            devices_stt->zb_app_id = simple_desc->app_device_id;

            switch(simple_desc->app_device_id){

				case ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID:         //OnOff Light Device
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee OnOff Light");
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						default:break;
					}
					break;
				case ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID:       //Dimmer Light
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Dimmable Light");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Dimmable Light");
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						default:break;
					}
					break;

				case 0x010D:    
				case 0x10C:					
				case ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID: //RGB and CCT Light
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Color Light");
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Color Light");
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Color Light");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						default:break;
					}
					break;

				case ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID:
					switch(*(simple_desc->app_cluster_list + i)){
						case 0xFC31:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Config Model");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = 0xFC31;
							break;
						default:break;
					}
					break;

				case ESP_ZB_HA_THERMOSTAT_DEVICE_ID:  			//1Light-1Fan
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Fan Control");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
							break;
						case ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee IR Blaster");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                           
							break;
						default:break;
					}
					break;

				case 0x0106: 									//Illuminance Sensor
				    //printf("Hello --> 0x0106\n");
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT:
						    //printf("ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT!!\n");
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Lux Sensor");
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						default:break;
					}
					break;

				case ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID: 	//Temperature & Humidity Sensor
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zb Temperature Sensor");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Humidity Sensor");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
							break;
						default:break;
					}
					break;

				case ESP_ZB_HA_IAS_ZONE_ID:  					//contact switch, motion sensor, gas leak
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee IAS Sensor");
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE;
							devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;

							nuos_zb_read_attributes_from_clusters(ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, devices_stt->endpoint[ep_index].ep, devices_stt->short_addr);

							ep_index++; 

							devices_stt->endpoint[ep_index].ep = 2;
							devices_stt->endpoint[ep_index].dev_id = 2;
							devices_stt->endpoint[ep_index].dev_type = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID;

							devices_stt->endpoint[ep_index].is_bind[0] = false;
							strcpy(devices_stt->endpoint[ep_index].dev_name, "Config Register");
							devices_stt->endpoint[ep_index].cluster_id[0] = 0xFC31;	
							devices_stt->ep_counts++;		
							devices_stt->endpoint[ep_index].cluster_counts = 1;		
							devices_stt->ep_counts = ep_index + 1;	
							break;
						default:break;
					}
					break;

				case ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID:   //OnOff Switch
				case ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID:  //wireless remote switch	
				case ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID:  //zigbee scene switch (Not Tested yet)
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
							devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
							devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
							char output_result[24];
							if(devices_stt->ep_counts == 4){
								const char* input_str = "Scene Switch";
								add_number_to_string(input_str, ep_index+1, output_result);
								strcpy(devices_stt->endpoint[ep_index].dev_name, output_result);
							}else{
								if(simple_desc->app_device_id == ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID){
									const char* input_str2 = "Curtain Switch";
									add_number_to_string(input_str2, ep_index+1, output_result);
									strcpy(devices_stt->endpoint[ep_index].dev_name, input_str2);
								}
							}
							//devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
						break;
						default: break;
					}
					break;

				case ESP_ZB_HA_HEATING_COOLING_UNIT_DEVICE_ID:  //zigbee scene switch (Not Tested yet)
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT:
						devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
						strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee IR Blaster");
						devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;


						break;
						default:break;
					}
					break;

				case ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID:
					switch(*(simple_desc->app_cluster_list + i)){
						case ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING:
						devices_stt->endpoint[ep_index].is_bind[cluster_counts] = false;
						strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Curtain Switch");
						devices_stt->endpoint[ep_index].cluster_id[cluster_counts++] = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING;
                        devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
                        ep_index++; 

	                    devices_stt->endpoint[ep_index].ep = 2;
						devices_stt->endpoint[ep_index].dev_id = 2;
						devices_stt->endpoint[ep_index].dev_type = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID;

						devices_stt->endpoint[ep_index].is_bind[0] = false;
						strcpy(devices_stt->endpoint[ep_index].dev_name, "Zigbee Curtain Stop");
						devices_stt->endpoint[ep_index].cluster_id[0] = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING;	
						devices_stt->ep_counts++;		
						devices_stt->endpoint[ep_index].cluster_counts = 1;		
						devices_stt->ep_counts = ep_index + 1;	
						break;
						default:break;
					}				
					break;
				default:break;
            }
        }
		devices_stt->endpoint[ep_index].dev_type = simple_desc->app_device_id;
		devices_stt->endpoint[ep_index].cluster_counts = cluster_counts;
        printf("desc==>devices_stt->short_addr:0x%4x ep_counts:%d dev_type:%d\n", 
			devices_stt->short_addr, 
			devices_stt->ep_counts, 
			devices_stt->endpoint[ep_index].dev_type);

        start_zigbee_cluster_ep_binding(devices_stt, ep_index);



        if(devices_stt->ep_counts >= 1){
			printf("ep_index: %d  %d\n", ep_index, devices_stt->ep_counts-1);
			if(ep_index >= devices_stt->ep_counts-1){
				printf("ep_index >= devices_stt->ep_counts-1\n");
				// printf("short_addr:%d\n", devices_stt->short_addr);

				strcpy(devices_stt->manuf_name, "manuf_name");
				strcpy(devices_stt->model_id, "model_id");
				strcpy(devices_stt->loc_name, "Living Room");

				devices_stt->room_id = 255;
				devices_stt->module_id = 255;

				//send_module_motion_sensor_unconfigured_over_uart(devices_stt->ieee_addr);

				vTaskDelay(pdMS_TO_TICKS(1));
				int16_t is_index = find_module_index_by_ieee(devices_stt->ieee_addr);
				if(is_index == -1){
					printf("Device NOT found in NVS with module_id:%d\n", devices_stt->module_id);
					if(save_or_update_light_bulb_device_to_nvs(devices_stt) == ESP_OK){
						memset(device_array, 0, sizeof(device_array));
						devices_count = 0;
						read_all_devices_from_nvs(NULL);
					}
					printf("After save to NVS devices_count:%d\n", devices_count);	
					remove_duplicate_devices();				
				}else{
					printf("Device already exists in NVS with module_id:%d\n", devices_stt->module_id);
					if(save_or_update_light_bulb_device_to_nvs(devices_stt) == ESP_OK){
						memset(device_array, 0, sizeof(device_array));
						devices_count = 0;
						read_all_devices_from_nvs(NULL);
					}
					printf("After save to NVS devices_count:%d\n", devices_count);	
					remove_duplicate_devices();
				}

				
				//send_module_unconfigured_over_uart(devices_stt->ieee_addr);

				/* Read peer Manufacture Name & Model Identifier */
				// esp_zb_zcl_read_attr_cmd_t read_req;
				// read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
				// read_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
				// read_req.zcl_basic_cmd.dst_endpoint = devices_stt->endpoint[0].ep;
				// read_req.zcl_basic_cmd.dst_addr_u.addr_short = devices_stt->short_addr;
				// read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

				// uint16_t attributes[] = {
				//    ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
				//    ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
				// };
				// read_req.attr_number = ARRAY_LENTH(attributes);
				// read_req.attr_field = attributes;
				//esp_zb_zcl_read_attr_cmd_req(&read_req);
				turn_off_commissioning_led_switch();


			}
        }
    }
}

// Optional: Callback to handle leave response
void leave_device_callback(esp_zb_zdp_status_t status, void *user_ctx) {
    if (status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        printf("Device was notified and has left the network.\n");
    } else {
        printf("Failed to notify device to leave the network.\n");
    }
}

static void permit_join_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
	if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
		ESP_LOGI(TAG, "Permit joining successful");
	} else {
		ESP_LOGE(TAG, "Permit joining failed with status: %d", zdo_status);
	}
}

void gateway_leave_device(uint16_t device_short_addr, esp_zb_ieee_addr_t device_ieee_addr, uint8_t rejoin) {

	esp_zb_zdo_permit_joining_req_param_t cmd_req;
	cmd_req.dst_nwk_addr = device_short_addr;
	cmd_req.permit_duration = 0; // Disable joining
	cmd_req.tc_significance = 0; // No special significance
    esp_zb_zdo_permit_joining_req(&cmd_req, permit_join_cb, NULL);

    //esp_zb_zdo_permit_joining_req_param_t *cmd_req, esp_zb_zdo_permit_join_callback_t user_cb, void *user_ctx
    esp_zb_zdo_mgmt_leave_req_param_t leave_req = {0};

    // Set the IEEE address of the device to be removed
    memcpy(leave_req.device_address, device_ieee_addr, sizeof(esp_zb_ieee_addr_t));

    // Set the NWK (short) address of the device to which the request is sent
    leave_req.dst_nwk_addr = device_short_addr;

    // Remove only the target device (not its children)
    leave_req.remove_children = 1;

    // Do not request the device to rejoin automatically
    leave_req.rejoin = rejoin;

    // Send the leave request
    esp_zb_zdo_device_leave_req(&leave_req, leave_device_callback, NULL);
}


static void ep_cb_broadcast(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {

		node_module_t* n_module = ((node_module_t*)user_ctx);
    	uint16_t short_addr = n_module->short_addr;
		n_module->is_active = true;
        //ESP_LOGW(TAG, "Active endpoint response: status(%d) short_addr(0x%x) endpoint count(%d)", zdo_status, short_addr, ep_count);

		// node_module_t * module = find_device_by_short_addr(short_addr);
		// if(module != NULL){
		//  	//printf("found type:%d\n", n_module->dev_id);
		// 	if(ep_count > 0){
		// 		send_uart_command_in_array(module, "active");
		// 	}
		// }
    }
}


void find_zigbee_nodes_task(void* args) {

	while(1){
		//vTaskDelay(pdMS_TO_TICKS(5000));
	    for(int k=0; k<devices_count; k++){
			esp_zb_zdo_active_ep_req_param_t active_ep_req;
			active_ep_req.addr_of_interest = device_array[k].short_addr;
			device_array[k].is_active = false;
			if(device_array[k].room_id != 255){
				esp_zb_lock_acquire(portMAX_DELAY);
				esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb_broadcast, (node_module_t*)&device_array[k]);
				esp_zb_lock_release();
			}
			vTaskDelay(pdMS_TO_TICKS(100));
	    }
		vTaskDelay(pdMS_TO_TICKS(60000));
		
	}
	vTaskDelete(NULL);
}

static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {

    	node_module_t* devices_stt = ((node_module_t*)user_ctx);
        ESP_LOGI(TAG, "Active endpoint response: status(%d) and endpoint count(%d)", zdo_status, ep_count);
        devices_stt->ep_counts = 0;
		devices_stt->is_active = true;
        for (int i = 0; i < ep_count; i++) {
        	if(ep_id_list[i] != 242){  //Ignore green power zigbee cluster
        		ESP_LOGI(TAG, "Endpoint ID List: %d", ep_id_list[i]);
        		devices_stt->endpoint[i].ep = ep_id_list[i];
        		devices_stt->ep_counts++;
        		//devices_stt->node_leave_info.is_bind[i] = false;
				// if(devices_stt->endpoint[i].dev_id == 0){
				// 	devices_stt->endpoint[i].dev_id = i+1;
				// }
        	}
        }

        for (int j = 0; j < devices_stt->ep_counts; j++) {
			/* get the node simple descriptor */
			printf("ep_cb=>devices_stt->short_addr:0x%4x\n", devices_stt->short_addr);
			esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
			simple_desc_req.addr_of_interest = devices_stt->short_addr;
			simple_desc_req.endpoint = devices_stt->endpoint[j].ep;
			esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, user_ctx);
        }
    }
}

node_module_t device_k = {0};

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    	memset(&device_k, 0, sizeof(device_k));
        ESP_LOGI(TAG, "Match desc response: status(%d), address(0x%x), endpoint(%d)", zdo_status, addr, endpoint);
        /* save into remote device record structure for future use */
        device_k.endpoint[0].ep = endpoint;
		device_k.endpoint[1].ep = endpoint+1;
		device_k.endpoint[2].ep = endpoint+2;
		device_k.endpoint[3].ep = endpoint+3;

        device_k.short_addr = addr;
		/* get the light ieee address */
		esp_zb_zdo_ieee_addr_req_param_t ieee_req;
		ieee_req.addr_of_interest = device_k.short_addr;
		ieee_req.dst_nwk_addr = device_k.short_addr;
		ieee_req.request_type = 0;
		ieee_req.start_index = 0;

		esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_cb, &device_k);
    }
}


void check_door_open_detection_handler(void* arg){
	if(start_getting_door_open_detetced_flag){
		if(door_open_detetced_counts++ > CHECK_TIME_FOR_DOOR_OPEN_DETECTION){
			start_getting_door_open_detetced_flag = false;
			door_open_detetced_counts = 0;
			printf("Door is still Opened!!\n");
			turn_off_light_switch();
			delete_door_open_detection_timer();
		}
	}
}


void check_motion_detection_handler(void* arg) {
	if(start_getting_motion_detetced_flag){
		if(motion_detetced_counts++ > CHECK_TIME_FOR_PIR_DETECTION){
			start_getting_motion_detetced_flag = false;
			motion_detetced_counts = 0;
			printf("No Sensor Motion Detected!!\n");
			turn_off_light_switch();
		}
	}

}


void init_timer() {
	if(!timer_initialized_ok){
		timer_initialized_ok = true;
		// Create a timer
		const esp_timer_create_args_t timer_args = {
			.callback = &check_motion_detection_handler,
			// name is optional, but may help identify the timer when debugging
			.name = "one_second_timer"
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_args, &my_timer));
	}

	if(!timer_door_open_initialized_ok){
		timer_door_open_initialized_ok = true;
		// Create a timer
		const esp_timer_create_args_t timer_args2 = {
			.callback = &check_door_open_detection_handler,
			// name is optional, but may help identify the timer when debugging
			.name = "one_second_timer"
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_args2, &my_timer_2));
	}
}

void start_motion_detection_timer(){
	if(!timer_started_flag){
		timer_started_flag = true;
		// Start the timer
		ESP_ERROR_CHECK(esp_timer_start_periodic(my_timer, 1000000)); // 1 second period
	}
	start_getting_motion_detetced_flag = true;
	motion_detetced_counts = 0;
}

void delete_motion_detection_timer(){
	if(timer_started_flag){
		timer_started_flag = false;
		if(my_timer != NULL)
			esp_timer_stop(my_timer);
	}
	start_getting_motion_detetced_flag = false;
	motion_detetced_counts = 0;
}

///////////
void start_door_open_detection_timer(){
	if(!timer_started_flag_2){
		timer_started_flag_2 = true;
		// Start the timer
		ESP_ERROR_CHECK(esp_timer_start_periodic(my_timer_2, 1000000)); // 1 second period
	}
	start_getting_door_open_detetced_flag = true;
	door_open_detetced_counts = 0;
}

void delete_door_open_detection_timer(){
	if(timer_started_flag_2){
		timer_started_flag_2 = false;
		if(my_timer_2 != NULL)
			esp_timer_stop(my_timer_2);
	}
	start_getting_door_open_detetced_flag = false;
	door_open_detetced_counts = 0;
}

static bool motion_sensor_one_time_enrollment_flag = false;
uint32_t current_time = 0;


static esp_err_t zb_custom_cluster_cmd_cb_handler(esp_zb_zcl_custom_cluster_command_message_t * message){
	esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
	ESP_LOGI(TAG, "zb_command_report_response_handler: endpoint(%d), cluster(0x%x)", message->info.dst_endpoint, message->info.cluster);					
    ESP_LOGI(TAG, "zb_command_report_response_handler: command(%d), dst_ep(0x%x)", message->info.command.id, message->info.dst_endpoint);
    
	if(message->info.command.id == 0xFD){
		if(message->info.cluster == 0xE000 || message->info.cluster == 0x0006){
			printf("short_addr: 0x%04X", message->info.src_address.u.short_addr);
			printf("\n\n");
			int16_t index = find_node_index_by_short_addr(message->info.src_address.u.short_addr);
			printf("index:%d   data_size:%d\n", index, message->data.size);


			if(index != -1){
				//device_array[index].endpoint[message->info.dst_endpoint-1].device_t.level = message->data.value;
				//device_array[index].endpoint[message->info.dst_endpoint-1].device_t.state = true;

				uint8_t _data = *(uint8_t *)message->data.value;
				printf("Attribute value (uint8): %u\n", _data);
                for(int k=0; k<device_array[index].ep_counts; k++){
					if(_data == k){
						device_array[index].endpoint[k].device_t.state = true;
						if(device_array[index].room_id == 0 || device_array[index].room_id == 255 || device_array[index].module_id == 0 || device_array[index].module_id == 255){
							send_module_scene_unconfigured_over_uart(device_array[index].ieee_addr);
						}else{
							device_array[index].endpoint[k].device_t.value = 255;
							send_module_ctrl_over_uart(&device_array[index], 1, k);
						}
					}else{
						device_array[index].endpoint[k].device_t.value = 0;
						device_array[index].endpoint[k].device_t.state = false;
					}
				}
				
			}

		}
	}
	return ret;
}

static esp_err_t zb_command_report_response_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message){
	esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "zb_command_report_response_handler: endpoint(%d), cluster(0x%x)", message->info.dst_endpoint, message->info.cluster);

	ESP_LOGI(TAG, "zb_command_report_response_handler: command(%d), dst_ep(0x%x)", message->info.command.id, message->info.dst_endpoint);
    return ret;
}
static esp_err_t zb_default_response_cb_handler(const esp_zb_zcl_cmd_default_resp_message_t *message){
	esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
						
    						
    ESP_LOGI(TAG, "zb_default_response_cb_handler: endpoint(%d), cluster(0x%x)", message->info.dst_endpoint, message->info.cluster);

	// printf("CMD id:%d\n", message->info.command.id);
	if(message->info.cluster == 0x06){	
		call_on_attribute_handler_callback();
	}  
	return ret;
}

static esp_err_t zb_write_attr_response_cb_handler(esp_zb_zcl_cmd_write_attr_resp_message_t * message){
esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "zb_write_attr_response_cb_handler: endpoint(%d), cluster(0x%x)", message->info.dst_endpoint, message->info.cluster);
    return ret;	
}

static esp_err_t zb_ias_zone_attribute_handler(const esp_zb_zcl_ias_zone_status_change_notification_message_t *message){
	esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "zb_ias_zone_attribute_handler: endpoint(%d), cluster(0x%x), zone_id(%d), zone_status(%d)", message->info.dst_endpoint, message->info.cluster,
             message->zone_id, message->zone_status);

	//if (message->info.dst_endpoint == 1) {
	if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE) {
		//print_ieee_addr(message->info.src_address.u.ieee_addr);
		printf("short_addr: 0x%04X", message->info.src_address.u.short_addr);
		printf("\n\n");
        //int16_t index = find_module_index_by_ieee(message->info.src_address.u.ieee_addr);
		int16_t index = find_node_index_by_short_addr(message->info.src_address.u.short_addr);
		printf("index:%d\n", index);
		switch(message->zone_id){
			case ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION:
			case 20: //ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION
				if(message->zone_status){
					printf("Motion Sensor Detected!!\n");

					if(index != -1){
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.level = 1;
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.state = true;
					}
					// switch ON Button
					// uint32_t press_duration = (esp_timer_get_time() / 1000) - current_time;
					//after immediate door open, check PIR input after 2 second
                    // if (press_duration > 2000) { //in ms
                    // 	printf("Turn ON Light Detected!!\n");
    				// 	if(start_getting_motion_detetced_flag){
    				// 		start_getting_motion_detetced_flag = false;
    				// 		turn_on_light_switch();
    				// 	}
                    // }
				}else{
					printf("Motion Sensor Idle!!\n");
					if(index != -1){
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.level = 0;
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.state = false;
					}					
				}
//
                if(!motion_sensor_one_time_enrollment_flag){
                	motion_sensor_one_time_enrollment_flag = true;
    				esp_zb_zcl_ias_zone_enroll_request_cmd_t cmd_req;
    				cmd_req.zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION;
    				cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    				cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = message->info.dst_address,
    				cmd_req.zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
    				cmd_req.zcl_basic_cmd.src_endpoint = message->info.dst_endpoint,
    				esp_zb_zcl_ias_zone_enroll_cmd_req(&cmd_req);

//    				esp_zb_zcl_ias_zone_enroll_response_cmd_t cmd_resp;
//    				cmd_resp.zone_id = message->zone_id;
//    				cmd_resp.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
//    				cmd_resp.zcl_basic_cmd.dst_addr_u.addr_short = message->info.dst_address,
//    				cmd_resp.zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
//    				cmd_resp.zcl_basic_cmd.src_endpoint = message->info.dst_endpoint,
//    				cmd_resp.enroll_rsp_code = 0;
//    				esp_zb_zcl_ias_zone_enroll_cmd_resp(&cmd_resp);
                }


				break;
			case 0: //ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH
				if(message->zone_status){
					printf("Door OPEN Detected!!\n");
					current_time = esp_timer_get_time() / 1000; // Time in milliseconds
					start_motion_detection_timer();
					start_door_open_detection_timer();
					if(index != -1){
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.level = 1;
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.state = true;
					}					
				}else{
					printf("Door CLOSE Detected!!\n");
					delete_door_open_detection_timer();
					if(index != -1){
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.level = 0;
						device_array[index].endpoint[message->info.dst_endpoint-1].device_t.state = false;
					}					
				}
				break;

			default:
				break;
		}

		
		if(index != -1){

			if(device_array[index].room_id == 0 || device_array[index].room_id == 255 || device_array[index].module_id == 0 || device_array[index].module_id == 255){
				send_module_motion_sensor_unconfigured_over_uart(device_array[index].ieee_addr);
				//send_module_remove_motion_sensor_on_uart(device_array[index].room_id, device_array[index].module_id, device_array[index].ieee_addr);
			}else{
				send_module_ctrl_over_uart(&device_array[index], 1, message->info.dst_endpoint-1);
			}
            
		}

	}
	return ret;
}
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "zb_attribute_handler: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
//    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
//        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
//            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
//                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
//                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
//                light_driver_set_power(light_state);
//            }
//        }
//    }
    return ret;
}

void reset_default_attr_callback(uint8_t endpoint, uint16_t cluster_id, esp_zb_zcl_attribute_t curr_attr){
	printf("OK DOne for ep:%d\n", endpoint);
}

// Returns true if the string is likely garbage, false if it seems valid
bool is_garbage_string(const char *str) {
    size_t len = strlen(str);
    if (len == 0 || len > 256) return true; // empty or too long

    // Check for mostly printable ASCII (allowing \n, \r)
    for (size_t i = 0; i < len; ++i) {
        char c = str[i];
        if (c != '\n' && c != '\r' && (c < 32 || c > 126)) {
            return true; // found non-printable char
        }
    }

    // Example: check for JSON format (starts with '{' and ends with '}')
    if (str[0] != '{' || str[len-1] != '}') return true;

    // Try parsing as JSON
    cJSON *root = cJSON_Parse(str);
    if (!root) return true;
    cJSON_Delete(root);

    return false; // Passed all checks, not garbage
}

void nvs_save_task(void *arg) {
    uint16_t node_index = *(int*)arg;
	//node_module_t *new_device = find_device_by_short_addr(short_addr);
	if(node_index < devices_count){
		if (save_or_update_light_bulb_device_to_nvs(&device_array[node_index]) == ESP_OK) {
			read_device_info();
			//send_uart_command(&device_array[node_index], 0, "update", 0);
		}
	}
    // free(new_device);  // Free the heap memory!
    vTaskDelete(NULL);
}



int is_printable_string(const char *str) {
    if (str == NULL) return 0;
    for (int i = 0; str[i] != '\0'; ++i) {
        if (!isprint((unsigned char)str[i])) {
            return 0; // Found non-printable character
        }
    }
    return 1; // All characters are printable
}

void check_string(const char *str) {
    if (str == NULL) {
        printf("String is NULL pointer\n");
    } else if (str[0] == '\0') {
        printf("String is empty\n");
    } else if (!is_printable_string(str)) {
        printf("String contains non-printable (garbage) characters\n");
    } else {
        printf("String is valid: \"%s\"\n", str);
    }
}

static esp_err_t  zb_read_attribute_response_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message){
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x))", message->info.dst_endpoint, message->info.cluster);
    const esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    // Iterate through the linked list of attribute responses
    uint8_t attr_counts = 0;
    while (variable != NULL) {
        // Print the status of the attribute read operation
        printf("Attribute ID: 0x%04x\n", variable->attribute.id);
        // Check the status of the attribute read operation
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
            // Print attribute data if the read was successful
            printf("Attribute type: 0x%02x\n", variable->attribute.data.type);
            printf("Attribute size: %d\n", variable->attribute.data.size);
            // Handle different attribute types and print the value
            switch (variable->attribute.data.type) {
                case ESP_ZB_ZCL_ATTR_TYPE_U8:
                    printf("Attribute value (uint8): %u\n", *(uint8_t *)variable->attribute.data.value);
                    break;
                case ESP_ZB_ZCL_ATTR_TYPE_U16:
                    printf("Attribute value (uint16): %u\n", *(uint16_t *)variable->attribute.data.value);
                    break;
                case ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING:
                    // Strings: First byte is the length, followed by the string itself
                    printf("Attribute value (string): %.*s\n", *(uint8_t *)variable->attribute.data.value,
                           (char *)variable->attribute.data.value + 1);
                    break;
                default:
                    printf("Unhandled attribute type\n");
                    break;
            }
        } else {
            printf("Failed to read attribute (status: 0x%02x)\n", variable->status);
        }
        attr_counts++;
        // Move to the next variable in the response
        variable = variable->next;
    }
    return ret;
}
static esp_err_t  zb_ias_zone_enroll_handler(const esp_zb_zcl_ias_zone_enroll_request_message_t *message){
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), manufacturer_code(0x%x), zone_type(%d)", message->info.dst_endpoint, message->info.cluster,
             message->manufacturer_code, message->zone_type);

//	esp_zb_zcl_ias_zone_enroll_request_cmd_t cmd_req;
//	cmd_req.zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION;
//	cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
//	cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = message->info.dst_address;
//	cmd_req.zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint;
//	cmd_req.zcl_basic_cmd.src_endpoint = message->info.dst_endpoint;
//	esp_zb_zcl_ias_zone_enroll_cmd_req(&cmd_req);
	esp_zb_zcl_ias_zone_enroll_response_cmd_t cmd_resp;
	cmd_resp.zone_id = 20;
	cmd_resp.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
	cmd_resp.zcl_basic_cmd.dst_addr_u.addr_short = message->info.dst_address,
	cmd_resp.zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
	cmd_resp.zcl_basic_cmd.src_endpoint = message->info.dst_endpoint,
	cmd_resp.enroll_rsp_code = ESP_ZB_ZCL_IAS_ZONE_ENROLL_RESPONSE_CODE_SUCCESS;
	esp_zb_zcl_ias_zone_enroll_cmd_resp(&cmd_resp);

	cmd_resp.zone_id = 	ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION;
	esp_zb_zcl_ias_zone_enroll_cmd_resp(&cmd_resp);
    return ret;
}

bool my_raw_command_handler(uint8_t bufid)
{
    // Example: Log that a raw command was received
    printf("Raw Zigbee command received. bufid: %d\n", bufid);
    // Return true if the command was handled, false otherwise
    return false;
}

// Define the callback function
void zcl_command_send_status_callback(const esp_zb_zcl_command_send_status_message_t *status_msg)
{
    printf("ZCL Command Send Status:\n");
    printf("  Status: %s\n", status_msg->status == ESP_OK ? "Success" : "Failure");
    printf("  Transaction Sequence Number: %d\n", status_msg->tsn);
    printf("  Source Endpoint: %d\n", status_msg->src_endpoint);
    printf("  Destination Endpoint: %d\n", status_msg->dst_endpoint);
}



static esp_err_t zb_cmd_privilege_command_handler(const esp_zb_zcl_custom_cluster_command_message_t *message){
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
   
    ESP_LOGI(TAG, "cluster(0x%x), command(0x%x), dst_ep(%d)", message->info.cluster, message->info.command.id, message->info.dst_endpoint);


    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
		ret = zb_read_attribute_response_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
    	break;
    case ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_ENROLL_REQUEST_ID:
		ret = zb_ias_zone_enroll_handler((esp_zb_zcl_ias_zone_enroll_request_message_t *)message);
    	break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_STATUS_CHANGE_NOT_ID:
    	ret = zb_ias_zone_attribute_handler((esp_zb_zcl_ias_zone_status_change_notification_message_t *) message);
    	break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
    	ret = zb_command_report_response_handler((esp_zb_zcl_cmd_config_report_resp_message_t *) message);
    	break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
    	ret = zb_default_response_cb_handler((esp_zb_zcl_cmd_default_resp_message_t *) message);
    	break;
    case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID:
		ret = zb_write_attr_response_cb_handler((esp_zb_zcl_cmd_write_attr_resp_message_t *) message);
		break;
	case ESP_ZB_CORE_CMD_PRIVILEGE_COMMAND_REQ_CB_ID:
		ret = zb_cmd_privilege_command_handler(message);
		break;  	
	case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
		ret = zb_custom_cluster_cmd_cb_handler((esp_zb_zcl_custom_cluster_command_message_t *) message);
		break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}


char *modules_to_json(const node_module_t *device_arr, uint16_t devices_count) {
    cJSON *json_array = cJSON_CreateArray();
    for (uint16_t i = 0; i < devices_count; ++i) {
        const node_module_t *node = &device_arr[i];
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "m", node->module_id);
        cJSON_AddNumberToObject(item, "app_id", node->zb_app_id);
        cJSON_AddBoolToObject(item, "active", node->is_active);

        // IEEE address as hex string
        char ieee_str[17] = {0};
        for (int j = 0; j < 8; ++j) {
            sprintf(ieee_str + j*2, "%02X", node->ieee_addr[j]);
        }
        cJSON_AddStringToObject(item, "ieee", ieee_str);
        cJSON_AddNumberToObject(item, "sa", node->short_addr);
        cJSON_AddNumberToObject(item, "epc", node->ep_counts);

        // Endpoints array
        cJSON *ep_array = cJSON_CreateArray();
        for (uint8_t j = 0; j < node->ep_counts; ++j) {
            cJSON *ep_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(ep_obj, "d", node->endpoint[j].dev_id);
            cJSON_AddNumberToObject(ep_obj, "mt", node->endpoint[j].dev_type);
            cJSON_AddStringToObject(ep_obj, "n", node->endpoint[j].dev_name);

            // Add device_status_handle_t fields
            cJSON *status_obj = cJSON_CreateObject();
            cJSON_AddBoolToObject(status_obj, "sta", node->endpoint[j].device_t.state);
            cJSON_AddNumberToObject(status_obj, "val", node->endpoint[j].device_t.level);
            cJSON_AddNumberToObject(status_obj, "col", node->endpoint[j].device_t.value);
            cJSON_AddItemToObject(ep_obj, "status", status_obj);

            cJSON_AddItemToArray(ep_array, ep_obj);
        }
        cJSON_AddItemToObject(item, "ep", ep_array);

        cJSON_AddStringToObject(item, "manuf", node->manuf_name);
        cJSON_AddStringToObject(item, "model", node->model_id);
        cJSON_AddStringToObject(item, "loc", node->loc_name);

        cJSON_AddItemToArray(json_array, item);
    }
    char *json_str = cJSON_PrintUnformatted(json_array);
    cJSON_Delete(json_array);
    return json_str; // Caller must free this string
}


char *modules_to_did_json(const node_module_t *device_arr, uint16_t devices_count) {
    cJSON *json_array = cJSON_CreateArray();
    for (uint16_t i = 0; i < devices_count; ++i) {
        const node_module_t *node = &device_arr[i];
        for (uint8_t j = 0; j < node->ep_counts; ++j) {
            cJSON *ep_obj = cJSON_CreateObject();

            cJSON_AddNumberToObject(ep_obj, "m", node->module_id);
            cJSON_AddNumberToObject(ep_obj, "sa", node->short_addr);

            cJSON_AddNumberToObject(ep_obj, "d", node->endpoint[j].dev_id);
            cJSON_AddNumberToObject(ep_obj, "mt", node->endpoint[j].dev_type);
            cJSON_AddStringToObject(ep_obj, "n", node->endpoint[j].dev_name);

            cJSON_AddNumberToObject(ep_obj, "val", node->endpoint[j].device_t.level);
            cJSON_AddItemToArray(json_array, ep_obj);
        }
    }
    char *json_str = cJSON_PrintUnformatted(json_array);
    cJSON_Delete(json_array);
    return json_str; // Caller must free this string
}

void send_all_data_to_serial(){
	char * json_str = modules_to_json(device_array, devices_count);
	printf("json_str_1:%s\n", json_str);
	send_serial(json_str);
}

void send_nuos_app_data_to_serial(){
	char * json_str = modules_to_did_json(device_array, devices_count);
	printf("json_str_2:%s\n", json_str);
	send_serial(json_str);
}

void read_device_info(){
	printf("--------read_device_info---------\n");
	memset(device_array, 0, sizeof(device_array));
	devices_count = 0;
	
	//clear_scene_nodes_nvs();

	read_all_devices_from_nvs(NULL);
	vTaskDelay(pdMS_TO_TICKS(100));

	remove_invalid_ieee_devices();

	remove_duplicate_devices();		

	// for(int k=0; k<devices_count; k++){
	// 	if(device_array[k].room_id != 255){
	// 		for(int j=0; j<device_array[k].ep_counts; j++){
	// 			start_zigbee_cluster_ep_binding(&device_array[k], j);
	// 		}
	// 	}
	// }
	print_devices_compact();

	load_scene_nodes();
	vTaskDelay(pdMS_TO_TICKS(100));
}



void esp_leave_callback(esp_zb_zdp_status_t zdo_status, void *user_ctx){

}



/* Note: Please select the correct console output port based on the development board in menuconfig */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
esp_err_t esp_zb_gateway_console_init(void)
{
    esp_err_t ret = ESP_OK;
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    esp_vfs_usb_serial_jtag_use_driver();
    esp_vfs_dev_uart_register();
    return ret;
}
#endif

#if(CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
static void esp_zb_gateway_update_rcp(void)
{
    /* Deinit uart to transfer UART to the serial loader */
    esp_zb_rcp_deinit();
    if (esp_rcp_update() != ESP_OK) {
        esp_rcp_mark_image_verified(false);
    }
    esp_restart();
}

static void esp_zb_gateway_board_try_update(const char *rcp_version_str)
{
    char version_str[RCP_VERSION_MAX_SIZE];
    if (esp_rcp_load_version_in_storage(version_str, sizeof(version_str)) == ESP_OK) {
        ESP_LOGI(TAG, "Storage RCP Version: %s", version_str);
        if (strcmp(version_str, rcp_version_str)) {
            ESP_LOGI(TAG, "*** NOT MATCH VERSION! ***");
            esp_zb_gateway_update_rcp();
        } else {
            ESP_LOGI(TAG, "*** MATCH VERSION! ***");
            esp_rcp_mark_image_verified(true);
        }
    } else {
        ESP_LOGI(TAG, "RCP firmware not found in storage, will reboot to try next image");
        esp_rcp_mark_image_verified(false);
        esp_restart();
    }
}

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t rcp_fw_conf = {
        .base_path = "/rcp_fw", .partition_label = "rcp_fw", .max_files = 10, .format_if_mount_failed = false
    };
    esp_vfs_spiffs_register(&rcp_fw_conf);
    return ESP_OK;
}
#endif
#define COMMISSIONING_TIMEOUT_MS (ZIGBEE_NW_REMAIN_OPEN_IN_SECS * 1000) // 2 minutes in milliseconds

static TimerHandle_t commissioning_timer = NULL;
static int commissioning_timer_counts = 0;
static bool gpio_state = false;

static void commissioning_timer_callback(TimerHandle_t xTimer) {
    
	if(commissioning_timer_counts++ >= COMMISSIONING_TIMEOUT_MS){
		ESP_LOGI(TAG, "Commissioning timeout expired. Stopping commissioning.");
		
		nuos_stop_commissioning();
	}else{
		gpio_state = !gpio_state;
        gpio_set_level(gpio_touch_led_pins[1], gpio_state);
	}
    
}

void nuos_start_commissioning() {
    if (commissioning_timer == NULL) {
        commissioning_timer = xTimerCreate(
            "commissioning_timer",
            pdMS_TO_TICKS(1000),
            pdTRUE, // pdTRUE = continuous timer, pdFALSE = one-shot timer
            NULL,
            commissioning_timer_callback
        );
        if (commissioning_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create commissioning timer");
            return;
        }
    }

	commissioning_timer_counts = 0;
    // Start or reset timer
    if (xTimerIsTimerActive(commissioning_timer)) {
        xTimerReset(commissioning_timer, 0);
        ESP_LOGI(TAG, "Commissioning timer reset.");
    } else {
        xTimerStart(commissioning_timer, 0);
        ESP_LOGI(TAG, "Commissioning timer started.");
    }

    if (esp_zb_bdb_is_factory_new()) {
        ESP_LOGI(TAG, "Start network formation");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
    } else {
        esp_zb_bdb_open_network(ZIGBEE_NW_REMAIN_OPEN_IN_SECS);
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
    }
}

void nuos_stop_commissioning() {
    if (commissioning_timer != NULL) {
		commissioning_timer_counts = 0;
		gpio_set_level(gpio_touch_led_pins[1], 0);
        xTimerStop(commissioning_timer, 0);
        xTimerDelete(commissioning_timer, 0);
        commissioning_timer = NULL;
        ESP_LOGI(TAG, "Commissioning timer stopped and deleted.");
    }

    esp_zb_bdb_cancel_formation();
    esp_zb_bdb_close_network();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}



void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
	esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    bool device_left_flag = false;

    switch (sig_type) {
		case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
	//	#if CONFIG_EXAMPLE_CONNECT_WIFI
	//    	example_connect();
	//		#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
	//			esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
	//			esp_coex_wifi_i154_enable();
	//		#else
	//			esp_wifi_set_ps(WIFI_PS_NONE);
	//		#endif
	//      start_http_server();
	//	#endif
			ESP_LOGI(TAG, "Initialize Zigbee stack");
			esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
			break;
		case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
		case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
			if (err_status == ESP_OK) {
				switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler);
				ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
				esp_zb_secur_link_key_exchange_required_set(false);
				if (esp_zb_bdb_is_factory_new()) {
					ESP_LOGI(TAG, "Start network formation");
					esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
				} else {
					ESP_LOGI(TAG, "Device rebooted");
					//esp_zb_bdb_cancel_formation();
					//esp_zb_bdb_close_network();
					esp_zb_bdb_open_network(ZIGBEE_NW_REMAIN_OPEN_IN_SECS);
					read_device_info();
					
					xTaskCreate(find_zigbee_nodes_task, "find_node_task", 4096, NULL, TASK_PRIORITY_ZB_FIND_NODES, NULL);
				}
			} else {
				ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
			}
			break;

		case ESP_ZB_BDB_SIGNAL_FORMATION:
			if (err_status == ESP_OK) {
				esp_zb_ieee_addr_t ieee_address;
				esp_zb_get_long_address(ieee_address);
				ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
						ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
						ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
						esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
				// esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
			} else {
				ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
				// esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
			}
			break;

		case ESP_ZB_BDB_SIGNAL_STEERING:
			if (err_status == ESP_OK) {
				ESP_LOGI(TAG, "Network steering started");
			}
			break;

		case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
			dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
			ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
			nuos_stop_commissioning();
			if(is_record_already_exists_in_nvs(dev_annce_params->device_short_addr) == ESP_ERR_NVS_NOT_FOUND){
				esp_zb_zdo_match_desc_req_param_t find_req2;
				uint16_t cluster_list2[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, 
					ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 
					ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
					ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, 
					ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
					0xFC31,
					ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
					ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE,
					ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, 
					ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
					ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, 
					ESP_ZB_ZCL_CLUSTER_ID_POLL_CONTROL};
				find_req2.num_in_clusters = 10;
				find_req2.num_out_clusters = 1;

				find_req2.dst_nwk_addr = dev_annce_params->device_short_addr;
				find_req2.addr_of_interest = dev_annce_params->device_short_addr;
				find_req2.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
				find_req2.cluster_list = cluster_list2;
				esp_zb_zdo_match_cluster(&find_req2, user_find_cb, NULL);
			}else{
				uint8_t k = find_node_index_by_short_addr(dev_annce_params->device_short_addr);
				send_module_ctrl_over_uart(&device_array[k], 0, 0);
			}
			break;

		case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
			if (err_status == ESP_OK) {
				if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
					ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
				} else {
					ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
				}
			}
			break;
		case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
			ESP_LOGI(TAG, "Production configuration is ready");
			if (err_status == ESP_OK) {
				app_production_config_t *prod_cfg = (app_production_config_t *)esp_zb_app_signal_get_params(p_sg_p);
				if (prod_cfg->version == APP_PROD_CFG_CURRENT_VERSION) {
					ESP_LOGI(TAG, "Manufacturer_code: 0x%x, manufacturer_name:%s", prod_cfg->manuf_code, prod_cfg->manuf_name);
					esp_zb_set_node_descriptor_manufacturer_code(prod_cfg->manuf_code);
				}
			} else {
				ESP_LOGW(TAG, "Production configuration is not present");
			}
			break;

		case ESP_ZB_ZDO_SIGNAL_LEAVE:         
			// Attempt to rejoin the network
			leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
			
			if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {  // Leave without rejoin -> Factory reset
				printf("Leave without rejoin, factory reset the device\n");
				device_left_flag = true;
			} else {  // Leave with rejoin -> Rejoin the network, only reboot the device
				printf("Leave with rejoin, only reboot the device\n");
			} 
        break;

		case ESP_ZB_ZDO_SIGNAL_LEAVE_INDICATION:
			esp_zb_zdo_signal_leave_indication_params_t *leave_params = (esp_zb_zdo_signal_leave_indication_params_t *)esp_zb_app_signal_get_params(p_sg_p);
			ESP_LOGI(TAG, "leave_type-> rejoin:%d short_addr:0x%x", leave_params->rejoin, leave_params->short_addr);

	
			//uint8_t node_index = find_node_index_by_short_addr(leave_params->short_addr);
			uint8_t node_index = find_node_index_by_ieee_addr(leave_params->device_addr);
			if(node_index == -1){
				ESP_LOGI(TAG, "Device with short address 0x%04hx not found in device array", leave_params->short_addr);
				break;
			}else{
				device_left_flag = true;
				send_module_remove_device_on_uart(device_array[node_index].room_id, device_array[node_index].module_id, device_array[node_index].ieee_addr);
				vTaskDelay(pdMS_TO_TICKS(100));
				gateway_leave_device(leave_params->short_addr, device_array[node_index].ieee_addr, false);
				if(remove_zigbee_device_from_nvs(leave_params->short_addr) == ESP_OK){
					printf("Device with short address 0x%04hx removed from NVS\n", leave_params->short_addr);
				}							
			}
			
		break;

		case ESP_ZB_ZDO_SIGNAL_DEVICE_UPDATE:
			esp_zb_zdo_signal_device_update_params_t *update_params = (esp_zb_zdo_signal_device_update_params_t *)esp_zb_app_signal_get_params(p_sg_p);
			printf("Device update signal received\n");
			if(device_left_flag){
				device_left_flag = false;
				ESP_LOGI(TAG, "device_left-> short_addr:0x%x", update_params->short_addr);
				//uint8_t node_index = find_node_index_by_short_addr(update_params->short_addr);
				uint8_t node_index = find_node_index_by_ieee_addr(update_params->long_addr);
				if(node_index == -1){
					ESP_LOGI(TAG, "Device with short address 0x%04hx not found in device array", leave_params->short_addr);
					break;
				}else{
					ESP_LOGI(TAG, "Device with short address 0x%04hx found at index %d", update_params->short_addr, node_index);
					send_module_remove_device_on_uart(device_array[node_index].room_id, device_array[node_index].module_id, device_array[node_index].ieee_addr);
					gateway_leave_device(update_params->short_addr, device_array[node_index].ieee_addr, false);
					if(remove_zigbee_device_from_nvs(update_params->short_addr) == ESP_OK){
						printf("Device with short address 0x%04hx removed from NVS\n", update_params->short_addr);
					}
				}
			
			}else{
				ESP_LOGI(TAG, "device_update-> status:%d short_addr:0x%x", update_params->status, update_params->short_addr);
			}
			
		break;
		case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:

			break;

		default:
			ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
					esp_err_to_name(err_status));
			break;
    }
}

void rcp_error_handler(void)
{
#if(CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
    ESP_LOGI(TAG, "Re-flashing RCP");
    esp_zb_gateway_update_rcp();
#endif
    esp_restart();
}

#if CONFIG_OPENTHREAD_SPINEL_ONLY
static esp_err_t check_ot_rcp_version(void)
{
    char internal_rcp_version[RCP_VERSION_MAX_SIZE];
    ESP_RETURN_ON_ERROR(esp_radio_spinel_rcp_version_get(internal_rcp_version, ESP_RADIO_SPINEL_ZIGBEE), TAG, "Fail to get rcp version from radio spinel");
    ESP_LOGI(TAG, "Running RCP Version: %s", internal_rcp_version);
#if(CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
    esp_zb_gateway_board_try_update(internal_rcp_version);
#endif
    return ESP_OK;
}
#endif

//esp_zb_bdb_reset_via_local_action(void);
#define TUYA_CLUSTER_ID 				0xEF00  // Most Tuya devices use this custom cluster

bool zb_apsde_data_indication_handler(esp_zb_apsde_data_ind_t ind)
{
    bool processed 				= false;
	static uint16_t last_src 	= 0;
	static uint8_t last_seq 	= 0xFF;

	if (ind.src_short_addr == last_src && ind.asdu[1] == last_seq) {
		// Duplicate, ignore
		return false;
	}
	last_src = ind.src_short_addr;
	last_seq = ind.asdu[1];

    if (ind.status != 0x00) {
        ESP_LOGE("APSDE INDICATION", "Invalid status of APSDE-DATA indication, error code: %d", ind.status);
        return false;
    }
    // Only process messages for endpoint 1, Home Automation profile, and On/Off or Tuya clusters
    if (ind.dst_endpoint != ESP_ZB_GATEWAY_ENDPOINT || ind.profile_id != ESP_ZB_AF_HA_PROFILE_ID ||
        (ind.cluster_id != 0x0006 && ind.cluster_id != TUYA_CLUSTER_ID)) {
        return false;
    }
    // Print basic info
    ESP_LOGI("APSDE INDICATION",
             "Received APSDE-DATA %s request with a length of %lu from endpoint %d, source address 0x%04hx to "
             "endpoint %d, destination address 0x%04hx",
             ind.dst_addr_mode == 0x01 ? "group" : "unicast", ind.asdu_length, ind.src_endpoint,
             ind.src_short_addr, ind.dst_endpoint, ind.dst_short_addr);
    ESP_LOG_BUFFER_HEX_LEVEL("APSDE INDICATION", ind.asdu, ind.asdu_length, ESP_LOG_INFO);

    // Check payload length for ZCL header
    if (ind.asdu_length < 3) {
        ESP_LOGW("APSDE INDICATION", "ASDU too short for ZCL command");
        return false;
    }

    uint8_t frame_control 	= ind.asdu[0];
    uint8_t seq_number   	= ind.asdu[1];
    uint8_t command_id   	= ind.asdu[2];
	uint8_t data 			= ind.asdu[3];
    // Tuya devices: command_id is typically 0x00 (set data), 0x01 (get data), 0x02 (report), 0x03 (status report), etc.
    switch (command_id) {
        case 0x00: // Tuya set data request
            ESP_LOGI("APSDE INDICATION", "Tuya Set Data Request (0x00) received");
            // Parse and handle Tuya set data here
            processed = true;
            break;
        case 0x01: // Tuya get data request
            ESP_LOGI("APSDE INDICATION", "Tuya Get Data Request (0x01) received");
            // Parse and handle Tuya get data here
            processed = true;
            break;
        case 0x02: // Tuya report data
            ESP_LOGI("APSDE INDICATION", "Tuya Report Data (0x02) received");
            // Parse and handle Tuya report data here
            processed = true;
            break;
        case 0x03: // Tuya status report
            ESP_LOGI("APSDE INDICATION", "Tuya Status Report (0x03) received");
            // Parse and handle Tuya status report here
            processed = true;
            break;
        case 0x04: // Tuya command response
            ESP_LOGI("APSDE INDICATION", "Tuya Command Response (0x04) received");
            // Parse and handle Tuya command response here
            processed = true;
            break;
        case 0x05: // Tuya heartbeat
            ESP_LOGI("APSDE INDICATION", "Tuya Heartbeat (0x05) received");
            // Parse and handle Tuya heartbeat here
            processed = true;
            break;
        case 0x06: // Tuya device info
            ESP_LOGI("APSDE INDICATION", "Tuya Device Info (0x06) received");
            // Parse and handle Tuya device info here
            processed = true;
            break;
        case 0xfd: // Tuya wireless scene switch
            ESP_LOGI("APSDE INDICATION", "Tuya Device Info (0xfd) received");
            // Parse and handle Tuya device info here
            processed = true;
			if(ind.asdu_length == 4){
				switch(data){
					case 0:  //single press
					printf("SINGLE KEY_%d PRESSED\n!!", ind.src_endpoint);
					break;
					case 1:  //double press
					printf("DOUBLE KEY_%d PRESS\n!!", ind.src_endpoint);
					break;
					case 2:  //long press
					printf("LONG KEY_%d PRESS\n!!", ind.src_endpoint);
					break;	
					default:
					break;									
				}
			}
            break;			
        // Add more cases as needed for your Tuya devices
        default:
            ESP_LOGW("APSDE INDICATION", "Unknown Tuya/cluster command ID: 0x%02X", command_id);
            // Optionally print payload for debugging
            ESP_LOG_BUFFER_HEX_LEVEL("APSDE INDICATION", ind.asdu, ind.asdu_length, ESP_LOG_INFO);
            break;
    }

    return processed;
}

    static int zb_zcl_custom_cluster_check_value_handler(uint16_t attr_id, uint8_t endpoint, uint8_t *value)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Hook for checking custom cluster attribute validity");
    return ret;
}

static void zb_zcl_custom_cluster_write_attr_handler(uint8_t endpoint, uint16_t attr_id, uint8_t *new_value, uint16_t manuf_code)
{
    ESP_LOGI(TAG, "Hook for indicating which attribute will be written");
}




static uint8_t custom_payload[2] = {0};   // MUST be static or heap

void write_custom_cluster_attr(uint16_t cluster_id, uint8_t custom_cmd_id, uint16_t dst_short_addr, uint8_t dst_ep, uint8_t custom_value)
{
    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {0};

	custom_payload[0] = custom_cmd_id;
	custom_payload[1] = custom_value;

    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.cluster_id   = cluster_id;
    cmd_req.profile_id   = ESP_ZB_AF_HA_PROFILE_ID;

    cmd_req.custom_cmd_id = 0;   // your custom command ID
    cmd_req.direction     = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;

    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;

    /* Manufacturer specific REQUIRED for custom clusters */
    cmd_req.manuf_specific = 0;//1;
    cmd_req.manuf_code     = 0;//0x1217;   // use your manufacturer ID
    cmd_req.dis_defalut_resp = 1;

    /* RAW payload */
	cmd_req.data.type = ESP_ZB_ZCL_ATTR_TYPE_U16;
    cmd_req.data.value = &custom_payload;
    cmd_req.data.size  = 2;

    esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);

}



esp_zb_attribute_list_t* nuos_zb_create_group_cluster(){
    //esp_zb_attribute_list_t *esp_zb_groups_cluster =  esp_zb_groups_cluster_create(NULL);
    esp_zb_groups_cluster_cfg_t group_cfg = {
        .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE
    };
    esp_zb_attribute_list_t *esp_zb_groups_cluster =  esp_zb_groups_cluster_create(&group_cfg);
    return esp_zb_groups_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_scene_cluster(){
    /* scenes cluster create with standard cluster + customized */
    //
    // #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
	esp_zb_scenes_cluster_cfg_t scene_cfg =
	{
		.scenes_count = 0,
		.current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
		.current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
		.scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
		.name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE,
	};
    esp_zb_attribute_list_t *esp_zb_scene_cluster = esp_zb_scenes_cluster_create(&scene_cfg);
    return esp_zb_scene_cluster;
}


static void esp_zb_task(void *pvParameters)
{
#if CONFIG_OPENTHREAD_SPINEL_ONLY
    esp_radio_spinel_register_rcp_failure_handler(rcp_error_handler, ESP_RADIO_SPINEL_ZIGBEE);
#endif
    esp_zb_overall_network_size_set(MAX_CHILDREN);
	const uint16_t table_size = MAX_CHILDREN;
    ESP_ERROR_CHECK(esp_zb_aps_src_binding_table_size_set(table_size));
    ESP_ERROR_CHECK(esp_zb_aps_dst_binding_table_size_set(table_size));
	esp_zb_nwk_set_max_children(table_size);
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
	
#if CONFIG_OPENTHREAD_SPINEL_ONLY
    ESP_ERROR_CHECK(check_ot_rcp_version());
#endif

	//ESP_ERROR_CHECK(esp_zb_zcl_scenes_table_set_size(table_size));
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ESP_ZB_GATEWAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_HOME_GATEWAY_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_attribute_list_t *basic_cluser = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_attribute_list_t *esp_zb_on_off_server_cluster = esp_zb_on_off_cluster_create(NULL);
	esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_server_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    uint8_t value = 0;
	// esp_zb_attribute_list_t *esp_zb_on_off_server_cluster_1 = esp_zb_on_off_cluster_create(NULL);
	// esp_zb_attribute_list_t *esp_zb_on_off_server_cluster_1 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
	// esp_zb_cluster_add_attr(esp_zb_on_off_server_cluster_1, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, 0xFD00,
	// 							ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &value);
	// esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_server_cluster_1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xEF00);
	esp_zb_cluster_list_add_custom_cluster(cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 

    esp_zb_attribute_list_t *esp_zb_tuya_private_cluster_1 = esp_zb_zcl_attr_list_create(0xE000);
	// esp_zb_cluster_add_attr(esp_zb_tuya_private_cluster_1, 0xE000, 0xFD00,
	// 							ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &value);
	esp_zb_cluster_list_add_custom_cluster(cluster_list, esp_zb_tuya_private_cluster_1, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE); 

	uint8_t ss = 0;
    esp_zb_attribute_list_t *esp_zb_tuya_private_cluster2 = esp_zb_zcl_attr_list_create(0xFC31);
	esp_zb_cluster_list_add_custom_cluster(cluster_list, esp_zb_tuya_private_cluster2, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

	esp_zb_attribute_list_t *ias_zone_cfg = esp_zb_ias_zone_cluster_create(NULL);
    esp_zb_cluster_list_add_ias_zone_cluster(cluster_list, ias_zone_cfg, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_attribute_list_t *temp_cluser = esp_zb_temperature_meas_cluster_create(NULL);
	esp_zb_attribute_list_t *humidity_cluser = esp_zb_humidity_meas_cluster_create(NULL);
	esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, humidity_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_attribute_list_t* esp_zb_groups_cluster = nuos_zb_create_group_cluster();
	esp_zb_cluster_list_add_groups_cluster(cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
	printf("Created Cluster Group\n");
	esp_zb_attribute_list_t *esp_zb_scenes_cluster = nuos_zb_create_scene_cluster();
	esp_zb_cluster_list_add_scenes_cluster(cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);  
	printf("Created Cluster Scene\n");
	
    esp_zb_ep_list_add_gateway_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);
	esp_zb_zcl_add_privilege_command(1, 0xE000, 0XFD);
    esp_zb_core_action_handler_register(zb_action_handler);
	//esp_zb_aps_data_indication_handler_register(zb_apsde_data_indication_handler);
	//esp_zb_factory_reset();
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    esp_rcp_update_deinit();

    vTaskDelete(NULL);
}

// void uart_evt_task(void *arg)
// {
//     node_module_evt_t evt;

//     while (1) {
//         if (xQueueReceive(send_to_uart_evt_queue, &evt, pdMS_TO_TICKS(100))) {
// 			send_module_ctrl_over_uart(&evt.module, 1, evt.index); 
//         }
//     }
// }



void app_main(void)
{
	nuos_zb_init_hardware();
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
	#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
		ESP_ERROR_CHECK(esp_zb_gateway_console_init());
	#endif

#if(CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
    esp_rcp_update_config_t rcp_update_config = ESP_ZB_RCP_UPDATE_CONFIG();
    ESP_ERROR_CHECK(init_spiffs());
    ESP_ERROR_CHECK(esp_rcp_update_init(&rcp_update_config));
#endif
    // Initialize the semaphore before starting callbacks
    task_done_semaphore = xSemaphoreCreateBinary();
    //Added by Nuos
    init_timer();
    //read_device_info();
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, TASK_PRIORITY_ZB_MAIN, NULL);


    // send_to_uart_evt_queue = xQueueCreate(20, sizeof(node_module_evt_t));
    // assert(send_to_uart_evt_queue);
	// xTaskCreate(uart_evt_task, "send_evt_task", 4096, NULL, TASK_PRIORITY_UART_EVT, NULL);

}

