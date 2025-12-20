
//#pragma once

#include "nuos_data_structs.h"

#ifndef _NUOS_COMMON_
#define _NUOS_COMMON_

#ifdef _DECLARE_MAIN_


#define MAX_SWITCHES_TO_CONTROL    					2
#define MAX_LOAD_TO_CONTROL    						1
#define ZIGBEE_NW_REMAIN_OPEN_IN_SECS               60

#ifdef __cplusplus
extern "C" {
#endif
// Function to find device by short_addr
node_module_t *find_device_by_short_addr(uint16_t target_short_addr);
bool find_device_by_dev_id(uint16_t module_id, uint16_t target_dev_id, uint8_t* nodeindex, uint8_t* epindex);
int find_node_index_by_short_addr(uint16_t target_short_addr);

void nuos_update_level_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
void nuos_update_onoff_cluster(uint8_t node_i, uint8_t ep_i, bool state);
void nuos_update_color_temperature(uint8_t node_i, uint8_t ep_i, uint16_t temp);
void nuos_update_color_hue(uint8_t node_i, uint8_t ep_i, uint16_t hue);

uint16_t find_device_type(uint8_t nodeindex, uint8_t epindex);
uint8_t get_total_endpoints(uint8_t nodeindex);
void nuos_start_commissioning();
void nuos_stop_commissioning();


//////////////////////////

esp_err_t nvs_store_or_update_scene(const device_scene_handle_t *new_scene);
esp_err_t nvs_read_scene_array(device_scene_handle_t *scene_handle);
esp_err_t nvs_read_scene_by_id(uint16_t group_id, uint8_t scene_id, device_scene_handle_t *scene);
esp_err_t nvs_delete_scene(uint16_t group_id, uint8_t scene_id);

void send_nuos_app_data_to_serial();

bool lock_nvs_save									= false;


void read_device_info();
void set_commissioning_task(void* arg) ;


void send_serial(const char* data);
void send_uart_command(node_module_t *node, uint8_t ep, const char *cmd, uint8_t val);

#ifdef __cplusplus
}
#endif
device_scene_handle_t  scene_data_t[SCENE_ARRAY_SIZE];
#else

#ifdef __cplusplus
extern "C" {
#endif
// Function to find device by short_addr
extern node_module_t *find_device_by_short_addr(uint16_t target_short_addr);
extern bool find_device_by_dev_id(uint16_t module_id, uint16_t target_dev_id, uint8_t* nodeindex, uint8_t* epindex);
extern int find_node_index_by_short_addr(uint16_t target_short_addr);

extern void nuos_update_level_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
extern void nuos_update_onoff_cluster(uint8_t node_i, uint8_t ep_i, bool state);
extern void nuos_update_color_temperature(uint8_t node_i, uint8_t ep_i, uint16_t temp);
extern void nuos_update_color_hue(uint8_t node_i, uint8_t ep_i, uint16_t hue);

extern uint16_t find_device_type(uint8_t nodeindex, uint8_t epindex);
extern uint8_t get_total_endpoints(uint8_t nodeindex);
extern void nuos_start_commissioning();
extern void nuos_stop_commissioning();


//////////////////////////



void send_nuos_app_data_to_serial();


extern bool lock_nvs_save;

extern void send_all_data_to_serial();
extern void read_device_info();
// extern void nuos_zb_leave_device(uint16_t module_id, uint16_t did);
extern void zigbee_cluster_fan_control_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t speed);
extern void zigbee_cluster_thermostat_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t temp);
extern bool is_fan_control(uint8_t nodeindex, uint8_t epindex);
extern bool is_thermostat_control(uint8_t nodeindex, uint8_t epindex);
extern void nuos_update_curtain_cluster(uint8_t node_i, uint8_t ep_i, uint8_t curtain_state);

extern void send_serial(const char* data);
extern void send_uart_command(node_module_t *node, uint8_t ep, const char *cmd, uint8_t val);

extern void read_device_info();
extern void set_commissioning_task(void* arg) ;
#ifdef __cplusplus
}
#endif
extern device_scene_handle_t  scene_data_t[SCENE_ARRAY_SIZE];
#endif
#endif
