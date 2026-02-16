

#ifndef FRAME_ENC_DEC_H
#define FRAME_ENC_DEC_H

#include "nuos_data_structs.h"



#define EP_RESPONSE_TIMEOUT_MS   600
#define MODULE_TIMEOUT_MS       3000

#ifdef __cplusplus
extern "C" {
#endif


    bool is_zigbee_busy();
    zb_state_t is_zigbee_state();
    void set_zigbee_state(zb_state_t state);
    void is_endpoint_matches(uint8_t end_point);
    
    void process_uart_data(const char* data, uint16_t len, esp_zb_ieee_addr_t ieee_addr, uint8_t* original_frame, uint8_t original_frame_length);
    void send_uart_command_in_array(node_module_t *node, const char *cmd);
    
    void send_module_ctrl_over_uart(node_module_t *node, int start_idx, int idx);                                                    
    //int uart_putc(uint8_t byte);    
    //int uart_put_all(const uint8_t *byte, const uint8_t len); 
    void send_module_remove_device_on_uart(uint8_t room_id, uint8_t module_id, esp_zb_ieee_addr_t ieee_addr);

    char* Fn_DECRYPT_DATA(const char* V_RECEIVED_DATA_STR);
    char* Fn_ENCRYPT_DATA(const char* V_RECEIVED_DATA_STR);
    void find_and_execute_ctrl_command(uint8_t room_id, uint8_t module_id, int didx, int *val, bool isArray);
    void process_decrypt_uart_receiver_data(const uint8_t* data, uint16_t len);
    extern uint8_t get_seq_ep_count();
    extern uint8_t get_seq_level();
    extern uint8_t get_seq_value();

    extern void send_module_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_ir_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_motion_sensor_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_remove_motion_sensor_on_uart(uint8_t room_id, uint8_t module_id, esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_door_sensor_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_temp_humidity_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_lux_sensor_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_gas_leak_sensor_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_rgbw_light_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void send_module_scene_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);
    extern void init_uart_queue();
    extern void call_on_attribute_handler_callback();

    extern void set_res_active_seq_response_received(bool received);
    extern bool is_res_active_seq_response_completed();
    extern uint8_t get_max_endpoint_in_active_seq();
    extern bool is_scene_add_request();
    extern uint8_t find_index();
    extern bool is_zigbee_busy();
    extern uint8_t get_module_idx_count();
    extern uint16_t get_module_short_addr();
    extern uint8_t get_current_module_index();
    extern void set_zigbee_busy(bool val);
    extern int find_free_module_slot(void);
    extern void clear_module_slot(uint8_t idx);
    extern void start_next_module_sequence(void);
    extern void send_next_endpoint_cmd(void);
    extern esp_err_t call_default_task();
    extern void update_device_handling_task(uint16_t short_addr, uint8_t end_point);
#ifdef __cplusplus
    }
#endif                      
#endif // FRAME_ENC_DEC_H
