

#ifndef FRAME_ENC_DEC_H
#define FRAME_ENC_DEC_H

#include "nuos_data_structs.h"



#ifdef __cplusplus
extern "C" {
#endif

    void process_uart_data(const char* data, uint16_t len, esp_zb_ieee_addr_t ieee_addr, uint8_t* original_frame, uint8_t original_frame_length);

    void send_uart_command_in_array(node_module_t *node, const char *cmd);
    // void send_module_addc_over_uart(int room_id,
    //                                 const node_module_t *module,
    //                                 int start_idx);
    extern void send_module_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr);

    void send_module_ctrl_over_uart(node_module_t *node, int start_idx, int idx); 
                                    
                                    
    int uart_putc(uint8_t byte);    
    int uart_put_all(uint8_t *byte, uint8_t len); 

    void send_module_remove_device_on_uart(uint8_t room_id, uint8_t module_id, esp_zb_ieee_addr_t ieee_addr);

    void decrypt_single_block(const char* base64_encrypted);
    char* Fn_DECRYPT_DATA(const char* V_RECEIVED_DATA_STR);
    char* Fn_ENCRYPT_DATA(const char* V_RECEIVED_DATA_STR);

    void find_and_execute_ctrl_command(uint8_t room_id, uint8_t module_id, int* didx, int *val, bool isArray);

    void process_decrypt_uart_receiver_data( uint8_t* data, uint16_t len);

#ifdef __cplusplus
    }
#endif                      
#endif // FRAME_ENC_DEC_H
