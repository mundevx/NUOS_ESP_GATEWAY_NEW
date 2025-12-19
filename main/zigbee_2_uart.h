

#ifndef ZIGBEE_2_UART_H
#define ZIGBEE_2_UART_H

#include "nuos_data_structs.h"
#ifdef __cplusplus
extern "C" {
#endif
    extern void uart_init(void);
    extern int uart_putc(uint8_t byte);    
    extern int uart_put_all(uint8_t *byte, uint8_t len);
    extern int uart_send_bytes_to_gateway(uint8_t *byte, uint8_t len);

#ifdef __cplusplus
    }
#endif                      
#endif // ZIGBEE_2_UART_H
