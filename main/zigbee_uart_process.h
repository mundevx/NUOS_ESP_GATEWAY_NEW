

#ifndef FRAME_ENC_DEC_H
#define FRAME_ENC_DEC_H

#include "nuos_data_structs.h"

#define UART_MAX_PAYLOAD        512

typedef enum
{
    CTRL_NONE = 0,
    CTRL_SWITCH,
    CTRL_DIMMER,
    CTRL_FAN,
    CTRL_CURTAIN_POSITION,
    CTRL_CURTAIN_COMMAND,
    CTRL_RGB,
    CTRL_MULTI_SWITCH
} control_type_t;




typedef struct
{
    uint8_t id;
    bool power;
    uint8_t brightness;
    uint8_t speed;
    uint8_t position;
    char command[16];

    uint8_t r;
    uint8_t g;
    uint8_t b;

} endpoint_ctrl_t;

typedef struct
{
    control_type_t type;

    uint8_t endpoint_count;
    endpoint_ctrl_t endpoint[4];

} control_msg_t;

typedef enum
{
    JSON_TYPE_INVALID = 0,
    JSON_TYPE_GATEWAY_CMD,
    JSON_TYPE_CONTROL
} json_type_t;
typedef enum
{
CMD_UNKNOWN = 0,
CMD_GET_UNCONFIGURED_DEVICES,
CMD_DISCOVER,
CMD_GET_CONFIGURED_DEVICES
} gateway_cmd_t;

typedef struct
{
    json_type_t type;

    gateway_cmd_t gateway_cmd;

    control_msg_t control;

} uart_json_msg_t;

#define EP_RESPONSE_TIMEOUT_MS   600
#define MODULE_TIMEOUT_MS       3000

typedef struct {
    uint16_t len;
    uint8_t  data[UART_MAX_PAYLOAD];
} uart_rx_msg_t;

#ifdef __cplusplus
extern "C" {
#endif




#ifdef __cplusplus
    }
#endif  

// extern void on_device_joined_fxn(
//     uint16_t short_addr,
//     esp_zb_ieee_addr_t ieee_addr);

#endif // FRAME_ENC_DEC_H
