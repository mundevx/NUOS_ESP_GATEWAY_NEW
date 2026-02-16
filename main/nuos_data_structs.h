
#include <esp_zigbee_type.h>

#ifndef _NUOS_DATA_
    #define _NUOS_DATA_


    #define MAX_DEVICES_IN_MODULE               6
    #define MAX_STR_LEN                         24
    #define SCENE_ARRAY_SIZE                    100

    // Structure linking room/module to group/scene
    typedef struct {
        uint8_t room_id;
        uint8_t module_id;
        uint8_t device_id;
        uint16_t group_id;
        uint8_t scene_id;
    } node_mapping_t;

    
    typedef struct device_ctx_s
    {
        bool state;          	// true or false
        bool last_state;
        uint8_t level;       	// brightness
        uint8_t last_level;
        uint32_t value; 		// rgb color
        uint32_t last_value;
    } device_status_handle_t;   //

    typedef struct endpoint_s { //device
        uint16_t dev_id;         // to be assigned by mobile app
        uint8_t  ep;
        uint16_t  dev_type;
        uint8_t cluster_counts;
        uint16_t cluster_id[12]; //take max 12 clusters
        bool is_bind[12];
        char dev_name[24];
        device_status_handle_t device_t;
        bool scene_add_command_flag;
    }endpoint_t;

    typedef struct node_module_s {
        uint8_t room_id;
        uint8_t module_id;
        uint16_t zb_app_id;
        bool is_active;
        bool in_scene;
        esp_zb_ieee_addr_t ieee_addr;
        uint16_t short_addr;
        uint8_t ep_counts;
        endpoint_t endpoint[MAX_DEVICES_IN_MODULE]; //devices
        char manuf_name[MAX_STR_LEN];
        char model_id[MAX_STR_LEN];
        char loc_name[MAX_STR_LEN];
    } node_module_t;


    typedef enum {
        ZB_IDLE,
        ZB_SEND_EP,
        ZB_WAIT_EP,
        ZB_MODULE_DONE
    } zb_state_t;


    #define MAX_MODULES             40
    #define MAX_EP                  6

   #define MAX_MODULES 40
#define MAX_EP      6

#define MAX_MODULES 40
#define MAX_EP      6

typedef struct {
    uint8_t room_id;
    uint8_t module_id;
    uint16_t short_addr;        // target device short address (for verification)
    uint8_t total;              // number of endpoints to process
    uint8_t current;            // next endpoint index to send
    int values[MAX_EP];         // level values for each endpoint
    uint8_t dids[MAX_EP];       // endpoint IDs (EP numbers)
    uint16_t node_type;         // device type of this module
    uint32_t last_tx_time_ms;   // timestamp of last sent command (per module)
    bool response_received[MAX_EP];
    bool completed;             // ← ADD
    //SemaphoreHandle_t done_sem;
} zb_module_seq_t;

// typedef struct {
//     uint8_t room_id[MAX_MODULES];
//     uint8_t module_id[MAX_MODULES];
//     uint16_t short_addr[MAX_MODULES];        // target device short address (for verification)
//     uint8_t total[MAX_MODULES];              // number of endpoints to process
//     uint8_t current[MAX_MODULES];            // next endpoint index to send
//     int values[MAX_MODULES][MAX_EP];         // level values for each endpoint
//     uint8_t dids[MAX_MODULES][MAX_EP];       // endpoint IDs (EP numbers)
//     uint16_t node_type[MAX_MODULES];         // device type of this module
//     uint32_t last_tx_time_ms[MAX_MODULES];   // timestamp of last sent command (per module)
//     bool response_received[MAX_MODULES][MAX_EP];
//     bool completed[MAX_MODULES];             // ← ADD
// } zb_module_seq_t;
#endif
