
#include <esp_zigbee_type.h>
#include <esp_err.h>
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
        uint16_t dummy_count;         // to be assigned by mobile app
        uint8_t  ep;
        uint16_t  dev_type;
        uint8_t cluster_counts;
        uint16_t cluster_id[12]; //take max 12 clusters
        bool is_bind[12];
        char dev_name[24];
        device_status_handle_t device_t;
        bool scene_add_command_flag;
    }endpoint_t;

    // typedef struct node_module_s {
    //     uint8_t room_id;
    //     uint8_t module_id;
    //     uint16_t zb_app_id;
    //     bool is_active;
    //     bool in_scene;
    //     esp_zb_ieee_addr_t ieee_addr;
    //     uint16_t short_addr;
    //     uint8_t ep_counts;
    //     endpoint_t endpoint[MAX_DEVICES_IN_MODULE]; //devices
    //     char manuf_name[MAX_STR_LEN];
    //     char model_id[MAX_STR_LEN];
    //     char loc_name[MAX_STR_LEN];
    // } node_module_t;

    typedef struct node_module_s {
        // ------------------------------------------------
        // Stable Zigbee identity
        // ------------------------------------------------
        esp_zb_ieee_addr_t ieee_addr;
        uint16_t short_addr;
        uint16_t zb_app_id;
        // ------------------------------------------------
        // User/application identity
        // ------------------------------------------------
        uint8_t room_id;
        uint8_t module_id;

        char device_name[MAX_STR_LEN];
        char loc_name[MAX_STR_LEN];
        // ------------------------------------------------
        // Manufacturer identity
        // ------------------------------------------------
        char manuf_name[MAX_STR_LEN];
        char model_id[MAX_STR_LEN];
        // ------------------------------------------------
        // Lifecycle
        // ------------------------------------------------
        bool is_active;
        bool in_scene;
        // ------------------------------------------------
        // Endpoints
        // ------------------------------------------------
        uint8_t ep_counts;
        endpoint_t endpoint[MAX_DEVICES_IN_MODULE];

    } node_module_t;

    typedef enum {
        ZB_IDLE,
        ZB_SEND_EP,
        ZB_WAIT_EP,
        ZB_MODULE_DONE
    } zb_state_t;


#define MAX_MODULES 40
#define MAX_EP      6

// typedef struct {
//     uint8_t room_id;
//     uint8_t module_id;
//     uint16_t short_addr;        // target device short address (for verification)
//     uint8_t total;              // number of endpoints to process
//     uint8_t current;            // next endpoint index to send
//     int values[MAX_EP];         // level values for each endpoint
//     uint8_t dids[MAX_EP];       // endpoint IDs (EP numbers)
//     uint16_t node_type[MAX_EP];         // device type of this module
//     uint32_t last_tx_time_ms;   // timestamp of last sent command (per module)
//     bool response_received[MAX_EP];
//     bool completed;             // ← ADD
// } zb_module_seq_t;


typedef enum {
    CLUSTER_TYPE_ONOFF,
    CLUSTER_TYPE_LEVEL,
    CLUSTER_TYPE_COLOR,
    CLUSTER_TYPE_MAX
} cluster_type_t;

typedef struct {
    uint8_t room_id;
    uint8_t module_id;
    uint16_t short_addr;

    uint8_t total;
    uint8_t current;

    uint8_t cluster_index;      // NEW → current cluster of endpoint

    int values[MAX_EP];
    uint8_t dids[MAX_EP];
    uint16_t node_type[MAX_EP];

    uint32_t last_tx_time_ms;

    bool response_received[MAX_EP];
    bool completed;

} zb_module_seq_t;
// One operation for a single endpoint and cluster
// typedef struct {
//     uint8_t endpoint;          // endpoint ID
//     uint16_t cluster;          // cluster ID (ON_OFF, LEVEL, etc.)
//     int16_t value;             // value to set
//     bool completed;            // true if response received
//     bool timeout;              // true if no response within timeout
//     uint32_t send_time;        // timestamp when sent (for timeout)
// } zb_operation_t;

// // Sequence for one module
// typedef struct {
//     uint8_t room_id;
//     uint8_t module_id;
//     uint8_t total_ops;                 // number of operations in this sequence
//     zb_operation_t ops[MAX_OPS];       // MAX_OPS = e.g., 16
//     uint8_t current_op;                 // index of currently sent operation
//     bool active;                         // sequence is being processed
//     uint32_t last_send_time;             // for timeout of current op
//     uint8_t raw_frame[MAX_UART_DATA_LEN]; // original encrypted frame to echo back
//     uint16_t raw_frame_len;
// } zb_sequence_t;

// // Global active sequence (only one module processed at a time)
// static zb_sequence_t g_seq = {0};
// static QueueHandle_t g_seq_queue = NULL;   // queue of pending sequences
// static TimerHandle_t g_op_timer = NULL;    // timer for operation timeout


typedef struct CtrlCommand_s {
  char command[20];      // "ctrl", "addc", "removec"
  uint8_t room_id;      // 1
  uint8_t module_id;    // 1
  uint8_t device_id;    // 1 or 0
  int16_t value;        // 0-255 or array of 6 values [-255..255]
  bool is_array;        // false for single, true for array
  int16_t values[6];    // for array mode
}CtrlCommand;
#endif
