
#include <esp_zigbee_type.h>

#ifndef _NUOS_DATA_
#define _NUOS_DATA_


#define MAX_DEVICES_IN_MODULE               6
#define MAX_STR_LEN                         24
#define SCENE_ARRAY_SIZE                    100

typedef struct device_scene_s
{
	uint16_t group_id;
	uint8_t scene_id;
	uint16_t short_addr; 
    uint16_t did;
} device_scene_handle_t;  



typedef struct device_ctx_s
{
	bool state;          	// true or false
	uint8_t level;       	// brightness
	uint32_t value; 		// rgb color
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
}endpoint_t;


typedef struct node_module_s {
    uint8_t room_id;
	uint8_t module_id;
    uint16_t zb_app_id;
    bool is_active;
    esp_zb_ieee_addr_t ieee_addr;
    uint16_t short_addr;
    uint8_t ep_counts;
    endpoint_t endpoint[MAX_DEVICES_IN_MODULE]; //devices
    char manuf_name[MAX_STR_LEN];
    char model_id[MAX_STR_LEN];
    char loc_name[MAX_STR_LEN];
} node_module_t;

#endif
