

#ifndef _NUOS_NVS_
#define _NUOS_NVS_

#include "esp_check.h"
#include "esp_log.h"
#include "nuos_data_structs.h"

#define MAX_DEVICES 	120//MAX_CHILDREN // Maximum number of stored devices
/*165 * 10 = 1650 bytes  i.e. 1.65 KB*/
// esp_err_t save_light_bulb_device_to_nvs(void* device_void);
#ifdef __cplusplus
extern "C" {
#endif
    esp_err_t read_all_devices_from_nvs(void* stored_device);
    esp_err_t clear_all_records_from_nvs();
    esp_err_t clear_single_record_from_nvs(int index);
    esp_err_t save_or_update_light_bulb_device_to_nvs(void* device_void);
    esp_err_t remove_zigbee_device_from_nvs(uint16_t short_addr);
    uint16_t is_record_already_exists_in_nvs(uint16_t short_addr);

    uint8_t get_previous_val(uint8_t nodeindex, uint8_t epindex);
    node_module_t *find_device_by_room_module_id(uint8_t r, uint8_t m);
    int find_device_index_by_room_module_id(uint8_t r, uint8_t m) ;
    node_module_t *find_device_by_node(uint16_t m, uint8_t did) ;
    uint16_t find_device_short_addr(uint16_t m, uint8_t did);
    bool find_group_and_scene(uint16_t short_addr, uint16_t did, uint16_t *group_id, uint8_t *scene_id);
    // esp_err_t nvs_store_or_update_scene(const void *new_scene);
    // esp_err_t nvs_read_scene_array(void *scene_handle);
    // esp_err_t nvs_read_scene_by_id(uint16_t group_id, uint8_t scene_id, void *scene);
    // esp_err_t nvs_delete_scene(uint16_t group_id, uint8_t scene_id);

    esp_err_t nvs_get_last_ids(uint16_t *group_id, uint8_t *scene_id);
    esp_err_t nvs_set_last_ids(uint16_t group_id, uint8_t scene_id);

    void print_devices_compact();
    int16_t find_module_index_by_ieee(const esp_zb_ieee_addr_t ieee_addr);
    int16_t find_module_by_ieee(esp_zb_ieee_addr_t ieee_addr);
    uint8_t remove_duplicate_devices();
    uint8_t remove_invalid_ieee_devices();
    esp_err_t load_scene_nodes(void);
    esp_err_t store_scene_nodes(void);
    esp_err_t clear_scene_nodes_nvs(void);
    uint16_t generate_next_for_node(uint8_t room_id, uint8_t module_id, uint8_t device_id);
#ifdef __cplusplus
}
#endif
#endif
