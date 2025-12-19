
#include "esp_zigbee_core.h"
#include "esp_err.h"
#include "esp_check.h"
#include "zdo/esp_zigbee_zdo_command.h"

static const char *TAG = "ESP_ZB_SCENE_COMMANDS";

extern void nuos_set_attribute_cluster_2(const esp_zb_zcl_set_attr_value_message_t *message);
extern esp_err_t nuos_set_state_attribute(uint8_t index);
extern esp_err_t nuos_set_color_temp_level_attribute(uint8_t index);
extern esp_err_t nuos_set_fan_attribute(uint8_t index);
extern void xyToRgb(uint16_t x, uint16_t y, float brightness, uint8_t *r, uint8_t *g, uint8_t *b);
extern void nuos_set_scene_group_cluster(const esp_zb_zcl_recall_scene_message_t *message);

void nuos_zb_scene_query_all_scenes_request(uint16_t group_id, uint8_t src_endpoint)
{
    esp_zb_zcl_scenes_get_scene_membership_cmd_t cmd_req;
    // memset(&cmd_req, 0, sizeof(cmd_req));
    cmd_req.group_id = group_id;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd_req.zcl_basic_cmd.dst_endpoint = 0; // Groupcast does not use endpoint
    cmd_req.zcl_basic_cmd.src_endpoint = src_endpoint;
    esp_zb_zcl_scenes_get_scene_membership_cmd_req(&cmd_req);
}


esp_zb_zcl_scenes_extension_field_t nuos_zb_scene_set_onoff_cluster_extension_field(uint8_t on_off_value, uint16_t cluster_id){
    // 1 for ON, 0 for OFF
    esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
        .cluster_id = cluster_id, // Cluster ID for On/Off cluster
        .length = sizeof(on_off_value),
        .extension_field_attribute_value_list = &on_off_value,
        .next = NULL // Initially, no next extension field
    };
    return on_off_extension_field;
}

esp_zb_zcl_scenes_extension_field_t nuos_zb_scene_set_onoff_level_cluster_extension_field(uint8_t on_off_value, uint8_t brightness_value){
    // brightness level (0-255)
    esp_zb_zcl_scenes_extension_field_t field1 = nuos_zb_scene_set_onoff_cluster_extension_field(on_off_value, 0x06);
    esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
        .cluster_id = 0x0008, // Cluster ID for Level Control cluster
        .length = sizeof(brightness_value),
        .extension_field_attribute_value_list = &brightness_value,
        .next = &field1 // Link to the On/Off extension field
    };
    return level_control_extension_field;
}


void nuos_zb_scenes_store_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
    esp_zb_zcl_scenes_store_scene_cmd_t cmd_req = {
        .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
        .zcl_basic_cmd.dst_endpoint = dst_ep,
        .zcl_basic_cmd.src_endpoint = src_ep,
        .group_id = group_id,
        .scene_id = scene_id,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
    };
    esp_zb_zcl_scenes_store_scene_cmd_req(&cmd_req);
}


void nuos_zb_scenes_remove_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, uint16_t address_mode){
    esp_zb_zcl_scenes_remove_scene_cmd_t cmd_req;
    cmd_req.group_id = group_id;
    cmd_req.scene_id = scene_id;
    cmd_req.address_mode = address_mode;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short;
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = src_ep;    
    esp_zb_zcl_scenes_remove_scene_cmd_req(&cmd_req);
}

void nuos_zb_scene_recall_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, 
                                        uint8_t dst_ep, uint16_t dst_addr_short, esp_zb_zcl_address_mode_t address_mode)
{
    esp_zb_zcl_scenes_recall_scene_cmd_t recall_scene_cmd = {
        .address_mode = address_mode,
        .zcl_basic_cmd.dst_addr_u.addr_short = group_id,
        .zcl_basic_cmd.src_endpoint = src_ep,
        .group_id = group_id,
        .scene_id = scene_id,
    };
	esp_zb_zcl_scenes_recall_scene_cmd_req(&recall_scene_cmd);
}

void nuos_zb_scene_add_scene_to_self_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, 
                                                uint8_t value_on_off, uint8_t value_level, uint16_t colorx, uint16_t colory){

    //nuos_zb_scenes_add_scene_request(group_id, scene_id, src_ep, dst_ep, esp_zb_get_short_address(), value_on_off, value_level, colorx, colory);
}

void nuos_zb_scene_add_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                                uint8_t value_on_off, uint8_t value_level){
    // nuos_zb_scenes_add_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short, value_on_off, value_level); //unicast cast working
}
void nuos_zb_scene_add_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level){
    // nuos_zb_scenes_add_scene_request(group_id, scene_id, src_ep, dst_ep, 0xffff, value_on_off, value_level); //broadcast working
}
void nuos_zb_scene_add_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level){
    // nuos_zb_scenes_add_scene_request(group_id, scene_id, 0, dst_ep, group_id, value_on_off, value_level); //groupcast not working
}



void nuos_zb_scene_remove_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
        nuos_zb_scenes_remove_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short,  ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}
void nuos_zb_scene_remove_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_remove_scene_request(group_id, scene_id, src_ep, dst_ep, 0xffff, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT); //broadcast working
}
void nuos_zb_scene_remove_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_remove_scene_request(group_id, scene_id, 0, dst_ep, group_id, ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT); //groupcast not working
}


void nuos_zb_scene_store_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
        nuos_zb_scenes_store_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short);
}
void nuos_zb_scene_store_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_store_scene_request(group_id, scene_id, src_ep, dst_ep, 0xffff); //broadcast not working
}
void nuos_zb_scene_store_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_store_scene_request(group_id, scene_id, 0, dst_ep, group_id);    //groupcast not working
}


void nuos_zb_scene_recall_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
    nuos_zb_scene_recall_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}

void nuos_zb_scene_recall_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep){
    nuos_zb_scene_recall_scene_request(group_id, scene_id, src_ep, 0xff, 0xffff,ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}

void nuos_zb_scene_recall_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep){
    nuos_zb_scene_recall_scene_request(group_id, scene_id, src_ep, 0, group_id, ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT);
}

/***********************ACTION HANDLERS**********************/
esp_err_t zb_add_scene_resp_handler(const esp_zb_zcl_scenes_operate_scene_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Add Scene Response: GROUP_ID(%d), SCENE_ID(%d)", message->group_id, message->scene_id);
    ESP_LOGI(TAG, "Add Scene Response: CLUSTER(%d), COMMAND(%d)", message->info.cluster, message->info.command.id);
    ESP_LOGI(TAG, "Add Scene Response: DST_ADDR(0x%x), DST_EP(%d), SRC_EP(%d)", 
                    message->info.dst_address, message->info.dst_endpoint, message->info.src_endpoint);
    return ESP_OK;
}

esp_err_t zb_view_scene_resp_handler(const esp_zb_zcl_scenes_view_scene_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "View Scene Response: GROUP_ID(%d), SCENE_ID(%d)", message->group_id, message->scene_id);
    ESP_LOGI(TAG, "View Scene Response: TRANSITION_TIME(%d)", message->transition_time);

    esp_zb_zcl_scenes_extension_field_t *field_set = message->field_set;
    while (field_set) {
        ESP_LOGI(TAG, "View Scene response: ex_cluster_id(%d), ex_length(%d)", field_set->cluster_id, field_set->length);
        field_set = field_set->next;
    }
    ESP_LOGI(TAG, "View Scene Response: CLUSTER(%d), COMMAND(%d)", message->info.cluster, message->info.command.id);
    ESP_LOGI(TAG, "View Scene Response: DST_ADDR(0x%x), DST_EP(%d), SRC_EP(%d)", message->info.dst_address, message->info.dst_endpoint, message->info.src_endpoint);
    return ESP_OK;
}

esp_err_t zb_get_scene_membership_resp_handler(const esp_zb_zcl_scenes_get_scene_membership_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                       message->info.status);

    ESP_LOGI(TAG, "GET Scene Membership Response: CAPACITY(%d), GROUP_ID(%d), SCENE_ID(%d)", message->capacity, message->group_id, message->scene_count);

    uint8_t *scene_list = message->scene_list;
    for(int i=0; i<message->scene_count; i++){
        ESP_LOGI(TAG, "scene_list[%d]: %d", i, scene_list[i]);
    }
    ESP_LOGI(TAG, "GET Scene Membership Response: CLUSTER(%d), COMMAND(%d)", message->info.cluster, message->info.command.id);
    ESP_LOGI(TAG, "GET Scene Membership Response: DST_ADDR(0x%x), DST_EP(%d), SRC_EP(%d)", message->info.dst_address, message->info.dst_endpoint, message->info.src_endpoint);

    return ESP_OK;
}


esp_err_t zb_get_scene_store_resp_handler(const esp_zb_zcl_store_scene_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    printf("zb_get_scene_store_resp_handler\n");
    printf("=================> %d <=================\n", message->info.dst_endpoint);
    printf("group_id: %d  scene_id: %d  cluster: %d\n", message->group_id, message->scene_id, message->info.cluster);


    return ESP_OK;
}

esp_err_t nuos_zb_scene_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
    esp_err_t ret = -1;
    //printf("On Scene Recall!!\n");
    switch (callback_id) {
        case ESP_ZB_CORE_SCENES_STORE_SCENE_CB_ID:
            ret = zb_get_scene_store_resp_handler ((esp_zb_zcl_store_scene_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_OPERATE_SCENE_RESP_CB_ID:
            ret =  zb_add_scene_resp_handler((esp_zb_zcl_scenes_operate_scene_resp_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_VIEW_SCENE_RESP_CB_ID:
            ret = zb_view_scene_resp_handler((esp_zb_zcl_scenes_view_scene_resp_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_GET_SCENE_MEMBERSHIP_RESP_CB_ID:
            ret = zb_get_scene_membership_resp_handler((esp_zb_zcl_scenes_get_scene_membership_resp_message_t *)message);
            break;
        default:
            break;
    }
    return ret;
}