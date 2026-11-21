#include "nuos_scene_engine.h"

#include <string.h>
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "esp_zigbee_gateway.h"
#include "nuos_nvs.h"
#include "common.h"

#define TAG "NUOS_SCENE"
#define ZCL_CLUSTER_GROUPS 0x0004
#define ZCL_CLUSTER_SCENES 0x0005
#define ZCL_CLUSTER_ONOFF  0x0006
#define ZCL_CLUSTER_LEVEL  0x0008
#define ZCL_CLUSTER_TUYA   0xEF00
#define NUOS_SCENE_CONTROLLER_CMD 0xE1

extern node_module_t device_array[MAX_DEVICES];
extern uint16_t devices_count;

static bool endpoint_has_cluster(const endpoint_t *ep, uint16_t cluster_id)
{
    if (!ep) return false;
    for (uint8_t i = 0; i < ep->cluster_counts; ++i) {
        if (ep->cluster_id[i] == cluster_id) return true;
    }
    return false;
}

static int find_endpoint(const node_module_t *device, uint8_t endpoint)
{
    if (!device) return -1;
    for (uint8_t i = 0; i < device->ep_counts && i < MAX_DEVICES_IN_MODULE; ++i) {
        if (device->endpoint[i].ep == endpoint) return (int)i;
    }
    return -1;
}

static esp_err_t add_group_locked(uint16_t dst_short_addr,
                                  uint8_t dst_endpoint,
                                  uint16_t group_id)
{
    esp_zb_zcl_groups_add_group_cmd_t cmd = {0};
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.group_id = group_id;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    cmd.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    (void)esp_zb_zcl_groups_add_group_cmd_req(&cmd);
    return ESP_OK;
}

static esp_err_t add_scene_locked(uint16_t dst_short_addr,
                                  uint8_t dst_endpoint,
                                  uint16_t group_id,
                                  uint8_t scene_id,
                                  const endpoint_t *ep)
{
    /*
     * Scenes AddScene stores the current On/Off and Level state of the target
     * endpoint when those clusters are present. This makes the webpage's
     * "Capture current state" action deterministic without inventing values.
     */
    uint8_t onoff_value = ep && ep->device_t.state ? 1 : 0;
    uint8_t level_value = ep ? ep->device_t.level : 0;

    esp_zb_zcl_scenes_extension_field_t onoff_field = {0};
    esp_zb_zcl_scenes_extension_field_t level_field = {0};
    esp_zb_zcl_scenes_extension_field_t *head = NULL;

    if (ep && endpoint_has_cluster(ep, ZCL_CLUSTER_ONOFF)) {
        onoff_field.cluster_id = ZCL_CLUSTER_ONOFF;
        onoff_field.length = 1;
        onoff_field.extension_field_attribute_value_list = &onoff_value;
        onoff_field.next = NULL;
        head = &onoff_field;
    }

    if (ep && endpoint_has_cluster(ep, ZCL_CLUSTER_LEVEL)) {
        level_field.cluster_id = ZCL_CLUSTER_LEVEL;
        level_field.length = 1;
        level_field.extension_field_attribute_value_list = &level_value;
        level_field.next = head;
        head = &level_field;
    }

    esp_zb_zcl_scenes_add_scene_cmd_t cmd = {0};
    cmd.group_id = group_id;
    cmd.scene_id = scene_id;
    cmd.transition_time = 0;
    cmd.extension_field = head;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    cmd.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;

    (void)esp_zb_zcl_scenes_add_scene_cmd_req(&cmd);
    return ESP_OK;
}

esp_err_t nuos_scene_configure_member(uint16_t dst_short_addr,
                                       uint8_t dst_endpoint,
                                       uint16_t group_id,
                                       uint8_t scene_id)
{
    if (dst_short_addr == 0xFFFF || dst_endpoint == 0 || group_id == 0)
        return ESP_ERR_INVALID_ARG;

    const endpoint_t *ep = NULL;
    for (uint16_t i = 0; i < devices_count; ++i) {
        if (device_array[i].short_addr == dst_short_addr) {
            int ep_index = find_endpoint(&device_array[i], dst_endpoint);
            if (ep_index < 0) return ESP_ERR_NOT_FOUND;
            ep = &device_array[i].endpoint[ep_index];
            break;
        }
    }
    if (!ep) return ESP_ERR_NOT_FOUND;

    esp_zb_lock_acquire(portMAX_DELAY);
    (void)add_group_locked(dst_short_addr, dst_endpoint, group_id);
    (void)add_scene_locked(dst_short_addr, dst_endpoint, group_id, scene_id, ep);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "AddGroup + AddScene queued addr=0x%04X ep=%u gid=%u sid=%u",
             dst_short_addr, dst_endpoint, group_id, scene_id);
    return ESP_OK;
}

esp_err_t nuos_scene_remove_member(uint16_t dst_short_addr,
                                   uint8_t dst_endpoint,
                                   uint16_t group_id,
                                   uint8_t scene_id)
{
    if (dst_short_addr == 0xFFFF || dst_endpoint == 0 || group_id == 0)
        return ESP_ERR_INVALID_ARG;

    esp_zb_zcl_scenes_remove_scene_cmd_t scene_cmd = {0};
    scene_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    scene_cmd.group_id = group_id;
    scene_cmd.scene_id = scene_id;
    scene_cmd.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    scene_cmd.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    scene_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;

    esp_zb_zcl_groups_add_group_cmd_t group_cmd = {0};
    group_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    group_cmd.group_id = group_id;
    group_cmd.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    group_cmd.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    group_cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);
    (void)esp_zb_zcl_scenes_remove_scene_cmd_req(&scene_cmd);
    (void)esp_zb_zcl_groups_remove_group_cmd_req(&group_cmd);
    esp_zb_lock_release();

    return ESP_OK;
}

esp_err_t nuos_scene_configure_controller(uint16_t dst_short_addr,
                                           uint8_t controller_endpoint,
                                           uint16_t group_id,
                                           uint8_t scene_id)
{
    if (dst_short_addr == 0xFFFF || controller_endpoint == 0 || group_id == 0)
        return ESP_ERR_INVALID_ARG;

    /* Existing NUOS scene-switch protocol: EF00 command E1 carries
     * endpoint + sceneId + groupId. This is the established device-side
     * configuration path and is deliberately kept separate from ZCL Scenes.
     */
    uint8_t payload[4];
    payload[0] = controller_endpoint;
    payload[1] = scene_id;
    payload[2] = (uint8_t)((group_id >> 8) & 0xFF);
    payload[3] = (uint8_t)(group_id & 0xFF);

    esp_zb_zcl_custom_cluster_cmd_req_t cmd = {0};
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.cluster_id = ZCL_CLUSTER_TUYA;
    cmd.custom_cmd_id = NUOS_SCENE_CONTROLLER_CMD;
    cmd.data.value = payload;
    cmd.data.size = sizeof(payload);
    cmd.data.type = ESP_ZB_ZCL_ATTR_TYPE_U32;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.profile_id = 0x0104;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    cmd.zcl_basic_cmd.dst_endpoint = controller_endpoint;
    cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;
    cmd.manuf_specific = 0;
    cmd.manuf_code = 0;
    cmd.dis_defalut_resp = 1;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_custom_cluster_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Controller scene configured addr=0x%04X ep=%u gid=%u sid=%u err=0x%x",
             dst_short_addr, controller_endpoint, group_id, scene_id, (unsigned)err);
    return err;
}

esp_err_t nuos_scene_recall_group(uint16_t group_id,
                                  uint8_t scene_id)
{
    if (group_id == 0) return ESP_ERR_INVALID_ARG;

    esp_zb_zcl_scenes_recall_scene_cmd_t cmd = {0};
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    cmd.group_id = group_id;
    cmd.scene_id = scene_id;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd.zcl_basic_cmd.dst_endpoint = 0;
    cmd.zcl_basic_cmd.src_endpoint = ESP_ZB_GATEWAY_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);
    (void)esp_zb_zcl_scenes_recall_scene_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "RecallScene groupcast gid=%u sid=%u queued", group_id, scene_id);
    return ESP_OK;
}
