/*
 * zigbee_uart_process.c
 *
 * Zigbee-side lifecycle integration for the UART protocol.
 *
 * Connect your existing Zigbee discovery/database code to the weak
 * "nuos_add_new_zigbee_device_to_database()" hook below.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#include "esp_zigbee_type.h"

#include "common.h"
#include "nuos_nvs.h"

#include "nuos_uart_protocol.h"

extern node_module_t device_array[MAX_DEVICES];
extern uint16_t devices_count;

extern void nuos_start_new_device_discovery(node_module_t *device);

static const char *TAG = "UART_ZB_PROCESS";
#define NVS_NAMESPACE "zb_storage"
/* -------------------------------------------------------------------------- */
/* Find device                                                                */
/* -------------------------------------------------------------------------- */
static int find_device_by_ieee_local(const esp_zb_ieee_addr_t ieee_addr) {
    for (uint16_t i = 0; i < devices_count; i++) {
        if (memcmp(device_array[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static esp_err_t delete_device_from_nvs_and_ram(uint16_t device_index);

/* Reconcile recycled NWK short addresses against stable IEEE identity. */
static void reconcile_short_address_collision(uint16_t short_addr,
                                              const esp_zb_ieee_addr_t ieee_addr)
{
    uint16_t i = 0;
    while (i < devices_count) {
        if (device_array[i].short_addr == short_addr &&
            memcmp(device_array[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) != 0) {
            node_module_t stale;
            memcpy(&stale, &device_array[i], sizeof(stale));
            ESP_LOGW(TAG, "Short address 0x%04X reused by another IEEE; removing stale record", short_addr);
            uart_send_device_removed_event(&stale);
            if (delete_device_from_nvs_and_ram(i) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to remove stale short-address record at index %u", (unsigned)i);
                return;
            }
            /* Array compacted; inspect the same index again. */
            continue;
        }
        i++;
    }
}

/* -------------------------------------------------------------------------- */
/* NVS database delete + compact                                                */
/* -------------------------------------------------------------------------- */

static esp_err_t delete_device_from_nvs_and_ram(
    uint16_t device_index)
{
    if (device_index >= devices_count)
        return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t err =
        nvs_open(
            NVS_NAMESPACE,
            NVS_READWRITE,
            &handle);

    if (err != ESP_OK)
        return err;

    /*
     * NVS keys are device_0, device_1, device_2...
     *
     * Delete the selected record first.
     */
    char key[16];

    snprintf(
        key, sizeof(key),
        "device_%u",
        device_index);

    err = nvs_erase_key(
        handle,
        key);

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);

    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);

    /*
     * Compact RAM.
     */
    for (uint16_t i = device_index;
         i + 1 < devices_count;
         i++) {
        memcpy(
            &device_array[i],
            &device_array[i + 1],
            sizeof(node_module_t));
    }

    devices_count--;

    memset(
        &device_array[devices_count],
        0,
        sizeof(node_module_t));

    /*
     * Rewrite all remaining records with contiguous keys.
     */
    err = nvs_open(
        NVS_NAMESPACE,
        NVS_READWRITE,
        &handle);

    if (err != ESP_OK)
        return err;

    for (uint16_t i = 0;
         i < devices_count;
         i++) {

        snprintf(
            key, sizeof(key),
            "device_%u", i);

        err = nvs_set_blob(
            handle,
            key,
            &device_array[i],
            sizeof(node_module_t));

        if (err != ESP_OK)
            break;
    }

    /*
     * Remove the old last key.
     */
    if (err == ESP_OK) {
        snprintf(
            key, sizeof(key),
            "device_%u",
            devices_count);

        nvs_erase_key(
            handle,
            key);

        err = nvs_commit(handle);
    }

    nvs_close(handle);

    return err;
}

/* -------------------------------------------------------------------------- */
/* New device database hook                                                    */
/* -------------------------------------------------------------------------- */

/*
 * Your existing Zigbee discovery code should do:
 *
 * 1. Allocate a node_module_t.
 * 2. Fill IEEE/short address.
 * 3. Discover endpoints.
 * 4. Read Basic manufacturer/model.
 * 5. Fill endpoint clusters.
 * 6. Set room_id = 0xFF.
 * 7. Set module_id = 0xFF.
 * 8. Set is_active = true.
 * 9. Save to NVS.
 *
 * Return the index in device_array[].
 */
int nuos_add_new_zigbee_device_to_database(
    uint16_t short_addr,
    esp_zb_ieee_addr_t ieee_addr)
{
    reconcile_short_address_collision(short_addr, ieee_addr);
    int existing = find_device_by_ieee_local(ieee_addr);
    if (existing >= 0) {
        device_array[existing].short_addr = short_addr;
        device_array[existing].is_active = true;
        nuos_start_new_device_discovery(&device_array[existing]);
        return existing;
    }

    if (devices_count >= MAX_DEVICES) {
        ESP_LOGE(TAG, "Device database full (%u)", (unsigned)MAX_DEVICES);
        return -1;
    }

    uint16_t index = devices_count;
    node_module_t *device = &device_array[index];
    memset(device, 0, sizeof(node_module_t));

    memcpy(device->ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    device->short_addr = short_addr;
    device->zb_app_id = 0;
    device->room_id = 0xFF;
    device->module_id = 0xFF;
    device->is_active = true;
    device->in_scene = false;
    device->ep_counts = 0;

    snprintf(device->loc_name, sizeof(device->loc_name), "Living Room");
    snprintf(device->manuf_name, sizeof(device->manuf_name), "manuf_name");
    snprintf(device->model_id, sizeof(device->model_id), "model_id");

    devices_count++;

    ESP_LOGI(TAG,
             "NEW Zigbee device added to RAM database: index=%u short=0x%04X",
             (unsigned)index, short_addr);

    ESP_LOGI(TAG,
             "IEEE: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             device->ieee_addr[7], device->ieee_addr[6],
             device->ieee_addr[5], device->ieee_addr[4],
             device->ieee_addr[3], device->ieee_addr[2],
             device->ieee_addr[1], device->ieee_addr[0]);

    nuos_start_new_device_discovery(device);
    return (int)index;
}

/* -------------------------------------------------------------------------- */
/* DEVICE JOIN / REJOIN                                                        */
/* -------------------------------------------------------------------------- */

void on_device_joined_fxn(
    uint16_t short_addr,
    esp_zb_ieee_addr_t ieee_addr)
{
    reconcile_short_address_collision(short_addr, ieee_addr);
    ESP_LOGI(
        TAG,
        "Zigbee device joined/rejoined: short=0x%04X",
        short_addr);

    int index =
        find_device_by_ieee_local(
            ieee_addr);

    /* ---------------------------------------------------------- */
    /* Existing device = REJOIN                                    */
    /* ---------------------------------------------------------- */

    if (index >= 0) {

        node_module_t *device =
            &device_array[index];

        /*
         * Short address may change after rejoin.
         */
        device->short_addr =
            short_addr;

        device->is_active =
            true;

        esp_err_t err =
            uart_save_device_to_nvs(
                (uint16_t)index);

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Failed to save rejoined device: %s",
                esp_err_to_name(err));
        }

        uart_send_device_rejoined_event(
            (uint16_t)index);

        uart_send_connectivity_event(
            (uint16_t)index,
            true);

        return;
    }

    /* ---------------------------------------------------------- */
    /* Unknown IEEE = NEW DEVICE                                  */
    /* ---------------------------------------------------------- */

    /*
     * IMPORTANT:
     *
     * This calls YOUR existing device discovery/database function.
     */
    index =
        nuos_add_new_zigbee_device_to_database(
            short_addr,
            ieee_addr);

    if (index < 0 ||
        index >= (int)devices_count) {

        ESP_LOGE(
            TAG,
            "New Zigbee device was not added to database");

        return;
    }

    node_module_t *device =
        &device_array[index];

    /*
     * New device is always unconfigured.
     */
    device->room_id =
        0xFF;

    device->module_id =
        0xFF;

    device->short_addr =
        short_addr;

    device->is_active =
        true;

    esp_err_t err =
        uart_save_device_to_nvs(
            (uint16_t)index);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to save new device: %s",
            esp_err_to_name(err));
    }

    /*
     * Do NOT announce device.added yet. Endpoint discovery is asynchronous.
     * Sending the event here publishes endpoints:[] and the C3/web cache then
     * keeps the incomplete device record. The endpoint-discovery callback will
     * publish the completed device event after all real endpoints are known.
     */
    ESP_LOGI(TAG,
             "New device queued for endpoint discovery; device.added deferred");
}

/* -------------------------------------------------------------------------- */
/* DEVICE REMOVED / LEFT                                                       */
/* -------------------------------------------------------------------------- */

void on_device_removed_fxn(
    esp_zb_ieee_addr_t ieee_addr)
{
    int index =
        find_device_by_ieee_local(
            ieee_addr);

    if (index < 0) {
        ESP_LOGW(
            TAG,
            "Removed device not found in database");
        return;
    }

    /*
     * Make a copy because the RAM/NVS record will be removed.
     */
    node_module_t removed_device;

    memcpy(
        &removed_device,
        &device_array[index],
        sizeof(node_module_t));

    removed_device.is_active =
        false;

    /*
     * Inform C3 before deleting local data.
     * The event contains the stable deviceId/IEEE address.
     */
    uart_send_device_removed_event(
        &removed_device);

    esp_err_t err =
        delete_device_from_nvs_and_ram(
            (uint16_t)index);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to delete removed device: %s",
            esp_err_to_name(err));
    }
    else {
        ESP_LOGI(
            TAG,
            "Device removed from Zigbee DB");
    }
}

/* -------------------------------------------------------------------------- */
/* CONNECTIVITY                                                                 */
/* -------------------------------------------------------------------------- */

void on_device_connectivity_changed(
    uint16_t device_index,
    bool online)
{
    if (device_index >= devices_count)
        return;

    bool old_state =
        device_array[device_index].is_active;

    device_array[device_index].is_active =
        online;

    /*
     * Send event only when state really changes.
     */
    if (old_state != online) {

        uart_save_device_to_nvs(
            device_index);

        uart_send_connectivity_event(
            device_index,
            online);
    }
}
