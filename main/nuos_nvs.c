

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_zigbee_type.h"
#include "esp_zigbee_gateway.h"
#include "string.h"
#include "stdlib.h"
#include "nuos_nvs.h"
#include "common.h"
#include "frame_enc_dec.h"

#define NVS_NAMESPACE "zb_storage"

extern node_module_t device_array[MAX_DEVICES];
extern uint16_t devices_count;

static const char* TAG = "Zigbee-NVS";

nvs_handle_t nvs_handle1;
esp_err_t clear_single_record_from_nvs(int index);

// extern uint16_t total_ep_counts;
// Function to print IEEE address
void print_ieee_addr(const esp_zb_ieee_addr_t addr) {
    // Ensure it's a valid pointer
    if (!addr) {
        ESP_LOGE(TAG, "Invalid IEEE address pointer");
        return;
    }

    // Print each byte of the IEEE address
    char ieee_addr_str[3 * 8];
    for (int i = 0; i < 8; ++i) {
        sprintf(&ieee_addr_str[i * 3], "%02X:", addr[i]);
    }
    // Remove the trailing colon
    ieee_addr_str[3 * 8 - 1] = '\0';

    printf("IEEE_ADDR=%s\n", ieee_addr_str);
}


uint16_t is_record_already_exists_in_nvs(uint16_t short_addr) {
    node_module_t stored_device;
    size_t size = sizeof(stored_device);

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1));
    // Search for duplicate short address and update
    for (int i = 0; i < MAX_DEVICES; i++) {
        char key[16];
        snprintf(key, sizeof(key), "device_%d", i);

        esp_err_t err = nvs_get_blob(nvs_handle1, key, &stored_device, &size);
        if (err == ESP_OK) {
            // If a device with the same short_addr exists, update its parameters
            if (stored_device.short_addr == short_addr) {
                ESP_LOGI(TAG, "Already existing device.");
                nvs_close(nvs_handle1);
                return i; // Return success or failure of update
            }
        }
    }
    nvs_close(nvs_handle1);
    return ESP_ERR_NVS_NOT_FOUND; // No space left to store new device
}


esp_err_t save_or_update_light_bulb_device_to_nvs(void* device_void) {
    node_module_t *new_device = (node_module_t *)device_void;
    node_module_t stored_device;
    size_t size = sizeof(stored_device);

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1));
    // Search for duplicate short address and update
    for (int i = 0; i < MAX_DEVICES; i++) {
        char key[16];
        snprintf(key, sizeof(key), "device_%d", i);

        esp_err_t err = nvs_get_blob(nvs_handle1, key, &stored_device, &size);

        //vTaskDelay(pdMS_TO_TICKS(1));
        if (err == ESP_OK) {
            // If a device with the same short_addr exists, update its parameters
            //if (stored_device.short_addr == new_device->short_addr) {
            if (memcmp(stored_device.ieee_addr, new_device->ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
                //ESP_LOGI(TAG, "Duplicate short address(0x%x) found, updating existing(0x%x) device.", new_device->short_addr, stored_device.short_addr);

                strcpy(stored_device.manuf_name, new_device->manuf_name);
                strcpy(stored_device.model_id, new_device->model_id);
            
                stored_device.module_id = new_device->module_id;
                stored_device.room_id = new_device->room_id;

                stored_device.short_addr = new_device->short_addr;


                // Copy endpoints (including device_t fields) from new_device to stored_device
                stored_device.ep_counts = new_device->ep_counts;
                for (int ij = 0; ij < new_device->ep_counts; ij++) {
            
                    stored_device.endpoint[ij] = new_device->endpoint[ij];
                }
                err = nvs_set_blob(nvs_handle1, key, &stored_device, sizeof(node_module_t));
                if (err == ESP_OK) {
                    //ESP_LOGI(TAG, "Device updated in NVS");
                    nvs_commit(nvs_handle1);
                }
                nvs_close(nvs_handle1);
                return err; // Return success or failure of update
            }
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            //new_device->module_id = i+1;
            // Slot is empty, save the new device
            err = nvs_set_blob(nvs_handle1, key, new_device, sizeof(node_module_t));
            if (err == ESP_OK) {
                //ESP_LOGI(TAG, "New device saved to NVS");
                nvs_commit(nvs_handle1);
                nvs_close(nvs_handle1);
                return ESP_OK; // Return success
            } else {
                ESP_LOGE(TAG, "Failed to save device: %s", esp_err_to_name(err));
                nvs_close(nvs_handle1);
                return err; // Return error on saving
            }
        } else {
            //ESP_LOGE(TAG, "Error reading from NVS: %s", esp_err_to_name(err));
            nvs_close(nvs_handle1);
            return err;
        }
    }

    
    nvs_close(nvs_handle1);
    ESP_LOGI(TAG, "No free space to store new device");
    return ESP_ERR_NO_MEM; // No space left to store new device
}
/**
 * @brief Read all devices stored in NVS
 */
esp_err_t read_all_devices_from_nvs(void* stored_device) {
	esp_err_t err = ESP_OK;
    // Cast the void pointer to the expected structure type
    //node_module_t *device = (node_module_t *)stored_device;
    size_t size = sizeof(node_module_t);

    devices_count = 0;

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1));

    for (int i = 0; i < MAX_DEVICES; i++) {
        // Generate the NVS key based on the index
        vTaskDelay(pdMS_TO_TICKS(1));
        char key[16];
        snprintf(key, sizeof(key), "device_%d", i);
		// Read the device from NVS
		err = nvs_get_blob(nvs_handle1, key, &device_array[devices_count], &size);
		if (err == ESP_OK) {
			// printf("---------Device read from NVS---------\n");
			// print_ieee_addr(device_array[devices_count].ieee_addr);
			// printf("SHORT ADDR=0x%04x\n", device_array[devices_count].short_addr);
			// printf("TOTAL EP=%d\n", device_array[devices_count].ep_counts);
			if(device_array[devices_count].ep_counts > MAX_DEVICES_IN_MODULE) device_array[devices_count].ep_counts = MAX_DEVICES_IN_MODULE;
            
			for(int j=0; j<device_array[devices_count].ep_counts; j++){
				// printf("EP[%d]=%d\n", j, device_array[devices_count].endpoint[j].ep);
				// printf("DEV_NAME[%d]=%s\n", j, device_array[devices_count].endpoint[j].dev_name);
				// printf("dev_type[%d]=%d\n", j, device_array[devices_count].endpoint[j].dev_type);
                // if(device_array[devices_count].endpoint[j].dev_id != 0){
                //     total_ep_counts++;
                // }
                vTaskDelay(pdMS_TO_TICKS(1));
                //printf("dev_id[%d]=%d\n", j, device_array[devices_count].endpoint[j].dev_id);
			}
			//printf("MANUF_NAME=%s\n", device_array[devices_count].manuf_name);
			//printf("MODEL_ID=%s\n", device_array[devices_count].model_id);
            if(device_array[devices_count].short_addr == 0x0000){
                if (nvs_erase_key(nvs_handle1 , key) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to erase all records from NVS");
                    err = ESP_FAIL;
                }
                nvs_commit(nvs_handle1);
            }else{
				//memcpy(&device_array[devices_count], device, sizeof(node_module_t));
				// for(uint8_t p=0; p<device_array[devices_count].ep_counts; p++) {
				// 	printf("endpoint=%d dev_type=%d\n", device_array[devices_count].endpoint[p].ep, device_array[devices_count].endpoint[p].dev_type);
                //     printf("state=%d level=%d\n", device_array[devices_count].endpoint[p].device_t.state, device_array[devices_count].endpoint[p].device_t.level);
				// }
				devices_count++;
                //printf("devices_count:%d\n", devices_count);

            }
		} else if (err == ESP_ERR_NVS_NOT_FOUND) {
			//ESP_LOGW(TAG, "No device found at index %d", i);
		} else {
			//ESP_LOGE(TAG, "Error reading from NVS: %s", esp_err_to_name(err));
		}
    }
    nvs_close(nvs_handle1);
    if(devices_count > 0) err = ESP_OK;
    return err;
}

// Existing structures and types...
esp_err_t clear_all_records_from_nvs() {
    esp_err_t err = ESP_OK;

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1));

    if (nvs_erase_all(nvs_handle1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase all records from NVS");
        err = ESP_FAIL;
    }
    nvs_commit(nvs_handle1);
    nvs_close(nvs_handle1);

    return err;
}

esp_err_t clear_single_record_from_nvs(int index) {
    esp_err_t err = ESP_OK;

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1));
    char key[16];
    snprintf(key, sizeof(key), "device_%d", index);
    if (nvs_erase_key(nvs_handle1 , key) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase all records from NVS");
        err = ESP_FAIL;
    }
    nvs_commit(nvs_handle1);
    nvs_close(nvs_handle1);

    return err;
}


esp_err_t remove_zigbee_device_from_nvs(uint16_t short_addr) {
    node_module_t stored_device;
    size_t size = sizeof(stored_device);
    //printf("remove_zigbee_device_from_nvs:0x%x\n", short_addr);
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1));
    // Search for duplicate short address and update
    for (int i = 0; i < MAX_DEVICES; i++) {
        char key[16];
        snprintf(key, sizeof(key), "device_%d", i);

        esp_err_t err = nvs_get_blob(nvs_handle1, key, &stored_device, &size);
        if (err == ESP_OK) {
            // If a device with the same short_addr exists, update its parameters
            if (stored_device.short_addr == short_addr) {
                ESP_LOGI(TAG, "Short address found, remove existing device.");
                if (nvs_erase_key(nvs_handle1 , key) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to erase all records from NVS");
                    nvs_close(nvs_handle1);
                    return ESP_FAIL;
                }
                nvs_commit(nvs_handle1);
                nvs_close(nvs_handle1);
                return ESP_OK;
            }
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // Slot is empty
            //ESP_LOGI(TAG, "ESP_ERR_NVS_NOT_FOUND");
        } else {
            //ESP_LOGE(TAG, "Error reading from NVS: %s", esp_err_to_name(err));
            nvs_close(nvs_handle1);
            return err;
        }
    }
    nvs_close(nvs_handle1);
    return ESP_ERR_NO_MEM; // No space left to store new device
}

// Function to find device by short_addr
node_module_t *find_device_by_short_addr(uint16_t target_short_addr) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_array[i].short_addr == target_short_addr) {
            return &device_array[i]; // Found matching device
        }
    }
    return NULL; // Device not found
}
int find_node_index_by_ieee_addr(esp_zb_ieee_addr_t ieee_addr){

    for (int i = 0; i < MAX_DEVICES; i++) {
        if (memcmp(device_array[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
            return i; // Found matching device
        }
    }
    return -1; // result not found
}
int find_node_index_by_short_addr(uint16_t target_short_addr) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_array[i].short_addr == target_short_addr) {
            return i; // Found matching device
        }
    }
    return -1; // result not found
}

bool find_device_by_dev_id(uint16_t room_id, uint16_t module_id, uint16_t target_dev_id, uint8_t* nodeindex, uint8_t* epindex) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if((room_id == device_array[i].room_id && device_array[i].module_id == module_id) ){
            for (int j = 0; j < 4; j++) {
                if (device_array[i].endpoint[j].ep == target_dev_id) {
                    *nodeindex = i;
                    *epindex = j;
                    return true;
                }
            }
        }
    }
    return false;
}

node_module_t *find_device_by_room_module_id(uint8_t r, uint8_t m) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_array[i].room_id == r && device_array[i].room_id != 255) {
            if (device_array[i].module_id == m ) {
                printf("-------------->device_array[i].ep_counts:%d\n", device_array[i].ep_counts);
                return &device_array[i]; // Found matching device
            }
        }
    }
    return NULL; // Device not found
}

int find_device_index_by_room_module_id(uint8_t r, uint8_t m) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_array[i].room_id == r && device_array[i].room_id != 255) {
            if (device_array[i].module_id == m ) {
                return i;
            }
        }
    }
    return -1; // Device not found
}

node_module_t *find_device_by_node(uint16_t m, uint8_t d) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_array[i].module_id == m && device_array[i].room_id != 255) {
            for (int j = 0; j < device_array[i].ep_counts; j++) {
                if (device_array[i].endpoint[j].ep == d) {
                    return &device_array[i]; // Found matching device
                }
            }
        }
    }
    return NULL; // Device not found
}

uint16_t find_device_short_addr(uint16_t m, uint8_t did) {
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_array[i].module_id == m && device_array[i].room_id != 255) {
            return device_array[i].short_addr;
        }
    }
    return 0; // Device not found
}

uint16_t find_device_type(uint8_t nodeindex, uint8_t epindex){
    return device_array[nodeindex].endpoint[epindex].dev_type;
}
uint8_t get_total_endpoints(uint8_t nodeindex){
    return device_array[nodeindex].ep_counts;
}

uint8_t get_previous_val(uint8_t nodeindex, uint8_t epindex){
    uint8_t level = 0;
    if(device_array[nodeindex].endpoint[epindex].device_t.state == 0) level = 0;
    else level = device_array[nodeindex].endpoint[epindex].device_t.level;
    return level;
}
/////////////////////////////////////////////////////////////////////////

bool find_group_and_scene(uint16_t short_addr, uint16_t did, uint16_t *group_id, uint8_t *scene_id)
{
    // for (int i = 0; i < 100; ++i) {
    //     if (scene_data_t[i].short_addr == short_addr &&
    //         scene_data_t[i].did == did) {
    //         *group_id = scene_data_t[i].group_id;
    //         *scene_id = scene_data_t[i].scene_id;
    //         return true; // Found
    //     }
    // }
    return false; // Not found
}


// Assuming you have functions like these for NVS
void save_devices_to_nvs() {
  
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1);
    if (err == ESP_OK) {
        // Save device count
        nvs_set_u16(nvs_handle1, "dev_count", devices_count);
        
        // Save device array
        nvs_set_blob(nvs_handle1, "device_array", device_array, devices_count * sizeof(node_module_t));
        
        nvs_commit(nvs_handle1);
        nvs_close(nvs_handle1);
        ESP_LOGI(TAG, "Saved %d devices to NVS", devices_count);
    }
}


bool is_ieee_invalid(const esp_zb_ieee_addr_t _ieee_addr) {
    uint64_t all_ff[8] = {0,0,0,0,0,0,0,0};

    for (int i = 0; i < 8; i++) {
        printf("ieee_addr[%d]=0x%02x\n", i, _ieee_addr[i]);
        if (_ieee_addr[i] == 0xFF) {
           all_ff[i] = 0xff; 
        }
    }
    if(all_ff[0] == 0xff && all_ff[1] == 0xff && all_ff[2] == 0xff && all_ff[3] == 0xff &&
       all_ff[4] == 0xff && all_ff[5] == 0xff && all_ff[6] == 0xff && all_ff[7] == 0xff) {
        return true;
    }
    return false;
}


uint8_t remove_invalid_ieee_devices() {
    // 3. Remove invalid devices
    printf("\nRemoving invalid devices...%d\n", devices_count);
    uint8_t removed = 0;
    for (uint16_t i = 0; i < devices_count; ) {
        if (is_ieee_invalid(device_array[i].ieee_addr) || device_array[i].room_id == 255) {
        //if (is_ieee_invalid(device_array[i].ieee_addr)) {
            printf("  Removing index %d: Module ID %d\n", i, device_array[i].module_id);

            send_module_remove_device_on_uart(device_array[i].room_id, device_array[i].module_id, device_array[i].ieee_addr);
            vTaskDelay(pdMS_TO_TICKS(100));
            gateway_leave_device(device_array[i].short_addr, device_array[i].ieee_addr, false);      
            remove_zigbee_device_from_nvs(device_array[i].short_addr);  
            // Shift devices
            for (uint16_t j = i; j < devices_count - 1; j++) {
                memcpy(&device_array[j], &device_array[j + 1], sizeof(node_module_t));
            }
            devices_count--;
            removed++;
        } else {
            i++;
        }
    }
    if (removed > 0) {
        printf("Removed %d device(s) with invalid IEEE address. Device count: %d\n",
               removed, devices_count);
               save_devices_to_nvs();
    }
    
    return removed;
}

uint8_t remove_invalid_ieee_devices1234() {
    uint8_t removed_count = 0;
    uint16_t original_count = devices_count;
    
    for (uint16_t i = 0; i < devices_count; ) {
        if (is_ieee_invalid(device_array[i].ieee_addr)) {
            printf("Removing device with invalid IEEE address at index %d (Module ID %d, Short Addr 0x%04X)\n",
                   i, device_array[i].module_id, device_array[i].short_addr);
            
            // Shift all devices after i one position left
            for (uint16_t j = i; j < devices_count - 1; j++) {
                memcpy(&device_array[j], &device_array[j + 1], sizeof(node_module_t));
            }
            
            devices_count--;
            removed_count++;

            remove_zigbee_device_from_nvs(device_array[i].short_addr);
            // Don't increment i because we shifted the next device to position i
        } else {
            i++;  // Only increment if we didn't remove
        }
    }
    
    if (removed_count > 0) {
        printf("Removed %d device(s) with invalid IEEE address. Device count: %d -> %d\n",
               removed_count, original_count, devices_count);
    }
    
    return removed_count;
}

int16_t find_module_index_by_ieee(const esp_zb_ieee_addr_t ieee_addr) {
    for (uint16_t i = 0; i < devices_count; i++) {
        // Compare each byte of the IEEE address
        if (memcmp(device_array[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
            return i;  // Found at index i
        }
    }
    return -1;  // Not found
}

// Find device by IEEE address
int16_t find_module_by_ieee(esp_zb_ieee_addr_t ieee_addr) {
    for (uint16_t i = 0; i < devices_count; i++) {
        if (memcmp(device_array[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
            return i;  // Found
        }
    }
    return -1;  // Not found
}

// // Find device by short address
// int16_t find_device_by_short_addr(uint16_t short_addr) {
//     for (uint16_t i = 0; i < devices_count; i++) {
//         if (device_array[i].short_addr == short_addr) {
//             return i;  // Found
//         }
//     }
//     return -1;  // Not found
// }

// Find endpoint by short address and endpoint ID
int16_t find_endpoint_in_device(uint16_t short_addr, uint8_t endpoint_id) {
    for (uint16_t i = 0; i < devices_count; i++) {
        if (device_array[i].short_addr == short_addr) {
            for (uint8_t ep_idx = 0; ep_idx < device_array[i].ep_counts; ep_idx++) {
                if (device_array[i].endpoint[ep_idx].ep == endpoint_id) {
                    return ep_idx;
                }
            }
        }
    }
    return -1;
}

uint8_t remove_duplicate_devices() {
    uint8_t removed_count = 0;
    uint16_t original_count = devices_count;
    
    for (uint16_t i = 0; i < devices_count; i++) {
        // For each device, check if same IEEE exists later
        for (uint16_t j = i + 1; j < devices_count; ) {
            if (memcmp(device_array[i].ieee_addr, device_array[j].ieee_addr, 
                      sizeof(esp_zb_ieee_addr_t)) == 0) {
                // Found duplicate at index j
                printf("Removing duplicate at index %d (Module ID %d, Short Addr 0x%04X)\n",
                       j, device_array[j].module_id, device_array[j].short_addr);
                
                // Shift all devices after j one position left
                for (uint16_t k = j; k < devices_count - 1; k++) {
                    memcpy(&device_array[k], &device_array[k + 1], sizeof(node_module_t));
                }
                
                devices_count--;
                removed_count++;

                remove_zigbee_device_from_nvs(device_array[j].short_addr);
                // Don't increment j because we shifted the next device to position j
            } else {
                j++;  // Only increment if we didn't remove
            }
        }
    }
    
    if (removed_count > 0) {
        printf("Removed %d duplicate device(s). Device count: %d -> %d\n",
               removed_count, original_count, devices_count);
    }
    
    return removed_count;
}

void print_devices_compact() {
    printf("\nDevices in database: %d\n", devices_count);
    printf("Idx | ModID | Room | ShortAddr | IEEE Address          | Active | EPs | Name | Device Type       |\n");
    printf("----|-------|------|-----------|-----------------------|--------|-----|------|-------------      |\n");
    
    for (uint16_t i = 0; i < devices_count; i++) {
        node_module_t *device = &device_array[i];
        
        // Format IEEE address
        char ieee_str[24];
        snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                device->ieee_addr[0], device->ieee_addr[1], device->ieee_addr[2],
                device->ieee_addr[3], device->ieee_addr[4], device->ieee_addr[5],
                device->ieee_addr[6], device->ieee_addr[7]);
               /////////////////////////////////////////////////////////////        
        printf("%3d | %5d | %4d |   0x%04X  | %s | %6s | %3d | %s",
               i, device->module_id, device->room_id, device->short_addr,
               ieee_str, device->is_active ? "Yes" : "No", 
               device->ep_counts, device->loc_name);
               /////////////////////////////////////////////////////////////
               printf(" | ");
               for(int j=0; j<device->ep_counts; j++){
                   printf(" (%d)0x%X", device->endpoint[j].ep, device->endpoint[j].dev_type);   
               }
        printf("\n");
    }
    printf("\n");
}

void print_endpoints_details() {
    printf("\n=== Endpoint Details ===\n");
    
    for (uint16_t i = 0; i < devices_count; i++) {
        node_module_t *device = &device_array[i];
        
        for (uint8_t ep_idx = 0; ep_idx < device->ep_counts; ep_idx++) {
            endpoint_t *ep = &device->endpoint[ep_idx];
            
            printf("Device[%d].EP[%d]: ID=0x%04X, Type=0x%04X, Name='%s'\n",
                   i, ep->ep, ep->ep, ep->dev_type, ep->dev_name);
            
            // For each cluster
            for (uint8_t cl_idx = 0; cl_idx < ep->cluster_counts; cl_idx++) {
                printf("  Cluster[%d]: 0x%04X %s\n", 
                       cl_idx, ep->cluster_id[cl_idx],
                       ep->is_bind[cl_idx] ? "(Bound)" : "");
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
esp_err_t clear_scene_nodes_nvs(void) {
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1);
    if (err != ESP_OK) return err;

    if (nvs_handle1 == 0) {
        ESP_LOGE("SCENE", "NVS handle not initialized");
        nvs_close(nvs_handle1);
        return ESP_FAIL;
    }
    
    // Erase both keys
    err = nvs_erase_key(nvs_handle1, "nodes_count");
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE("SCENE", "Failed to erase nodes_count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle1);
        return err;
    }
    
    err = nvs_erase_key(nvs_handle1, "nodes_array");
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE("SCENE", "Failed to erase nodes_array: %s", esp_err_to_name(err));
        return err;
    }
    
    // Commit changes
    err = nvs_commit(nvs_handle1);
    if (err != ESP_OK) {
        ESP_LOGE("SCENE", "Failed to commit clear operation: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("SCENE", "Cleared scene nodes from NVS");
    }
    nvs_close(nvs_handle1);
    return err;
}
// Store entire array + count to NVS as blobs
esp_err_t store_scene_nodes(void) {

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle1);
    if (err != ESP_OK) return err;

    // Store count
    err = nvs_set_u16(nvs_handle1, "nodes_count", scene_nodes_count);
    if (err != ESP_OK) {    
        nvs_close(nvs_handle1);
        return err;
    }

    if (scene_nodes_count > 0) {
        // Store array as blob (only used portion)
        err = nvs_set_blob(nvs_handle1, "nodes_array", 
                          scene_nodes, 
                          scene_nodes_count * sizeof(node_mapping_t));
    }

    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle1);
    }

    nvs_close(nvs_handle1);
    return err;
}

// Load entire array + count from NVS (populates globals)
esp_err_t load_scene_nodes(void) {

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle1);
    if (err != ESP_OK) return err;

    // Get count
    size_t count_size = sizeof(scene_nodes_count);
    err = nvs_get_u16(nvs_handle1, "nodes_count", &scene_nodes_count);
    if (err != ESP_OK) {
        scene_nodes_count = 0;
        ESP_LOGI("SCENE", "No nodes count found, starting fresh");
        nvs_close(nvs_handle1);
        return ESP_OK;
    }

    if (scene_nodes_count > MAX_SCENE_NODES) {
        ESP_LOGE("SCENE", "Invalid count %d > %d, resetting", scene_nodes_count, MAX_SCENE_NODES);
        scene_nodes_count = 0;
        nvs_close(nvs_handle1);
        return ESP_ERR_INVALID_SIZE;
    }

    if (scene_nodes_count > 0) {
        size_t required_size = 0;
        err = nvs_get_blob(nvs_handle1, "nodes_array", NULL, &required_size);
        if (err != ESP_OK || required_size != scene_nodes_count * sizeof(node_mapping_t)) {
            ESP_LOGW("SCENE", "Array size mismatch, count=%d, got_size=%d", 
                     scene_nodes_count, (int)required_size);
            scene_nodes_count = 0;
            nvs_close(nvs_handle1);
            return ESP_OK;
        }

        // Load array
        err = nvs_get_blob(nvs_handle1, "nodes_array", scene_nodes, &required_size);
        if (err != ESP_OK) {
            ESP_LOGE("SCENE", "Failed to read array");
            scene_nodes_count = 0;
        }
    }

    ESP_LOGI("SCENE", "Loaded %d scene nodes", scene_nodes_count);
    for(int i=0; i<scene_nodes_count; i++){
        printf("|--group--|--scene--|--devid--|--room--|--module--|\n");
        printf("    %d    ", scene_nodes[i].group_id);
                   printf("    %d   ", scene_nodes[i].scene_id);
                             printf("    %d   ", scene_nodes[i].device_id);
                                       printf("    %d   ", scene_nodes[i].room_id);
                                                 printf("    %d    \n", scene_nodes[i].module_id);

    }
    nvs_close(nvs_handle1);
    return ESP_OK;
}



uint16_t generate_next_for_node(uint8_t room_id, uint8_t module_id, uint8_t device_id) {
    if (scene_nodes_count >= MAX_SCENE_NODES) {
        ESP_LOGE("SCENE", "Max nodes reached");
        return 0xFFFF;
    }
    
    // Check if node exists
    for (uint16_t idx = 0; idx < scene_nodes_count; idx++) {
        if (scene_nodes[idx].room_id == room_id && 
            scene_nodes[idx].module_id == module_id && 
            scene_nodes[idx].device_id == device_id) {
                printf("Already Exists g_id:%d  s_id:%d\n", scene_nodes[scene_idx].group_id, scene_nodes[scene_idx].scene_id);
												
            // Node already exists, return existing index
            return idx;
        }
    }
    
    // Create new node with next available IDs
    uint16_t idx = scene_nodes_count;
    if(scene_nodes_count > 0){
        idx = scene_nodes_count - 1;
    }
    scene_nodes[idx].room_id = room_id;
    scene_nodes[idx].module_id = module_id;
    scene_nodes[idx].device_id = device_id;
    if (scene_nodes[idx].scene_id < 255) {
        scene_nodes[idx].scene_id++;
    } else{
        scene_nodes[idx].scene_id = 1; // Reset to 1 if overflow
    }
    scene_nodes[idx].group_id++;  
    scene_nodes_count++;

    printf("Newly create g_id:%d  s_id:%d  cnts:%d\n", scene_nodes[scene_idx].group_id, scene_nodes[scene_idx].scene_id, scene_nodes_count);
												
    store_scene_nodes();
    return idx;
}

