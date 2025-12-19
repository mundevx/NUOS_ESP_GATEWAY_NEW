

#include <esp_http_server.h>
#include <esp_log.h>
#include <stdio.h>
#include <esp_zigbee_type.h>
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <nuos_nvs.h>
#include "esp_zigbee_gateway.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG = "HTTP_SERVER";
//
////char webpage[46000];
////
//// Array to store devices
//extern light_bulb_device_params_t devices_zb_info[MAX_RECORDS];
//
//int num_devices = MAX_RECORDS; // Keep track of the number of devices
//
//extern nvs_handle_t nvs_handle1;
//
//
//
//#define MAX_DEVICES 10 // Set maximum number of devices
//#define MAX_JSON_LEN 2048
//
//
//
//const char *html_devices_2 =
//   	"{"
//           "ieee_addr: '00124b0012345678',"
//           "endpoint: [1, 2],"
//           "short_addr: '1234',"
//           "manuf_name: 'Manufacturer A',"
//           "model_id: 'Model X',"
//           "location_name: 'Living Room'"
//       "},"
//       "{"
//           "ieee_addr: '00124b0012345679',"
//           "endpoint: [1],"
//           "short_addr: '1235',"
//           "manuf_name: 'Manufacturer B',"
//           "model_id: 'Model Y',"
//           "location_name: 'Living Room'"
//		"},"
//		"{"
//           "ieee_addr: '00124b0012345680',"
//           "endpoint: [3],"
//           "short_addr: '1236',"
//           "manuf_name: 'Manufacturer C',"
//           "model_id: 'Model Z',"
//           "location_name: 'Bedroom'"
//       "},";
//const char *html_header =
//   "<!DOCTYPE html>"
//   "<html lang=\"en\">"
//   "<head>"
//   "<meta charset=\"UTF-8\">"
//   "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
//   "<title>Device Dashboard</title>"
//   "<style>"
//   "body { font-family: Arial, sans-serif; background-color: #f4f4f9; }"
//   "h1 { text-align: center; color: #333; }"
//   "#control-panel { display: flex; justify-content: center; margin: 20px; gap: 10px; }"
//   "#location-filter, #clear-all-btn, #commission-btn { padding: 10px; font-size: 16px; }"
//   ".device-container { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; padding: 20px; }"
//   ".device-box { width: 250px; height: 180px; border-radius: 10px; background-color: #ffffff; box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.1); padding: 20px; position: relative; transition: transform 0.3s ease; cursor: pointer; }"
//   ".device-box:hover { transform: scale(1.05); }"
//   ".location-name { position: absolute; top: 10px; left: 10px; font-weight: bold; font-size: 14px; color: #666; }"
//   ".device-info { display: flex; justify-content: center; align-items: center; flex-direction: column; height: 100%; }"
//   ".manuf-model { font-size: 20px; font-weight: bold; color: #333; text-align: center; }"
//   ".hidden-details { display: none; font-size: 12px; color: #999; }"
//   ".device-box.active .hidden-details { display: block; }"
//   ".delete-btn { position: absolute; bottom: 10px; right: 10px; padding: 5px 10px; font-size: 12px; background-color: #ff5c5c; color: white; border: none; border-radius: 5px; cursor: pointer; }"
//   ".delete-btn:hover { background-color: #ff2a2a; }"
//   "</style>"
//   "</head>"
//   "<body>"
//   "<h1>Device Dashboard</h1>"
//   "<div id=\"control-panel\">"
//   "<select id=\"location-filter\" onchange=\"filterDevices()\">"
//   "<option value=\"all\">All Locations</option>"
//   "</select>"
//   "<button id=\"clear-all-btn\" onclick=\"clearAllDevices()\">Clear All Records</button>"
//   "<button id=\"commission-btn\" onclick=\"toggleCommissioning()\">Start Commissioning</button>"
//   "</div>"
//   "<div class=\"device-container\" id=\"device-container\"></div>"
//	"<script>"
//	"const devices = [";
//
//const char *html_footer =
//   "let commissioning = false;"
//   "function loadDevices() {"
//   "    const container = document.getElementById('device-container');"
//   "    const locationFilter = document.getElementById('location-filter');"
//   "    container.innerHTML = '';"
//   "    let locations = new Set();"
//   "    devices.forEach(device => {"
//   "        locations.add(device.location_name);"
//   "        const box = document.createElement('div');"
//   "        box.classList.add('device-box');"
//   "        box.dataset.location = device.location_name;"
//   "        box.innerHTML = `"
//   "            <div class='location-name'>${device.location_name}</div>"
//   "            <div class='device-info'>"
//   "                <div class='manuf-model'>${device.manuf_name} ${device.model_id}</div>"
//   "                <div class='hidden-details'>"
//   "                    <p>IEEE Address: ${device.ieee_addr}</p>"
//   "                    <p>Short Address: ${device.short_addr}</p>"
//   "                    <p>Endpoints: ${device.endpoint.join(', ')}</p>"
//   "                </div>"
//   "                <button class='delete-btn' onclick='deleteDevice(\"${device.short_addr}\")'>Delete</button>"
//   "            </div>`;"
//   "        container.appendChild(box);"
//   "    });"
//   "    locationFilter.innerHTML += [...locations].map(location => `<option value='${location}'>${location}</option>`).join('');"
//   "}"
//   "function filterDevices() {"
//   "    const selectedLocation = document.getElementById('location-filter').value;"
//   "    const boxes = document.querySelectorAll('.device-box');"
//   "    boxes.forEach(box => {"
//   "        box.style.display = (selectedLocation === 'all' || box.dataset.location === selectedLocation) ? 'block' : 'none';"
//   "    });"
//   "}"
//   "function deleteDevice(short_addr) {"
//   "    fetch('/delete-device', {"
//   "        method: 'POST',"
//   "        headers: { 'Content-Type': 'application/json' },"
//   "        body: JSON.stringify({ short_addr })"
//   "    }).then(response => {"
//   "        if (response.ok) {"
//   "            alert('Device deleted successfully');"
//	"const deviceIndex = devices.findIndex(device => device.short_addr === short_addr);"
//	"devices.splice(deviceIndex, 1);"
//   "            loadDevices();"
//   "        } else {"
//   "            alert('Failed to delete device');"
//   "        }"
//   "    });"
//   "}"
//   "function clearAllDevices() {"
//   "    fetch('/clear-all', { method: 'POST' }).then(response => {"
//   "        if (response.ok) {"
//   "            alert('All records cleared');"
//   "            devices.length = 0;"
//   "            loadDevices();"
//   "        } else {"
//   "            alert('Failed to clear records');"
//   "        }"
//   "    });"
//   "}"
//   "function toggleCommissioning() {"
//   "    commissioning = !commissioning;"
//   "    const commissionBtn = document.getElementById('commission-btn');"
//   "    commissionBtn.textContent = commissioning ? 'Stop Commissioning' : 'Start Commissioning';"
//   "    fetch('/toggle-commissioning', {"
//   "        method: 'POST',"
//   "        headers: { 'Content-Type': 'application/json' },"
//   "        body: JSON.stringify({ commissioning })"
//   "    }).then(response => {"
//   "        if (response.ok) {"
//   "            alert(commissioning ? 'Commissioning started' : 'Commissioning stopped');"
//   "        } else {"
//   "            alert('Failed to toggle commissioning');"
//   "        }"
//   "    });"
//   "}"
//   "loadDevices();"
//   "</script>"
//   "</body>"
//   "</html>";
//
//
//extern char html_devices[MAX_JSON_LEN];  // Buffer to hold the JSON string
//// Web server handler to generate HTML
//esp_err_t generate_html_handler(httpd_req_t *req) {
//
//    strcpy(webpage, "");
//    // Send the HTML header
//    //httpd_resp_send_chunk(req, html_header, strlen(html_header));
//    strcat(webpage, html_header);
//    // Send the device information
//    //read_all_devices_and_convert_to_json();
//    //strcat(webpage, html_devices_2);
//    strcat(webpage, html_devices);
//   // printf("%s", html_devices_2);
//    // End the device JSON array
//    //httpd_resp_send_chunk(req, "];", 2);
//    strcat(webpage, "];");
//    // Send the closing footer of the HTML
//    //httpd_resp_send_chunk(req, html_footer, strlen(html_footer));
//    strcat(webpage, html_footer);
//    // Finish sending the response
//    //httpd_resp_sendstr(req, NULL); // Finalize the response
//    httpd_resp_send(req, webpage, strlen(webpage));
//    return ESP_OK;
//}
//
//// Handler to delete device based on ieee_addr
//esp_err_t delete_device_handler(httpd_req_t *req) {
//    char buf[100];
//    int ret = httpd_req_recv(req, buf, sizeof(buf)); // Receive data from the request
//    if (ret <= 0) {
//        return ESP_FAIL; // Error handling
//    }
//
//    // Parse JSON body
//    cJSON *json = cJSON_Parse(buf);
//    if (!json) {
//        return ESP_FAIL; // Error handling
//    }
//
//    // Get short_addr from the JSON object
//    const char *short_addr_str = cJSON_GetObjectItem(json, "short_addr")->valuestring;
//    uint16_t short_addr = (uint16_t)strtoul(short_addr_str, NULL, 16); // Convert hex string to uint16_t
//
//    int index = -1;
//    // Find and delete the device from the array
//    for (int i = 0; i < num_devices; i++) {
//        if (devices_zb_info[i].short_addr == short_addr) { // Compare short_addr instead of ieee_addr
//            // Shift the devices in the array
//            for (int j = i; j < num_devices - 1; j++) {
//            	devices_zb_info[j] = devices_zb_info[j + 1];
//            }
//            num_devices--; // Reduce count
//            index = i;
//            break;
//        }
//    }
//
//    cJSON_Delete(json);
//    if(index != -1){
//		esp_zb_zdo_mgmt_leave_req_param_t cmd_req;
//		memcpy(cmd_req.device_address, devices_zb_info[index].ieee_addr, sizeof(esp_zb_ieee_addr_t));
//		cmd_req.dst_nwk_addr = devices_zb_info[index].short_addr;
//		cmd_req.dst_nwk_addr = devices_zb_info[index].short_addr;
//		cmd_req.rejoin = 1;//leave_params->rejoin;
//		cmd_req.remove_children = 1; // Set to 1 if you want child devices to leave too
//		//esp_zb_zdo_device_leave_req(&cmd_req, esp_leave_callback, NULL);
//
//    	esp_err_t err = clear_single_record_from_nvs(index);
//    }
//
//    httpd_resp_sendstr(req, "Device deleted successfully");
//    return ESP_OK;
//}
//
//
//// Handler to clear all records
//esp_err_t clear_all_handler(httpd_req_t *req) {
//    num_devices = 0; // Clear all devices
//    clear_all_records_from_nvs();
//    printf("All RecordsCleared!!\n");
//    httpd_resp_sendstr(req, "All records cleared");
//    vTaskDelay(100);
//    esp_zb_factory_reset();
//    return ESP_OK;
//}
//
//// Handler to toggle commissioning
//esp_err_t toggle_commissioning_handler(httpd_req_t *req) {
//    char buf[100];
//    int ret = httpd_req_recv(req, buf, sizeof(buf));
//    if (ret <= 0) {
//        return ESP_FAIL; // Error handling
//    }
//
//    // Parse JSON to get commissioning state
//    cJSON *json = cJSON_Parse(buf);
//    if (!json) {
//        return ESP_FAIL; // Error handling
//    }
//
//    bool commissioning = cJSON_GetObjectItem(json, "commissioning")->valueint;
//    // Implement your commissioning logic here
//    cJSON_Delete(json);
//    httpd_resp_sendstr(req, commissioning ? "Commissioning started" : "Commissioning stopped");
//    return ESP_OK;
//}
//
//void start_webserver() {
//	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//	config.stack_size = 8192;  // Increase the stack size
//
//    httpd_handle_t server = NULL;
//
//    // Define URI handlers
//    httpd_uri_t uri_html = {
//        .uri = "/",
//        .method = HTTP_GET,
//        .handler = generate_html_handler,
//        .user_ctx = NULL
//    };
//
//    httpd_uri_t uri_delete_device = {
//        .uri = "/delete-device",
//        .method = HTTP_POST,
//        .handler = delete_device_handler,
//        .user_ctx = NULL
//    };
//
//    httpd_uri_t uri_clear_all = {
//        .uri = "/clear-all",
//        .method = HTTP_POST,
//        .handler = clear_all_handler,
//        .user_ctx = NULL
//    };
//
//    httpd_uri_t uri_toggle_commissioning = {
//        .uri = "/toggle-commissioning",
//        .method = HTTP_POST,
//        .handler = toggle_commissioning_handler,
//        .user_ctx = NULL
//    };
//
//    // Start the HTTP server
//    if (httpd_start(&server, &config) == ESP_OK) {
//        httpd_register_uri_handler(server, &uri_html);
//        httpd_register_uri_handler(server, &uri_delete_device);
//        httpd_register_uri_handler(server, &uri_clear_all);
//        httpd_register_uri_handler(server, &uri_toggle_commissioning);
//    }
//}
//
//
//void start_http_server(){
//	start_webserver();
//}


