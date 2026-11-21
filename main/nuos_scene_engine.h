#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * NUOS Scene Engine
 *
 * H2 owns all Zigbee ZCL execution. These APIs are intentionally independent
 * of the NUOS UART framing; the UART command parser calls them.
 */
esp_err_t nuos_scene_configure_member(uint16_t dst_short_addr,
                                       uint8_t dst_endpoint,
                                       uint16_t group_id,
                                       uint8_t scene_id);

esp_err_t nuos_scene_remove_member(uint16_t dst_short_addr,
                                   uint8_t dst_endpoint,
                                   uint16_t group_id,
                                   uint8_t scene_id);

esp_err_t nuos_scene_configure_controller(uint16_t dst_short_addr,
                                           uint8_t controller_endpoint,
                                           uint16_t group_id,
                                           uint8_t scene_id);

esp_err_t nuos_scene_recall_group(uint16_t group_id,
                                  uint8_t scene_id);

#ifdef __cplusplus
}
#endif
