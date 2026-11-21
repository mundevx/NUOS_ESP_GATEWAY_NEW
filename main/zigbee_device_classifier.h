#ifndef ZIGBEE_DEVICE_CLASSIFIER_H
#define ZIGBEE_DEVICE_CLASSIFIER_H

#include <stdint.h>
#include <stdbool.h>

/* Standard ZCL clusters used by the NUOS product set. */
#define ZCL_CLUSTER_ON_OFF             0x0006
#define ZCL_CLUSTER_LEVEL_CONTROL      0x0008
#define ZCL_CLUSTER_COLOR_CONTROL     0x0300
#define ZCL_CLUSTER_WINDOW_COVERING   0x0102
#define ZCL_CLUSTER_FAN_CONTROL       0x0202
#define ZCL_CLUSTER_THERMOSTAT        0x0201
#define ZCL_CLUSTER_TEMPERATURE       0x0402
#define ZCL_CLUSTER_HUMIDITY          0x0405
#define ZCL_CLUSTER_OCCUPANCY         0x0406
#define ZCL_CLUSTER_ILLUMINANCE       0x0400
#define ZCL_CLUSTER_IAS_ZONE          0x0500
#define ZCL_CLUSTER_POWER_CONFIG      0x0001

/* NUOS/Tuya application clusters present in the supplied product firmware. */
#define ZCL_CLUSTER_NUOS_E000         0xE000
#define ZCL_CLUSTER_NUOS_E001         0xE001
#define ZCL_CLUSTER_NUOS_ED00         0xED00
#define ZCL_CLUSTER_NUOS_EE00         0xEE00
#define ZCL_CLUSTER_NUOS_EF00         0xEF00
#define ZCL_CLUSTER_NUOS_FC31         0xFC31

/* Device capability result consumed by the NUOS V2 UART JSON layer. */
typedef struct {
    const char *device_type;
    bool power;
    bool brightness;
    bool color;
    bool color_temperature;
    bool curtain;
    bool fan;
    bool thermostat;
    bool temperature;
    bool humidity;
    bool occupancy;
    bool illuminance;
    bool contact;
    bool gas;
} device_capability_t;

void classify_zigbee_endpoint(uint16_t zigbee_device_type,
                              const uint16_t *clusters,
                              uint8_t cluster_count,
                              const char *manufacturer,
                              const char *model,
                              device_capability_t *capability);

#endif
