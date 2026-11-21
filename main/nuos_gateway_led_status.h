#ifndef NUOS_GATEWAY_LED_STATUS_H
#define NUOS_GATEWAY_LED_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ESP32-C3 -> ESP32-H2 RGB LED status bridge.
 * The physical WS2812 is on H2 GPIO8. */
void nuosLedBleCommissioning(void);
void nuosLedBleConnected(void);
void nuosLedBleActivity(void);

#ifdef __cplusplus
}
#endif

#endif /* NUOS_GATEWAY_LED_STATUS_H */
