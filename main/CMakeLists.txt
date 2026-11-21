idf_component_register(SRCS         
        "esp_zigbee_gateway.c"
        "zigbee_uart_process.c"
        "nuos_uart_protocol.c"
        "nuos_nvs.c"
        "switch_driver.c"
	"nuos_scene_engine.c"
        "zigbee_device_classifier.c"
                    INCLUDE_DIRS "."
                    REQUIRES json mbedtls esp_timer driver nvs_flash)