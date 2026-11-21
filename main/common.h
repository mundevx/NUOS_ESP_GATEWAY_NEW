
//#pragma once

#include "nuos_data_structs.h"

#ifndef _NUOS_COMMON_
#define _NUOS_COMMON_

#define TASK_PRIORITY_ZB_MAIN  			                5
#define TASK_PRIORITY_SWITCH_PRESS		                10
#define TASK_PRIORITY_ZB_FIND_NODES  	                22
#define TASK_PRIORITY_ZB_COMMISSIONING	                18
#define TASK_PRIORITY_ZB_SEND_CMD		                11
#define TASK_PRIORITY_NVS_SAVE			                20

#define TASK_PRIORITY_UART_PROCESS                      14
#define TASK_PRIORITY_UART_TX                           12
#define TASK_PRIORITY_UART_RXD                          9
#define TASK_PRIORITY_XBEE_TASK                         16
#define TASK_PRIORITY_TIMEOUT_TASK                      19


#define MAX_SCENE_NODES                                 200

#define MAX_SWITCHES_TO_CONTROL    					    2
#define MAX_LOAD_TO_CONTROL    						    1
#define ZIGBEE_NW_REMAIN_OPEN_IN_SECS                   120


#ifdef _DECLARE_MAIN_

    #ifdef __cplusplus
    extern "C" {
    #endif
    // Function to find device by short_addr
    node_module_t *find_device_by_short_addr(uint16_t target_short_addr);
    bool find_device_by_dev_id(uint16_t room_id, uint16_t module_id, uint16_t target_dev_id, uint8_t* nodeindex, uint8_t* epindex);
    int find_node_index_by_short_addr(uint16_t target_short_addr);
    int find_node_index_by_ieee_addr(esp_zb_ieee_addr_t ieee);
    void nuos_update_level_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
    void nuos_update_onoff_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
    void nuos_update_color_temperature(uint8_t node_i, uint8_t ep_i, uint16_t temp);
    void nuos_update_color_hue(uint8_t node_i, uint8_t ep_i, uint16_t hue);
    void nuos_update_scene_add_cmd(uint8_t ep_i, uint16_t short_addr, uint8_t scene_id, uint16_t group_id);

    uint16_t find_device_type(uint8_t nodeindex, uint8_t epindex);
    uint8_t get_total_endpoints(uint8_t nodeindex);
   // void nuos_start_commissioning();



    //////////////////////////

    // esp_err_t nvs_store_or_update_scene(const device_scene_handle_t *new_scene);
    // esp_err_t nvs_read_scene_array(device_scene_handle_t *scene_handle);
    // esp_err_t nvs_read_scene_by_id(uint16_t group_id, uint8_t scene_id, device_scene_handle_t *scene);
    esp_err_t nvs_delete_scene(uint16_t group_id, uint8_t scene_id);
    void send_nuos_app_data_to_serial();
    bool lock_nvs_save									= false;
    void read_device_info();
    void set_commissioning_task(void* arg) ;
    
    void send_serial(const char* data);
    void send_uart_command(node_module_t *node, uint8_t ep, const char *cmd, uint8_t val);
    void gateway_leave_device(uint16_t device_short_addr, esp_zb_ieee_addr_t device_ieee_addr, uint8_t rejoin);
    esp_err_t esp_zigbee_read_manuf_code_and_model_id(void* device_void);
    uint8_t map_1000_to_255(uint16_t value16);
    uint16_t map_255_to_1000(uint8_t value8);

    bool data_from_uart[120] = {0};
    bool level_from_uart[120] = {0};
    bool color_from_uart[120] = {0};
    bool uart_frame_rcvd = false;
    void turn_on_commissioning_led_switch();
     
    #ifdef __cplusplus
    }
    #endif
   
    void nuos_stop_commissioning();
    // device_scene_handle_t  scene_data_t[SCENE_ARRAY_SIZE];
    //bool esp_zb_cmd_sent_flag = false;
    node_mapping_t scene_nodes[MAX_SCENE_NODES];
    uint16_t scene_nodes_count = 0;
    uint16_t scene_idx = 0;
   
#else
   
    #ifdef __cplusplus
    extern "C" {
    #endif
    // Function to find device by short_addr
    extern node_module_t *find_device_by_short_addr(uint16_t target_short_addr);
    extern bool find_device_by_dev_id(uint16_t room_id, uint16_t module_id, uint16_t target_dev_id, uint8_t* nodeindex, uint8_t* epindex);
    extern int find_node_index_by_short_addr(uint16_t target_short_addr);
    extern int find_node_index_by_ieee_addr(esp_zb_ieee_addr_t ieee);
    extern void nuos_update_level_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
    extern void nuos_update_onoff_cluster(uint8_t node_i, uint8_t ep_i, bool state);
    extern void nuos_update_color_temperature(uint8_t node_i, uint8_t ep_i, uint16_t temp);
    extern void nuos_update_color_hue(uint8_t node_i, uint8_t ep_i, uint16_t hue);
    extern void nuos_update_scene_add_cmd(uint8_t ep_i, uint16_t short_addr, uint8_t scene_id, uint16_t group_id);
    extern uint16_t find_device_type(uint8_t nodeindex, uint8_t epindex);
    extern uint8_t get_total_endpoints(uint8_t nodeindex);
    extern void nuos_start_commissioning(uint16_t timeout_seconds);
    extern void nuos_stop_commissioning();
    extern void gateway_leave_device(uint16_t device_short_addr, esp_zb_ieee_addr_t device_ieee_addr, uint8_t rejoin);
    extern esp_err_t esp_zigbee_read_manuf_code_and_model_id(void* device_void);
    //////////////////////////

    extern void write_custom_cluster_attr(uint16_t cluster_id, uint8_t custom_cmd_id, uint16_t dst_short_addr, uint8_t dst_ep, uint8_t custom_value);
    extern void zigbee_cluster_thermostat_state_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t state);

    void send_nuos_app_data_to_serial();


    extern bool lock_nvs_save;

    extern void send_all_data_to_serial();
    extern void read_device_info();
    // extern void nuos_zb_leave_device(uint16_t module_id, uint16_t did);
    extern void zigbee_cluster_fan_control_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t speed);
    extern void zigbee_cluster_thermostat_temp_attr_update(uint8_t node_i, uint8_t ep_i, uint8_t temp);
    extern bool is_fan_control(uint8_t nodeindex, uint8_t epindex);
    extern bool is_thermostat_control(uint8_t nodeindex, uint8_t epindex);
    extern uint8_t fan_control_type(uint8_t nodeindex, uint8_t epindex);
    extern void nuos_update_curtain_cluster(uint8_t node_i, uint8_t ep_i, uint8_t curtain_state);
    extern void nuos_send_window_covering_command(uint8_t node_i, uint8_t ep_i, uint8_t command);
    extern void nuos_send_window_covering_lift_percent(uint8_t node_i, uint8_t ep_i, uint8_t percent);
    extern void send_serial(const char* data);
    extern void send_uart_command(node_module_t *node, uint8_t ep, const char *cmd, uint8_t val);

    extern void read_device_info();
    extern void set_commissioning_task(void* arg) ;
    extern void nuos_update_cct_color_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
    extern void nuos_update_cct_level_cluster(uint8_t node_i, uint8_t ep_i, uint8_t level);
    extern bool data_from_uart[120];
    extern bool level_from_uart[120];
    extern bool color_from_uart[120];
    extern bool uart_frame_rcvd;
    extern void turn_on_commissioning_led_switch(); 
    #ifdef __cplusplus
    }
    #endif

    // extern device_scene_handle_t  scene_data_t[SCENE_ARRAY_SIZE];
    //extern bool esp_zb_cmd_sent_flag;
    extern node_mapping_t scene_nodes[MAX_SCENE_NODES];
    extern uint16_t scene_nodes_count;
    extern uint16_t scene_idx;
    
#endif

enum DEVICE_TYPE
{
    DEVICE_TYPE_INVALID,                                    // 0
    DEVICE_TYPE_ON_OFF_FAN,                                 // 1
    DEVICE_TYPE_ON_OFF_LED,                                 // 2
    DEVICE_TYPE_ON_OFF_BULB,                                // 3
    DEVICE_TYPE_ON_OFF_CFL,                                 // 4
    DEVICE_TYPE_ON_OFF_RESERVED,                            // 5
    DEVICE_TYPE_DIMMABLE_FAN,                               // 6
    DEVICE_TYPE_DIMMABLE_TRIAC,                             // 7
    DEVICE_TYPE_DIMMABLE_Z10,                               // 8
    DEVICE_TYPE_DIMMABLE_TE,                                // 9
    DEVICE_TYPE_DIMMABLE_PROTOCOL,                          // 10
    DEVICE_TYPE_AC,                                         // 11
    
    DEVICE_TYPE_TEMPERATURE_SENSOR,                         // 12
    DEVICE_TYPE_HUMIDITY_SENSOR,                            // 13
    DEVICE_TYPE_DOOR_SENSOR,                                // 14
    DEVICE_TYPE_DOOR_VOLTAGE,                               // 15
    DEVICE_TYPE_OCCUPANCY_SENSOR,                           // 16
    DEVICE_TYPE_WIRED_DOOR_LOCK,                            // 17
    DEVICE_TYPE_CURTAIN_MOTOR,                              // 18
    DEVICE_TYPE_SOCKET,                                     // 19
    DEVICE_TYPE_CHANDELIER,                                 // 20
    DEVICE_TYPE_GEYSER,                                     // 21
    DEVICE_TYPE_GENERIC_SENSOR,                             // 22
    DEVICE_TYPE_TV,                                         // 23
    DEVICE_TYPE_DOORBELL,                                   // 24
    DEVICE_TYPE_REFRIDGERATOR,                              // 25
    
    DEVICE_TYPE_OCCUPANCY_ON_OFF_TIME_MIN,                  // 26
    DEVICE_TYPE_LUX_SENSOR,                                 // 27
    DEVICE_TYPE_SMOKE_SENSOR,                               // 28
    DEVICE_TYPE_CO2_SENSOR,                                 // 29
    DEVICE_TYPE_AIRQ_SENSOR,                                // 30
    DEVICE_TYPE_COLOR_TEMP_SENSOR,                          // 31
    DEVICE_TYPE_VRMS,                                       // 32
    DEVICE_TYPE_IRMS,                                       // 33
    DEVICE_TYPE_ACTIVE_PWR,                                 // 34
    DEVICE_TYPE_APARANT_PWR,                                // 35
    DEVICE_TYPE_PF,                                         // 36
    DEVICE_TYPE_FREQ,                                       // 37
    DEVICE_TYPE_CAMERA,                                     // 38
    DEVICE_TYPE_RFID,                                       // 39
    DEVICE_TYPE_SCENE,                                      // 40
    DEVICE_TYPE_SCENE_PAIR,                                 // 41
    DEVICE_TYPE_GAS_LEAK_SENSOR,                            // 42
    
    DEVICE_TYPE_IR_TEMP_SET,                                // 43
    DEVICE_TYPE_IR_POWER_ONOFF,                             // 44
    DEVICE_TYPE_IR_VOLUME_UPDOWN,                           // 45
    DEVICE_TYPE_IR_CHANNEL_UPDOWN,                          // 46
    DEVICE_TYPE_IR_CHANNEL,                                 // 47
    DEVICE_TYPE_SOURCE_UPDOWN,                              // 48
    
    DEVICE_TYPE_OC_POWER,                                   // 49  
    DEVICE_TYPE_OC_FLOW,                                    // 50  
    DEVICE_TYPE_OC_O2,                                      // 51  
    DEVICE_TYPE_OC_TEMPERATURE,                             // 52  
    DEVICE_TYPE_OC_ELAPSED_TIME,                            // 53  
    DEVICE_TYPE_OC_FLOW_SETPOINT,                           // 54
    
    DEVICE_TYPE_CURTAIN_MOTOR_OPEN,                         // 55
    DEVICE_TYPE_CURTAIN_MOTOR_CLOSE,                        // 56
    
    // BMS Device types
    DEVICE_TYPE_BMS_ESCALATOR_ONOFF_STATUS,                 // 57
    DEVICE_TYPE_BMS_ESCALATOR_DIRECTION,                    // 58
    DEVICE_TYPE_BMS_ESCALATOR_TRIP_STATUS,                  // 59
    DEVICE_TYPE_BMS_ESCALATOR_EMERGENCY_STOP,               // 60
    DEVICE_TYPE_BMS_ESCALATOR_MAINTENANCE,                  // 61
    DEVICE_TYPE_BMS_ESCALATOR_SPEED,                        // 62
    
    DEVICE_TYPE_BMS_LIFT_ALARM,                             // 63
    DEVICE_TYPE_BMS_LIFT_POWER,                             // 64
    DEVICE_TYPE_BMS_LIFT_MAINTENANCE,                       // 65
    DEVICE_TYPE_BMS_LIFT_FAULT,                             // 66
    DEVICE_TYPE_BMS_LIFT_STATUS,                            // 67
    DEVICE_TYPE_BMS_LIFT_DIRECTION,                         // 68
    
    DEVICE_TYPE_BMS_LIGHT,                                  // 69
    DEVICE_TYPE_BMS_LIGHT_HEALTHY_STATUS,                   // 70
    
    DEVICE_TYPE_BMS_FIRE_SAFETY_PANEL_HEALTHY,              // 71 
    DEVICE_TYPE_BMS_FIRE_SAFETY_FIRE_ACTIVATION,            // 72
    DEVICE_TYPE_BMS_FIRE_SAFETY_GAS_FLOODING,               // 73
    
    DEVICE_TYPE_BMS_FIREHYDRANT_PUMP_STATUS,                // 74  (6 bits)
        // 1-bit for Hydrant Pump On/Off Status, 
        // 1-bit for Sprinkler Pump On/Off Status, 
        // 1-bit for Jockey Pump On/Off Status, 
        // 1-bit for Fire Pump Run Status
        // 1-bit for Fire Pump Trip Status
        // 1-bit for Fire Pump Auto/Manual Status
    DEVICE_TYPE_BMS_FIREHYDRANT_DOMESTIC_PUMP_LEVEL,        // 75
    DEVICE_TYPE_BMS_FIREHYDRANT_FIREHYDRANT_PUMP_LEVEL,     // 76   NA
    DEVICE_TYPE_BMS_FIREHYDRANT_FIREHYDRANT_PUMP_PRESSURE,  // 77   NA
    DEVICE_TYPE_BMS_FIREHYDRANT_DOMESTIC_PUMP_LEVEL1,       // 78   NA
    DEVICE_TYPE_BMS_FIREHYDRANT_FIREHYDRANT_PUMP_LEVEL1,     // 79   NA
    
    DEVICE_TYPE_BMS_FM200_SYSTEM_HEALTHY_STATUS,            // 80
    DEVICE_TYPE_BMS_FM200_SYSTEM_FIRE_ACTIVATION,           // 81
    
    DEVICE_TYPE_BMS_SLPITACSYSTEM_ONOFF_CONTROL,            // 82
    DEVICE_TYPE_BMS_SLPITACSYSTEM_ONOFF_STATUS,             // 83
    DEVICE_TYPE_BMS_SLPITACSYSTEM_FAULT_STATUS,             // 84
    
    DEVICE_TYPE_BMS_LTPANEL_PhaseId,                        // 85
    DEVICE_TYPE_BMS_LTPANEL_VoltagePhase,                   // 86
    DEVICE_TYPE_BMS_LTPANEL_CurrentPhase,                   // 87
    DEVICE_TYPE_BMS_LTPANEL_FREQUENCY,                      // 88
    DEVICE_TYPE_BMS_LTPANEL_POWERCONSUMPTION,               // 89
    DEVICE_TYPE_BMS_LTPANEL_POWERFACTOR,                    // 90
    
    DEVICE_TYPE_BMS_UPS_PhaseId,                            // 91   
    DEVICE_TYPE_BMS_UPS_Input2LineVoltageRMSPhase,          // 92   30050, 30051, 30052 
    DEVICE_TYPE_BMS_UPS_InputCurrentRMSPhase,               // 93   30005, 30006, 30007
    DEVICE_TYPE_BMS_UPS_Output2LineVoltageRMSPhase,         // 94   30056, 30057, 30058
    DEVICE_TYPE_BMS_UPS_OutputCurrentRMSPhase,              // 95   30015, 30016, 30017
    DEVICE_TYPE_BMS_UPS_OutputPercentagePhase,              // 96   30021, 30022, 30023

    DEVICE_TYPE_BMS_UPS_OutputPowerKWPhase,                 // 97   30018, 30019, 30020
    DEVICE_TYPE_BMS_UPS_OutputPowerKVAPhase,                // 98   30047, 30048, 30049
    DEVICE_TYPE_BMS_UPS_OutputFrequency,                    // 99   30014
    DEVICE_TYPE_BMS_UPS_DCNegativeVoltage,                  // 100  30031
    DEVICE_TYPE_BMS_UPS_DCPositiveVoltage,                  // 101  30032
    
    DEVICE_TYPE_BMS_UPS_BatteryLevelPercentage,             // 102  30037
    DEVICE_TYPE_BMS_UPS_BatteryRemainTime,                  // 103  30038
    DEVICE_TYPE_BMS_UPS_SystemStatus,                       // 104  30071
        // 0 = Initialize
        // 1 = Standby Operation
        // 2 = Output is Off
        // 3 = Bypass Operation
        // 4 = Normal Operation
        // 5 = Battery Operation
        // 6 = Battery Self-test
          
        // 8 = ECO Mode
        // 9 = EPO Activated
        // 10 = Maintenance Bypass Operation
          
        // 13 = Hybrid Operation
        // 14 = Self-aging Mode
          
        // 20 = REC is OFF
        // 21 = Input
        // 22 = On Battery
          
        // 30 = Inverter Standby
        // 31 = Inverter Output
        // 32 = Inverterself-aging
          
        // 40 = ECU is Off
        // 41 = ECU Standby
        // 42 = ECU Active
    DEVICE_TYPE_BMS_UPS_ErrorBits1,                         // 105  
        // 0 bit for Rectifier Inoperable                           // 10065
        // 1 bit for Inverter Inoperable                            // 10066
        // 2 bit for REC Power Supply Lost                          // 10067   
        // 3 bit for Fan Inoperable                                 // 10068
        // 4 bit for Input SCR Inoperable                           // 10069
        // 5 bit for INV Relay Short-circuit                        // 10070
        // 6 bit for Bypass SCR Shorted                             // 10071
        // 7 bit for INV Fuse Blown                                 // 10072
        // 8 bit for REC Cannot Initialize                          // 10074
        // 9 bit for Charger SCR Broken                             // 10075
        // 10 bit for Charger inoperable                            // 10076
        // 11 bit for Parallel Alarm                                // 10077
        // 12 bit for Input Phases Reversed                         // 10080
        // 13 bit for Neutral Missing on Input                      // 10081
        // 14 bit for Bypass Phases Reversed                        // 10082
        // 15 bit for Battery Polarity Reversed                     // 10083
    DEVICE_TYPE_BMS_UPS_ErrorBits2,                         // 106 
        // 0 bit for No Battery Connected                           // 10084
        // 1 bit for Backfeed on Bypass                             // 10085
        // 2 bit for Inverter Overload                              // 10088
        // 3 bit for Parallel Overload                              // 10089
        // 4 bit for Battery End of Discharge                       // 10090
        // 5 bit for Battery Overvoltage                            // 10091
        // 6 bit for Low Battery Warning                            // 10092
        // 7 bit for Bypass Out of Sync                             // 10095
        // 8 bit for Bypass Not Available                           // 10096
        // 9 bit for LBS Communication Lost                         // 10098
        // 10 bit for Inverter Out of Sync                          // 10099
        // 11 bit for Short Circuit on Output                       // 10114
        // 12 bit for Parallel Cable Damaged                        // 10121
        // 13 bit for Bypass SCR Broken                             // 10126
        // 14 bit for Bypass Over-temperature                       // 10127
        // 15 bit for Bypass Overload                               // 10147
    DEVICE_TYPE_BMS_UPS_ErrorBits3,                         // 107  
        // 0 bit for Incorrect Use of MBB                           // 10148
        // 1 bit for Parallel Cable Abnormal                        // 10150
        // 2 bit for Parallel Cable Missing                         // 10154
        // 3 bit for Service Recommended                            // 10159
        // 4 bit for Battery Temperature >45°C or <-20°C            // 10163
        // 5 bit for Room Temperature >40°C or <-20°C               // 10164
        // 6 bit for Mains Input Voltage or Frequency Out of Range  // 10201
        // 7 bit for Battery Voltage Low                            // 10202
        // 8 bit for UPS Important alarm                            // 10204
        // 9 bit for UPS warning                                    // 10206
        // 10 bit for UPS over temperature                          // 10207
    
    DEVICE_TYPE_BMS_DG_PhaseId,                             // 108  
    DEVICE_TYPE_BMS_DG_Mains2LineVoltageRMSPhase,           // 109  // 1068, 1070, 1072 - 2res
    DEVICE_TYPE_BMS_DG_MainsLineVoltageRMSPhase,            // 110  // 1062, 1064, 1066 - 2res
    DEVICE_TYPE_BMS_DG_MainsCurrentRMSPhase,                // 111  // 1078, 1080, 1082 - 2res
    DEVICE_TYPE_BMS_DG_MainsFrequency,                      // 112  // 1060             - 1res
    DEVICE_TYPE_BMS_DG_MainsEarthCurrent,                   // 113  // 1084             - 2res
    
    // DEVICE_TYPE_BMS_DG_PhaseId,                             
    DEVICE_TYPE_BMS_DG_Generator2LineVoltageRMSPhase,       // 114  // 1040, 1042, 1044 - 2res
    DEVICE_TYPE_BMS_DG_GeneratorLineVoltageRMSPhase,        // 115  // 1034, 1036, 1038 - 2res
    DEVICE_TYPE_BMS_DG_GeneratorCurrentRMSPhase,            // 116  // 1046, 1048, 1050 - 2res
    DEVICE_TYPE_BMS_DG_GeneratorFrequency,                  // 117  // 1032             - 1res
    DEVICE_TYPE_BMS_DG_GeneratorEarthCurrent,               // 118  // 1052             - 2res

    // DEVICE_TYPE_BMS_DG_PhaseId, 
    DEVICE_TYPE_BMS_DG_MainsLoadWatts,                      // 119  // 1085, 1087, 1089 - 2res
    DEVICE_TYPE_BMS_DG_GeneratorLoadWatts,                  // 120  // 1054, 1056, 1058 - 2res
    DEVICE_TYPE_BMS_DG_GeneratorPF,                         // 121  // 1555, 1556, 1557 - 1res
    DEVICE_TYPE_BMS_DG_GeneratorLoadVAr,                    // 122  // 1547, 1549, 1551 - 2res
    DEVICE_TYPE_BMS_DG_GeneratorTotalVAr,                   // 123  // 1553             - 2res
    
    DEVICE_TYPE_BMS_DG_GeneratorPositiveKWh,                // 124  // 1801             - 2res
    DEVICE_TYPE_BMS_DG_GeneratorPositiveKVAh,               // 125  // 1805             - 2res
    DEVICE_TYPE_BMS_DG_GeneratorPositiveKVArh,              // 126  // 1807             - 2res
    DEVICE_TYPE_BMS_DG_EngineSpeed,                         // 127  // 1031             - 1res
    DEVICE_TYPE_BMS_DG_ChargeAlternatorVoltage,             // 128  // 1029             - 1res
    DEVICE_TYPE_BMS_DG_EngineBatteryVoltage,                // 129  // 1030             - 1res
    
    DEVICE_TYPE_BMS_DG_OilPressure,                         // 130  // 1025             - 1res
    DEVICE_TYPE_BMS_DG_CoolantTemperature,                  // 131  // 1026             - 1res
    DEVICE_TYPE_BMS_DG_EngineRunHours,                      // 132  // 1798             - 2res
    DEVICE_TYPE_BMS_DG_FuelLevel,                           // 133  // 1028             - 1res
    DEVICE_TYPE_BMS_DG_Status,                              // 134  
    DEVICE_TYPE_BMS_DG_ErrorBit,                            // 135  (7 bits)
        // 0 bit for AlternatorFieldFailure
        // 1 bit for TripDueToEarthFault
        // 2 bit for TripDueToOverload
        // 3 bit for AlternatorFailToBuildUpVoltage
        // 4 bit for AutoSynchronisingPanelStatus
        // 5 bit for EngineCommonAlarm
        // 6 bit for EngineFailToStart

    DEVICE_TYPE_BMS_FAN_RunStatus,                          // 136
    DEVICE_TYPE_BMS_FAN_OnOffCommand,                       // 137
    
    DEVICE_TYPE_BMS_ACCESSCONTROL_OnlineStatus,             // 138
    
    DEVICE_TYPE_BMS_WATERPUMP_HealthyStatus,                // 139
    DEVICE_TYPE_BMS_WATERPUMP_OnOffStatus,                  // 140

    DEVICE_TYPE_BMS_ANEMOMETER,                             // 141
    
    DEVICE_TYPE_BMS_TEMPERATURE,                            // 142
    DEVICE_TYPE_BMS_HUMIDITY,                               // 143
    
    DEVICE_TYPE_BMS_SEISMOMETER,                            // 144

    DEVICE_TYPE_BMS_VESDA_PANEL_HEALTHY,                    // 145

    DEVICE_TYPE_BMS_HARDPOINTS,                             // 146

    DEVICE_TYPE_BMS_LTPANEL_POWER_KWH,                      // 147
    DEVICE_TYPE_BMS_LTPANEL_POWER_KW,                       // 148

    DEVICE_TYPE_BMS_LTPANEL_SLD_HARDPOINT,                  // 149

    DEVICE_TYPE_BMS_DG_Error1,                              // 150
    DEVICE_TYPE_BMS_DG_Error2,                              // 151
    DEVICE_TYPE_BMS_DG_Error3,                              // 152
    DEVICE_TYPE_BMS_DG_Error4,                              // 153
    DEVICE_TYPE_BMS_DG_Error5,                              // 154
    DEVICE_TYPE_BMS_DG_Error6,                              // 155
    DEVICE_TYPE_BMS_DG_Error7,                              // 156

    DEVICE_TYPE_BMS_UPS_ErrorBits1_40kVA,                   // 157 
    DEVICE_TYPE_BMS_UPS_ErrorBits2_40kVA,                   // 158 
    DEVICE_TYPE_BMS_UPS_ErrorBits3_40kVA,                   // 159

    DEVICE_TYPE_BMS_LTPANEL_SLD_CONTROL,                    // 160

    DEVICE_TYPE_BMS_STULZPRECISION_ACSYSTEM_ONOFF_STATUS,   // 161
    DEVICE_TYPE_BMS_STULZPRECISION_ACSYSTEM_FAULT_STATUS,   // 162
    DEVICE_TYPE_BMS_STULZPRECISION_ACSYSTEM_UNIT_TEMP,      // 163
    DEVICE_TYPE_BMS_STULZPRECISION_ACSYSTEM_UNIT_HUMIDITY,  // 164

    DEVICE_TYPE_BMS_UPS_ErrorBits1_Vertiv,                  // 165 
    DEVICE_TYPE_BMS_UPS_ErrorBits2_Vertiv,                  // 166 
    DEVICE_TYPE_BMS_UPS_ErrorBits3_Vertiv,                  // 167

    DEVICE_TYPE_BMS_FIREHYDRANT_PUMP_STATUS_MANYPUMPS,      // 168  (6 bits)

    DEVICE_TYPE_BMS_LTPANEL_SLD_HARDPOINT_AUTOMANUAL,       // 169

    DEVICE_TYPE_BMS_TOSHIBA_ACSYSTEM_ID,                    // 170
    DEVICE_TYPE_BMS_TOSHIBA_ACSYSTEM_ONOFF_STATUS,          // 171
    DEVICE_TYPE_BMS_TOSHIBA_ACSYSTEM_FAULT_STATUS,          // 172
    DEVICE_TYPE_BMS_TOSHIBA_ACSYSTEM_UNIT_TEMP,             // 173
    DEVICE_TYPE_BMS_TOSHIBA_ACSYSTEM_UNIT_TEMP_SETTING,     // 174
    
    DEVICE_TYPE_MAX_COUNT                                   // 175
};

enum MODULE_TYPE
{
    MODULE_TYPE_INVALID,                // 0
    MODULE_TYPE_CONTROLLER,             // 1
    MODULE_TYPE_IR_WITH_ECOSENSE,       // 2
    MODULE_TYPE_DOOR,                   // 3
    MODULE_TYPE_MOTION,                 // 4
    MODULE_TYPE_OCCUPANCY,              // 5
    MODULE_TYPE_FAN,                    // 6
    MODULE_TYPE_POWER_METER,            // 7
    MODULE_TYPE_GENERIC_SENSOR,         // 8
    MODULE_TYPE_LUX_SENSOR,             // 9
    MODULE_TYPE_TEMP_HUMIDITY_SENSOR,   // 10
    MODULE_TYPE_SMOKE_SENSOR,           // 11
    MODULE_TYPE_CO2_SENSOR,             // 12
    MODULE_TYPE_AIRQ_SENSOR,            // 13
    MODULE_TYPE_COLOR_TEMP_SENSOR,      // 14
    MODULE_TYPE_CAMERA,                 // 15
    MODULE_TYPE_RFID,                   // 16
    MODULE_TYPE_SCENE,                  // 17
    MODULE_TYPE_GAS_LEAK_SENSOR,        // 18
    MODULE_TYPE_SCENE_GATEWAY_SAVE,     // 19
    MODULE_TYPE_IR_BLASTER,             // 20
    MODULE_TYPE_OXYGEN_CONCENTRATOR,    // 21
    MODULE_TYPE_RGBW,                   // 22
    
    // BMS Module Types
    MODULE_TYPE_BMS_ESCALATOR,          // 23   // System 1
    MODULE_TYPE_BMS_LIFT,               // 24   // System 2 
    MODULE_TYPE_BMS_LIGHTING,           // 25   // System 3
    MODULE_TYPE_BMS_FIRESAFTY,          // 26   // System 4
    MODULE_TYPE_BMS_FIREHYDRANT,        // 27   // System 5
    MODULE_TYPE_BMS_FM200VesdaSYSTEM,   // 28   // System 4
    MODULE_TYPE_BMS_SLPITACSYSTEM,      // 29   // System 6
    MODULE_TYPE_BMS_LTPANEL,            // 30   // System 7
    MODULE_TYPE_BMS_UPS,                // 31   // System 8
        MODULE_TYPE_BMS_UP1,            // 32
        MODULE_TYPE_BMS_UP2,            // 33
        MODULE_TYPE_BMS_UP3,            // 34
    MODULE_TYPE_BMS_DG,                 // 35   // System 9
        MODULE_TYPE_BMS_DG1,            // 36
        MODULE_TYPE_BMS_DG2,            // 37
        MODULE_TYPE_BMS_DG3,            // 38
        MODULE_TYPE_BMS_DG4,            // 39
        MODULE_TYPE_BMS_DG5,            // 40
    MODULE_TYPE_BMS_FAN,                // 41   // System 10
    MODULE_TYPE_BMS_ACCESSCONTROL,      // 42   // System NA
    MODULE_TYPE_BMS_WATERPUMP,          // 43   // System NA
    MODULE_TYPE_BMS_ANEMOMETER,         // 44   // System 3
    MODULE_TYPE_BMS_TEMPERATURE,        // 45   // System 3
    MODULE_TYPE_BMS_SEISMOMETER,        // 46   // System 3
    MODULE_TYPE_BMS_HARDPOINTS,         // 47   // System NA
    MODULE_TYPE_BMS_ANEMSEISCOMBINED,   // 48   // System NA
    MODULE_TYPE_BMS_LTPANEL_POWER,      // 49   // System NA
    MODULE_TYPE_BMS_LTPANEL_SLD,        // 50   // System NA
    MODULE_TYPE_BMS_UP40,               // 51
    MODULE_TYPE_BMS_DG6,                // 52
    MODULE_TYPE_BMS_SLDCONTROL,         // 53
    MODULE_TYPE_BMS_TANKLEVELPRESSURE,  // 54
    MODULE_TYPE_BMS_STULZPRECISIONAC,   // 55
    MODULE_TYPE_BMS_TANKLEVEL,          // 56
    MODULE_TYPE_BMS_LTPANEL_SLD_AUTOMANUAL,         // 57
    MODULE_TYPE_BMS_TOSHIBAAC,          // 58

    MODULE_TYPE_MAX_COUNT               
};
enum TOPIC
{
    // connection message
    TOPIC_JS00,
    TOPIC_CONN,     // gateway state json
    TOPIC_SCHD,     // gateway schedule json
    TOPIC_WIFI,     // gateway wifi json
    TOPIC_PWRS,     // gateway module devices power json
    TOPIC_UNCN,     // gateway unconfigured module json
    TOPIC_HTBJ,     // gateway heartbeat module json
    TOPIC_RSSJ,     // gateway modules rssi module json
    TOPIC_LIFJ,     // lift on/down time json
    TOPIC_ESCJ,     // escalator on/down time json
    TOPIC_MODX,     // gateway module address json
    TOPIC_LCAL,     // local mode or cloud mode
    
    TOPIC_RSTG,     // reset gateway
    TOPIC_STIP,     // updating static ip
    TOPIC_GTWI,     // identify gateway
    TOPIC_CHLK,     // child lock code enable/disable
    TOPIC_CHLD,     // child lock code json synce
    
    TOPIC_RNGM,     // rename module xbees with rId and mId
    TOPIC_STAT,     // update/sync from module after powerup
    TOPIC_DBUG,     // debuge received from Controller/Sensor Module
    TOPIC_UNFL,     // filter particular unconfigured module
    TOPIC_UNOK,     // ignore unconfigured module
    TOPIC_RSTM,     // reset module   
    TOPIC_CHMT,     // change module type
    TOPIC_LGFL,     // filter and show the log msmg only of the module type
    TOPIC_LGOK,     // ignore filter and show log msmgs from all of the module type 
    TOPIC_UPDB,     // update module address and room id and module id in db

    TOPIC_ERRR,     // error msmg
    TOPIC_MSMG,     // msmg to user app
    TOPIC_WHAT,     // topic to know the module type in text
    
    TOPIC_ATXS,     // at command to xbee self
    TOPIC_ATXR,     // at command to remote xbee
    TOPIC_ATXA,     // at command to all xbee's
    
    TOPIC_HTBT,     // heartbeat received from module
    TOPIC_IDEN,     // identify the unconfigured module
    
    TOPIC_GTWA,     // add/edit/delete gateway - only delete gateway can be called from the phone
    TOPIC_GTWE,  
    TOPIC_GTWD,   
    
    TOPIC_ROMA,     // add/edit/delete room - only add and delete can be called from the phone
    TOPIC_ROME,  
    TOPIC_ROMD,   
    
    TOPIC_ADDD,     // add/edit/delete device
    TOPIC_EDTD,   
    TOPIC_DELD,   
    
    TOPIC_AD00,
    TOPIC_ADDC,     // add/edit/delete controller module
    TOPIC_ADDI,     // add/edit/delete ac/ecosense module
    TOPIC_ADDO,     // add/edit/delete door module
    TOPIC_ADDM,     // add/edit/delete motion module
    TOPIC_ADDU,     // add/edit/delete occupancy module
    TOPIC_ADDF,     // add/edit/delete fan module
    TOPIC_ADDP,     // add/edit/delete power measurement module
    TOPIC_ADDS,     // add/edit/delete generic sensor module
    TOPIC_ADDL,     // add/edit/delete lux sensor module
    TOPIC_ADDE,     // add/edit/delete temp & humidity (ecosense) sensor module
    TOPIC_ADDK,     // add/edit/delete smoke sensor module
    TOPIC_ADD2,     // add/edit/delete co2 sensor module
    TOPIC_ADDQ,     // add/edit/delete air quality sensor module
    TOPIC_ADDY,     // add/edit/delete color temp sensor module
    TOPIC_ADDV,     // add/edit/delete camera OV module
    TOPIC_ADDR,     // add/edit/delete rfid module
    TOPIC_ADD4,     // add/edit/delete scene control module
    TOPIC_ADDG,     // add/edit/delete gas leak sensor module
    TOPIC_ADD8,     // add/edit/delete scene gateway save control module
    TOPIC_ADDB,     // add/edit/delete ir blaster module
    TOPIC_ADD0,     // add/edit/delete oxygen concentrator module
    TOPIC_ADD3,     // add/edit/delete rgb module
    TOPIC_ADES,     // add/edit/delete escalator       
    TOPIC_ADLF,     // add/edit/delete lift
    TOPIC_ADLI,     // add/edit/delete lighting
    TOPIC_ADFR,     // add/edit/delete fire safety system
    TOPIC_ADFH,     // add/edit/delete fire hydrant
    TOPIC_ADFV,     // add/edit/delete fm200
    TOPIC_ADAC,     // add/edit/delete split ac
    TOPIC_ADLT,     // add/edit/delete lt panel
    TOPIC_ADUP,     // add/edit/delete ups
        TOPIC_ADU1,
        TOPIC_ADU2,
        TOPIC_ADU3,
    TOPIC_ADGS,     // add/edit/delete disel genset
        TOPIC_ADG1,
        TOPIC_ADG2,
        TOPIC_ADG3,
        TOPIC_ADG4,
        TOPIC_ADG5,
        TOPIC_ADG6,
    TOPIC_ADFN,     // add/edit/delete fan system
    TOPIC_ADAS,     // add/edit/delete access control system
    TOPIC_ADWP,     // add/edit/delete water pump system
    TOPIC_ADAN,     // add/edit/delete anemometer
    TOPIC_ADTH,     // add/edit/delete temperature humidity sensor
    TOPIC_ADSE,     // add/edit/delete seismometer sensor
    TOPIC_ADHP,     // add/edit/delete hard points
    TOPIC_ADCS,     // add/edit/delete combined sensor of anemometer and seismometer
    TOPIC_ADL1,     // add/edit/delete lt panel power
    TOPIC_ADSL,     // add/edit/delete lt panel sld
    TOPIC_ADSC,     // add/edit/delete lt panel sld control
    TOPIC_ADLP,     // add/edit/delete tank level and pressure sensor
    TOPIC_ADST,     // add/edit/delete Stulz precision AC
    TOPIC_ADLS,     // add/edit/delete tank level sensor
    TOPIC_ADAM,     // add/edit/delete lt panel sld auto manual status
    TOPIC_ADAT,     // add/edit/delete Toshiba AC
    
    TOPIC_ED00,
    TOPIC_EDTC, 
    TOPIC_EDTI,
    TOPIC_EDTO,
    TOPIC_EDTM,
    TOPIC_EDTU,
    TOPIC_EDTF,
    TOPIC_EDTP,
    TOPIC_EDTS,
    TOPIC_EDTL,
    TOPIC_EDTE,
    TOPIC_EDTK,
    TOPIC_EDT2,
    TOPIC_EDTQ,
    TOPIC_EDTY,
    TOPIC_EDTV,
    TOPIC_EDTR,
    TOPIC_EDT4,
    TOPIC_EDTG,
    TOPIC_EDT8,
    TOPIC_EDTB,
    TOPIC_EDT0,
    TOPIC_EDT3,
    TOPIC_EDES,     // add/edit/delete escalator       
    TOPIC_EDLF,     // add/edit/delete lift
    TOPIC_EDLI,     // add/edit/delete lighting
    TOPIC_EDFR,     // add/edit/delete fire safety system
    TOPIC_EDFH,     // add/edit/delete fire hydrant
    TOPIC_EDFV,     // add/edit/delete fm200
    TOPIC_EDAC,     // add/edit/delete split ac
    TOPIC_EDLT,     // add/edit/delete lt panel
    TOPIC_EDUP,     // add/edit/delete ups
        TOPIC_EDU1,
        TOPIC_EDU2,
        TOPIC_EDU3,
    TOPIC_EDGS,     // add/edit/delete disel genset
        TOPIC_EDG1,
        TOPIC_EDG2,
        TOPIC_EDG3,
        TOPIC_EDG4,
        TOPIC_EDG5,
        TOPIC_EDG6,
    TOPIC_EDFN,     // add/edit/delete fan system
    TOPIC_EDAS,     // add/edit/delete access control system
    TOPIC_EDWP,     // add/edit/delete water pump system
    TOPIC_EDAN,     // add/edit/delete anemometer
    TOPIC_EDTH,     // add/edit/delete temperature humidity sensor
    TOPIC_EDSE,     // add/edit/delete seismometer sensor
    TOPIC_EDHP,     // add/edit/delete hard points
    TOPIC_EDCS,     // add/edit/delete combined sensor of anemometer and seismometer
    TOPIC_EDL1,     // add/edit/delete lt panel power
    TOPIC_EDSL,     // add/edit/delete lt panel sld
    TOPIC_EDSC,     // add/edit/delete lt panel sld control
    TOPIC_EDLP,     // add/edit/delete tank level and pressure sensor
    TOPIC_EDST,     // add/edit/delete Stulz precision AC
    TOPIC_EDLS,     // add/edit/delete tank level sensor
    TOPIC_EDAM,     // add/edit/delete lt panel sld auto manual status
    TOPIC_EDAT,     // add/edit/delete Toshiba AC
    
    TOPIC_DE00,
    TOPIC_DELC,  
    TOPIC_DELI,   
    TOPIC_DELO,   
    TOPIC_DELM,
    TOPIC_DELU,
    TOPIC_DELF,  
    TOPIC_DELP, 
    TOPIC_DELS,   
    TOPIC_DELL,  
    TOPIC_DELE,  
    TOPIC_DELK,   
    TOPIC_DEL2,   
    TOPIC_DELQ,  
    TOPIC_DELY, 
    TOPIC_DELV,
    TOPIC_DELR,
    TOPIC_DEL4,
    TOPIC_DELG,
    TOPIC_DEL8,
    TOPIC_DELB,
    TOPIC_DEL0,
    TOPIC_DEL3,
    TOPIC_DBES,     // add/edit/delete escalator       
    TOPIC_DBLF,     // add/edit/delete lift
    TOPIC_DBLI,     // add/edit/delete lighting
    TOPIC_DBFR,     // add/edit/delete fire safety system
    TOPIC_DBFH,     // add/edit/delete fire hydrant
    TOPIC_DBFV,     // add/edit/delete fm200
    TOPIC_DBAC,     // add/edit/delete split ac
    TOPIC_DBLT,     // add/edit/delete lt panel
    TOPIC_DBUP,     // add/edit/delete ups
        TOPIC_DBU1,
        TOPIC_DBU2,  
        TOPIC_DBU3, 
    TOPIC_DBGS,     // add/edit/delete disel genset
        TOPIC_DBG1,
        TOPIC_DBG2,
        TOPIC_DBG3,
        TOPIC_DBG4,
        TOPIC_DBG5,
        TOPIC_DBG6,
    TOPIC_DBFN,     // add/edit/delete fan system
    TOPIC_DBAS,     // add/edit/delete access control system
    TOPIC_DBWP,     // add/edit/delete water pump system
    TOPIC_DBAN,     // add/edit/delete anemometer
    TOPIC_DBTH,     // add/edit/delete temperature humidity sensor
    TOPIC_DBSE,     // add/edit/delete seismometer sensor
    TOPIC_DBHP,     // add/edit/delete hard points
    TOPIC_DBCS,     // add/edit/delete combined sensor of anemometer and seismometer
    TOPIC_DBL1,     // add/edit/delete lt panel power
    TOPIC_DBSL,     // add/edit/delete lt panel sld
    TOPIC_DBSC,     // add/edit/delete lt panel sld control
    TOPIC_DBLP,     // add/edit/delete tank level and pressure sensor
    TOPIC_DBST,     // add/edit/delete Stulz precision AC
    TOPIC_DBLS,     // add/edit/delete tank level sensor
    TOPIC_DBAM,     // add/edit/delete lt panel sld auto manual status
    TOPIC_DBAT,     // add/edit/delete Toshiba AC
    
    // unconfigure msmgs
    TOPIC_UN00,
    TOPIC_CNTR,  
    TOPIC_ECOS,  
    TOPIC_DOOR,  
    TOPIC_MOTN,
    TOPIC_OCCU,
    TOPIC_FANM,
    TOPIC_POWR,
    TOPIC_GENS,
    TOPIC_LUXS,
    TOPIC_TEHU,
    TOPIC_SMOK,
    TOPIC_CO2S,
    TOPIC_AIRQ,
    TOPIC_COLR,
    TOPIC_CAMR,
    TOPIC_RFID,
    TOPIC_SCEN,
    TOPIC_GASL,
    TOPIC_SCGW,
    TOPIC_IRBL,  //ADD AC--addb 
    TOPIC_OXYG,
    TOPIC_RGBW,
    TOPIC_ESCL,     // add/edit/delete escalator       
    TOPIC_LIFT,     // add/edit/delete lift
    TOPIC_LIGT,     // add/edit/delete lighting
    TOPIC_FIRE,     // add/edit/delete fire safety system
    TOPIC_FIHY,     // add/edit/delete fire hydrant
    TOPIC_F200,     // add/edit/delete fm200
    TOPIC_SPAC,     // add/edit/delete split ac
    TOPIC_LTPL,     // add/edit/delete lt panel
    TOPIC_UPSS,     // add/edit/delete ups
        TOPIC_UPS1,
        TOPIC_UPS2,
        TOPIC_UPS3,
    TOPIC_DGEN,     // add/edit/delete disel genset
        TOPIC_DGE1,
        TOPIC_DGE2,
        TOPIC_DGE3,
        TOPIC_DGE4,
        TOPIC_DGE5,
        TOPIC_DGE6,
    TOPIC_FANS,     // add/edit/delete fan system
    TOPIC_ACCS,     // add/edit/delete access control system
    TOPIC_WATR,     // add/edit/delete water pump system
    TOPIC_ANEM,     // add/edit/delete anemometer
    TOPIC_TEMH,     // add/edit/delete temperature humidity sensor
    TOPIC_SMOM,     // add/edit/delete seismometer sensor
    TOPIC_HARD,     // add/edit/delete hard points
    TOPIC_ANSE,     // add/edit/delete combined sensor of anemometer and seismometer
    TOPIC_LTP1,     // add/edit/delete lt panel power
    TOPIC_SLDP,     // add/edit/delete lt panel sld
    TOPIC_SLDC,     // add/edit/delete lt panel sld control
    TOPIC_LEVP,     // add/edit/delete tank level and pressure sensor
    TOPIC_STAC,     // add/edit/delete Stulz precision AC
    TOPIC_LEVL,     // add/edit/delete tank level sensor
    TOPIC_SLAM,     // add/edit/delete lt panel sld auto manual status
    TOPIC_TOAC,     // add/edit/delete Toshiba AC
    
    // sensor data received
    TOPIC_SE00,
    TOPIC_SENI,     // ir eco
    TOPIC_SENO,     // door 
    TOPIC_SENM,     // motion
    TOPIC_SENU,     // occupancy
    TOPIC_SENP,     // power
    TOPIC_SENS,     // generic sensor module
    TOPIC_SENL,     // lux
    TOPIC_SENE,     // temp & Humidity (ecosense)
    TOPIC_SENK,     // smoke
    TOPIC_SEN2,     // co2
    TOPIC_SENQ,     // air quality
    TOPIC_SENY,     // color temp
    TOPIC_SENR,     // rfid module
    TOPIC_SCES,     // saved scene sent to gateway from scene module to be converted and sent to room modules
    TOPIC_SCEA,     // scene configure/save/add command to gateway from scene module
    TOPIC_SCAA,     // gateway to send the room module addresses to the scene module for storing
    TOPIC_SWHP,     // scene command for saving switch pairing
    TOPIC_SENG,     // gas leak
    TOPIC_SCGS,     // scene sent to gateway
    TOPIC_SCGA,     // scene configure/save/add command to gateway
    TOPIC_SCGD,     // scene remove/delete command to gateway
    TOPIC_SEN0,     // sensed values for oxygen purity, flow, temperature, time elapsed
    TOPIC_SEES,     // escalator
    TOPIC_SELF,     // lift
    TOPIC_SELI,     // light
    TOPIC_SEFR,     // fire safety
    TOPIC_SEFH,     // fire hydrant
    TOPIC_SEFV,     // fm200
    TOPIC_SEAC,     // split ac
    TOPIC_SELT,     // lt panel
    TOPIC_SEUP,     // ups
        TOPIC_SEU1,
        TOPIC_SEU2,
        TOPIC_SEU3,
    TOPIC_SEGS,     // disel genset
        TOPIC_SEG1,
        TOPIC_SEG2,
        TOPIC_SEG3,
        TOPIC_SEG4,
        TOPIC_SEG5,
        TOPIC_SEG6,
    TOPIC_SEFN,     // fan system
    TOPIC_SEAS,     // access control system
    TOPIC_SEWP,     // water pump system
    TOPIC_SEAN,     // anemometer
    TOPIC_SETH,     // temperature humidity sensor
    TOPIC_SESE,     // temperature seismometer sensor
    TOPIC_SEHP,     // hard points
    TOPIC_SECS,     // combined sensor - anemometer and seismometer
    TOPIC_SEL1,     // add/edit/delete lt panel power
    TOPIC_SESL,     // add/edit/delete lt panel sld
    TOPIC_SESC,     // add/edit/delete lt panel sld control
    TOPIC_SELP,     // add/edit/delete tank level and pressure sensor
    TOPIC_SEST,     // add/edit/delete Stulz precision AC
    TOPIC_SELS,     // add/edit/delete tank level sensor
    TOPIC_SEAM,     // add/edit/delete lt panel sld auto manual status
    TOPIC_SEAT,     // add/edit/delete Toshiba AC

    TOPIC_CTRL,     // control individual device
    TOPIC_ALON,     // control all room devices together
    TOPIC_ALOF,     // control all devices off
    
    TOPIC_HRDR,     // to know the hard reset reason from module
    
    TOPIC_SECU,     // enable/disable - home, door, occupancy security
    TOPIC_AUTO,     // enable/disable automatic appliances
    
    TOPIC_ADDA,     // add/edit/delete schedule/alarm
    TOPIC_EDTA,   
    TOPIC_DELA,  
    
    TOPIC_ADDN,     //notification through GCM
    TOPIC_EDTN,  
    TOPIC_DELN,  
    
    TOPIC_ADDT,     // text sms notification
    TOPIC_EDTT,
    TOPIC_DELT,
    
    TOPIC_ADDW,     //add/edit wifi credentials
    TOPIC_EDTW,
    TOPIC_DELW,
    
    TOPIC_XDDR,
    
    TOPIC_MAX_COUNT  
};
#endif
