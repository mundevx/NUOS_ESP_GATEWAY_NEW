#include "aws_mqtt_gateway.h"
#include "aws_gateway_config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "nuos_gateway_led_status.h"

extern uint16_t sendCommand(const char *json);

/* -------------------------------------------------------------------------- */
/* H2 RGB status LED bridge                                                   */
/* -------------------------------------------------------------------------- */
/* The WS2812 is physically connected to H2 GPIO8. C3 owns BLE/Wi-Fi, so it
 * reports those application states to H2 through the existing JSON UART
 * command path. These helpers are intentionally tiny so BLE code can call
 * them without depending on the H2 LED implementation. */
static void nuosSetLedStatus(const char *state) {
  StaticJsonDocument<256> d;
  d["schema"] = "nuos.gateway.v2";
  d["type"] = "command";
  d["requestId"] = String("led_") + String(millis());
  d["command"] = "status.led";
  d["arguments"]["state"] = state;
  String json;
  serializeJson(d, json);
  sendCommand(json.c_str());
}

static void nuosLedEvent(const char *event) {
  StaticJsonDocument<256> d;
  d["schema"] = "nuos.gateway.v2";
  d["type"] = "command";
  d["requestId"] = String("ledevent_") + String(millis());
  d["command"] = "status.led";
  d["arguments"]["event"] = event;
  String json;
  serializeJson(d, json);
  sendCommand(json.c_str());
}

/* BLE/provisioning integration hooks. Call these from the BLE commissioning
 * callbacks in the C3 application. */
void nuosLedBleCommissioning(void) { nuosSetLedStatus("ble_commissioning"); }
void nuosLedBleConnected(void)    { nuosSetLedStatus("ble_connected"); }
void nuosLedBleActivity(void)     { nuosLedEvent("ble_activity"); }
static WiFiClientSecure tls;
static PubSubClient mqtt(tls);
static AwsMqttConfig cfg;
static uint32_t lastConnectAttempt=0;
static bool apMode=false;
static bool staEverConnected=false;
static uint32_t staStartMillis=0;
static const char *AP_SSID="NUOS-C3-SETUP";
static const char *AP_PASSWORD="NUOS1234";
static void startProvisioningAP(){
  if(apMode && WiFi.getMode()==WIFI_AP) return;
  nuosSetLedStatus("wifi_ap_mode");

  // Provisioning owns the radio. Completely stop the previous STA state
  // before creating the AP. In AP-only mode WiFi.status() is not a useful
  // STA indicator and may return 254, so the rest of the code uses apMode.
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true, false);
  delay(100);

  WiFi.mode(WIFI_AP);
  delay(100);

  // Use a fixed provisioning address so the UI is always reachable at the
  // documented address, independent of the Arduino core's DHCP defaults.
  IPAddress apIP(192,168,4,1);
  IPAddress apGW(192,168,4,1);
  IPAddress apMask(255,255,255,0);
  WiFi.softAPConfig(apIP, apGW, apMask);

  // Explicitly recreate the AP after switching radio modes.
  WiFi.softAPdisconnect(true);
  delay(100);
  bool ok=WiFi.softAP(AP_SSID, AP_PASSWORD, 1, false, 4);
  apMode=ok;

  if(ok){
    delay(100);
    Serial.printf("[WIFI] AP MODE: SSID=%s IP=%s GW=%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str(),
                  WiFi.softAPIP().toString().c_str());
    Serial.printf("[WIFI] AP clients=%d | AP channel=1\n", WiFi.softAPgetStationNum());
    Serial.println("[WIFI] STA retry disabled while provisioning AP is active");
  } else {
    Serial.println("[WIFI] AP MODE START FAILED");
    nuosSetLedStatus("error");
  }
}
static void stopProvisioningAP(){
  if(!apMode) return;
  WiFi.softAPdisconnect(true);
  apMode=false;
}

static String gatewayId(){return cfg.thingName.length()?cfg.thingName:String("NUOS_GATEWAY");}
static String topic(const String &t){String r=t; r.replace("{gatewayId}",gatewayId()); return r;}

static String makeRequestId(const char *prefix){
  static uint32_t n=1;
  return String(prefix)+String(millis())+String("_")+String(n++);
}

/* Convert a public NUOS Ecosystem V2 command into the private H2 command
 * vocabulary. Cloud/application engineers never need to know these names. */
static bool buildH2Command(JsonDocument &d, String &out){
  if (!d.containsKey("requestId")) {
    if (d.containsKey("commandId")) d["requestId"] = d["commandId"];
    else d["requestId"] = makeRequestId("mqtt_");
  }
  d.remove("commandId");
  d["type"]="command";

  if (!d.containsKey("command")) {
    if (d.containsKey("set")) d["command"]="device.set";
    else if (d.containsKey("get")) d["command"]="device.get_state";
    else if (d.containsKey("action")) d["command"]="device.action";
  }
  if (!d.containsKey("command")) return false;

  /* Public endpoint is simply "endpoint"; H2 also accepts endpointId. */
  if (d.containsKey("endpoint") && !d.containsKey("endpointId"))
    d["endpointId"] = d["endpoint"];

  serializeJson(d,out);
  return true;
}

static void publishWrapped(const char *kind,const char *h2Json){
  if(!mqtt.connected()||!h2Json)return;
  StaticJsonDocument<8192> in;
  if(deserializeJson(in,h2Json)) return;

  StaticJsonDocument<8192> out;
  out["schema"]="nuos.gateway.v2";
  out["gatewayId"]=gatewayId();
  out["timestamp"]=time(nullptr);

  const char *eventType=in["eventType"]|"";
  const char *publicType=kind;
  if(strcmp(kind,"zigbee_event")==0){
    if(strcmp(eventType,"property.changed")==0) publicType="state_changed";
    else if(strcmp(eventType,"connectivity.changed")==0) publicType="availability";
    else if(strcmp(eventType,"device.added")==0 || strcmp(eventType,"device.rejoined")==0) publicType="device";
    else if(strcmp(eventType,"device.removed")==0) publicType="device_removed";
    else publicType="event";
  }
  out["type"]=publicType;
  JsonObject data=out.createNestedObject("data");

  for(JsonPair p:in.as<JsonObject>()){
    const char *k=p.key().c_str();
    if(!strcmp(k,"commandId")) { data["requestId"]=p.value(); continue; }
    if(!strcmp(k,"schema") || !strcmp(k,"type")) continue;
    if(!strcmp(k,"command")) continue; /* private H2 command */
    if(!strcmp(k,"eventType")) continue;
    if(!strcmp(k,"endpointNumber")) { data["endpoint"]=p.value(); continue; }
    if(!strcmp(k,"value") && strcmp(publicType,"state_changed")==0) continue;
    data[k]=p.value();
  }

  if(strcmp(publicType,"state_changed")==0){
    JsonObject state=data.createNestedObject("state");
    const char *prop=in["property"]|"value";
    state[prop]=in["value"];
  }
  if(strcmp(publicType,"availability")==0){
    data["availability"]=in["connectivity"]["state"]|"offline";
  }

  String payload; serializeJson(out,payload);
  mqtt.publish(topic(cfg.eventTopic).c_str(),payload.c_str(),cfg.qos==1);

  /* State snapshots/events also go to the dedicated state topic. */
  if(!strcmp(publicType,"state_changed") || !strcmp(publicType,"availability") ||
     (!strcmp(kind,"response") && in["data"]["devices"].is<JsonArray>())){
    mqtt.publish(topic(cfg.stateTopic).c_str(),payload.c_str(),cfg.qos==1);
  }
}

static void mqttCallback(char *t,uint8_t *payload,unsigned int len){
  StaticJsonDocument<8192> d; if(deserializeJson(d,payload,len))return;
  const char *type=d["type"]|"command";
  if(strcmp(type,"command")!=0)return;
  String h2;
  if(!buildH2Command(d,h2)) return;
  sendCommand(h2.c_str());
}

static bool connectMqtt(){
  if(!cfg.endpoint.length()) return false;
  String root=awsReadCertificate("root_ca.pem"); String cert=awsReadCertificate("device_cert.pem"); String key=awsReadCertificate("private_key.pem");
  if(root.length()) tls.setCACert(root.c_str()); else tls.setInsecure();
  if(cert.length()&&key.length()) tls.setCertificate(cert.c_str()),tls.setPrivateKey(key.c_str());
  mqtt.setServer(cfg.endpoint.c_str(),cfg.port); mqtt.setKeepAlive(cfg.keepAlive); mqtt.setBufferSize(4096); mqtt.setCallback(mqttCallback);
  String will=topic(cfg.availabilityTopic); bool ok;
  if(cfg.username.length()) ok=mqtt.connect(cfg.clientId.c_str(),cfg.username.c_str(),cfg.password.c_str(),will.c_str(),cfg.lwtQos,cfg.lwtRetain,cfg.lwtPayload.c_str(),cfg.cleanSession);
  else ok=mqtt.connect(cfg.clientId.c_str(),will.c_str(),cfg.lwtQos,cfg.lwtRetain,cfg.lwtPayload.c_str());
  if(ok){mqtt.subscribe(topic(cfg.commandTopic).c_str(),cfg.qos);mqtt.publish(will.c_str(),"online",cfg.lwtQos==1);nuosSetLedStatus("normal");}
  return ok;
}
static const char *wifiStatusName(wl_status_t status){
  switch(status){
    case WL_IDLE_STATUS:       return "IDLE";
    case WL_NO_SSID_AVAIL:     return "NO_SSID_AVAIL";
    case WL_CONNECTED:         return "CONNECTED";
    case WL_CONNECT_FAILED:    return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:   return "CONNECTION_LOST";
    case WL_DISCONNECTED:      return "DISCONNECTED";
    default:                   return "UNKNOWN";
  }
}

static void printTargetNetworkScan(const String &targetSsid){
  Serial.printf("[WIFI] Scanning 2.4 GHz networks for SSID: %s\n", targetSsid.c_str());
  int count = WiFi.scanNetworks(false, true);
  if(count < 0){
    Serial.printf("[WIFI] Scan failed: %d\n", count);
    return;
  }

  bool found = false;
  for(int i=0; i<count; ++i){
    String ssid = WiFi.SSID(i);
    if(ssid == targetSsid){
      found = true;
      Serial.printf("[WIFI] FOUND target: SSID=%s CH=%d RSSI=%d dBm\n",
                    ssid.c_str(), WiFi.channel(i), WiFi.RSSI(i));
    }
  }

  if(!found){
    Serial.printf("[WIFI] TARGET SSID NOT FOUND: %s\n", targetSsid.c_str());
    Serial.println("[WIFI] Check that the router has a 2.4 GHz WLAN enabled and that the SSID is not hidden.");
  }
  Serial.printf("[WIFI] Scan complete: %d network(s) found\n", count);
  WiFi.scanDelete();
}

static void wifiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info){
  switch(event){
    case ARDUINO_EVENT_WIFI_STA_START:
      nuosSetLedStatus("wifi_sta_connecting");
      Serial.println("[WIFI EVENT] STA START");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      nuosSetLedStatus("wifi_sta_connected");
      Serial.println("[WIFI EVENT] STA CONNECTED to access point");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      staEverConnected = true;
      nuosSetLedStatus("wifi_sta_connected");
      Serial.printf("[WIFI EVENT] GOT IP: %s | GW=%s | RSSI=%d dBm\n",
                    WiFi.localIP().toString().c_str(),
                    WiFi.gatewayIP().toString().c_str(),
                    WiFi.RSSI());
      // If STA recovered after a previous disconnect, the provisioning AP
      // is no longer needed. awsMqttLoop() also enforces this.
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      // Once provisioning AP is active, STA is intentionally stopped.
      if(apMode) break;
      uint8_t reason = info.wifi_sta_disconnected.reason;
      const char *reasonName = WiFi.STA.disconnectReasonName((wifi_err_reason_t)reason);
      Serial.printf("[WIFI EVENT] STA DISCONNECTED: reason=%u (%s)\n",
                    reason, reasonName ? reasonName : "UNKNOWN");

      // Do not start AP for the normal initial connection attempt.
      // Once STA has successfully connected at least once, any later STA
      // disconnect immediately enables the local provisioning AP.
      if(staEverConnected && cfg.wifiSsid.length()){
        Serial.println("[WIFI] STA disconnected after successful connection - starting AP fallback");
        mqtt.disconnect();
        startProvisioningAP();
      }
      break;
    }

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("[WIFI EVENT] STA LOST IP");
      if(staEverConnected && cfg.wifiSsid.length()){
        Serial.println("[WIFI] STA lost IP - starting AP fallback");
        mqtt.disconnect();
        startProvisioningAP();
      }
      break;

    default:
      break;
  }
}

static void printWifiProgress(){
  static uint32_t lastPrint = 0;
  if(apMode) return;
  if(millis() - lastPrint < 2000) return;
  lastPrint = millis();

  Serial.printf("[WIFI] STA status=%d (%s), SSID=%s, elapsed=%lus\n",
                (int)WiFi.status(),
                wifiStatusName(WiFi.status()),
                cfg.wifiSsid.c_str(),
                (unsigned long)((millis()-staStartMillis)/1000));
}

void awsMqttBegin(){
  awsConfigLoad(cfg);
  mqtt.setCallback(mqttCallback);
  configTime(0,0,"pool.ntp.org","time.nist.gov");
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(wifiEventHandler);

  if(cfg.wifiSsid.length()){
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // Diagnostic scan before connecting. This confirms that the configured
    // SSID is actually visible to the ESP32-C3 2.4 GHz radio.
    printTargetNetworkScan(cfg.wifiSsid);

    nuosSetLedStatus("wifi_sta_connecting");
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
    staStartMillis=millis();
    Serial.printf("[WIFI] STA connecting to: %s\n",cfg.wifiSsid.c_str());
    Serial.printf("[WIFI] Password length: %u\n",(unsigned)cfg.wifiPassword.length());
  } else {
    startProvisioningAP();
  }
}

bool awsMqttApplyWifiConfig(uint32_t timeoutMs){
  AwsMqttConfig newCfg;
  if(!awsConfigLoad(newCfg)){
    Serial.println("[WIFI] Failed to reload saved configuration");
    startProvisioningAP();
    return false;
  }

  cfg = newCfg;

  if(!cfg.wifiSsid.length()){
    Serial.println("[WIFI] No STA SSID configured - starting provisioning AP");
    startProvisioningAP();
    return false;
  }

  Serial.printf("[WIFI] Applying new STA configuration: SSID=%s\n", cfg.wifiSsid.c_str());
  printTargetNetworkScan(cfg.wifiSsid);

  // Keep the provisioning AP available while testing the new credentials.
  // AP mode was intentionally used to stop the old STA retry loop.
  WiFi.mode(WIFI_AP_STA);
  if(!apMode) startProvisioningAP();

  WiFi.setAutoReconnect(true);
  WiFi.disconnect(false, false);
  delay(150);
  WiFi.setAutoReconnect(true);
  nuosSetLedStatus("wifi_sta_connecting");
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
  staStartMillis=millis();

  uint32_t start=millis();
  while(millis()-start < timeoutMs){
    wl_status_t status=WiFi.status();
    if(status==WL_CONNECTED){
      staEverConnected = true;
      Serial.printf("[WIFI] STA CONNECTED: SSID=%s IP=%s RSSI=%d dBm\n",
                    WiFi.SSID().c_str(),WiFi.localIP().toString().c_str(),WiFi.RSSI());
      return true;
    }
    printWifiProgress();
    delay(100);
    yield();
  }

  Serial.printf("[WIFI] STA connection FAILED after %lu ms. Final status=%d (%s)\n",
                (unsigned long)timeoutMs,
                (int)WiFi.status(),
                wifiStatusName(WiFi.status()));

  // Failed credentials: stop STA completely and leave only the provisioning AP.
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  delay(50);
  WiFi.mode(WIFI_AP);
  if(!apMode) startProvisioningAP();
  Serial.println("[WIFI] New credentials failed - remaining in provisioning AP mode");
  return false;
}

void awsMqttFinishWifiApply(bool staConnected){
  if(staConnected && apMode){
    delay(200);
    stopProvisioningAP();
    Serial.printf("[WIFI] Provisioning AP stopped. STA IP=%s\n", WiFi.localIP().toString().c_str());
  }
}

void awsMqttLoop(){
  if(WiFi.status()!=WL_CONNECTED){
    printWifiProgress();
    if(!apMode && cfg.wifiSsid.length() && !staEverConnected && (millis()-staStartMillis)>15000){
      Serial.println("[WIFI] STA connection timeout (15s) - stopping STA and starting provisioning AP");
      startProvisioningAP();
    }
    return;
  }

  if(apMode){
    Serial.println("[WIFI] STA CONNECTED - stopping provisioning AP");
    stopProvisioningAP();
  }

  if(!mqtt.connected()){
    if(millis()-lastConnectAttempt>5000){
      lastConnectAttempt=millis();
      connectMqtt();
    }
  }else{
    mqtt.loop();
  }
}
void awsMqttPublishH2Message(uint8_t h2Type,const char *h2Json){
  const char *kind="h2_message";
  if(h2Type==4) kind="zigbee_event";
  else if(h2Type==2) kind="ack";
  else if(h2Type==3) kind="response";
  else if(h2Type==5) kind="error";
  publishWrapped(kind,h2Json);
}
