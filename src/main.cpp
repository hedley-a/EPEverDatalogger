/*Lot of ideas comes from here:
 * https://github.com/glitterkitty/EpEverSolarMonitor
 *
 */
#include <Arduino.h>
#include "main.h"
#include <ModbusMaster.h>
#include "epregister.h"

#include <EEPROM.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncWiFiManager.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <UnixTime.h>
#include <WebSerialLite.h>

#include "Settings.h" //settings functions

#include "html.h" //the HTML content
#include "htmlProzessor.h" // The html Prozessor

String topic = "/"; // Default first part of topic. We will add device ID in setup

// flag for saving data and other things
bool shouldSaveConfig = false;
bool restartNow = false;
bool updateProgress = false;
unsigned long mqtttimer = 0;
unsigned long getDataTimer = 0;
unsigned long RestartTimer = 0;
byte wsReqInvNum = 1;
char mqtt_server[80];
char mqttClientId[80];
int errorcode;
uint32_t bootcount = 0;

WiFiClient client;
Settings _settings;
PubSubClient mqttclient(client);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient;
DNSServer dns;
ModbusMaster epnode; // instantiate ModbusMaster object
UnixTime uTime(3);
StaticJsonDocument<JSON_BUFFER> liveJson;
JsonObject liveData = liveJson.createNestedObject("LiveData");
JsonObject statsData = liveJson.createNestedObject("StatsData");
#include "status-LED.h"
ADC_MODE(ADC_VCC);
//----------------------------------------------------------------------
void saveConfigCallback()
{
  DEBUG_WEBLN(F("Should save config"));
  shouldSaveConfig = true;
}

static void handle_update_progress_cb(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if (!index)
  {
    Update.runAsync(true);
    if (!Update.begin(free_space))
    {
      Update.printError(EPEVER_SERIAL);
    }
  }

  if (Update.write(data, len) != len)
  {
    Update.printError(EPEVER_SERIAL);
  }

  if (final)
  {
    if (!Update.end(true))
    {
      Update.printError(EPEVER_SERIAL);
    }
    else
    {
      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML_REBOOT, htmlProcessor);
      request->send(response);
      DEBUG_WEBLN(F("Update complete"));
      RestartTimer = millis();
      restartNow = true; // Set flag so main loop can issue restart call
    }
  }
}

void preTransmission()
{
  digitalWrite(EPEVER_DE_RE, 1);
}

void postTransmission()
{
  digitalWrite(EPEVER_DE_RE, 0);
}

void notifyClients()
{
  if (wsClient != nullptr && wsClient->canSend())
  {
    char data[JSON_BUFFER];
    size_t len = serializeJson(liveJson, data);
    wsClient->text(data, len);
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    updateProgress = true;
    if (String((char *)data).substring(0, 9) == "wsSelInv_") // get the inverter number to display on the web
    {
      wsReqInvNum = String((char *)data).substring(9, 10).toInt();
    }
    if (String((char *)data).substring(0, 11) == "loadSwitch_") // get switch data from web loadSwitch_1_1
    {
      epnode.setSlaveId(String((char *)data).substring(11, 12).toInt());
      epnode.writeSingleCoil(0x0002, String((char *)data).substring(13, 14).toInt());
    }
    updateProgress = false;
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    wsClient = client;
    // if (getEpData(1))
    getEpData(1);
    getJsonData(1);
    notifyClients();
    DEBUG_WEBF("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    DEBUG_WEBF("WebSocket client #%u disconnected\n", client->id());
    wsClient = nullptr;
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
  String d = "";
  for (uint i = 0; i < len; i++)
  {
    d += char(data[i]);
  }
  WebSerial.println(d);
}

bool resetCounter(bool count)
{

  if (count)
  {
    if (ESP.getResetInfoPtr()->reason == 6)
    {
      ESP.rtcUserMemoryRead(16, &bootcount, sizeof(bootcount));

      if (bootcount >= 10 && bootcount < 20)
      {
        // bootcount = 0;
        // ESP.rtcUserMemoryWrite(16, &bootcount, sizeof(bootcount));
        _settings.reset();
        ESP.eraseConfig();
        ESP.restart();
      }
      else
      {
        bootcount++;
        ESP.rtcUserMemoryWrite(16, &bootcount, sizeof(bootcount));
      }
    }
    else
    {
      bootcount = 0;
      ESP.rtcUserMemoryWrite(16, &bootcount, sizeof(bootcount));
    }
  }
  else
  {
    bootcount = 0;
    ESP.rtcUserMemoryWrite(16, &bootcount, sizeof(bootcount));
  }
  return true;
}

void setup()
{
  pinMode(EPEVER_DE_RE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);
  resetCounter(true);
  _settings.load();
  WiFi.persistent(true);              // fix wifi save bug
  AsyncWiFiManager wm(&server, &dns); // create wifimanager instance

  EPEVER_SERIAL.begin(EPEVER_BAUD);
  epnode.setResponseTimeout(100);
  epnode.begin(1, EPEVER_SERIAL);
  epnode.preTransmission(preTransmission);
  epnode.postTransmission(postTransmission);

  wm.setSaveConfigCallback(saveConfigCallback);

  sprintf(mqttClientId, "%s-%06X", _settings.data.deviceName, ESP.getChipId());

  AsyncWiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT server", NULL, 32);
  AsyncWiFiManagerParameter custom_mqtt_user("mqtt_user", "MQTT User", NULL, 32);
  AsyncWiFiManagerParameter custom_mqtt_pass("mqtt_pass", "MQTT Password", NULL, 32);
  AsyncWiFiManagerParameter custom_mqtt_topic("mqtt_topic", "MQTT Topic", "EPEver", 32);
  AsyncWiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", NULL, 6);
  AsyncWiFiManagerParameter custom_mqtt_refresh("mqtt_refresh", "MQTT Send Interval", "300", 4);
  AsyncWiFiManagerParameter custom_mqtt_triggerpath("mqtt_triggerpath", "MQTT Data Trigger Path", NULL, 80);
  AsyncWiFiManagerParameter custom_device_name("device_name", "Device Name", "EPEver2MQTT", 32);
  AsyncWiFiManagerParameter custom_device_quantity("device_quantity", "Device Quantity", "1", 2);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_mqtt_topic);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_refresh);
  wm.addParameter(&custom_mqtt_triggerpath);
  wm.addParameter(&custom_device_name);
  wm.addParameter(&custom_device_quantity);

  bool res = wm.autoConnect("EPEver2MQTT-AP");

  wm.setConnectTimeout(30);       // how long to try to connect for before continuing
  wm.setConfigPortalTimeout(120); // auto close configportal after n seconds

  // save settings if wifi setup is fire up
  if (shouldSaveConfig)
  {
    strncpy(_settings.data.mqttServer, custom_mqtt_server.getValue(), 40);
    strncpy(_settings.data.mqttUser, custom_mqtt_user.getValue(), 40);
    strncpy(_settings.data.mqttPassword, custom_mqtt_pass.getValue(), 40);
    _settings.data.mqttPort = atoi(custom_mqtt_port.getValue());
    strncpy(_settings.data.deviceName, custom_device_name.getValue(), 40);
    strncpy(_settings.data.mqttTopic, custom_mqtt_topic.getValue(), 40);
    _settings.data.mqttRefresh = atoi(custom_mqtt_refresh.getValue());
    strncpy(_settings.data.mqttTriggerPath, custom_mqtt_triggerpath.getValue(), 80);
    _settings.data.deviceQuantity = atoi(custom_device_quantity.getValue()) <= 0 ? 1 : atoi(custom_device_quantity.getValue());

    _settings.save();
    ESP.restart();
  }

  topic = _settings.data.mqttTopic;
  mqttclient.setServer(_settings.data.mqttServer, _settings.data.mqttPort);
  mqttclient.setCallback(callback);
  mqttclient.setBufferSize(MQTT_BUFFER);
  // check is WiFi connected

  if (res)
  {
    // set the device name
    MDNS.begin(_settings.data.deviceName);
    WiFi.hostname(_settings.data.deviceName);

    liveJson["DEVICE_NAME"] = _settings.data.deviceName;

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML_MAIN, htmlProcessor);
      request->send(response); });

    server.on("/livejson", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                AsyncResponseStream *response = request->beginResponseStream("application/json");
                serializeJson(liveJson, *response);
                request->send(response); });

    server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML_REBOOT, htmlProcessor);
                request->send(response);
                restartNow = true;
                RestartTimer = millis(); });

    server.on("/confirmreset", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML_CONFIRM_RESET, htmlProcessor);
      request->send(response); });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Device is Erasing...");
                response->addHeader("Refresh", "15; url=/");
                response->addHeader("Connection", "close");
                request->send(response);
                delay(500);
                _settings.reset();
                ESP.eraseConfig();
                ESP.restart(); });

    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML_SETTINGS, htmlProcessor);
      request->send(response); });

    server.on("/settingsedit", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", HTML_SETTINGS_EDIT, htmlProcessor);
      request->send(response); });

    server.on("/settingssave", HTTP_POST, [](AsyncWebServerRequest *request)
              {
                strncpy(_settings.data.mqttServer, request->arg("post_mqttServer").c_str(), 40);
                _settings.data.mqttPort = request->arg("post_mqttPort").toInt();
                strncpy(_settings.data.mqttUser, request->arg("post_mqttUser").c_str(), 40);
                strncpy(_settings.data.mqttPassword, request->arg("post_mqttPassword").c_str(), 40);
                strncpy(_settings.data.mqttTopic, request->arg("post_mqttTopic").c_str(), 40);
                _settings.data.mqttRefresh = request->arg("post_mqttRefresh").toInt() < 1 ? 1 : request->arg("post_mqttRefresh").toInt(); // prevent lower numbers
                strncpy(_settings.data.deviceName, request->arg("post_deviceName").c_str(), 40);
                _settings.data.deviceQuantity = request->arg("post_deviceQuanttity").toInt() <= 0 ? 1 : request->arg("post_deviceQuanttity").toInt();
                _settings.data.mqttJson = (request->arg("post_mqttjson") == "true") ? true : false;
                strncpy(_settings.data.mqttTriggerPath, request->arg("post_mqtttrigger").c_str(), 80);
                _settings.save();
                request->redirect("/reboot"); });

    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      AsyncWebParameter *p = request->getParam(0);
      if (p->name() == "datetime")
      {
        uint8_t rtcSetY  = atoi (request->getParam("datetime")->value().substring(0, 2).c_str ());
        uint8_t rtcSetM  = atoi (request->getParam("datetime")->value().substring(2, 4).c_str ());
        uint8_t rtcSetD  = atoi (request->getParam("datetime")->value().substring(4, 6).c_str ());
        uint8_t rtcSeth  = atoi (request->getParam("datetime")->value().substring(6, 8).c_str ());
        uint8_t rtcSetm  = atoi (request->getParam("datetime")->value().substring(8, 10).c_str ());
        uint8_t rtcSets  = atoi (request->getParam("datetime")->value().substring(10, 12).c_str ());

      for (size_t i = 1; i <= ((size_t)_settings.data.deviceQuantity); i++)
      {
        epnode.setSlaveId(i);
        epnode.setTransmitBuffer(0, ((uint16_t)rtcSetm << 8) | rtcSets); // minute | secund
        epnode.setTransmitBuffer(1, ((uint16_t)rtcSetD << 8) | rtcSeth); // day | hour
        epnode.setTransmitBuffer(2, ((uint16_t)rtcSetY << 8) | rtcSetM); // year | month
        epnode.writeMultipleRegisters(0x9013, 3); //write registers
        delay(50);
      }
        }
     request->send(200, "text/plain", "message received"); });

    server.on(
        "/update", HTTP_POST, [](AsyncWebServerRequest *request)
        {
      Serial.end();
      updateProgress = true;
      ws.enable(false);
      ws.closeAll(); },
        handle_update_progress_cb);

    server.onNotFound([](AsyncWebServerRequest *request)
                      { request->send(418, "text/plain", "418 I'm a teapot"); });

    ws.onEvent(onEvent);
    server.addHandler(&ws);

    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.onMessage(recvMsg);

    server.begin();
    MDNS.addService("http", "tcp", 80);
  }
  analogWrite(LED_PIN, 255);
  resetCounter(false);
  MDNS.update();
}

// end void setup

//----------------------------------------------------------------------
void loop()
{
  // Make sure wifi is in the right mode
  if (WiFi.status() == WL_CONNECTED)
  {                      // No use going to next step unless WIFI is up and running.
    ws.cleanupClients(); // clean unused client connections
    // MDNS.update();
    mqttclient.loop(); // Check if we have something to read from MQTT

    if (millis() > (getDataTimer + 500) && !updateProgress && wsClient != nullptr && wsClient->canSend())
    {
      for (size_t i = 1; i <= ((size_t)_settings.data.deviceQuantity); i++)
      {
        if (wsReqInvNum == i && getEpData(i)) // only send the data to web was selected
        {
          getJsonData(i);
          notifyClients();
        }
      }
      getDataTimer = millis();
    }

    if (updateProgress)
    {
      getDataTimer = millis();
    }
    if (millis() > (mqtttimer + (_settings.data.mqttRefresh * 1000)) && !updateProgress)
    {
      for (size_t i = 1; i <= ((size_t)_settings.data.deviceQuantity); i++)
      {
        if (getEpData(i))
        {
          // need change when no data from epever connect to mqtt
          getJsonData(i);
          sendtoMQTT(i); // Update data to MQTT server if we should
        }
      }
      mqtttimer = millis();
    }
  }

  if (restartNow && millis() >= (RestartTimer + 500))
  {
    DEBUG_WEBLN("Restart");
    ESP.restart();
  }
  notificationLED(); // notification LED routine
}
// End void loop

bool getEpData(int invNum)
{
  errorcode = 0;
  epnode.setSlaveId(invNum);

  // clear buffers
  memset(rtc.buf, 0, sizeof(rtc.buf));
  memset(live.buf, 0, sizeof(live.buf));
  memset(stats.buf, 0, sizeof(stats.buf));
  batteryCurrent = 0;
  batterySOC = 0;
  uTime.setDateTime(0, 0, 0, 0, 0, 0);

  // Read registers for clock 0x9013 ------------------------------- combine with statistic reading
  epnode.clearResponseBuffer();
  result = epnode.readHoldingRegisters(RTC_CLOCK, RTC_CLOCK_CNT);
  if (result == epnode.ku8MBSuccess)
  {
    rtc.buf[0] = epnode.getResponseBuffer(0);
    rtc.buf[1] = epnode.getResponseBuffer(1);
    rtc.buf[2] = epnode.getResponseBuffer(2);
    uTime.setDateTime((2000 + rtc.r.y), rtc.r.M, rtc.r.d, (rtc.r.h + 1), rtc.r.m, rtc.r.s);

    errorcode = result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read registers for clock Failed");
    errorcode = result;
    return false;
  }
  // read LIVE-Data 0x3100
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(LIVE_DATA, LIVE_DATA_CNT);
  if (result == epnode.ku8MBSuccess)
  {

    for (i = 0; i < LIVE_DATA_CNT; i++)
      live.buf[i] = epnode.getResponseBuffer(i);

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read LIVE-Dat Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Statistical Data 0x3300
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(STATISTICS, STATISTICS_CNT);
  if (result == epnode.ku8MBSuccess)
  {
    for (i = 0; i < STATISTICS_CNT; i++)
      stats.buf[i] = epnode.getResponseBuffer(i);

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Statistical Data Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Battery SOC 0x311A ---------------------------------- move to new 0x31XX reading construct
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(BATTERY_SOC, 1);
  if (result == epnode.ku8MBSuccess)
  {
    batterySOC = epnode.getResponseBuffer(0);

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Battery SOC Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Battery Net Current = Icharge - Iload
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(BATTERY_CURRENT_L, 2);
  if (result == epnode.ku8MBSuccess)
  {
    batteryCurrent = epnode.getResponseBuffer(0);
    batteryCurrent |= epnode.getResponseBuffer(1) << 16;

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Battery Net Current = Icharge - Iload Failed");
    errorcode = errorcode + result;
    return false;
  }
  // State of the Load Switch
  epnode.clearResponseBuffer();
  result = epnode.readCoils(LOAD_STATE, 1);
  if (result == epnode.ku8MBSuccess)
  {
    loadState = epnode.getResponseBuffer(0) ? true : false;

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read State of the Load Switch Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Read Status Flags
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(0x3200, 2);
  if (result == epnode.ku8MBSuccess)
  {
    uint16_t temp = epnode.getResponseBuffer(0);
    status_batt.volt = temp & 0b1111;
    status_batt.temp = (temp >> 4) & 0b1111;
    status_batt.resistance = (temp >> 8) & 0b1;
    status_batt.rated_volt = (temp >> 15) & 0b1;

    temp = epnode.getResponseBuffer(1);

    // for(i=0; i<16; i++) DEBUG_WEB( (temp >> (15-i) ) & 1 );

    // charger_input     = ( temp & 0b0000000000000000 ) >> 15 ;
    charger_mode = (temp & 0b0000000000001100) >> 2;
    // charger_input     = ( temp & 0b0000000000000000 ) >> 12 ;
    // charger_operation = ( temp & 0b0000000000000000 ) >> 0 ;

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Read Status Flags Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Device Temperature ---------------------------------- move to new 0x31XX reading construct
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(DEVICE_TEMPERATURE, 1);
  if (result == epnode.ku8MBSuccess)
  {
    deviceTemperature = epnode.getResponseBuffer(0);

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Device Temperature Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Battery temperature ---------------------------------- move to new 0x31XX reading construct
  epnode.clearResponseBuffer();
  result = epnode.readInputRegisters(BATTERY_TEMPERATURE, 1);
  if (result == epnode.ku8MBSuccess)
  {
    batteryTemperature = epnode.getResponseBuffer(0);

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Battery temperature Failed");
    errorcode = errorcode + result;
    return false;
  }
  // Settings Data 0x9000 ------------------------------------ combine with clock
  epnode.clearResponseBuffer();
  result = epnode.readHoldingRegisters(DEVICE_SETTINGS, DEVICE_SETTINGS_CNT);
  if (result == epnode.ku8MBSuccess)
  {
    for (i = 0; i < DEVICE_SETTINGS_CNT; i++)
      settingParam.buf[i] = epnode.getResponseBuffer(i);

    errorcode = errorcode + result;
  }
  else
  {
    DEBUG_WEB(epnode.ku8MBSuccess);
    DEBUG_WEB(" ");
    DEBUG_WEB(result);
    DEBUG_WEBLN(" Read Settings Data Failed");
    errorcode = errorcode + result;
    return false;
  }
  if (errorcode == 0)
  {
    DEBUG_WEBLN("Transmission OK.");
  }
  return true;
}

bool getJsonData(int invNum)
{
  if ((size_t)_settings.data.deviceQuantity > 1)
  {
    liveJson["DEVICE_NAME"] = _settings.deviceNameStr + "_" + (invNum);
  }
  else
  {
    liveJson["DEVICE_NAME"] = _settings.data.deviceName;
  }
  liveJson["DEVICE_QUANTITY"] = _settings.data.deviceQuantity;
  liveJson["DEVICE_TIME"] = uTime.getUnix();
  liveJson["DEVICE_TEMPERATURE"] = deviceTemperature / 100.f;

  liveJson["DEVICE_FREE_HEAP"] = ESP.getFreeHeap();
  liveJson["DEVICE_JSON_MEMORY"] = liveJson.memoryUsage();
  liveJson["ESP_VCC"] = ESP.getVcc() / 1000.0;
  liveJson["Wifi_RSSI"] = WiFi.RSSI();

  liveData["SOLAR_VOLTS"] = live.l.pvV / 100.f;
  liveData["SOLAR_AMPS"] = live.l.pvA / 100.f;
  liveData["SOLAR_WATTS"] = live.l.pvW / 100.f;

  liveData["BATT_VOLTS"] = live.l.battV / 100.f;
  liveData["BATT_AMPS"] = batteryCurrent / 100.f;
  liveData["BATT_WATTS"] = (int(live.l.battV / 10) * int(batteryCurrent / 10) / 100.f);

  liveData["LOAD_VOLTS"] = live.l.loadV / 100.f;
  liveData["LOAD_AMPS"] = live.l.loadA / 100.f;
  liveData["LOAD_WATTS"] = live.l.loadW / 100.f;
  liveData["BATTERY_SOC"] = batterySOC / 1.0f;
  liveJson["BATTERY_TEMPERATURE"] = batteryTemperature / 100.f;
  liveJson["LOAD_STATE"] = loadState;

  statsData["SOLAR_MAX"] = stats.s.pVmax / 100.f;
  statsData["SOLAR_MIN"] = stats.s.pVmin / 100.f;
  statsData["BATT_MAX"] = stats.s.bVmax / 100.f;
  statsData["BATT_MIN"] = stats.s.bVmin / 100.f;

  statsData["CONS_ENERGY_DAY"] = stats.s.consEnerDay / 100.f;
  statsData["CONS_ENGERY_MON"] = stats.s.consEnerMon / 100.f;
  statsData["CONS_ENGERY_YEAR"] = stats.s.consEnerYear / 100.f;
  statsData["CONS_ENGERY_TOT"] = stats.s.consEnerTotal / 100.f;
  statsData["GEN_ENERGY_DAY"] = stats.s.genEnerDay / 100.f;
  statsData["GEN_ENERGY_MON"] = stats.s.genEnerMon / 100.f;
  statsData["GEN_ENERGY_YEAR"] = stats.s.genEnerYear / 100.f;
  statsData["GEN_ENERGY_TOT"] = stats.s.genEnerTotal / 100.f;
  statsData["CO2_REDUCTION"] = stats.s.c02Reduction / 100.f;

  liveJson["BATT_VOLT_STATUS"] = batt_volt_status[status_batt.volt];
  liveJson["BATT_TEMP"] = batt_temp_status[status_batt.temp];

  liveJson["CHARGER_INPUT_STATUS"] = charger_input_status[charger_input];
  liveJson["CHARGER_MODE"] = charger_charging_status[charger_mode];
  return true;
}

bool connectMQTT()
{
  if (!mqttclient.connected())
  {
    if (mqttclient.connect(mqttClientId, _settings.data.mqttUser, _settings.data.mqttPassword, (topic + "/Alive").c_str(), 0, true, "false", true))
    {
      mqttclient.publish((topic + String("/IP")).c_str(), String(WiFi.localIP().toString()).c_str());
      mqttclient.publish((topic + String("/Alive")).c_str(), "true", true); // LWT online message must be retained!

      if(strlen(_settings.data.mqttTriggerPath) > 0)
      {
        DEBUG_WEBLN("MQTT Data Trigger Subscribed");
        mqttclient.subscribe(_settings.data.mqttTriggerPath);
      }

      if ((size_t)_settings.data.deviceQuantity > 1) // if more than one inverter avaible, subscribe to all topics
      {
        for (size_t i = 1; i < ((size_t)_settings.data.deviceQuantity + 1); i++)
        {
          if (!_settings.data.mqttJson) // classic mqtt DP
            mqttclient.subscribe((topic + "/" + _settings.deviceNameStr + "_" + i + "/LOAD_STATE").c_str());
          else // subscribe json
            mqttclient.subscribe((topic + "/" + _settings.deviceNameStr + "_" + i + "/DATA").c_str());
        }
      }
      else // if only one inverter avaible subscribe to one topic
      {
        if (!_settings.data.mqttJson) // classic mqtt DP
          mqttclient.subscribe((topic + "/" + _settings.deviceNameStr + "/LOAD_STATE").c_str());
        else // subscribe json
          mqttclient.subscribe((topic + "/" + _settings.deviceNameStr + "/DATA").c_str());
      }
      return true;
    }
    else
    {
      return false;
    }
    return false;
  }
  else
  {
    return true;
  }
}

bool sendtoMQTT(int invNum)
{

  if (!connectMQTT())
  {
    return false;
  }
  String mqttDeviceName;

  if ((size_t)_settings.data.deviceQuantity > 1)
  {
    mqttDeviceName = _settings.deviceNameStr + "_" + invNum;
  }
  else
  {
    mqttDeviceName = _settings.data.deviceName;
  }

  //-----------------------------------------------------

  if (!_settings.data.mqttJson)
  {
    // Device Data
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/DEVICE_TIME").c_str(), String(uTime.getUnix()).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/DEVICE_TEMPERATURE").c_str(), String(deviceTemperature / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/LOAD_STATE").c_str(), String(loadState ? "true" : "false").c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/BATT_VOLT_STATUS").c_str(), String(batt_volt_status[status_batt.volt]).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/BATT_TEMP").c_str(), String(batt_temp_status[status_batt.temp]).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/CHARGER_INPUT_STATUS").c_str(), String(charger_input_status[charger_input]).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/CHARGER_MODE").c_str(), String(charger_charging_status[charger_mode]).c_str());
    // Device Settings Data
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/BATTERY_TYPE").c_str(), String(settingParam.s.bTyp).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/BATTERY_CAPACITY").c_str(), String(settingParam.s.bCapacity / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/TEMPERATURE_COMPENSATION").c_str(), String(settingParam.s.tempCompensation / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/HIGH_VOLT_DISCONNECT").c_str(), String(settingParam.s.highVDisconnect / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/CHARGING_LIMIT_VOLTS").c_str(), String(settingParam.s.chLimitVolt / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/OVER_VOLTS_RECONNECT").c_str(), String(settingParam.s.overVoltRecon / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/EQUALIZATION_VOLTS").c_str(), String(settingParam.s.equVolt / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/BOOST_VOLTS").c_str(), String(settingParam.s.boostVolt / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/FLOAT_VOLTS").c_str(), String(settingParam.s.floatVolt / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/BOOST_RECONNECT_VOLTS").c_str(), String(settingParam.s.boostVoltRecon / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/LOW_VOLTS_RECONNECT").c_str(), String(settingParam.s.lowVoltRecon / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/UNDER_VOLTS_RECOVER").c_str(), String(settingParam.s.underVoltRecov / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/UNDER_VOLTS_WARNING").c_str(), String(settingParam.s.underVoltWarning / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/LOW_VOLTS_DISCONNECT").c_str(), String(settingParam.s.lowVoltDiscon / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/DeviceData/DISCHARGING_LIMIT_VOLTS").c_str(), String(settingParam.s.dischLimitVolt / 100.f).c_str());
    // Live Solar
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/SOLAR_VOLTS").c_str(), String(live.l.pvV / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/SOLAR_AMPS").c_str(), String(live.l.pvA / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/SOLAR_WATTS").c_str(), String(live.l.pvW / 100.f).c_str());
    // Live Batt
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/BATT_VOLTS").c_str(), String(live.l.battV / 100.f).c_str());

    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/BATT_AMPS").c_str(), String(batteryCurrent / 100.f).c_str());

    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/BATT_WATTS").c_str(), String((int(live.l.battV / 10) * int(batteryCurrent / 10) / 100.f)).c_str());

    // Live Load
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/LOAD_VOLTS").c_str(), String(live.l.loadV / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/LOAD_AMPS").c_str(), String(live.l.loadA / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/LOAD_WATTS").c_str(), String(live.l.loadW / 100.f).c_str());
    // Live Battery
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/BATTERY_SOC").c_str(), String(batterySOC / 1.0f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/LiveData/BATTERY_TEMPERATURE").c_str(), String(batteryTemperature / 100.f).c_str());
    // stats Solar min / max
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/SOLAR_MAX").c_str(), String(stats.s.pVmax / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/SOLAR_MIN").c_str(), String(stats.s.pVmin / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/BATT_MAX").c_str(), String(stats.s.bVmax / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/BATT_MIN").c_str(), String(stats.s.bVmin / 100.f).c_str());
    // stats sonsumed, generated, Co2
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/CONS_ENERGY_DAY").c_str(), String(stats.s.consEnerDay / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/CONS_ENGERY_MON").c_str(), String(stats.s.consEnerMon / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/CONS_ENGERY_YEAR").c_str(), String(stats.s.consEnerYear / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/CONS_ENGERY_TOT").c_str(), String(stats.s.consEnerTotal / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/GEN_ENERGY_DAY").c_str(), String(stats.s.genEnerDay / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/GEN_ENERGY_MON").c_str(), String(stats.s.genEnerMon / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/GEN_ENERGY_YEAR").c_str(), String(stats.s.genEnerYear / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/GEN_ENERGY_TOT").c_str(), String(stats.s.genEnerTotal / 100.f).c_str());
    mqttclient.publish((topic + "/" + mqttDeviceName + "/StatsData/CO2_REDUCTION").c_str(), String(stats.s.c02Reduction / 100.f).c_str());
  }
  else
  {
    char data[JSON_BUFFER];
    serializeJson(liveJson, data);
    mqttclient.setBufferSize(JSON_BUFFER + 100);
    mqttclient.publish((String(topic + "/" + mqttDeviceName + "/DATA")).c_str(), data);
  }

  return true;
}

void callback(char *top, byte *payload, unsigned int length) // Need rework
{
  updateProgress = true; // stop servicing data
  if (!_settings.data.mqttJson)
  {
    String messageTemp;
    for (unsigned int i = 0; i < length; i++)
    {
      messageTemp += (char)payload[i];
    }

    if ((size_t)_settings.data.deviceQuantity > 1)
    {
      for (size_t k = 1; k < ((size_t)_settings.data.deviceQuantity + 1); k++)
      {
        if (strcmp(top, (topic + "/" + _settings.deviceNameStr + "_" + k + "/LOAD_STATE").c_str()) == 0)
        {
          epnode.setSlaveId(k);
          if (messageTemp == "true")
            epnode.writeSingleCoil(0x0002, 1);
          if (messageTemp == "false")
            epnode.writeSingleCoil(0x0002, 0);
        }
      }
    }
    else
    {
      if (strcmp(top, (topic + "/" + _settings.deviceNameStr + "/LOAD_STATE").c_str()) == 0)
      {
        epnode.setSlaveId(1);
        if (messageTemp == "true")
          epnode.writeSingleCoil(0x0002, 1);
        if (messageTemp == "false")
          epnode.writeSingleCoil(0x0002, 0);
      }
    }
  }
  else
  {
    StaticJsonDocument<1024> mqttJsonAnswer;
    deserializeJson(mqttJsonAnswer, (const byte *)payload, length);

    if ((size_t)_settings.data.deviceQuantity > 1)
    {
      for (size_t k = 1; k < ((size_t)_settings.data.deviceQuantity + 1); k++)
      {
        if (mqttJsonAnswer["DEVICE_NAME_" + k] == (_settings.deviceNameStr + "_" + k))
        {
          epnode.setSlaveId(k);
          if (mqttJsonAnswer["LOAD_STATE"] == true)
            epnode.writeSingleCoil(0x0002, 1);
          if (mqttJsonAnswer["LOAD_STATE"] == false)
            epnode.writeSingleCoil(0x0002, 0);
        }
      }
    }
    else
    {
      epnode.setSlaveId(1);
      if (mqttJsonAnswer["LOAD_STATE"] == true)
        epnode.writeSingleCoil(0x0002, 1);
      if (mqttJsonAnswer["LOAD_STATE"] == false)
        epnode.writeSingleCoil(0x0002, 0);
    }
  }

  if (strlen(_settings.data.mqttTriggerPath) > 0 && strcmp(top, _settings.data.mqttTriggerPath) == 0)
      {
        DEBUG_WEBLN("MQTT Data Trigger Firered Up");
        mqtttimer = 0;
      }
  updateProgress = false; // start data servicing again
}