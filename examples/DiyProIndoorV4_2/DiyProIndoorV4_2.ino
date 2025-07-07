/*
This is the code for the AirGradient DIY PRO 4.2 Air Quality Monitor with an D1
ESP8266 Microcontroller.

It is an air quality monitor for PM2.5, CO2, Temperature and Humidity with a
small display and can send data over Wifi.

=== MQTT PAYLOAD STRUCTURE ===
This firmware publishes comprehensive sensor data to MQTT in JSON format.

Expected payload fields for Indoor Monitor (DIY Pro V4.2):
- PM Sensor (PMS5003): pm01, pm02, pm10, pm003Count, pm02Compensated
- Temperature/Humidity (SHT3x/SHT40): atmp, rhum, atmpCompensated, rhumCompensated
- CO2 Sensor (S8): rco2
- TVOC/NOx Sensor (SGP41): tvocIndex, noxIndex, tvocRaw, noxRaw
- Device Info: serialno, firmware, model, wifi (RSSI), boot, bootCount

MQTT Topics:
- readings: mui/airgradient/readings/{deviceId}

All messages are published with retain=true for reliability.
Buffer size: 256 bytes for comprehensive payload.

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
*/

#include "AgApiClient.h"
#include "AgConfigure.h"
#include "AgSchedule.h"
#include "AgWiFiConnector.h"
#include "LocalServer.h"
#include "OpenMetrics.h"
#include "Libraries/pubsubclient-2.8/src/PubSubClient.h"
#include <AirGradient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>

#define LED_BAR_ANIMATION_PERIOD 100         /** ms */
#define DISP_UPDATE_INTERVAL 2500            /** ms */
#define SERVER_CONFIG_SYNC_INTERVAL 60000    /** ms */
#define SERVER_SYNC_INTERVAL 60000           /** ms */
#define MQTT_SYNC_INTERVAL 10000             /** ms */
#define SENSOR_CO2_CALIB_COUNTDOWN_MAX 5     /** sec */
#define SENSOR_TVOC_UPDATE_INTERVAL 1000     /** ms */
#define SENSOR_CO2_UPDATE_INTERVAL 4000      /** ms */
#define SENSOR_PM_UPDATE_INTERVAL 2000       /** ms */
#define SENSOR_TEMP_HUM_UPDATE_INTERVAL 6000 /** ms */
#define DISPLAY_DELAY_SHOW_CONTENT_MS 2000   /** ms */

static AirGradient ag(DIY_PRO_INDOOR_V4_2);
static Configuration configuration(Serial);
static AgApiClient apiClient(Serial, configuration);
static Measurements measurements(configuration);
static OledDisplay oledDisplay(configuration, measurements, Serial);
static StateMachine stateMachine(oledDisplay, Serial, measurements, configuration);
static WifiConnector wifiConnector(oledDisplay, Serial, stateMachine, configuration);
static OpenMetrics openMetrics(measurements, configuration, wifiConnector, apiClient);
static LocalServer localServer(Serial, openMetrics, measurements, configuration, wifiConnector);

// Direct MQTT only (ลบ MqttClient class และ secure clients)
static WiFiClient mqttWifiClient;
static PubSubClient directMqttClient(mqttWifiClient);
static bool mqttConnected = false;

static uint32_t factoryBtnPressTime = 0;
static AgFirmwareMode fwMode = FW_MODE_I_42PS;
static String fwNewVersion;

// Function declarations (ลบ functions ที่ไม่ใช้)
static void boardInit(void);
static void failedHandler(String msg);
static void configurationUpdateSchedule(void);
static void appDispHandler(void);
static void oledDisplaySchedule(void);
static void updateTvoc(void);
static void updatePm(void);
static void sendDataToServer(void);
static void tempHumUpdate(void);
static void co2Update(void);
static void mdnsInit(void);
static void factoryConfigReset(void);
static void wdgFeedUpdate(void);
static bool sgp41Init(void);
static void wifiFactoryConfigure(void);
static int calculateMaxPeriod(int updateInterval);
static void setMeasurementMaxPeriod();
static void initDirectMqtt(void);
static void directMqttHandle(void);
static bool connectDirectMqtt(void);
static void sendDataToMongoDB(void); // เพิ่ม

AgSchedule dispLedSchedule(DISP_UPDATE_INTERVAL, oledDisplaySchedule);
AgSchedule configSchedule(SERVER_CONFIG_SYNC_INTERVAL, configurationUpdateSchedule);
AgSchedule agApiPostSchedule(SERVER_SYNC_INTERVAL, sendDataToServer);
AgSchedule co2Schedule(SENSOR_CO2_UPDATE_INTERVAL, co2Update);
AgSchedule pmsSchedule(SENSOR_PM_UPDATE_INTERVAL, updatePm);
AgSchedule tempHumSchedule(SENSOR_TEMP_HUM_UPDATE_INTERVAL, tempHumUpdate);
AgSchedule tvocSchedule(SENSOR_TVOC_UPDATE_INTERVAL, updateTvoc);
AgSchedule watchdogFeedSchedule(60000, wdgFeedUpdate);
AgSchedule mqttSchedule(MQTT_SYNC_INTERVAL, directMqttHandle);

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("Serial nr: " + ag.deviceId());

  configuration.begin();

  Wire.begin(ag.getI2cSdaPin(), ag.getI2cSclPin());
  delay(1000);

  configuration.setAirGradient(&ag);
  oledDisplay.setAirGradient(&ag);
  stateMachine.setAirGradient(&ag);
  wifiConnector.setAirGradient(&ag);
  apiClient.setAirGradient(&ag);
  openMetrics.setAirGradient(&ag);
  localServer.setAirGraident(&ag);
  measurements.setAirGradient(&ag);

  boardInit();
  setMeasurementMaxPeriod();

  bool connectToWifi = false;

  oledDisplay.setText("Press now for", configuration.isOfflineMode() ? "online mode" : "offline mode", "");
  uint32_t startTime = millis();
  while (true)
  {
    if (ag.button.getState() == ag.button.BUTTON_PRESSED)
    {
      configuration.setOfflineMode(!configuration.isOfflineMode());
      oledDisplay.setText("Offline Mode", configuration.isOfflineMode() ? " = True" : "  = False", "");
      delay(1000);
      break;
    }
    uint32_t periodMs = (uint32_t)(millis() - startTime);
    if (periodMs >= 3000)
    {
      Serial.println("Set for offline mode timeout");
      break;
    }
    delay(1);
  }
  connectToWifi = !configuration.isOfflineMode();

  if (connectToWifi)
  {
    apiClient.begin();

    if (wifiConnector.connect())
    {
      if (wifiConnector.isConnected())
      {
        mdnsInit();
        localServer.begin();

        // Initialize only direct MQTT
        initDirectMqtt();

        Serial.println("=== Testing Production Direct MQTT ===");
        if (connectDirectMqtt())
        {
          Serial.println("✓ Production Direct MQTT ready!");
        }

        sendDataToAg();

        if (configuration.getConfigurationControl() != ConfigurationControl::ConfigurationControlLocal)
        {
          apiClient.fetchServerConfiguration();
        }
        configSchedule.update();
        if (apiClient.isFetchConfigurationFailed())
        {
          if (apiClient.isNotAvailableOnDashboard())
          {
            stateMachine.displaySetAddToDashBoard();
            stateMachine.displayHandle(AgStateMachineWiFiOkServerOkSensorConfigFailed);
          }
          else
          {
            stateMachine.displayClearAddToDashBoard();
          }
          delay(DISPLAY_DELAY_SHOW_CONTENT_MS);
        }
      }
      else
      {
        if (wifiConnector.isConfigurePorttalTimeout())
        {
          oledDisplay.showRebooting();
          delay(2500);
          oledDisplay.setText("", "", "");
          ESP.restart();
        }
      }
    }
  }

  if (wifiConnector.hasConfigurated() == false)
  {
    Serial.println("Set offline mode cause wifi is not configurated");
    configuration.setOfflineModeWithoutSave(true);
  }

  oledDisplay.setText("Warming Up", "Serial Number:", ag.deviceId().c_str());
  delay(DISPLAY_DELAY_SHOW_CONTENT_MS);

  Serial.println("Display brightness: " + String(configuration.getDisplayBrightness()));
  oledDisplay.setBrightness(configuration.getDisplayBrightness());

  appDispHandler();
}

void loop()
{
  dispLedSchedule.run();
  configSchedule.run();
  agApiPostSchedule.run();

  if (configuration.hasSensorS8)
  {
    co2Schedule.run();
  }
  if (configuration.hasSensorPMS1)
  {
    pmsSchedule.run();
    ag.pms5003.handle();
  }
  if (configuration.hasSensorSHT)
  {
    tempHumSchedule.run();
  }
  if (configuration.hasSensorSGP)
  {
    tvocSchedule.run();
  }

  watchdogFeedSchedule.run();
  wifiConnector.handle();
  factoryConfigReset();
  configUpdateHandle();
  localServer._handle();

  if (configuration.hasSensorSGP)
  {
    ag.sgp41.handle();
  }

  MDNS.update();
  mqttSchedule.run();

  // Keep direct MQTT alive (ลบ secure branch)
  if (mqttConnected)
  {
    directMqttClient.loop();
  }
}

static void co2Update(void)
{
  if (!configuration.hasSensorS8)
  {
    return;
  }

  int value = ag.s8.getCo2();
  if (utils::isValidCO2(value))
  {
    measurements.update(Measurements::CO2, value);
  }
  else
  {
    measurements.update(Measurements::CO2, utils::getInvalidCO2());
  }
}

static void mdnsInit(void)
{
  Serial.println("mDNS init");
  if (!MDNS.begin(localServer.getHostname().c_str()))
  {
    Serial.println("Init mDNS failed");
    return;
  }

  MDNS.addService("_airgradient", "_tcp", 80);
  MDNS.addServiceTxt("_airgradient", "_tcp", "model", AgFirmwareModeName(fwMode));
  MDNS.addServiceTxt("_airgradient", "_tcp", "serialno", ag.deviceId());
  MDNS.addServiceTxt("_airgradient", "_tcp", "fw_ver", ag.getVersion());
  MDNS.addServiceTxt("_airgradient", "_tcp", "vendor", "AirGradient");

  MDNS.announce();
}

static void factoryConfigReset(void)
{
  if (ag.button.getState() == ag.button.BUTTON_PRESSED)
  {
    if (factoryBtnPressTime == 0)
    {
      factoryBtnPressTime = millis();
    }
    else
    {
      uint32_t ms = (uint32_t)(millis() - factoryBtnPressTime);
      if (ms >= 2000)
      {
        if (ag.isOne() || ag.isPro4_2())
        {
          oledDisplay.setText("Factory reset", "keep pressed", "for 8 sec");
        }
        else
        {
          Serial.println("Factory reset, keep pressed for 8 sec");
        }

        int count = 7;
        while (ag.button.getState() == ag.button.BUTTON_PRESSED)
        {
          delay(1000);
          String str = "for " + String(count) + " sec";
          oledDisplay.setText("Factory reset", "keep pressed", str.c_str());

          count--;
          if (count == 0)
          {
            wifiConnector.reset();
            configuration.reset();
            oledDisplay.setText("Factory reset", "successful", "");
            delay(3000);
            oledDisplay.setText("", "", "");
            ESP.restart();
          }
        }

        factoryBtnPressTime = 0;
        appDispHandler();
      }
    }
  }
  else
  {
    if (factoryBtnPressTime != 0)
    {
      appDispHandler();
    }
    factoryBtnPressTime = 0;
  }
}

static void wdgFeedUpdate(void)
{
  ag.watchdog.reset();
  Serial.println("External watchdog feed!");
}

static bool sgp41Init(void)
{
  ag.sgp41.setNoxLearningOffset(configuration.getNoxLearningOffset());
  ag.sgp41.setTvocLearningOffset(configuration.getTvocLearningOffset());
  if (ag.sgp41.begin(Wire))
  {
    Serial.println("Init SGP41 success");
    configuration.hasSensorSGP = true;
    return true;
  }
  else
  {
    Serial.println("Init SGP41 failuire");
    configuration.hasSensorSGP = false;
  }
  return false;
}

static void wifiFactoryConfigure(void)
{
  WiFi.persistent(true);
  WiFi.begin("airgradient", "cleanair");
  WiFi.persistent(false);
  oledDisplay.setText("Configure WiFi", "connect to", "\'airgradient\'");
  delay(2500);
  oledDisplay.setText("Rebooting...", "", "");
  delay(2500);
  oledDisplay.setText("", "", "");
  ESP.restart();
}

static void sendDataToAg()
{
  stateMachine.displayHandle(AgStateMachineWiFiOkServerConnecting);

  delay(1500);
  if (apiClient.sendPing(wifiConnector.RSSI(), measurements.bootCount()))
  {
    stateMachine.displayHandle(AgStateMachineWiFiOkServerConnected);
  }
  else
  {
    stateMachine.displayHandle(AgStateMachineWiFiOkServerConnectFailed);
  }
  delay(DISPLAY_DELAY_SHOW_CONTENT_MS);
}

void dispSensorNotFound(String ss)
{
  ss = ss + " not found";
  oledDisplay.setText("Sensor init", "Error:", ss.c_str());
  delay(2000);
}

static void boardInit(void)
{
  oledDisplay.begin();

  Serial.println("Firmware Version: " + ag.getVersion());

  oledDisplay.setText("AirGradient ONE", "FW Version: ", ag.getVersion().c_str());
  delay(DISPLAY_DELAY_SHOW_CONTENT_MS);

  ag.button.begin();
  ag.watchdog.begin();

  oledDisplay.setText("Press now for", "factory WiFi", "configure");

  uint32_t stime = millis();
  while (true)
  {
    if (ag.button.getState() == ag.button.BUTTON_PRESSED)
    {
      wifiFactoryConfigure();
    }
    delay(1);
    uint32_t ms = (uint32_t)(millis() - stime);
    if (ms >= 3000)
    {
      break;
    }
    delay(1);
  }

  oledDisplay.setText("Sensor", "initializing...", "");

  if (sgp41Init() == false)
  {
    dispSensorNotFound("SGP41");
  }

  if (ag.sht.begin(Wire) == false)
  {
    Serial.println("SHTx sensor not found");
    configuration.hasSensorSHT = false;
    dispSensorNotFound("SHT");
  }

  if (ag.s8.begin(&Serial) == false)
  {
    Serial.println("CO2 S8 sensor not found");
    configuration.hasSensorS8 = false;
    dispSensorNotFound("S8");
  }

  configuration.hasSensorPMS1 = true;
  configuration.hasSensorPMS2 = false;
  if (ag.pms5003.begin(&Serial) == false)
  {
    Serial.println("PMS sensor not found");
    configuration.hasSensorPMS1 = false;
    dispSensorNotFound("PMS");
  }

  if (configuration.hasSensorS8)
  {
    if (ag.s8.setAbcPeriod(configuration.getCO2CalibrationAbcDays() * 24))
    {
      Serial.println("Set S8 AbcDays successful");
    }
    else
    {
      Serial.println("Set S8 AbcDays failure");
    }
  }

  localServer.setFwMode(FW_MODE_I_42PS);
}

static void failedHandler(String msg)
{
  while (true)
  {
    Serial.println(msg);
    delay(1000);
  }
}

static void configurationUpdateSchedule(void)
{
  if (configuration.isOfflineMode() || configuration.getConfigurationControl() == ConfigurationControl::ConfigurationControlLocal)
  {
    Serial.println("Ignore fetch server configuration. Either mode is offline or configurationControl set to local");
    apiClient.resetFetchConfigurationStatus();
    return;
  }

  if (apiClient.fetchServerConfiguration())
  {
    configUpdateHandle();
  }
}

static void configUpdateHandle()
{
  if (configuration.isUpdated() == false)
  {
    return;
  }

  stateMachine.executeCo2Calibration();

  // Reinitialize direct MQTT if needed
  if (!mqttConnected)
  {
    initDirectMqtt();
  }

  if (configuration.hasSensorSGP)
  {
    if (configuration.noxLearnOffsetChanged() || configuration.tvocLearnOffsetChanged())
    {
      ag.sgp41.end();

      int oldTvocOffset = ag.sgp41.getTvocLearningOffset();
      int oldNoxOffset = ag.sgp41.getNoxLearningOffset();
      bool result = sgp41Init();
      const char *resultStr = "successful";
      if (!result)
      {
        resultStr = "failure";
      }
      if (oldTvocOffset != configuration.getTvocLearningOffset())
      {
        Serial.printf("Setting tvocLearningOffset from %d to %d hours %s\r\n", oldTvocOffset, configuration.getTvocLearningOffset(), resultStr);
      }
      if (oldNoxOffset != configuration.getNoxLearningOffset())
      {
        Serial.printf("Setting noxLearningOffset from %d to %d hours %s\r\n", oldNoxOffset, configuration.getNoxLearningOffset(), resultStr);
      }
    }
  }

  if (configuration.isDisplayBrightnessChanged())
  {
    oledDisplay.setBrightness(configuration.getDisplayBrightness());
  }

  appDispHandler();
}

static void appDispHandler(void)
{
  AgStateMachineState state = AgStateMachineNormal;

  if (configuration.isOfflineMode() == false)
  {
    if (wifiConnector.isConnected() == false)
    {
      state = AgStateMachineWiFiLost;
    }
    else if (apiClient.isFetchConfigurationFailed())
    {
      state = AgStateMachineSensorConfigFailed;
      if (apiClient.isNotAvailableOnDashboard())
      {
        stateMachine.displaySetAddToDashBoard();
      }
      else
      {
        stateMachine.displayClearAddToDashBoard();
      }
    }
    else if (apiClient.isPostToServerFailed())
    {
      state = AgStateMachineServerLost;
    }
  }
  stateMachine.displayHandle(state);
}

static void oledDisplaySchedule(void)
{
  if (factoryBtnPressTime == 0)
  {
    appDispHandler();
  }
}

static void updateTvoc(void)
{
  if (!configuration.hasSensorSGP)
  {
    return;
  }

  measurements.update(Measurements::TVOC, ag.sgp41.getTvocIndex());
  measurements.update(Measurements::TVOCRaw, ag.sgp41.getTvocRaw());
  measurements.update(Measurements::NOx, ag.sgp41.getNoxIndex());
  measurements.update(Measurements::NOxRaw, ag.sgp41.getNoxRaw());
}

static void updatePm(void)
{
  if (ag.pms5003.connected())
  {
    measurements.update(Measurements::PM01, ag.pms5003.getPm01Ae());
    measurements.update(Measurements::PM25, ag.pms5003.getPm25Ae());
    measurements.update(Measurements::PM10, ag.pms5003.getPm10Ae());
    measurements.update(Measurements::PM03_PC, ag.pms5003.getPm03ParticleCount());
  }
  else
  {
    measurements.update(Measurements::PM01, utils::getInvalidPmValue());
    measurements.update(Measurements::PM25, utils::getInvalidPmValue());
    measurements.update(Measurements::PM10, utils::getInvalidPmValue());
    measurements.update(Measurements::PM03_PC, utils::getInvalidPmValue());
  }
}

static void sendDataToServer(void)
{
  int bootCount = measurements.bootCount() + 1;
  measurements.setBootCount(bootCount);

  if (configuration.isOfflineMode() || !configuration.isPostDataToAirGradient())
  {
    Serial.println("Skipping transmission of data to AG server. Either mode is offline or post data to server disabled");
    return;
  }

  if (wifiConnector.isConnected() == false)
  {
    Serial.println("WiFi not connected, skipping data transmission to AG server");
    return;
  }

  String syncData = measurements.toString(false, fwMode, wifiConnector.RSSI());
  if (apiClient.postToServer(syncData))
  {
    Serial.println();
    Serial.println("Online mode and isPostToAirGradient = true");
    Serial.println();
  }
}

static void tempHumUpdate(void)
{
  if (ag.sht.measure())
  {
    float temp = ag.sht.getTemperature();
    float rhum = ag.sht.getRelativeHumidity();

    measurements.update(Measurements::Temperature, temp);
    measurements.update(Measurements::Humidity, rhum);

    if (configuration.hasSensorSGP)
    {
      ag.sgp41.setCompensationTemperatureHumidity(temp, rhum);
    }
  }
  else
  {
    measurements.update(Measurements::Temperature, utils::getInvalidTemperature());
    measurements.update(Measurements::Humidity, utils::getInvalidHumidity());
    Serial.println("SHT read failed");
  }
}

void setMeasurementMaxPeriod()
{
  measurements.maxPeriod(Measurements::CO2, calculateMaxPeriod(SENSOR_CO2_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::TVOC, calculateMaxPeriod(SENSOR_TVOC_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::TVOCRaw, calculateMaxPeriod(SENSOR_TVOC_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::NOx, calculateMaxPeriod(SENSOR_TVOC_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::NOxRaw, calculateMaxPeriod(SENSOR_TVOC_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::PM25, calculateMaxPeriod(SENSOR_PM_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::PM01, calculateMaxPeriod(SENSOR_PM_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::PM10, calculateMaxPeriod(SENSOR_PM_UPDATE_INTERVAL));
  measurements.maxPeriod(Measurements::PM03_PC, calculateMaxPeriod(SENSOR_PM_UPDATE_INTERVAL));

  if (configuration.hasSensorSHT)
  {
    measurements.maxPeriod(Measurements::Temperature, calculateMaxPeriod(SENSOR_TEMP_HUM_UPDATE_INTERVAL));
    measurements.maxPeriod(Measurements::Humidity, calculateMaxPeriod(SENSOR_TEMP_HUM_UPDATE_INTERVAL));
  }
  else
  {
    measurements.maxPeriod(Measurements::Temperature, calculateMaxPeriod(SENSOR_PM_UPDATE_INTERVAL));
    measurements.maxPeriod(Measurements::Humidity, calculateMaxPeriod(SENSOR_PM_UPDATE_INTERVAL));
  }
}

int calculateMaxPeriod(int updateInterval)
{
  return (SERVER_SYNC_INTERVAL - (SERVER_SYNC_INTERVAL * 0.5)) / updateInterval;
}

static void initDirectMqtt(void) // Connect To Host & Port
{
  Serial.println("Initializing Direct MQTT...");
  Serial.println("Configuring for non-secure connection (Port 1883)");

  directMqttClient.setServer("mui-nose.asia", 1883);
  directMqttClient.setBufferSize(512); // ขยายจาก 256
  directMqttClient.setKeepAlive(15);
  directMqttClient.setSocketTimeout(5);

  Serial.println("Direct MQTT (non-secure) initialized");
  mqttConnected = false;
}

static bool connectDirectMqtt(void) // Connect By Username & Password
{
  if (mqttConnected && directMqttClient.connected())
  {
    return true; // Already connected
  }

  if (directMqttClient.connected())
  {
    directMqttClient.disconnect();
    delay(100);
  }

  String clientId = "airgradient-direct-" + ag.deviceId() + "-" + String(millis());
  Serial.println("Connecting direct MQTT with client ID: " + clientId);

  bool connected = directMqttClient.connect(clientId.c_str(), "muimqtt", "muiroboticsmqtt");

  if (connected && directMqttClient.connected())
  {
    mqttConnected = true;
    Serial.println("✓ Direct MQTT connection successful!");
    delay(100);
    return true;
  }
  else
  {
    mqttConnected = false;
    Serial.println("✗ Direct MQTT connection failed. State: " + String(directMqttClient.state()));
    return false;
  }
}

static void directMqttHandle(void) // Part การส่งข้อมูลไปยัง MQTT
{
  if (!WiFi.isConnected())
  {
    Serial.println("WiFi not connected, skipping MQTT");
    return;
  }

  uint32_t freeHeap = ESP.getFreeHeap();
  Serial.println("Free heap before MQTT: " + String(freeHeap) + " bytes");

  if (freeHeap < 3000) // Keep low memory threshold
  {
    Serial.println("WARNING: Low memory, skipping MQTT to prevent crash");
    ESP.getFreeHeap(); // Garbage collection attempt
    delay(100);
    return;
  }

  if (!directMqttClient.connected())
  {
    String clientId = "airgradient-" + ag.deviceId() + "-" + String(millis());
    Serial.println("Attempting MQTT connection to mui-nose.asia:1883 with client ID: " + clientId);

    if (directMqttClient.connect(clientId.c_str(), "muimqtt", "muiroboticsmqtt"))
    {
      Serial.println("Direct MQTT connection established");
    }
    else
    {
      Serial.println("Direct MQTT connection failed, state: " + String(directMqttClient.state()));
      return;
    }
  }

  if (!directMqttClient.connected())
  {
    Serial.println("MQTT client not connected after connection attempt");
    return;
  }

  directMqttClient.loop();

  // Simplified sensor status
  Serial.println("=== SENSOR STATUS ===");
  Serial.println("PMS:" + String(configuration.hasSensorPMS1 ? "Y" : "N") +
                 " SHT:" + String(configuration.hasSensorSHT ? "Y" : "N") +
                 " S8:" + String(configuration.hasSensorS8 ? "Y" : "N") +
                 " SGP:" + String(configuration.hasSensorSGP ? "Y" : "N"));

  String deviceId = ag.deviceId();
  uint32_t freeHeapBeforePayload = ESP.getFreeHeap();

  String payload = "";

  if (freeHeapBeforePayload < 5000) // Increased threshold for comprehensive payload
  {
    // Minimal payload for low memory
    payload = "{\"error\":\"low_memory\",\"serialno\":\"" + ag.deviceId() + "\"}";
  }
  else
  {
    // Create comprehensive payload
    payload = "{";

    if (configuration.hasSensorPMS1)
    {
      int pm01 = measurements.get(Measurements::PM01);
      int pm25 = measurements.get(Measurements::PM25);
      int pm10 = measurements.get(Measurements::PM10);
      int pm003Count = measurements.get(Measurements::PM03_PC);

      if (pm01 >= 0 && pm01 < 9999)
      {
        payload += "\"pm01\":" + String(pm01) + ",";
      }
      if (pm25 >= 0 && pm25 < 9999)
      {
        payload += "\"pm02\":" + String(pm25) + ",";
        // Add compensated PM2.5 (same value for ESP8266)
        payload += "\"pm02Compensated\":" + String(pm25) + ",";
      }
      if (pm10 >= 0 && pm10 < 9999)
      {
        payload += "\"pm10\":" + String(pm10) + ",";
      }
      if (pm003Count >= 0 && pm003Count < 999999)
      {
        payload += "\"pm003Count\":" + String(pm003Count) + ",";
      }
    }

    if (configuration.hasSensorSHT)
    {
      float temp = measurements.getFloat(Measurements::Temperature);
      float hum = measurements.getFloat(Measurements::Humidity);

      if (temp > -40 && temp < 80)
      {
        payload += "\"atmp\":" + String(temp, 2) + ",";
        // Add compensated temperature (same value for ESP8266)
        payload += "\"atmpCompensated\":" + String(temp, 2) + ",";
      }
      if (hum >= 0 && hum <= 100)
      {
        payload += "\"rhum\":" + String(hum, 2) + ",";
        // Add compensated humidity (same value for ESP8266)
        payload += "\"rhumCompensated\":" + String(hum, 2) + ",";
      }
    }

    if (configuration.hasSensorS8)
    {
      int co2 = measurements.get(Measurements::CO2);
      if (co2 >= 400 && co2 <= 5000)
      {
        payload += "\"rco2\":" + String(co2) + ",";
      }
    }

    if (configuration.hasSensorSGP)
    {
      int tvoc = measurements.get(Measurements::TVOC);
      int nox = measurements.get(Measurements::NOx);
      int tvocRaw = measurements.get(Measurements::TVOCRaw);
      int noxRaw = measurements.get(Measurements::NOxRaw);

      if (tvoc >= 0 && tvoc <= 500)
      {
        payload += "\"tvocIndex\":" + String(tvoc) + ",";
      }
      if (nox >= 0 && nox <= 500)
      {
        payload += "\"noxIndex\":" + String(nox) + ",";
      }
      if (tvocRaw >= 0 && tvocRaw <= 65535)
      {
        payload += "\"tvocRaw\":" + String(tvocRaw) + ",";
      }
      if (noxRaw >= 0 && noxRaw <= 65535)
      {
        payload += "\"noxRaw\":" + String(noxRaw) + ",";
      }
    }

    // Add boot count
    int bootCount = measurements.bootCount();
    payload += "\"boot\":" + String(bootCount) + ",";
    payload += "\"bootCount\":" + String(bootCount) + ",";

    int rssi = wifiConnector.RSSI();
    if (rssi < 0 && rssi > -120)
    {
      payload += "\"wifi\":" + String(rssi) + ",";
    }

    payload += "\"serialno\":\"" + ag.deviceId() + "\",";
    payload += "\"firmware\":\"" + ag.getVersion() + "\",";
    payload += "\"model\":\"DIY-PRO-I-4.2PS\"";
    payload += "}";
  }

  String topic = "mui/airgradient/readings/" + deviceId;

  if (payload.length() > 0 && directMqttClient.connected())
  {
    bool success = directMqttClient.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Publish result: " + String(success ? "SUCCESS" : "FAILED"));

    // เพิ่มตรงนี้
    Serial.print("MQTT client state: ");
    Serial.println(directMqttClient.state());

    if (success)
    {
      Serial.println("✓ MQTT sync success to mui-nose.asia");
      Serial.println("Payload size: " + String(payload.length()) + " bytes");
    }

    directMqttClient.loop();
  }

  // เพิ่ม
  sendToMongoDB(payload);

  uint32_t finalFreeHeap = ESP.getFreeHeap();
  Serial.println("Final free heap: " + String(finalFreeHeap) + " bytes");
}

// เพิ่ม
void sendToMongoDB(String payload)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClient wifiClient;
    HTTPClient http;
    http.begin(wifiClient, "http://192.168.1.164:3000/api/airgradient");
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(payload);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    http.end();
  }
  else
  {
    Serial.println("WiFi not connected");
  }
}