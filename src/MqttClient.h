#ifndef _AG_MQTT_CLIENT_H_
#define _AG_MQTT_CLIENT_H_

#ifdef ESP32
#include "mqtt_client.h"
#else
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#endif /** ESP32 */
#include "Main/PrintLog.h"
#include <Arduino.h>

class MqttClient : public PrintLog
{
private:
  bool isBegin = false;
  String uri;
#ifdef ESP32
  esp_mqtt_client_handle_t client;
#else
  WiFiClient __wifiClient; // Change back to reference
  WiFiClientSecure __wifiClientSecure;
  void *client;
  String password;
  String user;
  String server;
  uint16_t port;
  bool useSecure;
#endif
  bool connected = false;
  int connectionFailedCount = 0;

public:
  MqttClient(Stream &debugLog);
  ~MqttClient();

  bool begin(String uri);
  void end(void);
  void _updateConnected(bool connected);
  bool publish(const char *topic, const char *payload, int len);
  bool isCurrentUri(String &uri);
  bool isConnected(void);
  int getConnectionFailedCount(void);
  bool connect(String id);
  void handle(void);
  void resetMqttClient(void); // Reset the MQTT client completely
};

#endif /** _AG_MQTT_CLIENT_H_ */
