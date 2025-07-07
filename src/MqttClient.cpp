#include "MqttClient.h"
#include "Libraries/pubsubclient-2.8/src/PubSubClient.h"

#ifdef ESP32
static void __mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                 int32_t event_id, void *event_data);
#else
#define CLIENT() ((PubSubClient *)client)
#endif

MqttClient::MqttClient(Stream &debugLog) : PrintLog(debugLog, "MqttClient")
{
#ifdef ESP32
#else
  client = NULL;
  useSecure = false;
#endif
}

MqttClient::~MqttClient() {}

bool MqttClient::begin(String uri)
{
  if (isBegin)
  {
    logInfo("Already begin, calll 'end' and try again");
    return true;
  }
  if (uri.isEmpty())
  {
    Serial.println("Mqtt uri is empty");
    return false;
  }

  this->uri = uri;
  logInfo("Init uri: " + uri);

#ifdef ESP32
  /** config esp_mqtt client */
  esp_mqtt_client_config_t config = {
      .uri = this->uri.c_str(),
  };

  /** init client */
  client = esp_mqtt_client_init(&config);
  if (client == NULL)
  {
    logError("Init client failed");
    return false;
  }

  /** Register event */
  if (esp_mqtt_client_register_event(client, MQTT_EVENT_ANY,
                                     __mqtt_event_handler, this) != ESP_OK)
  {
    logError("Register event failed");
    return false;
  }

  if (esp_mqtt_client_start(client) != ESP_OK)
  {
    logError("Client start failed");
    return false;
  }
#else
  // mqtt://<Username>:<Password>@<Host>:<Port>
  bool hasUser = false;
  for (unsigned int i = 0; i < this->uri.length(); i++)
  {
    if (this->uri[i] == '@')
    {
      hasUser = true;
      break;
    }
  }

  user = "";
  password = "";
  server = "";
  port = 0;

  char *serverPort = NULL;
  char *buf = (char *)this->uri.c_str();
  if (hasUser)
  {
    // mqtt://<Username>:<Password>@<Host>:<Port>
    char *userPass = strtok(buf, "@");
    serverPort = strtok(NULL, "@");

    if (userPass == NULL)
    {
      logError("User and Password invalid");
      return false;
    }
    else
    {
      if ((userPass[5] == '/') && (userPass[6] == '/'))
      { /** Check mqtt:// */
        userPass = &userPass[7];
      }
      else if ((userPass[6] == '/') &&
               (userPass[7] == '/'))
      { /** Check mqtts:// */
        userPass = &userPass[8];
      }
      else
      {
        logError("Server invalid");
        return false;
      }

      buf = strtok(userPass, ":");
      if (buf == NULL)
      {
        logError("User invalid");
        return false;
      }
      user = String(buf);

      buf = strtok(NULL, "@");
      if (buf == NULL)
      {
        logError("Password invalid");
        return false;
      }
      password = String(buf);

      logInfo("Username: " + user);
      logInfo("Password: " + password);
    }

    if (serverPort == NULL)
    {
      logError("Server and port invalid");
      return false;
    }
  }
  else
  {
    // mqtt://<Host>:<Port>
    if ((buf[5] == '/') && (buf[6] == '/'))
    { /** Check mqtt:// */
      serverPort = &buf[7];
    }
    else if ((buf[6] == '/') && (buf[7] == '/'))
    { /** Check mqtts:// */
      serverPort = &buf[8];
    }
    else
    {
      logError("Server invalid");
      return false;
    }
  }

  if (serverPort == NULL)
  {
    logError("Server and port invalid");
    return false;
  }

  buf = strtok(serverPort, ":");
  if (buf == NULL)
  {
    logError("Server invalid");
    return false;
  }
  server = String(buf);
  logInfo("Server: " + server);

  buf = strtok(NULL, ":");
  if (buf == NULL)
  {
    logError("Port invalid");
    return false;
  }
  port = (uint16_t)String(buf).toInt();
  logInfo("Port: " + String(port));

  // For MUI server, try non-secure connection first even on port 8883
  // since the test code works with regular WiFiClient on port 8883
  useSecure = false; // Force non-secure for now
  logInfo("Using non-secure connection for MUI server on port: " + String(port));
  if (client == NULL)
  {
    // Create PubSubClient with WiFiClient reference (back to simple approach)
    logInfo("Creating new PubSubClient instance with WiFiClient reference");
    client = new PubSubClient(__wifiClient);

    if (client == NULL)
    {
      logError("Failed to create PubSubClient");
      return false;
    }
    logInfo("PubSubClient created successfully");
  }

  if (client == NULL)
  {
    logError("Failed to create PubSubClient");
    return false;
  }
  logInfo("PubSubClient created successfully");
}

// Configure PubSubClient settings AFTER creation
CLIENT()->setServer(server.c_str(), port);
CLIENT()->setBufferSize(256);   // Reduce buffer size to 256
CLIENT()->setKeepAlive(30);     // Shorter keep-alive
CLIENT()->setSocketTimeout(10); // Shorter socket timeout

logInfo("PubSubClient configured - Server: " + server + ":" + String(port));
connected = false;
#endif

  isBegin = true;
  connectionFailedCount = 0;
  return true;
}

void MqttClient::end(void)
{
  if (!isBegin)
  {
    logWarning("Already end, call 'begin' and try again");
    return;
  }
#ifdef ESP32
  esp_mqtt_client_disconnect(client);
  esp_mqtt_client_stop(client);
  esp_mqtt_client_destroy(client);
  client = NULL;
#else
  if (CLIENT() != NULL)
  {
    CLIENT()->disconnect();
    delete (PubSubClient *)client;
    client = NULL;
  }
  // No need to delete WiFiClient reference
#endif
  isBegin = false;
  this->uri = "";

  logInfo("end");
}

void MqttClient::_updateConnected(bool connected)
{
  this->connected = connected;
  if (connected)
  {
    connectionFailedCount = 0;
  }
  else
  {
    connectionFailedCount++;
    logWarning("Connection failed count " + String(connectionFailedCount));
  }
}

bool MqttClient::publish(const char *topic, const char *payload, int len)
{
  if (!isBegin)
  {
    logError("No-initialized");
    return false;
  }

  if (!connected)
  {
    logError("Client disconnected");
    return false;
  }

#ifdef ESP8266
  // Just call loop() to maintain connection without checking status
  CLIENT()->loop();
#endif

#ifdef ESP32
  if (esp_mqtt_client_publish(client, topic, payload, len, 0, 0) == ESP_OK)
  {
    logInfo("Publish success");
    return true;
  }
#else
  if (CLIENT()->publish(topic, payload))
  {
    logInfo("Publish success");
    return true;
  }
#endif
  logError("Publish failed");
  return false;
}

/**
 * @brief Check that URI is same as current initialized  URI
 *
 * @param uri Target URI
 * @return true Same
 * @return false Difference
 */
bool MqttClient::isCurrentUri(String &uri)
{
  if (this->uri == uri)
  {
    return true;
  }
  return false;
}

/**
 * @brief Get MQTT client connected status
 *
 * @return true Connected
 * @return false Disconnected
 */
bool MqttClient::isConnected(void)
{
  // Add debug to see what's happening with connection status
  bool actualStatus = false;
  int clientState = -999;
  if (isBegin && CLIENT() != NULL)
  {
    actualStatus = CLIENT()->connected();
    clientState = CLIENT()->state();
  }

  Serial.println("[DEBUG] isConnected() - internal: " + String(connected) + ", actual: " + String(actualStatus) + ", state: " + String(clientState));

  // For debugging: return actual status from PubSubClient
  // return connected;
  return actualStatus; // Use actual status to see real connection state
}

/**
 * @brief Get number of connection failed
 *
 * @return int
 */
int MqttClient::getConnectionFailedCount(void) { return connectionFailedCount; }

#ifdef ESP8266
bool MqttClient::connect(String id)
{
  if (isBegin == false)
  {
    logError("MQTT not initialized");
    return false;
  }

  if (this->uri.isEmpty())
  {
    logError("MQTT URI is empty");
    return false;
  }

  logInfo("Attempting MQTT connection with ID: " + id);
  logInfo("Server: " + server + ":" + String(port));
  logInfo("Authentication - User: " + user + " (empty: " + String(user.isEmpty() ? "yes" : "no") + ")");

  // Reset connection status
  connected = false;

  // Check if PubSubClient is properly initialized
  if (CLIENT() == NULL)
  {
    logError("PubSubClient is NULL");
    return false;
  }

  // Check network connectivity first
  if (WiFi.status() != WL_CONNECTED)
  {
    logError("WiFi not connected");
    return false;
  }

  // Ensure we disconnect any previous connection
  if (CLIENT()->connected())
  {
    logInfo("Disconnecting previous connection");
    CLIENT()->disconnect();
    delay(100);
  }

  // Re-set server to ensure fresh connection
  CLIENT()->setServer(server.c_str(), port);

  bool connectResult = false;
  if (user.isEmpty())
  {
    logInfo("Attempting connection without auth");
    connectResult = CLIENT()->connect(id.c_str());
  }
  else
  {
    logInfo("Attempting connection with auth - User: " + user);
    // Use clean session (last parameter = true)
    connectResult = CLIENT()->connect(id.c_str(), user.c_str(), password.c_str(), NULL, 0, false, NULL, true);
  }
  if (connectResult)
  {
    connected = true;
    Serial.println("[DEBUG] connect() - Connection successful, setting internal connected = true");
    logInfo("MQTT connection successful");
    _updateConnected(true);

    // Don't call loop() here - let handle() do it
    Serial.println("[DEBUG] connect() - Skipping loop() call in connect method");

    // Check immediately if the connection is still valid
    delay(100); // Small delay to allow connection to stabilize
    bool immediateCheck = CLIENT()->connected();
    Serial.println("[DEBUG] connect() - Immediate check after connect: " + String(immediateCheck));
    Serial.println("[DEBUG] connect() - Client state after connect: " + String(CLIENT()->state()));

    if (!immediateCheck)
    {
      Serial.println("[DEBUG] connect() - WARNING: Connection lost immediately after connect!");

      // Try reconnecting once more with slight delay
      Serial.println("[DEBUG] connect() - Attempting immediate reconnect...");
      delay(500);

      bool retryResult = false;
      if (user.isEmpty())
      {
        retryResult = CLIENT()->connect(id.c_str());
      }
      else
      {
        retryResult = CLIENT()->connect(id.c_str(), user.c_str(), password.c_str(), NULL, 0, false, NULL, true);
      }

      if (retryResult && CLIENT()->connected())
      {
        Serial.println("[DEBUG] connect() - Immediate reconnect successful!");
        connected = true;
      }
      else
      {
        Serial.println("[DEBUG] connect() - Immediate reconnect also failed");
        connected = false;
      }
    }
    else
    {
      // Try a small loop() call to stabilize the connection
      Serial.println("[DEBUG] connect() - Connection stable, calling loop() once");
      CLIENT()->loop();
      delay(50);

      // Check again after loop
      bool afterLoop = CLIENT()->connected();
      Serial.println("[DEBUG] connect() - Status after loop(): " + String(afterLoop));
      if (!afterLoop)
      {
        Serial.println("[DEBUG] connect() - Connection lost after loop()!");
        connected = false;
      }
    }
  }
  else
  {
    int state = CLIENT()->state();
    logError("MQTT connection failed, state: " + String(state));

    // Decode MQTT client state
    String stateMsg = "";
    switch (state)
    {
    case -4:
      stateMsg = "MQTT_CONNECTION_TIMEOUT";
      break;
    case -3:
      stateMsg = "MQTT_CONNECTION_LOST";
      break;
    case -2:
      stateMsg = "MQTT_CONNECT_FAILED";
      break;
    case -1:
      stateMsg = "MQTT_DISCONNECTED";
      break;
    case 0:
      stateMsg = "MQTT_CONNECTED";
      break;
    case 1:
      stateMsg = "MQTT_CONNECT_BAD_PROTOCOL";
      break;
    case 2:
      stateMsg = "MQTT_CONNECT_BAD_CLIENT_ID";
      break;
    case 3:
      stateMsg = "MQTT_CONNECT_UNAVAILABLE";
      break;
    case 4:
      stateMsg = "MQTT_CONNECT_BAD_CREDENTIALS";
      break;
    case 5:
      stateMsg = "MQTT_CONNECT_UNAUTHORIZED";
      break;
    default:
      stateMsg = "UNKNOWN_STATE";
      break;
    }
    logError("State meaning: " + stateMsg);
    _updateConnected(false);
  }

  return connected;
}
void MqttClient::handle(void)
{
  Serial.println("[DEBUG] handle() - Alternative approach without loop()");

  if (isBegin == false)
  {
    Serial.println("[DEBUG] handle() - not initialized");
    return;
  }

  if (CLIENT() == NULL)
  {
    Serial.println("[DEBUG] handle() - CLIENT() is NULL");
    return;
  }

  // Check WiFi status first
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[DEBUG] handle() - WiFi not connected");
    connected = false;
    return;
  }

  // Alternative approach: Check connection status without calling loop()
  // PubSubClient will automatically handle the connection in publish() calls
  bool clientConnected = CLIENT()->connected();
  Serial.println("[DEBUG] handle() - PubSubClient reports connected: " + String(clientConnected));

  // Update our internal status
  connected = clientConnected;

  // If disconnected, we'll let the main code handle reconnection
  if (!connected)
  {
    Serial.println("[DEBUG] handle() - Connection lost, state: " + String(CLIENT()->state()));
  }

  Serial.println("[DEBUG] handle() - Alternative approach completed, internal status: " + String(connected));
}

void MqttClient::resetMqttClient(void)
{
  Serial.println("[DEBUG] resetMqttClient() - Resetting MQTT client");

  if (CLIENT() != NULL)
  {
    CLIENT()->disconnect();
    delete (PubSubClient *)client;
    client = NULL;
    Serial.println("[DEBUG] resetMqttClient() - Old PubSubClient deleted");
  }

  // No need to recreate WiFiClient - just use the existing reference
  Serial.println("[DEBUG] resetMqttClient() - Reusing existing WiFiClient reference");

  // Create fresh PubSubClient with existing WiFiClient
  logInfo("Creating fresh PubSubClient instance");
  client = new PubSubClient(__wifiClient);

  if (client != NULL)
  {
    CLIENT()->setServer(server.c_str(), port);
    CLIENT()->setBufferSize(512);
    CLIENT()->setKeepAlive(60);
    CLIENT()->setSocketTimeout(15);
    logInfo("Fresh PubSubClient created and configured");
    Serial.println("[DEBUG] resetMqttClient() - Fresh client configured");
  }
  else
  {
    logError("Failed to create fresh PubSubClient");
  }

  connected = false;
}
#endif

#ifdef ESP32
bool MqttClient::connect(String id)
{
  // For ESP32, the connection is handled automatically by the event system
  // This is just a placeholder for compatibility
  logInfo("ESP32 connect called - connection handled by event system");
  return connected;
}

void MqttClient::handle(void)
{
  // For ESP32, maintenance is handled automatically by the event system
  // This is just a placeholder for compatibility
}

void MqttClient::resetMqttClient(void)
{
  // For ESP32, reset the client by stopping and restarting
  if (client != NULL)
  {
    esp_mqtt_client_disconnect(client);
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    client = NULL;
  }

  // Reinitialize would require the URI, which we have stored
  if (!uri.isEmpty())
  {
    esp_mqtt_client_config_t config = {
        .uri = uri.c_str(),
    };

    client = esp_mqtt_client_init(&config);
    if (client != NULL)
    {
      esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, __mqtt_event_handler, this);
      esp_mqtt_client_start(client);
    }
  }

  connected = false;
}

static void __mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                 int32_t event_id, void *event_data)
{
  MqttClient *mqtt = (MqttClient *)handler_args;

  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  esp_mqtt_client_handle_t client = event->client;

  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id)
  {
  case MQTT_EVENT_CONNECTED:
    mqtt->logInfo("MQTT_EVENT_CONNECTED");
    mqtt->_updateConnected(true);
    break;
  case MQTT_EVENT_DISCONNECTED:
    mqtt->logInfo("MQTT_EVENT_DISCONNECTED");
    mqtt->_updateConnected(false);
    break;
  case MQTT_EVENT_SUBSCRIBED:
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    break;
  case MQTT_EVENT_PUBLISHED:
    break;
  case MQTT_EVENT_DATA:
    break;
  case MQTT_EVENT_ERROR:
    Serial.println("MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
    {
      mqtt->logError("Reported from esp-tls: " +
                     String(event->error_handle->esp_tls_last_esp_err));
      mqtt->logError("Reported from tls stack: " +
                     String(event->error_handle->esp_tls_stack_err));
      mqtt->logError("Captured as transport's socket errno: " +
                     String(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    Serial.printf("Other event id:%d\r\n", event->event_id);
    break;
  }
}
#endif
