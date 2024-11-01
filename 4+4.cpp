#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// Wi-Fi and MQTT details
const char* mqtt_server = "dev.coppercloud.in";
const char* mqtt_topic = "esp8266/led";

// Define hardware
#define LED_PIN_D4 5     // D4 on NodeMCU
#define LED_PIN_D0 4    // D0 on NodeMCU
#define LED_PIN_D1 14     // D1 on NodeMCU
#define LED_PIN_D2 12    // D2 on NodeMCU

#define RESET_PIN 2    // D3 on NodeMCU (Connect a button to this pin and GND)
#define TOGGLE_PIN_D7 10 // D7 on NodeMCU (Toggles D4 LED)
#define TOGGLE_PIN_D8 0 // D8 on NodeMCU (Toggles D0 LED)
#define TOGGLE_PIN_D5 13 // D5 on NodeMCU (Toggles D1 LED)
#define TOGGLE_PIN_D6 3 // D6 on NodeMCU (Toggles D2 LED)

WiFiClient espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80);

// EEPROM storage addresses for LED states
#define FLAG_ADDR 100           // Flag to indicate saved credentials
#define LED_STATE_D4_ADDR 101   // Address for storing D4 LED state
#define LED_STATE_D0_ADDR 102   // Address for storing D0 LED state
#define LED_STATE_D1_ADDR 103   // Address for storing D1 LED state
#define LED_STATE_D2_ADDR 104   // Address for storing D2 LED state

bool wifiConnected = false;
unsigned long buttonPressTime = 0;
bool resetTriggered = false;

// Track the last states of each toggle pin
bool lastToggleStateD7 = HIGH;
bool lastToggleStateD8 = HIGH;
bool lastToggleStateD5 = HIGH;
bool lastToggleStateD6 = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN_D4, OUTPUT);
  pinMode(LED_PIN_D0, OUTPUT);
  pinMode(LED_PIN_D1, OUTPUT);
  pinMode(LED_PIN_D2, OUTPUT);
  
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_PIN_D7, INPUT_PULLUP);
  pinMode(TOGGLE_PIN_D8, INPUT_PULLUP);
  pinMode(TOGGLE_PIN_D5, INPUT_PULLUP);
  pinMode(TOGGLE_PIN_D6, INPUT_PULLUP);
  EEPROM.begin(512);

  checkForReset();
  
  // Restore last LED states from EEPROM
  restoreLEDState(LED_PIN_D4, LED_STATE_D4_ADDR);
  restoreLEDState(LED_PIN_D0, LED_STATE_D0_ADDR);
  restoreLEDState(LED_PIN_D1, LED_STATE_D1_ADDR);
  restoreLEDState(LED_PIN_D2, LED_STATE_D2_ADDR);

  // Attempt to connect to Wi-Fi with saved credentials, or start AP mode
  if (EEPROM.read(FLAG_ADDR) == 1 && !resetTriggered) {
    connectToWiFi();
  } else {
    startAccessPoint();
  }
}

void loop() {
  server.handleClient();

  // Handle reset button
  handleReset();

  // Reconnect to MQTT if needed
  if (wifiConnected && !client.connected()) {
    connectToMQTT();
  }
  client.loop();

  // Handle toggle actions
  handleToggle(TOGGLE_PIN_D7, LED_PIN_D4, LED_STATE_D4_ADDR, lastToggleStateD7, "D4");
  handleToggle(TOGGLE_PIN_D8, LED_PIN_D0, LED_STATE_D0_ADDR, lastToggleStateD8, "D0");
  handleToggle(TOGGLE_PIN_D5, LED_PIN_D1, LED_STATE_D1_ADDR, lastToggleStateD5, "D1");
  handleToggle(TOGGLE_PIN_D6, LED_PIN_D2, LED_STATE_D2_ADDR, lastToggleStateD6, "D2");
}

void startAccessPoint() {
  WiFi.softAP("ESP8266_AP", "password123");
  Serial.println("Access Point started. Connect to 'ESP8266_AP'");

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<h1>Send Wi-Fi Credentials in JSON</h1>");
  });

  server.on("/saveWiFi", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "Bad Request");
      return;
    }
    
    String jsonData = server.arg("plain");
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error) {
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }

    String ssid = doc["ssid"];
    String password = doc["password"];
    saveCredentials(ssid, password);

    server.send(200, "text/plain", "Credentials Saved. Rebooting...");
    delay(1000);
    ESP.restart();
  });

  server.begin();
  Serial.println("Web server started.");
}

void connectToWiFi() {
  String ssid = readStringFromEEPROM(0);
  String password = readStringFromEEPROM(32);
  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.print("Connecting to Wi-Fi");
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi!");
    wifiConnected = true;
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);
    connectToMQTT();
  } else {
    startAccessPoint();
  }
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received: ");
  Serial.println(message);

  if (message == "D4_ON") {
    updateLED(LED_PIN_D4, LED_STATE_D4_ADDR, LOW, "D4_ON");
  } else if (message == "D4_OFF") {
    updateLED(LED_PIN_D4, LED_STATE_D4_ADDR, HIGH, "D4_OFF");
  } else if (message == "D0_ON") {
    updateLED(LED_PIN_D0, LED_STATE_D0_ADDR, LOW, "D0_ON");
  } else if (message == "D0_OFF") {
    updateLED(LED_PIN_D0, LED_STATE_D0_ADDR, HIGH, "D0_OFF");
  } else if (message == "D1_ON") {
    updateLED(LED_PIN_D1, LED_STATE_D1_ADDR, LOW, "D1_ON");
  } else if (message == "D1_OFF") {
    updateLED(LED_PIN_D1, LED_STATE_D1_ADDR, HIGH, "D1_OFF");
  } else if (message == "D2_ON") {
    updateLED(LED_PIN_D2, LED_STATE_D2_ADDR, LOW, "D2_ON");
  } else if (message == "D2_OFF") {
    updateLED(LED_PIN_D2, LED_STATE_D2_ADDR, HIGH, "D2_OFF");
  }
}

void saveCredentials(String ssid, String password) {
  writeStringToEEPROM(0, ssid);
  writeStringToEEPROM(32, password);
  EEPROM.write(FLAG_ADDR, 1);
  EEPROM.commit();
}

void checkForReset() {
  if (digitalRead(RESET_PIN) == LOW) {
    Serial.println("Reset detected. Clearing credentials...");
    resetTriggered = true;
    clearCredentials();
    ESP.restart();
  }
}

void clearCredentials() {
  for (int i = 0; i < 64; i++) EEPROM.write(i, 0);
  EEPROM.write(FLAG_ADDR, 0);
  EEPROM.commit();
}

void handleReset() {
  if (digitalRead(RESET_PIN) == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
    } else if (millis() - buttonPressTime >= 1000) {
      clearCredentials();
      Serial.println("Credentials cleared. Restarting...");
      delay(500);
      ESP.restart();
    }
  } else {
    buttonPressTime = 0;
  }
}

void handleToggle(uint8_t togglePin, uint8_t ledPin, int eepromAddr, bool& lastState, const char* ledName) {
  bool currentState = digitalRead(togglePin);
  if (currentState == LOW && lastState == HIGH) {
    bool ledState = !digitalRead(ledPin);
    digitalWrite(ledPin, ledState);
    EEPROM.write(eepromAddr, ledState);
    EEPROM.commit();
    String mqttMessage = String(ledName) + (ledState ? "_ON" : "_OFF");
    client.publish(mqtt_topic, mqttMessage.c_str());
  }
  lastState = currentState;
}

void updateLED(uint8_t ledPin, int eepromAddr, bool state, const char* mqttMessage) {
  digitalWrite(ledPin, state);
  EEPROM.write(eepromAddr, state);
  EEPROM.commit();
  client.publish(mqtt_topic, mqttMessage);
}

void restoreLEDState(uint8_t ledPin, int eepromAddr) {
  digitalWrite(ledPin, EEPROM.read(eepromAddr) ? LOW : HIGH);
}

void writeStringToEEPROM(int addr, const String& str) {
  for (int i = 0; i < str.length(); i++) EEPROM.write(addr + i, str[i]);
  EEPROM.write(addr + str.length(), 0); // Null-terminate the string
  EEPROM.commit();
}

String readStringFromEEPROM(int addr) {
  String str;
  char ch;
  for (int i = 0; i < 32; i++) {
    ch = EEPROM.read(addr + i);
    if (ch == 0) break;
    str += ch;
  }
  return str;
}
