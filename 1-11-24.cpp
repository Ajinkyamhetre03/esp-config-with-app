#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// Wi-Fi and MQTT details
const char* mqtt_server = "dev.coppercloud.in";
const char* mqtt_topic = "esp8266/led";

// Define hardware
#define LED_PIN_D4 2   // D4 on NodeMCU
#define LED_PIN_D0 16  // D0 on NodeMCU
#define RESET_PIN 0    // D3 on NodeMCU (Connect a button to this pin and GND)
#define TOGGLE_PIN_D7 10  // D7 on NodeMCU (Connect to GND to toggle D4)
WiFiClient espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80);

// EEPROM storage addresses
#define SSID_ADDR 0
#define PASS_ADDR 32
#define FLAG_ADDR 100     // Flag to indicate saved credentials
#define LED_STATE_D4_ADDR 101 // Address for storing D4 LED state
#define LED_STATE_D0_ADDR 102 // Address for storing D0 LED state

bool wifiConnected = false;
unsigned long buttonPressTime = 0;
bool resetTriggered = false;
bool lastTogglePinState = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN_D4, OUTPUT);
  pinMode(LED_PIN_D0, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);  // Configure reset button pin
  pinMode(TOGGLE_PIN_D7, INPUT_PULLUP); // Configure toggle pin
  EEPROM.begin(512);

  // Check if reset button is held on startup
  checkForReset();

  // Restore the last saved LED states
  int savedStateD4 = EEPROM.read(LED_STATE_D4_ADDR);
  int savedStateD0 = EEPROM.read(LED_STATE_D0_ADDR);
  digitalWrite(LED_PIN_D4, savedStateD4 == 1 ? LOW : HIGH);
  digitalWrite(LED_PIN_D0, savedStateD0 == 1 ? LOW : HIGH);

  // If credentials are saved, attempt to connect to Wi-Fi
  if (EEPROM.read(FLAG_ADDR) == 1 && !resetTriggered) {
    String ssid = readStringFromEEPROM(SSID_ADDR);
    String password = readStringFromEEPROM(PASS_ADDR);

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
      return;
    }
  }

  // If no Wi-Fi credentials, start in AP mode
  startAccessPoint();
}

void loop() {
  server.handleClient();

  // Check if reset button is held for 1 second
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

  // Reconnect to MQTT if disconnected and Wi-Fi is connected
  if (wifiConnected && !client.connected()) {
    connectToMQTT();
  }
  client.loop();

  // Check for changes on the TOGGLE_PIN_D7
  bool currentTogglePinState = digitalRead(TOGGLE_PIN_D7);
  if (currentTogglePinState == LOW && lastTogglePinState == HIGH) {
    // Toggle D4 LED state
    int currentD4State = digitalRead(LED_PIN_D4);
    if (currentD4State == HIGH) {
      digitalWrite(LED_PIN_D4, LOW);  // Turn on LED D4
      EEPROM.write(LED_STATE_D4_ADDR, 1);  // Save D4 state as ON
      client.publish(mqtt_topic, "D4_ON");  // Send MQTT message
    } else {
      digitalWrite(LED_PIN_D4, HIGH); // Turn off LED D4
      EEPROM.write(LED_STATE_D4_ADDR, 0);  // Save D4 state as OFF
      client.publish(mqtt_topic, "D4_OFF"); // Send MQTT message
    }
    EEPROM.commit();
    delay(50);  // Debounce delay
  }
  lastTogglePinState = currentTogglePinState;
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
    digitalWrite(LED_PIN_D4, LOW);
    EEPROM.write(LED_STATE_D4_ADDR, 1);  // Save LED D4 state as ON
    EEPROM.commit();
  } else if (message == "D4_OFF") {
    digitalWrite(LED_PIN_D4, HIGH);
    EEPROM.write(LED_STATE_D4_ADDR, 0);  // Save LED D4 state as OFF
    EEPROM.commit();
  } else if (message == "D0_ON") {
    digitalWrite(LED_PIN_D0, LOW);
    EEPROM.write(LED_STATE_D0_ADDR, 1);  // Save LED D0 state as ON
    EEPROM.commit();
  } else if (message == "D0_OFF") {
    digitalWrite(LED_PIN_D0, HIGH);
    EEPROM.write(LED_STATE_D0_ADDR, 0);  // Save LED D0 state as OFF
    EEPROM.commit();
  }
}

void saveCredentials(String ssid, String password) {
  writeStringToEEPROM(SSID_ADDR, ssid);
  writeStringToEEPROM(PASS_ADDR, password);
  EEPROM.write(FLAG_ADDR, 1);
  EEPROM.commit();
}

void clearCredentials() {
  for (int i = SSID_ADDR; i < SSID_ADDR + 64; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.write(FLAG_ADDR, 0);
  EEPROM.commit();
}

void writeStringToEEPROM(int address, String data) {
  for (int i = 0; i < data.length(); i++) {
    EEPROM.write(address + i, data[i]);
  }
  EEPROM.write(address + data.length(), 0);
}

String readStringFromEEPROM(int address) {
  String data = "";
  char ch;
  for (int i = address; i < address + 32; i++) {
    ch = EEPROM.read(i);
    if (ch == 0) break;
    data += ch;
  }
  return data;
}

void checkForReset() {
  if (digitalRead(RESET_PIN) == LOW) {
    delay(1000);
    if (digitalRead(RESET_PIN) == LOW) {
      Serial.println("Long press detected. Clearing credentials...");
      clearCredentials();
      resetTriggered = true;
      delay(500);
    }
  }
}
