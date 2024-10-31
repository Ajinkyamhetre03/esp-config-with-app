#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

const char *ap_ssid = "ESP8266_AP";         // Access Point SSID
const char *ap_password = "password123";    // Access Point password

const char *mqtt_server = "dev.coppercloud.in";  // MQTT broker address
const int mqtt_port = 1883;                      // MQTT port

ESP8266WebServer server(80);   // Web server on port 80
WiFiClient espClient;
PubSubClient client(espClient);

#define LED_D4 D4    // LED connected to GPIO2 (D4)
#define LED_D0 D0    // LED connected to GPIO16 (D0)

// Variables to track pin usage
bool useD4 = false;
bool useD0 = false;

// Function declarations
void setup_wifi(const char *ssid, const char *password);
void connectToMQTT();
void handleData();
void mqttCallback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);

  // Set up ESP8266 as an Access Point (AP)
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point created");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Initialize pins for LEDs (we’ll set these based on received settings)
  pinMode(LED_D4, OUTPUT);
  pinMode(LED_D0, OUTPUT);
  digitalWrite(LED_D4, LOW);  // Ensure LEDs are off initially
  digitalWrite(LED_D0, LOW);

  // Set up route to receive JSON data
  server.on("/sendData", HTTP_POST, handleData);
  server.begin();
  Serial.println("Server started");

  // Set up MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

void handleData() {
  if (server.hasArg("plain")) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      server.send(400, "application/json", "{\"status\":\"Bad JSON\"}");
      return;
    }

    String wifi_ssid = doc["WiFi"];
    String wifi_password = doc["password"];
    useD4 = doc["D4"];
    useD0 = doc["D0"];

    Serial.println("Received WiFi credentials:");
    Serial.print("SSID: ");
    Serial.println(wifi_ssid);
    Serial.print("Password: ");
    Serial.println(wifi_password);
    Serial.print("Use D4: ");
    Serial.println(useD4 ? "True" : "False");
    Serial.print("Use D0: ");
    Serial.println(useD0 ? "True" : "False");

    // Disconnect from AP mode and connect to WiFi with received credentials
    WiFi.softAPdisconnect(true);
    setup_wifi(wifi_ssid.c_str(), wifi_password.c_str());
  } else {
    server.send(400, "application/json", "{\"status\":\"No Data\"}");
  }
}

void setup_wifi(const char *ssid, const char *password) {
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server.send(200, "application/json", "{\"status\":\"Connected to WiFi\"}");

    // Now that Wi-Fi is connected, start MQTT connection
    connectToMQTT();
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    WiFi.softAP(ap_ssid, ap_password);  // Re-enable AP mode if WiFi connection fails
  }
}

void connectToMQTT() {
  if (client.connect("ESP8266Client")) {  // No username and password
    Serial.println("Connected to MQTT broker");
    client.subscribe("home/led/control");  // Subscribe to a single topic for D4 and D0 control
  } else {
    Serial.println("Failed to connect to MQTT broker");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(message);

  // Control LED based on MQTT message
  if (message.startsWith("D0")) {
    if (message == "D0ON" && useD0) {
      digitalWrite(LED_D0, HIGH);  // Turn LED D0 on
    } else if (message == "D0OFF" && useD0) {
      digitalWrite(LED_D0, LOW);   // Turn LED D0 off
    }
  } else if (message.startsWith("D4")) {
    if (message == "D4ON" && useD4) {
      digitalWrite(LED_D4, HIGH);  // Turn LED D4 on
    } else if (message == "D4OFF" && useD4) {
      digitalWrite(LED_D4, LOW);   // Turn LED D4 off
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client")) {  // No username and password
      Serial.println("connected");
      client.subscribe("home/led/control");  // Subscribe to a single topic for D4 and D0 control
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  server.handleClient();  // Handle incoming HTTP requests

  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    reconnect();
  }
  client.loop();  // Handle MQTT client tasks
}
