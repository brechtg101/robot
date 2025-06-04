#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include <WiFi.h>
#include <PubSubClient.h>

// Debug control - must be defined before any debug macros
#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED false  // Set to true to enable debug messages
#endif

// Debug print macros
#define DEBUG_PRINT(x) if (DEBUG_ENABLED) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_ENABLED) { Serial.println(x); }
#define DEBUG_PRINTF(...) if (DEBUG_ENABLED) { Serial.printf(__VA_ARGS__); }

// WiFi credentials
const char* WIFI_SSID = "Sonos";
const char* WIFI_PASSWORD = "Gu9510112!";

// MQTT Broker settings
const char* MQTT_BROKER = "mqtt.eclipseprojects.io";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_Autobrecht";


// MQTT Topics
const char* TOPIC_DISTANCES = "autobrecht/sensors/distances";
const char* TOPIC_LINE_SENSORS = "autobrecht/sensors/line";
const char* TOPIC_MOTOR_SPEEDS = "autobrecht/motors/speeds";
const char* TOPIC_STATUS = "autobrecht/status";

// Global variables
WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_DELAY = 5000;  // 5 seconds

void setupWiFi() {
    Serial.println("Connecting to WiFi...");  // Always print WiFi connection status
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi connected");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
}

void setupMqtt() {
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback([](char* topic, byte* payload, unsigned int length) {
        if (DEBUG_ENABLED) {
            Serial.printf("Message received on topic %s\n", topic);
        }
    });
}

bool connectMqtt() {
    if (mqtt.connected()) {
        return true;
    }
    
    Serial.println("Connecting to MQTT broker...");
    Serial.printf("Broker: %s, Port: %d\n", MQTT_BROKER, MQTT_PORT);
    
    // Check WiFi connection first
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected! Cannot connect to MQTT broker.");
        return false;
    }
    
    // Try to connect with a longer timeout
    mqtt.setKeepAlive(60);  // Increase keepalive to 60 seconds
    mqtt.setSocketTimeout(10);  // Increase socket timeout to 10 seconds
    
    if (mqtt.connect(MQTT_CLIENT_ID)) {
        Serial.println("Connected to MQTT broker");
        return true;
    }
    
    // Get the connection state for more detailed error reporting
    int state = mqtt.state();
    Serial.printf("Failed to connect to MQTT broker. Error code: %d\n", state);
    switch (state) {
        case -4: Serial.println("Connection timeout"); break;
        case -3: Serial.println("Server unreachable"); break;
        case -2: Serial.println("Server rejected connection"); break;
        case -1: Serial.println("Client disconnected"); break;
        case 1: Serial.println("Unacceptable protocol version"); break;
        case 2: Serial.println("Identifier rejected"); break;
        case 3: Serial.println("Server unavailable"); break;
        case 4: Serial.println("Bad username or password"); break;
        case 5: Serial.println("Not authorized"); break;
        default: Serial.println("Unknown error"); break;
    }
    
    return false;
}

void publishDistances(float center) {
    if (!mqtt.connected()) return;
    
    char message[100];
    snprintf(message, sizeof(message), 
             "{\"center\":%.1f}", 
             center);
    mqtt.publish(TOPIC_DISTANCES, message);
    DEBUG_PRINTF("Published distance - Center: %.1f cm\n", center);
}

void publishLineSensors(bool left, bool right) {
    if (!mqtt.connected()) return;
    
    char message[100];
    snprintf(message, sizeof(message), 
             "{\"left\":%d,\"right\":%d}", 
             left, right);
    mqtt.publish(TOPIC_LINE_SENSORS, message);
}

void publishMotorSpeeds(int left, int right) {
    if (!mqtt.connected()) return;
    
    char message[100];
    snprintf(message, sizeof(message), 
             "{\"left\":%d,\"right\":%d}", 
             left, right);
    mqtt.publish(TOPIC_MOTOR_SPEEDS, message);
    DEBUG_PRINTF("Published motor speeds - Left: %d%%, Right: %d%%\n", 
                 left, right);
}

void publishStatus(const char* status) {
    if (!mqtt.connected()) return;
    mqtt.publish(TOPIC_STATUS, status);
    DEBUG_PRINTF("Published status: %s\n", status);
}

void publishMovement(const char* movement) {
    if (!mqtt.connected()) return;
    mqtt.publish("autobrecht/movement", movement);
}

void updateMqtt() {
    if (!mqtt.connected()) {
        unsigned long now = millis();
        if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_DELAY) {
            lastMqttReconnectAttempt = now;
            if (connectMqtt()) {
                lastMqttReconnectAttempt = 0;
            }
        }
    } else {
        mqtt.loop();
    }
}

#endif // MQTT_COMM_H 