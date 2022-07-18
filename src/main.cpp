#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <NewPing.h>

#define FLAG_DEBUG_WIFI true

#define LED_RED_PIN D2
#define LED_GREEN_PIN D3

#define AP_SSID "SM_AP"
#define AP_SECRET "abcDEF123!"

#define MQTT_SERVER_IP "192.168."
#define MQTT_SERVER_PORT 1883

#define TOPIC_HEARTBEAT "sm/heartbeat"
#define TOPIC_MAIL_ARRIVED "sm/mail-arrived"
#define TOPIC_CALIBRATE "sm/calibrate"
#define TOPIC_BATTERY_STATUS "sm/battery-status"

#define FILE_CONFIG "/config.json"

#define DISTANCE_SENSOR_TRIGGER_PIN  D1
#define DISTANCE_SENSOR_ECHO_PIN D0
#define DISTANCE_SENSOR_MAX_DISTANCE 300

#define ADC_PIN A0 // analog to digital converter pin

const ulong LAST_HEARTBEAT_FREQUENCY = 3000; // send heartbeat every 3 seconds
const ulong LAST_DISTANCE_UPDATE_FREQUENCY = 100; // check distance every 100 milliseconds
const ulong LAST_MAIL_ARRIVED_UPDATE_FREQUENCY = 3000; // check if mail has arrived after 3 seconds
const ulong LAST_BATTERY_STATUS_UPDATE_FREQUENCE = 3000; // check batter status every 3 seconds
const ulong MAX_DELTA_TO_DISTANCE = 0; // the maximum delta to distance after subtracting calibration value

/*
 * SETUP
 */

ulong CURRENT_DISTANCE = 0;
ulong CURRENT_CALIBRATION_VALUE = 0;

void onButNotReady() {
    analogWrite(LED_RED_PIN, 255);
    delay(1000);
}

void onAndReady() {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 255);
}

void setupWifi() {
    WiFi.mode(WIFI_STA);
    Serial.println("[Wifi] MAC address: " + WiFi.macAddress());

    WiFiManager wifiManager;
    wifiManager.setDebugOutput(FLAG_DEBUG_WIFI);
    // wifiManager.resetSettings();

    bool isConnected = wifiManager.autoConnect(AP_SSID, AP_SECRET);

    if (isConnected) {
        Serial.println("[WifiManager] Connected");
    } else {
        Serial.println("[WifiManager] Connection failed");
    }
}

void updateConfig() {
    DynamicJsonDocument doc(256);
    doc["calibrationValue"] = CURRENT_DISTANCE;

    File config = LittleFS.open(FILE_CONFIG, "w");
    if (config) {
        serializeJson(doc, Serial);
        serializeJson(doc, config);
        config.flush();
        config.close();
    }
}

void loadConfig() {
    if (LittleFS.begin()) {
        if (LittleFS.exists(FILE_CONFIG)) {
            File config = LittleFS.open(FILE_CONFIG, "r");
            if (config) {
                DynamicJsonDocument doc(256);

                DeserializationError error = deserializeJson(doc, config);
                if (error) return;

                CURRENT_CALIBRATION_VALUE = (long) doc["calibrationValue"];
            }
        }
    }
}

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setupMqtt() {
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    mqttClient.setCallback([](char *topic, byte *payload, unsigned int length) {
        Serial.println("[MqttClient] Topic: " + String(topic));
        Serial.println("[MqttClient] Payload: ");
        for (int i = 0; i < length; i++) {
            Serial.print((char) payload[i]);
        }
        Serial.println();

        if (String(topic).equals(TOPIC_CALIBRATE)) {
            updateConfig();
        }
    });
}

[[maybe_unused]] void setup() {
    Serial.begin(9600);
    Serial.println();

    onButNotReady();

    setupWifi();
    setupMqtt();

    loadConfig();

    onAndReady();
}

/*
 * LOOP
 */

ulong LAST_HEARTBEAT = 0;
ulong LAST_DISTANCE_UPDATE = 0;
ulong LAST_MAIL_ARRIVED_UPDATE = 0;
ulong LAST_BATTERY_STATUS_UPDATE = 0;

void reconnectMqtt() {
    while (!mqttClient.connected()) {
        Serial.print("[MqttClient] Try to reconnect...");
        String id = String(random(0xffff), HEX);
        if (mqttClient.connect(id.c_str())) {
            Serial.println("[MqttClient] Connected");
            mqttClient.subscribe(TOPIC_CALIBRATE);
        } else {
            Serial.println("[MqttClient] Connection failed");
            Serial.println("[MqttClient] State:" + String(mqttClient.state()));
            Serial.println("[MqttClient] Try again in 5 seconds");
            delay(5000);
        }
    }
}

NewPing distanceSensor(
        DISTANCE_SENSOR_TRIGGER_PIN,
        DISTANCE_SENSOR_ECHO_PIN,
        DISTANCE_SENSOR_MAX_DISTANCE
);

// https://github.com/esdkrwl/UpcycledMailbox/blob/master/Software/Firmware/src/main.cpp#L217
double determineBatteryVoltage() {
    analogRead(ADC_PIN);
    delay(5);
    int input = 0;
    for (int i = 0; i < 10; i++) {
        input += analogRead(ADC_PIN);
        delay(5);
    }
    return ((double) input / 10.0) / 1024.0;
}

[[maybe_unused]] void loop() {
    if (!mqttClient.connected()) {
        reconnectMqtt();
    }
    mqttClient.loop();

    ulong currentTime = millis();

    if (currentTime - LAST_HEARTBEAT >= LAST_HEARTBEAT_FREQUENCY) {
        LAST_HEARTBEAT = currentTime;
        mqttClient.publish(TOPIC_HEARTBEAT, R"({"msg":"I am alive"})");
    }

    if (currentTime - LAST_DISTANCE_UPDATE >= LAST_DISTANCE_UPDATE_FREQUENCY) {
        LAST_DISTANCE_UPDATE = currentTime;

        CURRENT_DISTANCE = distanceSensor.ping_cm();

        Serial.println("[Smart Mailbox] Current distance: " + String(CURRENT_DISTANCE) + " cm");

        loadConfig();

        Serial.println("[Smart Mailbox] Current calibration value: " + String(CURRENT_CALIBRATION_VALUE) + " cm");

        if (CURRENT_CALIBRATION_VALUE > 0
            && (CURRENT_DISTANCE < CURRENT_CALIBRATION_VALUE - MAX_DELTA_TO_DISTANCE
                || CURRENT_DISTANCE > CURRENT_CALIBRATION_VALUE - MAX_DELTA_TO_DISTANCE)
                ) {
            if (currentTime - LAST_MAIL_ARRIVED_UPDATE >= LAST_MAIL_ARRIVED_UPDATE_FREQUENCY) {
                LAST_MAIL_ARRIVED_UPDATE = currentTime;
                mqttClient.publish(TOPIC_MAIL_ARRIVED, R"({"msg":"Mail has arrived"})");
            }
        }
    }

    if (currentTime - LAST_BATTERY_STATUS_UPDATE >= LAST_BATTERY_STATUS_UPDATE_FREQUENCE) {
        LAST_BATTERY_STATUS_UPDATE = currentTime;
        auto batteryVoltageInPercent = (uint8_t) abs((determineBatteryVoltage() * 100) - 100);
        Serial.println("[Smart Mailbox] Battery status: " + String(batteryVoltageInPercent) + "%");
        mqttClient.publish(
                TOPIC_BATTERY_STATUS,
                String("{\"batteryStatus\":" + String(batteryVoltageInPercent) + "}").c_str()
        );
    }
}
