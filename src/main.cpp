#include <AsyncMqttClient.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <EEPROM.h>
#include "config.h"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

const char *stateToString(uint8_t state = 2) {
    if (state == 2) {
        state = digitalRead(config::RELAY_PIN);
    }

    return state == HIGH ? "false" : "true";
}

void connectToWifi() {
    Serial.println(F("Connecting to Wi-Fi..."));
    WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println(F("Connecting to MQTT..."));
    mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &event) {
    connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event) {
    Serial.print(F("Disconnected from Wi-Fi: "));
    Serial.println(event.reason);
    digitalWrite(LED_BUILTIN, LOW);

    mqttReconnectTimer.detach();
    wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool) {
    Serial.println(F("Connected to MQTT."));
    digitalWrite(LED_BUILTIN, HIGH);

    // Subscribe to topics:
    mqttClient.subscribe("switch/room1-fan/set", 0);
    mqttClient.subscribe("switch/room1-fan/toggle", 0);

    // Send current state
    mqttClient.publish("switch/room1-fan", 0, false, stateToString());
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.print(F("Disconnected from MQTT. Reason: "));
    Serial.println((int) reason);
    digitalWrite(LED_BUILTIN, LOW);

    if (WiFi.isConnected()) {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

char payloadBuffer[6];

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties, size_t len, size_t, size_t) {
    uint8_t newState = HIGH;

    if (strcmp(topic, "switch/room1-fan/toggle") == 0) {
        if (digitalRead(config::RELAY_PIN) == HIGH) {
            newState = LOW;
        }

        mqttClient.publish("switch/room1-fan", 0, false, newState == HIGH ? "false" : "true");
    } else {
        payloadBuffer[len] = '\0';
        strncpy(payloadBuffer, payload, len);

        if (strncmp(payloadBuffer, "true", 4) == 0) {
            newState = LOW;  // Turn on
        }

        mqttClient.publish("switch/room1-fan", 0, false, payloadBuffer);
    }

    digitalWrite(config::RELAY_PIN, newState);

    // Save last state to the memory
    EEPROM.put(0, newState);
    EEPROM.commit();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(config::RELAY_PIN, OUTPUT);
    pinMode(config::BTN_PIN, INPUT_PULLUP);

    digitalWrite(LED_BUILTIN, LOW);

    bool lastState = HIGH;  // off by default

    EEPROM.begin(sizeof(bool));    // 1 byte is enough for boolean
    EEPROM.get(0, lastState);

    digitalWrite(config::RELAY_PIN, lastState);

    Serial.begin(115200);
    Serial.println();
    Serial.println();

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(config::MQTT_HOST, config::MQTT_PORT);
    mqttClient.setClientId(config::MQTT_ID);
    mqttClient.setCredentials("device", config::MQTT_PASSWORD);

    connectToWifi();
}

unsigned long lastBtnTime = 0;
bool lastBtnState = HIGH;

void loop() {
    unsigned long time = millis();

    if (time - lastBtnTime >= config::BTN_DEBOUNCE_PERIOD) {
        bool btnState = digitalRead(config::BTN_PIN);

        if (btnState != lastBtnState) {
            if (btnState == LOW) {   // button pressed
                const bool newState = !digitalRead(config::RELAY_PIN);

                digitalWrite(config::RELAY_PIN, newState);

                mqttClient.publish("switch/room1-fan", 0, false, stateToString(newState));

                Serial.println("Button pressed");

                // Save last state to the memory
                EEPROM.put(0, newState);
                EEPROM.commit();
            }

            lastBtnState = btnState;
        }

        lastBtnTime = time;
    }
}