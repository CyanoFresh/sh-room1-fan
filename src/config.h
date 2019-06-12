#ifndef ROOM1_SECONDARY_LIGHT_CONFIG_H
#define ROOM1_SECONDARY_LIGHT_CONFIG_H

#include <Arduino.h>

namespace config {
    const char WIFI_SSID[] = "Solomaha";
    const char WIFI_PASSWORD[] = "solomakha21";

    const auto MQTT_HOST = IPAddress(192, 168, 1, 230);
    const uint16_t MQTT_PORT = 1883;
    const char MQTT_ID[] = "room1-fan";
    const char MQTT_PASSWORD[] = "svcnr6u2tc3tnex23nbgdf7zm7eyd24v73gvcn63m273";

    const uint8_t RELAY_PIN = D1;
    const uint8_t BTN_PIN = D2;

    const uint8_t BTN_DEBOUNCE_PERIOD = 70;
}

#endif
