// Tracking firmware: keeps the wormbot centred in the camera image by jogging
// the SCARA arm through FluidNC over telnet.
//
// core 1 (loop):  camera -> JPEG decode -> crop/resize -> FOMO -> Detection
// core 0 (task):  FluidNC telnet, visual servoing, calibration, safety
// core 0 (httpd): dashboard at http://<xiao>/

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "controller.h"
#include "secrets.h"
#include "vision.h"

void web_begin();

static void wifi_event(WiFiEvent_t ev) {
    if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP)
        Serial.printf("WiFi: http://%s/\n", WiFi.localIP().toString().c_str());
    else if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
        controller::requestDisable("WiFi lost");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nwormbot tracker");
    if (!psramFound()) Serial.println("WARNING: no PSRAM");
    if (!vision::begin()) {
        Serial.println("camera init failed, halting");
        while (true) delay(1000);
    }
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // power save adds 100+ ms of jitter to every jog
    WiFi.setHostname("wormbot-tracker");
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(wifi_event);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    controller::begin();
    web_begin();
}

void loop() {
    vision::step(controller::detectionThreshold());
}
