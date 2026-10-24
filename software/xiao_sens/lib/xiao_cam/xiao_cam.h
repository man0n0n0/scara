#pragma once
// Camera setup shared by the capture and tracker firmwares.
//
// Sensor settings live in NVS (namespace "xcam"). The capture firmware edits and
// saves them; the tracker only loads them. Flashing one firmware over the other
// keeps NVS, so the model runs on images taken with the same exposure, gain and
// white balance as its training set. `pio run -t erase` wipes them.

#include <Arduino.h>
#include <esp_camera.h>

struct CamSettings {
    uint8_t  version;
    int8_t   framesize;    // framesize_t; QVGA (320x240) recommended
    int8_t   quality;      // JPEG quality, 4..63, lower = better
    int8_t   brightness;   // -2..2
    int8_t   contrast;     // -2..2
    int8_t   saturation;   // -2..2
    int8_t   ae_level;     // -2..2, auto-exposure target
    uint8_t  aec;          // 1 = auto exposure
    uint16_t aec_value;    // 0..1200, manual exposure when aec = 0
    uint8_t  agc;          // 1 = auto gain
    uint8_t  agc_gain;     // 0..30, manual gain when agc = 0
    uint8_t  awb;          // 1 = auto white balance
    uint8_t  vflip;
    uint8_t  hmirror;
};

namespace xcam {
    void   defaults(CamSettings& s);
    bool   load(CamSettings& s);        // false if nothing saved, s then holds defaults
    bool   save(const CamSettings& s);
    bool   begin(const CamSettings& s); // init the sensor, then apply s
    void   apply(const CamSettings& s);
    // Set one field by name, as used by the /control endpoint. False if unknown.
    bool   set(CamSettings& s, const char* var, int val);
    String to_json(const CamSettings& s);
}
