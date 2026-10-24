#include "xiao_cam.h"
#include <Preferences.h>

// XIAO ESP32S3 Sense camera connector (OV2640 / OV3660)
#define CAM_PIN_XCLK  10
#define CAM_PIN_SIOD  40
#define CAM_PIN_SIOC  39
#define CAM_PIN_D7    48
#define CAM_PIN_D6    11
#define CAM_PIN_D5    12
#define CAM_PIN_D4    14
#define CAM_PIN_D3    16
#define CAM_PIN_D2    18
#define CAM_PIN_D1    17
#define CAM_PIN_D0    15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF  47
#define CAM_PIN_PCLK  13

static const uint8_t SETTINGS_VERSION = 1;

namespace xcam {

void defaults(CamSettings& s) {
    s = {};
    s.version   = SETTINGS_VERSION;
    s.framesize = FRAMESIZE_QVGA;
    s.quality   = 12;
    s.aec       = 1;
    s.aec_value = 300;
    s.agc       = 1;
    s.awb       = 1;
}

bool load(CamSettings& s) {
    defaults(s);
    Preferences p;
    if (!p.begin("xcam", true)) return false;
    CamSettings tmp;
    bool ok = p.getBytesLength("s") == sizeof(tmp) && p.getBytes("s", &tmp, sizeof(tmp)) == sizeof(tmp) &&
              tmp.version == SETTINGS_VERSION;
    p.end();
    if (ok) s = tmp;
    return ok;
}

bool save(const CamSettings& s) {
    Preferences p;
    if (!p.begin("xcam", false)) return false;
    bool ok = p.putBytes("s", &s, sizeof(s)) == sizeof(s);
    p.end();
    return ok;
}

bool begin(const CamSettings& s) {
    camera_config_t c = {};
    c.pin_pwdn     = -1;
    c.pin_reset    = -1;
    c.pin_xclk     = CAM_PIN_XCLK;
    c.pin_sccb_sda = CAM_PIN_SIOD;
    c.pin_sccb_scl = CAM_PIN_SIOC;
    c.pin_d7       = CAM_PIN_D7;
    c.pin_d6       = CAM_PIN_D6;
    c.pin_d5       = CAM_PIN_D5;
    c.pin_d4       = CAM_PIN_D4;
    c.pin_d3       = CAM_PIN_D3;
    c.pin_d2       = CAM_PIN_D2;
    c.pin_d1       = CAM_PIN_D1;
    c.pin_d0       = CAM_PIN_D0;
    c.pin_vsync    = CAM_PIN_VSYNC;
    c.pin_href     = CAM_PIN_HREF;
    c.pin_pclk     = CAM_PIN_PCLK;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer   = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    // JPEG buffers are sized from the init frame size: init at the largest size
    // we allow, then switch to the configured one.
    c.frame_size   = FRAMESIZE_VGA;
    c.jpeg_quality = s.quality;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    // LATEST drops stale frames: what the tracker sees is as fresh as possible.
    c.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        log_e("camera init failed: 0x%x", err);
        return false;
    }
    apply(s);
    return true;
}

void apply(const CamSettings& s) {
    sensor_t* t = esp_camera_sensor_get();
    if (!t) return;
    framesize_t fs = (framesize_t)s.framesize;
    if (fs > FRAMESIZE_VGA) fs = FRAMESIZE_VGA;
    t->set_framesize(t, fs);
    t->set_quality(t, s.quality);
    t->set_brightness(t, s.brightness);
    t->set_contrast(t, s.contrast);
    t->set_saturation(t, s.saturation);
    t->set_exposure_ctrl(t, s.aec);
    t->set_aec_value(t, s.aec_value);
    t->set_ae_level(t, s.ae_level);
    t->set_gain_ctrl(t, s.agc);
    t->set_agc_gain(t, s.agc_gain);
    t->set_whitebal(t, s.awb);
    t->set_vflip(t, s.vflip);
    t->set_hmirror(t, s.hmirror);
}

bool set(CamSettings& s, const char* var, int val) {
    if      (!strcmp(var, "framesize"))  s.framesize  = constrain(val, 0, (int)FRAMESIZE_VGA);
    else if (!strcmp(var, "quality"))    s.quality    = constrain(val, 4, 63);
    else if (!strcmp(var, "brightness")) s.brightness = constrain(val, -2, 2);
    else if (!strcmp(var, "contrast"))   s.contrast   = constrain(val, -2, 2);
    else if (!strcmp(var, "saturation")) s.saturation = constrain(val, -2, 2);
    else if (!strcmp(var, "ae_level"))   s.ae_level   = constrain(val, -2, 2);
    else if (!strcmp(var, "aec"))        s.aec        = val ? 1 : 0;
    else if (!strcmp(var, "aec_value"))  s.aec_value  = constrain(val, 0, 1200);
    else if (!strcmp(var, "agc"))        s.agc        = val ? 1 : 0;
    else if (!strcmp(var, "agc_gain"))   s.agc_gain   = constrain(val, 0, 30);
    else if (!strcmp(var, "awb"))        s.awb        = val ? 1 : 0;
    else if (!strcmp(var, "vflip"))      s.vflip      = val ? 1 : 0;
    else if (!strcmp(var, "hmirror"))    s.hmirror    = val ? 1 : 0;
    else return false;
    return true;
}

String to_json(const CamSettings& s) {
    char b[320];
    snprintf(b, sizeof(b),
             "{\"framesize\":%d,\"quality\":%d,\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
             "\"ae_level\":%d,\"aec\":%d,\"aec_value\":%u,\"agc\":%d,\"agc_gain\":%u,\"awb\":%d,"
             "\"vflip\":%d,\"hmirror\":%d}",
             s.framesize, s.quality, s.brightness, s.contrast, s.saturation, s.ae_level, s.aec, s.aec_value,
             s.agc, s.agc_gain, s.awb, s.vflip, s.hmirror);
    return String(b);
}

}  // namespace xcam
