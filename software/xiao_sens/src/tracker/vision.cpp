#include "vision.h"
#include <esp_camera.h>
#include <img_converters.h>
#include "config.h"
#include "xiao_cam.h"
#ifndef TRACKER_STUB
#include "ei_model.h"
#endif

namespace vision {

static portMUX_TYPE      det_mux = portMUX_INITIALIZER_UNLOCKED;
static Detection         det     = {};
static SemaphoreHandle_t new_frame;
static SemaphoreHandle_t snap_mutex;
static uint8_t*          snap_buf = nullptr;
static size_t            snap_len = 0, snap_cap = 0;
static uint8_t*          rgb      = nullptr;   // decoded frame, BGR888 (see get_data)
static size_t            rgb_cap  = 0;
static char              info[96];
static CamSettings       cam;   // live sensor settings; changed from the dashboard (web task)

String cam_json() { return xcam::to_json(cam); }

bool cam_set(const char* var, int v) {
    if (!xcam::set(cam, var, v)) return false;
    xcam::apply(cam);
    Serial.printf("camera: %s = %d\n", var, v);
    return true;
}

bool cam_save() { return xcam::save(cam); }

#ifdef TRACKER_STUB
static float    fake_u, fake_v;
static uint32_t fake_until = 0;
void fake(float u, float v, uint32_t ttl_ms) {
    fake_u = u;
    fake_v = v;
    fake_until = millis() + ttl_ms;
}
#else
// Image sharpness: mean absolute Laplacian of the luminance of the model input
// (BGR888, w x h). Motion blur or bad focus makes it drop; compare with the
// value seen when everything is still.
static float sharpness(const uint8_t* bgr, int w, int h) {
    auto Y = [&](int x, int y) {
        const uint8_t* p = bgr + (y * w + x) * 3;
        return (int)p[0] + 2 * p[1] + p[2];  // B + 2G + R, 0..1020
    };
    uint32_t sum = 0;
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++)
            sum += abs(4 * Y(x, y) - Y(x - 1, y) - Y(x + 1, y) - Y(x, y - 1) - Y(x, y + 1));
    return sum / (4.0f * (w - 2) * (h - 2));
}

// Edge Impulse pulls pixels through this. Each value is a packed 0xRRGGBB.
// fmt2rgb888 actually writes B,G,R (espressif/esp32-camera#379), hence the swap.
static int get_data(size_t offset, size_t length, float* out) {
    const uint8_t* p = rgb + offset * 3;
    for (size_t i = 0; i < length; i++, p += 3) out[i] = (p[2] << 16) + (p[1] << 8) + p[0];
    return 0;
}
#endif

bool begin() {
    new_frame  = xSemaphoreCreateBinary();
    snap_mutex = xSemaphoreCreateMutex();
    bool saved = xcam::load(cam);
    Serial.printf("camera settings (%s): %s\n", saved ? "saved (capture firmware or dashboard)" : "DEFAULTS - none saved",
                  xcam::to_json(cam).c_str());
    if (!xcam::begin(cam)) return false;
#ifdef TRACKER_STUB
    snprintf(info, sizeof(info), "STUB detector (no model) - use /fake");
#else
    snprintf(info, sizeof(info), "%s v%d, %dx%d, label '%s'", EI_CLASSIFIER_PROJECT_NAME,
             EI_CLASSIFIER_PROJECT_DEPLOY_VERSION, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT,
             TARGET_LABEL);
#endif
    Serial.printf("model: %s\n", info);
    return true;
}

static void keep_snapshot(const camera_fb_t* fb) {
    if (xSemaphoreTake(snap_mutex, 0) != pdTRUE) return;  // dashboard is reading, skip one
    if (fb->len > snap_cap) {
        free(snap_buf);
        snap_cap = fb->len + 4096;
        snap_buf = (uint8_t*)ps_malloc(snap_cap);
        if (!snap_buf) snap_cap = 0;
    }
    if (snap_buf) {
        memcpy(snap_buf, fb->buf, fb->len);
        snap_len = fb->len;
    }
    xSemaphoreGive(snap_mutex);
}

void step(float min_conf) {
    Detection d = {};
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        delay(10);
        return;
    }
    d.t_cap = millis();
    keep_snapshot(fb);

#ifdef TRACKER_STUB
    (void)min_conf;
    esp_camera_fb_return(fb);
    delay(60);  // pace like a real model
    if ((int32_t)(fake_until - d.t_cap) > 0) {
        d.found = true;
        d.u     = fake_u;
        d.v     = fake_v;
        d.conf  = 1.0f;
        d.cells = 1;
    }
#else
    size_t need = (size_t)fb->width * fb->height * 3;
    if (need > rgb_cap) {
        free(rgb);
        rgb     = (uint8_t*)ps_malloc(need);
        rgb_cap = rgb ? need : 0;
    }
    uint16_t w = fb->width, h = fb->height;
    bool ok = rgb && fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
    esp_camera_fb_return(fb);
    if (!ok) {
        log_e("JPEG decode failed");
        return;
    }
    // Centre-crop to the model's aspect, then resize in place. Matches Studio's
    // "fit shortest axis" - set the impulse's resize mode to that.
    ei::image::processing::crop_and_interpolate_rgb888(rgb, w, h, rgb, EI_CLASSIFIER_INPUT_WIDTH,
                                                        EI_CLASSIFIER_INPUT_HEIGHT);
    d.sharp = sharpness(rgb, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    signal_t sig;
    sig.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    sig.get_data     = &get_data;
    ei_impulse_result_t res = {};
    EI_IMPULSE_ERROR err = run_classifier(&sig, &res, false);
    if (err != EI_IMPULSE_OK) {
        log_e("run_classifier: %d", err);
        return;
    }
    // Confidence-weighted centroid of all cells of the target class.
    float sw = 0, su = 0, sv = 0;
    for (uint32_t i = 0; i < res.bounding_boxes_count; i++) {
        const auto& bb = res.bounding_boxes[i];
        if (bb.value < min_conf || strcmp(bb.label, TARGET_LABEL) != 0) continue;
        su += bb.value * (bb.x + bb.width * 0.5f);
        sv += bb.value * (bb.y + bb.height * 0.5f);
        sw += bb.value;
        if (bb.value > d.conf) d.conf = bb.value;
        d.cells++;
    }
    if (sw > 0) {
        d.found = true;
        d.u     = 2.0f * (su / sw) / EI_CLASSIFIER_INPUT_WIDTH - 1.0f;
        d.v     = 2.0f * (sv / sw) / EI_CLASSIFIER_INPUT_HEIGHT - 1.0f;
    }
#endif
    d.infer_ms = millis() - d.t_cap;

    portENTER_CRITICAL(&det_mux);
    d.seq = det.seq + 1;
    det   = d;
    portEXIT_CRITICAL(&det_mux);
    xSemaphoreGive(new_frame);
}

Detection latest() {
    portENTER_CRITICAL(&det_mux);
    Detection d = det;
    portEXIT_CRITICAL(&det_mux);
    return d;
}

bool wait_new(uint32_t timeout_ms) {
    return xSemaphoreTake(new_frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

uint8_t* snapshot(size_t* len) {
    *len = 0;
    if (xSemaphoreTake(snap_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return nullptr;
    uint8_t* out = snap_len ? (uint8_t*)ps_malloc(snap_len) : nullptr;
    if (out) {
        memcpy(out, snap_buf, snap_len);
        *len = snap_len;
    }
    xSemaphoreGive(snap_mutex);
    return out;
}

const char* model_info() {
    return info;
}

}  // namespace vision
