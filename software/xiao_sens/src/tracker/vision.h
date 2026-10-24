#pragma once
#include <Arduino.h>

// One processed camera frame.
struct Detection {
    uint32_t seq;        // increments every frame
    uint32_t t_cap;      // millis() when the frame was grabbed
    bool     found;
    float    u, v;       // target centre, -1..1 across the model's input square, +u right, +v down
    float    conf;       // best cell confidence
    uint8_t  cells;      // FOMO cells that contributed
    uint16_t infer_ms;   // decode + resize + inference
    float    sharp;      // mean |Laplacian| of the model input: drops when the image is blurred
};

namespace vision {
    bool        begin();
    // Grab, run the model, publish. Blocks for one frame. Runs on core 1.
    void        step(float min_conf);
    // Newest result. Thread-safe.
    Detection   latest();
    // Wait up to timeout_ms for a frame newer than the last call. Single consumer.
    bool        wait_new(uint32_t timeout_ms);
    // Copy of the last JPEG for the dashboard. Caller frees with free().
    uint8_t*    snapshot(size_t* len);
    const char* model_info();
    // Camera sensor settings (shared with the capture firmware, NVS "xcam"):
    String      cam_json();                      // current settings
    bool        cam_set(const char* var, int v); // change one and apply it now (not saved)
    bool        cam_save();                      // save the current settings to NVS
#ifdef TRACKER_STUB
    // Bench test without a model: report a target at (u, v) for ttl_ms.
    void        fake(float u, float v, uint32_t ttl_ms);
#endif
}
