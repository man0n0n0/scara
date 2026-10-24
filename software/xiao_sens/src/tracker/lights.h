#pragma once
// Light controller (ESP32-C3, tools/light_c3/main.py): UDP text commands
// "spot 0|1", "light 0|1", "all 0", "state"; it answers with its state.
#include <stddef.h>

namespace lights {
    // Fire and forget, from one task only (keeps one socket open).
    void send(const char* cmd);
    // Send and wait up to LIGHTS_REPLY_MS for the reply ("spot 1 light 0").
    // Returns its length, 0 if none. Any task (own socket per call).
    int  query(const char* cmd, char* reply, size_t n);
    // Non-blocking: one datagram received on LIGHTS_PORT ("human 0"...), or 0.
    // Control task only. Our own broadcast commands come back here too.
    int  receive(char* msg, size_t n);
}
