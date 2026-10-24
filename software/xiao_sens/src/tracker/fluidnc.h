#pragma once
// Minimal FluidNC client over telnet (raw TCP, port 23).
// Owned by the control task: not thread-safe, call everything from one task.

#include <Arduino.h>
#include <WiFi.h>

class FluidNC {
public:
    struct Status {
        char     state[12];   // Idle, Jog, Run, Hold, Alarm, Door, Home, ...
        int      sub;         // sub-state ("Hold:0" = hold complete), -1 = none
        float    mpos[3];     // machine position, X Y Z
        uint32_t t_ms;        // when it arrived; 0 = never
    };

    void begin(const char* host, uint16_t port);
    // Connect/reconnect, read replies, send the periodic '?'. Never blocks long,
    // except connect() which waits up to 1 s when the link is down. A link that
    // stays up but sends no status is dropped and reconnected.
    void poll();

    bool connected() { return _client.connected(); }
    bool sendLine(const char* line);   // appends '\n'; counts towards outstanding()
    void jogCancel();                  // realtime 0x85: decelerate, flush queued jogs
    void feedHold() { _rt('!'); }
    void softReset();                  // realtime 0x18: stop everything, drop queued lines

    const Status& status() const { return _st; }
    bool          stateIs(const char* s) const { return strcmp(_st.state, s) == 0; }
    int           outstanding() const { return _outstanding; }
    int           lastError() const { return _last_error; }
    uint32_t      lastErrorMs() const { return _last_error_ms; }
    const char*   lastAlarm() const { return _last_alarm; }
    void          clearAlarm() { _last_alarm[0] = 0; }  // handled: don't act on it twice
    uint32_t      connectedSince() const { return _connected_since; }
    uint32_t      reconnects() const { return _reconnects; }  // links lost since boot

private:
    void _rt(uint8_t c);
    void _line(char* s);
    void _parseStatus(char* s);

    WiFiClient _client;
    const char* _host = nullptr;
    uint16_t    _port = 23;
    uint32_t    _next_connect = 0, _next_status = 0, _last_reply = 0, _connected_since = 0;
    uint32_t    _reconnects = 0;
    bool        _was_connected = false;
    char        _buf[256];
    size_t      _len = 0;
    Status      _st = {};
    float       _wco[3] = {0, 0, 0};
    int         _outstanding = 0;
    int         _last_error = 0;
    uint32_t    _last_error_ms = 0;
    char        _last_alarm[24] = "";
};
