#include "fluidnc.h"
#include "config.h"

void FluidNC::begin(const char* host, uint16_t port) {
    _host = host;
    _port = port;
}

void FluidNC::poll() {
    uint32_t now = millis();
    if (!_client.connected()) {
        if (_was_connected) {
            Serial.println("fluidnc: disconnected");
            _was_connected = false;
            _reconnects++;
            _st.t_ms       = 0;
            _outstanding   = 0;
        }
        if (WiFi.status() != WL_CONNECTED || (int32_t)(now - _next_connect) < 0) return;
        _next_connect = now + 2000;
        if (!_client.connect(_host, _port, 1000)) return;
        _client.setNoDelay(true);
        _was_connected   = true;
        _connected_since = millis();
        _len             = 0;
        Serial.printf("fluidnc: connected to %s:%u\n", _host, _port);
        // A jog left queued by a previous session (e.g. we rebooted) is cancelled.
        _rt(0x85);
        return;
    }

    while (_client.available()) {
        int c = _client.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            _buf[_len] = 0;
            if (_len) _line(_buf);
            _len = 0;
        } else if (_len < sizeof(_buf) - 1) {
            _buf[_len++] = (char)c;
        }
    }

    // Socket still "connected" but FluidNC silent (rebooted, WiFi dropped without
    // a FIN): drop it so the reconnect above runs.
    uint32_t heard = _st.t_ms ? _st.t_ms : _connected_since;
    if ((int32_t)(millis() - heard) > FLUIDNC_STALE_RECONNECT_MS) {
        Serial.println("fluidnc: no status - dropping the link to reconnect");
        _client.stop();
        return;
    }

    if ((int32_t)(now - _next_status) >= 0) {
        _next_status = now + STATUS_PERIOD_MS;
        _rt('?');
    }
    // A lost reply must not wedge the jog pacing forever.
    if (_outstanding > 0 && now - _last_reply > 2000) _outstanding = 0;
}

void FluidNC::_rt(uint8_t c) {
    if (_client.connected()) _client.write(&c, 1);
}

bool FluidNC::sendLine(const char* line) {
    if (!_client.connected()) return false;
    size_t n = strlen(line);
    if (_client.write((const uint8_t*)line, n) != n || _client.write((const uint8_t*)"\n", 1) != 1) return false;
    if (_outstanding == 0) _last_reply = millis();
    _outstanding++;
    return true;
}

void FluidNC::jogCancel() {
    _rt(0x85);
}

void FluidNC::softReset() {
    _rt(0x18);
    _outstanding = 0;  // FluidNC drops whatever was buffered: no 'ok' will come
}

void FluidNC::_line(char* s) {
    if (s[0] == '<') {
        _parseStatus(s);
        return;
    }
    if (!strcmp(s, "ok")) {
        if (_outstanding > 0) _outstanding--;
        _last_reply = millis();
        return;
    }
    if (!strncmp(s, "error:", 6)) {
        if (_outstanding > 0) _outstanding--;
        _last_reply    = millis();
        _last_error    = atoi(s + 6);
        _last_error_ms = millis();
        Serial.printf("fluidnc: %s\n", s);
        return;
    }
    if (!strncmp(s, "ALARM:", 6)) {
        strlcpy(_last_alarm, s, sizeof(_last_alarm));
        Serial.printf("fluidnc: %s\n", s);
        return;
    }
    if (s[0] == '[') Serial.printf("fluidnc: %s\n", s);  // [MSG:...] etc.
}

// <Jog|MPos:120.000,850.000,-10.000|FS:600,0|WCO:0,0,0>
// Reports either MPos or WPos; with WPos, MPos = WPos + WCO (WCO is sent now and then).
void FluidNC::_parseStatus(char* s) {
    char* end = strchr(s, '>');
    if (end) *end = 0;
    s++;
    bool  have_pos = false, is_wpos = false;
    float pos[3]   = {0, 0, 0};
    char* save     = nullptr;
    for (char* f = strtok_r(s, "|", &save); f; f = strtok_r(nullptr, "|", &save)) {
        if (f == s) {
            // state, possibly with a sub-state ("Hold:0", "Door:1")
            size_t n = strcspn(f, ":");
            if (n >= sizeof(_st.state)) n = sizeof(_st.state) - 1;
            memcpy(_st.state, f, n);
            _st.state[n] = 0;
            _st.sub      = f[n] == ':' ? atoi(f + n + 1) : -1;
        } else if (!strncmp(f, "MPos:", 5) || !strncmp(f, "WPos:", 5)) {
            is_wpos  = f[0] == 'W';
            have_pos = sscanf(f + 5, "%f,%f,%f", &pos[0], &pos[1], &pos[2]) == 3;
        } else if (!strncmp(f, "WCO:", 4)) {
            sscanf(f + 4, "%f,%f,%f", &_wco[0], &_wco[1], &_wco[2]);
        }
    }
    if (!have_pos) return;
    for (int i = 0; i < 3; i++) _st.mpos[i] = is_wpos ? pos[i] + _wco[i] : pos[i];
    _st.t_ms = millis();
}
