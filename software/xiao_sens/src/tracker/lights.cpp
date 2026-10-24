#include "lights.h"
#include <Arduino.h>
#include <lwip/sockets.h>
#include "config.h"

namespace lights {

static sockaddr_in dest() {
    sockaddr_in to = {};
    to.sin_family      = AF_INET;
    to.sin_port        = htons(LIGHTS_PORT);
    to.sin_addr.s_addr = inet_addr(LIGHTS_HOST);
    return to;
}

void send(const char* cmd) {
    static int s = -1;
    if (s < 0) s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return;
    sockaddr_in to = dest();
    sendto(s, cmd, strlen(cmd), 0, (sockaddr*)&to, sizeof(to));
}

int query(const char* cmd, char* reply, size_t n) {
    reply[0] = 0;
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return 0;
    timeval tv = {0, LIGHTS_REPLY_MS * 1000};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in to = dest();
    sendto(s, cmd, strlen(cmd), 0, (sockaddr*)&to, sizeof(to));
    int got = recv(s, reply, n - 1, 0);
    close(s);
    if (got < 0) got = 0;
    reply[got] = 0;
    return got;
}

int receive(char* msg, size_t n) {
    static int s = -1;
    msg[0] = 0;
    if (s < 0) {
        s = socket(AF_INET, SOCK_DGRAM, 0);
        if (s < 0) return 0;
        int one = 1;
        setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in me = {};
        me.sin_family      = AF_INET;
        me.sin_port        = htons(LIGHTS_PORT);
        me.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(s, (sockaddr*)&me, sizeof(me)) < 0) {
            close(s);
            s = -1;
            return 0;
        }
        fcntl(s, F_SETFL, O_NONBLOCK);
    }
    int got = recv(s, msg, n - 1, 0);
    if (got < 0) got = 0;
    msg[got] = 0;
    return got;
}

}  // namespace lights
