# HLK-LD2450 24GHz radar -> ESP32-C3 (MicroPython)
# Variation of main.py: NO capture/replay. When more than one body is detected,
# the head streams random moves to FluidNC (floating-in-space look).
# UART1: TX=GPIO3 (-> sensor RX), RX=GPIO4 (<- sensor TX). Sensor: 256000 8N1, 10 fps.
import struct
import time
import math
import random
import network
import socket
from machine import UART, Pin

uart = UART(1, baudrate=256000, bits=8, parity=None, stop=1, tx=3, rx=4, rxbuf=512)
boot = Pin(9, Pin.IN, Pin.PULL_UP)   # ESP32-C3 BOOT button, active low (pressed = 0)

DEBUG = True

# --- FluidNC over Telnet (see stream.py) ---
WIFI_SSID = "rail"
WIFI_PASS = "12345678"
HOST, PORT = "192.168.0.1", 23
HOMING = True                 # False: skip $H and just $X unlock (no limit switches)
HOMING_TIMEOUT = 30000        # ms to wait for homing before giving up (avoid hang)
FEED = 7000                   # brush speed, mm/min
XMAX, YMAX, ZMAX = 1400, 260, 2000   # machine work area (mm)
# --- brush stroke: sweep X end-to-end, ramp Y, breathe Z, dwell, reverse ---
X_START, X_END = 0, XMAX      # brush sweeps X between these
Y_MAIN, Y_END = 0, YMAX       # Y random start/end picked within [Y_MAIN, Y_END]
Z_MEAN_VALUE = 680             # Z breathes around this value (floating-in-space)
Z_FLOAT = 10                  # +/- mm breath amplitude on Z
BREATH_MS = 4000              # breath period in ms (one full Z up-down cycle)
SUBDIV = 33                   # segments per stroke (Z/Y resolution along X)
T_SLEEP_MIN = 6.0             # min dwell at stroke end (s), randomly varied
T_SLEEP_MAX = 13.0             # max dwell at stroke end (s)
T_DETECTION = 1500            # ms the trigger must hold before moving (debounce)
LASER_S = 100                 # laser power (M3 S value) while triggered
sock = None                   # FluidNC socket, opened lazily on first send
cur_y = Y_MAIN                # last Y reached (next stroke starts here)
detect_since = None           # ticks_ms when the trigger first became true
laser_on = False              # current laser state (avoid resending M3)

DATA_HEAD = b"\xaa\xff\x03\x00"   # data frame header
DATA_TAIL = b"\x55\xcc"          # data frame tail
CFG_HEAD = b"\xfd\xfc\xfb\xfa"   # config frame header
CFG_TAIL = b"\x04\x03\x02\x01"   # config frame tail


def debug(msg):
    if DEBUG:
        print(msg)


def cfg_cmd(word, value=b"", wait=0.1):
    # word/value little-endian; intra-frame length = len(word)+len(value)
    payload = word + value
    frame = CFG_HEAD + struct.pack("<H", len(payload)) + payload + CFG_TAIL
    uart.write(frame)
    time.sleep(wait)
    ack = uart.read()
    print("  cmd", word.hex(), "->", ack.hex() if ack else "no-ack")
    return ack


def configure():
    debug("Configuring LD2450...")
    cfg_cmd(b"\xff\x00", b"\x01\x00")   # enable config mode
    cfg_cmd(b"\xa0\x00")               # read firmware version
    cfg_cmd(b"\x90\x00")               # multi-target tracking mode
    cfg_cmd(b"\xfe\x00")               # end config mode (resume data output)
    debug("Configured. Streaming targets...\n")


def decode_signed(raw):
    # 16-bit value: MSB = sign flag (1 = positive). lower 15 bits = magnitude.
    return (raw - 0x8000) if (raw & 0x8000) else -raw


def parse(frame):
    targets = []
    for i in range(3):
        x, y, spd, res = struct.unpack_from("<HHHH", frame, 4 + i * 8)
        targets.append((decode_signed(x), decode_signed(y), decode_signed(spd), res))
    return targets


def present(targets):
    # only bodies that are actually detected (non-zero placement)
    return [(n, x, y, spd) for n, (x, y, spd, res) in enumerate(targets)
            if x or y or spd]


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wait_wifi():
    # block until the ESP32 is associated with the FluidNC AP (retry forever)
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    while not wlan.isconnected():
        print("wifi: connecting to", WIFI_SSID, "...")
        try:
            wlan.connect(WIFI_SSID, WIFI_PASS)
        except OSError as e:
            print("wifi connect error:", e)
        for _ in range(40):                # ~12s per attempt, then re-issue connect
            if wlan.isconnected():
                break
            time.sleep_ms(300)
    print("wifi up", wlan.ifconfig()[0])
    return wlan


def connect_fluidnc():
    # wait for WiFi, then open the Telnet socket; retries forever until connected
    global sock
    wait_wifi()                            # block here until the AP is reachable
    while sock is None:
        try:
            s = socket.socket()
            s.connect(socket.getaddrinfo(HOST, PORT)[0][-1])
            s.setblocking(False)
            time.sleep_ms(300)
            try:
                s.recv(512)                # drain welcome banner
            except OSError:
                pass
            print("FluidNC connected", HOST)
            sock = s
        except OSError as e:
            print("FluidNC connect failed, retrying:", e)
            sock = None
            time.sleep_ms(1000)
            wait_wifi()                    # WiFi may have dropped; re-establish first
    return sock


def send_line(s, line, timeout_ms=None):
    # send one line, block until FluidNC replies ok / error (GRBL protocol)
    s.send(line + "\n")
    buf = ""
    start = time.ticks_ms()
    while True:
        try:
            buf += s.recv(256).decode()
        except OSError:
            if timeout_ms and time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                print("timeout waiting for ok:", line)
                return None
            time.sleep_ms(10)
            continue
        while "\n" in buf:
            resp, buf = buf.split("\n", 1)
            resp = resp.strip()
            if resp == "ok":
                return True
            if resp.startswith("error"):
                print("ERR", line, "->", resp)
                return False


def fluid_reset():
    # GRBL/FluidNC soft-reset (Ctrl-X realtime byte); clears alarm/abort, drains banner
    sock.send(b"\x18")
    time.sleep_ms(500)
    try:
        sock.recv(512)
    except OSError:
        pass


def home():
    # connect and run the homing cycle at startup (GRBL/FluidNC)
    global sock
    if sock is None:
        connect_fluidnc()
    if sock is None:
        print("home skipped: FluidNC offline")
        return
    fluid_reset()            # soft-reset before the startup commands
    if HOMING:
        send_line(sock, "$H", timeout_ms=HOMING_TIMEOUT)   # homing cycle (bounded)
    else:
        send_line(sock, "$X")                              # unlock, no homing
    send_line(sock, "G21")   # mm
    send_line(sock, "G90")   # absolute
    print("homed" if HOMING else "unlocked")


def wait_idle(s):
    # poll status (? realtime) until FluidNC reports Idle -> motion finished
    while True:
        try:
            s.send(b"?")
        except OSError:
            return
        time.sleep_ms(100)
        try:
            r = s.recv(256).decode()
        except OSError:
            r = ""
        if "Idle" in r:
            return


def set_laser(on):
    # turn the laser on (M3 S<power>) or off (M3 S0); only sends on a state change
    global laser_on
    if sock is None or on == laser_on:
        return
    send_line(sock, "M3 S%d" % (LASER_S if on else 0))
    laser_on = on
    debug("laser " + ("ON" if on else "OFF"))


def breath_z(at_ms):
    # Z from the breath sine at a given clock time (ms)
    phase = 2 * math.pi * at_ms / BREATH_MS
    return clamp(Z_MEAN_VALUE + Z_FLOAT * math.sin(phase), 0, ZMAX)


def brush_stroke():
    # laser-OFF travel to a random X, then laser-ON stroke to another random X
    global sock, cur_y
    if sock is None:
        connect_fluidnc()
    if sock is None:
        return                              # offline; skip
    x0 = random.uniform(X_START, X_END)     # random entry point on X
    x1 = random.uniform(X_START, X_END)     # random exit point within the X range
    y0 = cur_y                              # start where the last stroke ended
    y1 = random.uniform(Y_MAIN, Y_END)      # new random terminal Y each stroke

    # 1) travel to the random start with the laser OFF (no mark)
    set_laser(False)
    send_line(sock, "G1 X%.1f Y%.1f Z%.1f F%d"
              % (clamp(x0, 0, XMAX), clamp(y0, 0, YMAX), breath_z(time.ticks_ms()), FEED))
    wait_idle(sock)                         # reach the start before firing

    # 2) laser ON, then stroke across to the random end
    set_laser(True)
    move_ms = abs(x1 - x0) / (FEED / 60.0) * 1000 if FEED > 0 else 0
    now = time.ticks_ms()
    debug("brush x %.0f -> %.0f" % (x0, x1))
    for k in range(1, SUBDIV + 1):
        f = k / SUBDIV
        x = clamp(x0 + (x1 - x0) * f, 0, XMAX)   # keep emitted coords inside work area
        y = clamp(y0 + (y1 - y0) * f, 0, YMAX)
        z = breath_z(now + move_ms * f)     # Z at this segment's projected time (already clamped)
        # stream continuously so segments blend -> one smooth brush stroke
        send_line(sock, "G1 X%.1f Y%.1f Z%.1f F%d" % (x, y, z, FEED))
    wait_idle(sock)                         # let the stroke finish at the end

    # 3) laser OFF at the end of the stroke
    set_laser(False)
    t_sleep = random.uniform(T_SLEEP_MIN, T_SLEEP_MAX)   # random dwell, kept regardless
    debug("dwell %.1fs" % t_sleep)
    time.sleep(t_sleep)
    cur_y = y1                              # next stroke continues from here


# ---execution
configure()
home()                            # connect FluidNC and home at startup
last_present = []                 # latest detected bodies
buf = bytearray()
while True:
    chunk = uart.read()
    if chunk:
        buf += chunk
    # drain the buffer, keeping only the MOST RECENT detection (no stale backlog)
    while True:
        i = buf.find(DATA_HEAD)
        if i < 0:
            if len(buf) > 3:          # keep tail bytes (header may be split)
                buf = buf[-3:]
            break
        if len(buf) - i < 30:          # incomplete frame, wait for more bytes
            buf = buf[i:]
            break
        frame = bytes(buf[i:i + 30])
        buf = buf[i + 30:]
        if frame[28:30] != DATA_TAIL:
            continue                   # bad tail, resync on next header
        last_present = present(parse(frame))
    debug(last_present)
    # trigger must hold for T_DETECTION before moving -> ignores fake/brief readings
    cond = len(last_present) > 1 or boot.value() == 0
    if cond:
        if detect_since is None:
            detect_since = time.ticks_ms()
        if time.ticks_diff(time.ticks_ms(), detect_since) >= T_DETECTION:
            brush_stroke()             # handles its own laser on/off around the stroke
    else:
        set_laser(False)               # laser OFF (M3 S0) when condition is false
        detect_since = None            # lost the trigger -> reset debounce timer
    time.sleep_ms(20)
