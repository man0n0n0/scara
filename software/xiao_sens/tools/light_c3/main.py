# ESP32-C3 light controller (MicroPython)
# GPIO3 -> spot MOSFET, GPIO4 -> general light MOSFET (XY-MOS, PWM capable)
# UDP port 4210, commands: "spot 1", "spot 0", "light 1", "light 0", "all 0", "state"
import network, socket, time, machine
from machine import Pin, PWM

SSID, PASSWORD, PORT = "ok", "123456789", 4210
RAMP_MS = 500  # soft start: the lamp's inrush current can brown out the ESP

# 1 = power-on or brown-out, 2 = hard reset/panic, 3 = watchdog, 4 = deep sleep wake, 5 = soft reboot
print("reset cause:", machine.reset_cause())

pwm = {"spot": PWM(Pin(3), freq=1000, duty_u16=0), "light": PWM(Pin(4), freq=1000, duty_u16=0)}
on = {"spot": 0, "light": 0}

def set_light(name, v):
    if v and not on[name]:
        for i in range(1, 21):
            pwm[name].duty_u16(65535 * i // 20)
            time.sleep_ms(RAMP_MS // 20)
    pwm[name].duty_u16(65535 if v else 0)
    on[name] = v

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
while not wlan.isconnected():
    time.sleep(0.2)
print("IP:", wlan.ifconfig()[0], "port", PORT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

def state():
    return "spot %d light %d" % (on["spot"], on["light"])

while True:
    data, addr = sock.recvfrom(64)
    parts = data.decode().strip().lower().split()
    if len(parts) == 2 and parts[1] in ("0", "1"):
        names = on if parts[0] == "all" else [parts[0]] if parts[0] in on else []
        for n in list(names):
            set_light(n, int(parts[1]))
    sock.sendto(state().encode(), addr)
