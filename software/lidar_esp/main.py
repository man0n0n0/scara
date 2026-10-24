# VL53L5CX 8x8 distance matrix over I2C (SCL=GPIO4, SDA=GPIO3)
#
# Requires the mp-extras driver on the board at /lib/vl53l5cx/ :
#   __init__.py  _config_file.py  mp.py  vl_fw_config.bin
#   https://github.com/mp-extras/vl53l5cx
#
# Emits one line per frame on the USB serial console:
#   INFO {...}                 once at boot
#   F <n> <d0> <d1> ... <dN>   n = grid side, -1 = invalid zone

import gc
import sys
import json
import binascii
import machine
from machine import I2C, Pin
from vl53l5cx import RESOLUTION_8X8, DATA_DISTANCE_MM, DATA_TARGET_STATUS, STATUS_VALID
from vl53l5cx.mp import VL53L5CXMP

N = 8
FREQ_HZ = 10

# Set to the GPIO wired to LPn if you have one; None if LPn is just pulled
# high to IOVDD through 47k (then no hardware reset is possible from here).
LPN_PIN = None

i2c = I2C(0, scl=Pin(4), sda=Pin(3), freq=400_000)

lpn = Pin(LPN_PIN, Pin.OUT, value=1) if LPN_PIN is not None else None
tof = VL53L5CXMP(i2c, lpn=lpn)
if lpn:
    tof.reset()
tof.init()                       # uploads the ~84 KB firmware, takes a few seconds
tof.resolution = RESOLUTION_8X8
tof.ranging_freq = FREQ_HZ       # max 15 Hz in 8x8

gc.collect()
print("INFO", json.dumps({
    "board": sys.implementation._machine if hasattr(sys.implementation, "_machine") else sys.platform,
    "micropython": ".".join(str(v) for v in sys.implementation.version),
    "uid": binascii.hexlify(machine.unique_id()).decode(),
    "cpu_hz": machine.freq() if isinstance(machine.freq(), int) else machine.freq()[0],
    "mem_free": gc.mem_free(),
    "i2c": "scl=4 sda=3 @400kHz",
    "sensor": "VL53L5CX",
    "grid": N,
    "hz": FREQ_HZ,
}))

tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})

while True:
    if not tof.check_data_ready():
        continue
    r = tof.get_ranging_data()
    d, s = r.distance_mm, r.target_status
    print("F", N, " ".join(
        str(d[i]) if s[i] == STATUS_VALID else "-1" for i in range(N * N)
    ))
