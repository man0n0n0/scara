"""Known ESP32-classic camera pin maps, for camprobe.py and main.py.

Transcribed from arduino-esp32 CameraWebServer/camera_pins.h; data_pins is
D0..D7 (Y2..Y9 on the schematics).  ESP32-S2/S3 boards are left out: the
esp32-camera component does not build for the chip in an ESP32-CAM anyway.
"""

BOARDS = {
    "AI_THINKER": {"data_pins": [5, 18, 19, 21, 36, 39, 34, 35],
        "pclk_pin": 22, "vsync_pin": 25, "href_pin": 23,
        "sda_pin": 26, "scl_pin": 27, "xclk_pin": 0,
        "powerdown_pin": 32, "reset_pin": -1},
    # ESP32-WROVER-DEV with camera, Freenove ESP32-Wrover CAM, ESP-WROVER-KIT
    "WROVER_KIT": {"data_pins": [4, 5, 18, 19, 36, 39, 34, 35],
        "pclk_pin": 22, "vsync_pin": 25, "href_pin": 23,
        "sda_pin": 26, "scl_pin": 27, "xclk_pin": 21,
        "powerdown_pin": -1, "reset_pin": -1},
    "ESP_EYE": {"data_pins": [34, 13, 14, 35, 39, 38, 37, 36],
        "pclk_pin": 25, "vsync_pin": 5, "href_pin": 27,
        "sda_pin": 18, "scl_pin": 23, "xclk_pin": 4,
        "powerdown_pin": -1, "reset_pin": -1},
    "ESP32_CAM_BOARD": {"data_pins": [34, 13, 14, 35, 39, 21, 19, 36],
        "pclk_pin": 25, "vsync_pin": 5, "href_pin": 27,
        "sda_pin": 18, "scl_pin": 23, "xclk_pin": 4,
        "powerdown_pin": 32, "reset_pin": 33},
    "M5STACK_PSRAM": {"data_pins": [32, 35, 34, 5, 39, 18, 36, 19],
        "pclk_pin": 21, "vsync_pin": 22, "href_pin": 26,
        "sda_pin": 25, "scl_pin": 23, "xclk_pin": 27,
        "powerdown_pin": -1, "reset_pin": 15},
    "M5STACK_WIDE": {"data_pins": [32, 35, 34, 5, 39, 18, 36, 19],
        "pclk_pin": 21, "vsync_pin": 25, "href_pin": 26,
        "sda_pin": 22, "scl_pin": 23, "xclk_pin": 27,
        "powerdown_pin": -1, "reset_pin": 15},
    "M5STACK_ESP32CAM": {"data_pins": [17, 35, 34, 5, 39, 18, 36, 19],
        "pclk_pin": 21, "vsync_pin": 22, "href_pin": 26,
        "sda_pin": 25, "scl_pin": 23, "xclk_pin": 27,
        "powerdown_pin": -1, "reset_pin": 15},
    "TTGO_T_JOURNAL": {"data_pins": [17, 35, 34, 5, 39, 18, 36, 19],
        "pclk_pin": 21, "vsync_pin": 22, "href_pin": 26,
        "sda_pin": 25, "scl_pin": 23, "xclk_pin": 27,
        "powerdown_pin": 0, "reset_pin": 15},
}
