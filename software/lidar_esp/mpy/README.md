# Pole tracker — MicroPython / ESP32-CAM

Serves its own web UI. No host script needed once it runs.

## 1. Firmware

Stock MicroPython has no camera driver. Flash a build from
<https://github.com/cnadler86/micropython-camera-API/releases> — the
**AI_THINKER** variant (needs PSRAM, which the ESP32-CAM has).

```sh
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 <firmware>.bin
```

## 2. Copy the app

```sh
mpremote connect /dev/ttyUSB0 fs cp detector.py homography.py webapp.py index.html main.py :
```

## 3. Use it

Board boots an access point `poletracker` / `pole1234`, then
<http://192.168.4.1/>. To join your own network instead, set `STA_SSID` /
`STA_PASS` at the top of `main.py`.

Order of operations:

1. **Live view** — check the image is neither black nor saturated. Adjust
   `exposure` / `gain`, press Apply.
2. **Relearn background** with the scene *empty*. If the pole is present during
   learning it becomes part of the decor and stays invisible.
3. Put the pole in. The score bar should cross the threshold; the overlay marks
   its edges (red) and base (blue).
4. **Calibration** — place the pole at 4+ known floor positions, enter each
   coordinate, Capture, then Solve. Residuals are reported in mm.
5. Floor position panel then shows live XY.

## Files

| file | role |
|---|---|
| `detector.py` | viper hot loops: profiles, background subtraction, matched filter, base row |
| `homography.py` | 8x8 least-squares solve, no numpy; persists to `calib.json` |
| `webapp.py` | asyncio HTTP server, JSON API |
| `index.html` | the UI |
| `main.py` | camera setup, wifi, detection loop |
| `test_detector.py`, `test_webapp.py` | run on a PC, not the board |

## Tests (on a PC, from this directory)

```sh
python3 test_detector.py   # detector vs synthetic QVGA scenes
python3 test_webapp.py     # HTTP API + calibration round trip
```

## Si le firmware n'a pas l'emetteur natif

`AttributeError: 'module' object has no attribute 'viper'` au premier import de
`detector.py` veut dire que le binaire a ete compile sans `MICROPY_EMIT_NATIVE`.
`detector.py` retombe alors tout seul sur `micropython.native`, puis sur le
bytecode, et passe `row_step` a 4 (une ligne d'image sur 4) pour compenser.
Pour savoir ce qui tourne reellement :

    import micropython
    print([n for n in ('viper','native','asm_xtensawin') if hasattr(micropython,n)])

La page web affiche la meme information dans son en-tete (`viper` / `native` /
`bytecode`). En bytecode, compter ~1 image/s : utilisable pour la calibration et
le suivi lent, mais un binaire avec viper est 20 a 50x plus rapide. Le reglage
`row step` de la page permet d'arbitrer vitesse contre sensibilite -- il faut
refaire le fond apres l'avoir change, les profils sont des sommes sur les lignes
echantillonnees et ne valent que pour le pas qui les a produits.

## Brochage camera

`TypeError: 'data_pins' argument required` veut dire que le binaire est le build
generique, pas un build `MICROPY_CAMERA_MODEL_AI_THINKER` : `Camera()` reclame
alors tout le brochage. Il est code en dur dans `main.py` (dict `AI_THINKER`) et
passe systematiquement, avec repli sans broches si un build specifique a une
carte les refuse. Brochage AI-Thinker ESP32-CAM :

    D0..D7   5, 18, 19, 21, 36, 39, 34, 35     (Y2..Y9)
    PCLK 22   VSYNC 25   HREF 23
    SDA  26   SCL   27   (bus de controle du capteur, pas le I2C systeme)
    XCLK  0 a 20 MHz     PWDN 32     RESET -1 (non cable)

GPIO 4 est la LED flash, GPIO 0 est pris par XCLK : ne rien brancher dessus.

Sur ESP32-WROVER-DEV / Freenove ESP32-Wrover CAM (`BOARD = "WROVER_KIT"`, le
defaut) le brochage est different : XCLK sur 21, pas de ligne power-down, et
GPIO 4 est D0 -- une broche de donnees camera, pas la LED. GPIO 0 est libre.

Les noms des proprietes du capteur changent d'un build a l'autre. `lock_sensor()`
les essaie une par une et la page web affiche `sensor locks` : `all locked`, ou
la liste de celles qui n'ont pas pris. Si l'exposition auto ne se verrouille pas,
le fond appris ne vaut rien -- pour voir les noms reellement exposes :

    from camera import Camera
    print([n for n in dir(Camera) if not n.startswith('_')])

## Camera() echoue : camprobe.py

`OSError: [Errno 95] EOPNOTSUPP` (ESP_ERR_NOT_SUPPORTED) a exactement deux
origines dans le driver esp32-camera : une demande de JPEG sur un capteur qui
ne le fait pas, ou bien *Detected camera not supported* -- aucun capteur n'a
repondu sur le bus SCCB a aucune adresse connue. On ne demande jamais de JPEG,
donc c'est toujours la seconde : le capteur n'a pas ete trouve. Le format de
pixel n'a rien a voir.

    mpremote connect /dev/ttyUSB0 run camprobe.py

balaie les brochages de `boards.py` x 20/10/8 MHz et dit lequel repond. Reporter
le resultat dans `BOARD` et `XCLK_HZ` de `main.py`. Si rien ne repond, par ordre
de probabilite : alimentation (l'OV2640 veut du 5 V avec de la marge en courant,
le 3,3 V d'un adaptateur USB-serie ne suffit pas), nappe mal enfoncee, module
camera mort, carte dont le brochage n'est pas dans `boards.py`.
