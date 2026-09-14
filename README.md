# Pomodoro timer

Firmware for the Waveshare ESP32-S3-Touch-AMOLED-1.8 with an OLED display and a QMI8658 accelerometer. The UI is a retro pixel look (LVGL `unscii_16`) inspired by the [app-pixels pomodoro](https://github.com/app-pixels/pomodoro) — one focus phase, no work/break cycles.

## Demo

// TODO

## How it works

- Stand the device on **one of the four side edges** to select a duration and start the timer immediately.
  - **Top** edge → 30 minutes
  - **Right** edge → 10 minutes
  - **Bottom** edge → 5 minutes
  - **Left** edge → 60 minutes
- Lay it flat **screen-up** while the timer is running to pause it (pill shows `PAUSED`) and a **RESET** button appears; put it back on the same edge to resume, or tap **RESET** to cancel the timer back to idle.
- Each edge flips the screen so the UI stays readable.
- When the timer expires it plays a soft two-note chime and shows `TIME IS UP`. Laying it flat after that returns to idle.
- When the device is lying flat the timer shows the idle screen: a `PLACE ON A SIDE` hint plus grey labels on each side with the matching minutes, so you always know which edge gives which time.

## Setting the times

Durations (in minutes) live in a single set of constants at the top of `main/main.c`:

```c
#define MIN_TOP    30
#define MIN_RIGHT  10
#define MIN_BOTTOM  5
#define MIN_LEFT   60
```

Both the side labels on the idle screen and the timer itself read from these constants, so you only need to change one number per side.

## Flash

Find the serial port first. `XXXX` is not a path — the number changes each time you plug the board in:

```sh
ls /dev/cu.usbmodem*
```

Then:

```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash
```

Use the name `ls` printed, not `usbmodemXXXX`. If nothing appears, reconnect the data cable and try BOOT + RESET.

## Hardware

- Waveshare 1.8" AMOLED Touch (368x448)
- QMI8658 6-axis IMU on the internal I2C bus, used for edge/orientation detection
- ES8311 codec + speaker for the completion beep

## Enclosure

The enclosure model ([build123d_models, pomodoro_timer_enclosure](https://github.com/khlebobul/build123d_models/tree/a051084d6fe956c4d0c5485d98b21841a44b6e1f/pomodoro_timer_enclosure)) is attached as a **git submodule** at `enclosure/`, pinned to commit `a051084d` and sparse-checked out so only `pomodoro_timer_enclosure/` is materialized:

- `pomodoro_timer_enclosure.py` — the parametric build123d script
- `buttons/` and `no_buttons/` — with/without side button cutouts, each split into `labeled/` (debossed edge labels for the idle screen) and `unlabeled/`
- `body` and `lid` parts in STL/STEP; `buttons.png` / `no_buttons.png` previews

After cloning this repo, fetch it with:

```sh
git submodule update --init --recursive
git -C enclosure sparse-checkout init --cone
git -C enclosure sparse-checkout set pomodoro_timer_enclosure
```