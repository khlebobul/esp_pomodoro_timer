# Pomodoro timer

Waveshare ESP32-S3-Touch-AMOLED-1.8 firmware controlled by orientation.

- Put it on one of four side edges to select and start a 5, 10, 30, or 60-minute timer.
- Set it nearly flat to pause; small tilts keep current timer running.
- Turn it to another edge to restart with that edge's duration.
- The UI rotates with each selected edge. Touch **Reset** to restart current timer.

```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash
```
