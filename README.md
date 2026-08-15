# Mini Drone

A project by Nils and Valéry, Kirchenfeldrobotics.

A 3D printed mini drone with a flight controller written from scratch. Instead of
flashing Betaflight onto a ready-made board, everything runs on an ESP32 in C++ on
ESP-IDF: reading the IMU, the PID loop, the DShot signals for the ESC and the telemetry.
The drone hangs in the local Wi-Fi network and is flown from a laptop over UDP. It is a
learning project — the point is to understand every layer of a flight controller, not to
build a better one than what you can buy.

## Flight Controller

`software/flight-controller/` is the ESP-IDF project. The work is split across FreeRTOS
tasks pinned to the two cores: sensor reading, control and motor output on core 1,
networking and telemetry on core 0. The tasks exchange data through single-element
queues, so each one always sees the newest value instead of a backlog.

| Component | Purpose |
| --- | --- |
| `mpu6050` | IMU over I2C at 400 kHz, gyro at ±500 °/s, accelerometer at ±4 g, fused into roll, pitch and yaw |
| `pid` | PID loop on pitch, roll and yaw rate, with a clamped integral term |
| `dshot` | DShot frames for the 4-in-1 ESC, generated with the RMT peripheral |
| `communications` | Wi-Fi station mode and the UDP server on port 5555 |
| `analytics` | Battery voltage and ESC current over ADC, with a lookup table for the 2S HV charge level |
| `utils` | Status LED patterns for the start-up sequence |

Control packets are 24 bytes, little endian, and carry a magic number, a sequence
number, the target pitch and roll angles, the target yaw rate, the base throttle and an
arm flag. Packets of the wrong size or with the wrong magic number are dropped, and a
gap in the sequence numbers is logged as a warning.

The Wi-Fi credentials are not in the repository. Copy the example file and fill in your
own network before building:

```sh
cd software/flight-controller
cp main/include/wifi_credentials.example.hpp main/include/wifi_credentials.hpp
idf.py set-target esp32
idf.py build flash monitor
```

`.devcontainer/` contains an ESP-IDF container, so the toolchain does not have to be
installed on the host.

## Web App

`software/web-app/backend/` is a FastAPI server that holds the current control state and
sends it to the drone as UDP packets at a fixed rate. The browser only talks to the
server, the server does the timing:

| Endpoint | Purpose |
| --- | --- |
| `POST /api/control` | Set target angles, throttle and arm state |
| `GET /api/state` | Read back the state the server is currently sending |
| `POST /api/disarm` | Emergency stop |

```sh
cd software/web-app/backend
pip install -r requirements.txt
python server.py
```

The server runs on port 8888, the drone address is set in `server.py` (`ESP_IP`).
`manuell_control.py` sends a single control packet and is useful for testing without a
frontend. `software/web-app/frontend/drone-webapp/` is a Next.js app and is still the
scaffold — the control interface is not written yet.

## Hardware

The frame is 3D printed, the electronics are off-the-shelf: an ESP32-S3 with camera
module, a 40 A 4-in-1 ESC, four 8500 kv brushless motors, an MPU6050 and a 2S 650 mAh
LiPo. `BOM.csv` lists every part with price and source, at around 80 francs in total.
The CAD is drawn in Onshape.

## Repository Layout

| Folder | Contents |
| --- | --- |
| `3d-files/` | Printable parts, not published yet |
| `software/flight-controller/` | The ESP-IDF firmware |
| `software/web-app/` | FastAPI backend and Next.js frontend |
| `BOM.csv` | Bill of materials |

## Status

The drone does not fly yet. Sensors, PID loop, telemetry and the radio link work; the
motor task is still disabled in `main.cpp` while the DShot output is being verified on
the bench. The 3D files and the Onshape document will be published once the frame is
final.

## License

PolyForm Noncommercial License 1.0.0: use, modification and distribution are permitted
for noncommercial purposes, details in [LICENSE](LICENSE).
