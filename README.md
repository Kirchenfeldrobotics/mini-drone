# Mini Drone

A small quadcopter we are building from scratch. We write our own flight
controller firmware for an ESP32, design the frame ourselves in Onshape and
control the drone from a web app over WiFi. The point of the project is to
understand how a drone actually works instead of just buying one.

It is a two person project by Nils and Valéry, two high school students. We
split the work about equally between us.

## Status

The software is basically finished. The firmware reads the sensor, stabilizes
the drone, drives the motors and sends telemetry back, so the drone could fly
with this code. The hardware is not finished yet. The frame exists as a 3D model
in Onshape but it is not production ready, so nothing has been printed,
assembled or flown so far.

## Hardware

| Part | Notes |
| --- | --- |
| 4x brushless motor | 8500kv, made for 2S |
| 4in1 ESC | 40A, driven with DShot150 |
| ESP32 | flight controller, runs the whole flight loop |
| MPU6050 | IMU, connected over I2C |
| 2S LiPo | 650 mAh |
| Buck converter | 7.4V down to 5V for the ESP32 |

The complete parts list with prices and links is in `BOM.csv`. The frame is
modelled in Onshape and still gets changed.

## Software

### Flight controller

`software/flight-controller`, written in C++ with ESP-IDF. Everything runs as
FreeRTOS tasks that are split over both cores of the ESP32. WiFi and battery
monitoring sit on core 0, the flight loop (sensor, PID, motors) sits on core 1,
so network traffic can never delay the control loop. The tasks wait for each
other with a FreeRTOS event group so they always start in the right order.

| Component | What it does |
| --- | --- |
| `mpu6050` | Reads accelerometer and gyroscope at 1 kHz over I2C. A complementary filter (98% gyro, 2% accel) turns that into a pitch and roll angle, yaw is used as a rate. |
| `pid` | One PID controller per axis. Pitch and roll are controlled by angle, yaw by rate. The integral part is clamped so it cannot wind up. |
| `dshot` | Sends DShot150 frames with the RMT peripheral, one channel per motor. A mixer turns the PID output into four throttle values and the base throttle is ramped so it cannot jump. |
| `communications` | UDP server on port 5555. Packets are binary structs with a magic number and a sequence number. It receives the control input and sends telemetry back at 50 Hz. |
| `analytics` | Measures battery voltage and ESC current with the ADC, smooths both with an EMA filter and converts the voltage to a percentage with a 2S LiPo lookup table. |
| `log_server` | Streams the ESP log output over UDP port 5556 so we can debug the drone while it is in the air. |

For safety the motors only spin while the ground station sets the armed flag in
every single packet. The ESCs first get a zero throttle arming sequence and they
are armed again after every disarm.

### Backend

`software/web-app/backend`, Python with FastAPI. It takes the control input over
a REST endpoint, packs it into the binary packet format and sends it to the
drone 50 times per second over UDP. At the same time it listens for the
telemetry packets and serves the latest values on `/api/analytics`. There is
also an `/api/disarm` route as an emergency stop.

### Frontend

`software/web-app/frontend`, Next.js with TypeScript and Tailwind. This is the
part that still has to be built. It should show the telemetry and let us steer
the drone with the keyboard or a gamepad.

## Still open

* finish the Onshape model and print the frame
* build the frontend UI
* camera component (the file exists but is empty)
* first flight and tuning the PID values

## Build and run

```bash
# flight controller
cd software/flight-controller
idf.py build flash monitor

# backend
cd software/web-app/backend
pip install -r requirements.txt
python server.py

# frontend
cd software/web-app/frontend/drone-webapp
npm install
npm run dev
```

The WiFi credentials go into `main/include/wifi_credentials.hpp` and the IP of
the drone has to be set in `server.py`.
