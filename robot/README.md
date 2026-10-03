# Balancing robot firmware

Runs a learned PPO policy on the ESP32-S3. On each control tick it reads the BMI160 IMU and the wheel encoders, evaluates the policy, and drives the two TB6612 motors. A WiFi web page shows the logs and telemetry, sends velocity commands and starts the IMU calibration.

## Build and flash

```sh
source ~/.espressif/tools/activate_idf_v6.1.sh
cd robot
idf.py build flash monitor
```

## Use

1. Join the WiFi network `goose` with the password `goosegoose`, then open http://192.168.4.1.
2. Press **Calibrate** once while holding the robot still and upright. The offsets are saved in flash and restored on every boot.
3. Hold the robot upright and press **Start**. Drive with the pad or with W A S D. Space or Esc stops it.

The robot stops on its own in these cases:

- Its pitch or roll passes the policy's training tilt limit (0.6 rad, 34 deg) while balancing, or `FALL_ANGLE_DEG` in the motor test.
- The IMU fails to respond.
- The browser stops sending commands. After `CMD_TIMEOUT_MS` the command drops to zero, and after `LINK_LOST_STOP_MS` the motors are cut.

**Check the observation signs** before the first run with a trained policy. They follow the training environment, and the Observation panel shows them. Tilt the robot by hand and check each value:

- Leaning forward makes `pitch` and `pitch rate` negative.
- Leaning to the right makes `roll` and `roll rate` positive.
- Turning left (counter-clockwise from above) makes `yaw rate` positive.
- Rolling the robot forward makes `wheel L` and `wheel R` positive.

`wheel L` must be the left wheel. If a value is wrong, fix `IMU_BODY_*` or `ENCODER_x_SIGN` in `main/config.h`.

**Motor test** drives each wheel directly. Use it with the wheels off the ground to check that +1 turns both wheels forward. Also check that motor A is the left wheel. Then fix `MOTOR_x_SIGN` and `ENCODER_x_SIGN` in `main/config.h` if needed.

## Loading a trained policy

A policy is the file `models/policy.bin` that `make export` writes in `software_stack`. The default model is `models/ppo_balance.zip`; pick another with `EXPORT_ARGS="--model <zip>"`. The format is described in `main/policy_net.h`. The firmware has two sources for it:

- **Uploaded:** stop the robot, then in the web page's **Policy** panel choose the file and press **Upload**. No rebuild is needed. The file is written to the `policy` flash partition and checked there. It replaces the current policy only if it passes, and it stays in use after reboots and reflashing. **Use built-in** erases it again. A rejected or interrupted upload is erased too, and the robot then falls back to the built-in policy. `curl --data-binary @models/policy.bin http://192.168.4.1/policy` also works.
- **Built-in:** `main/policy.bin`, embedded in the firmware. It is used when nothing (usable) is uploaded. To change it, copy the file there, rebuild and flash.

A policy is used only if all of these hold. Otherwise Start is refused and the log says why.
- The file is intact (CRC).
- Its observation (field names, frame size, history length) matches `build_frame()` and `OBS_HISTORY` in `main/control.c`.
- Its control rate, command limits, half-track and wheel speed match `main/config.h`.
- The firmware's forward pass reproduces the SB3 actions stored in the file.

The log and the web page show the active policy's source, name and CRC, and the log also shows its inference time.

To check a file on the PC with the firmware's own parser, run this in a shell without the ESP-IDF environment:
```sh
cd test
cc -O2 -std=c11 -I../main policy_host_test.c ../main/policy_net.c -lm -o policy_host_test
./policy_host_test ../main/policy.bin
```

If the training environment changes, update the observation in `main/control.c` (`OBS_FRAME_FIELDS`, `build_frame()`, `OBS_HISTORY`) and the "Policy interface" constants in `main/config.h` to match.

## Structure

| File | Role |
|---|---|
| `main/config.h` | All pins, rates, limits and WiFi settings |
| `main/control.c` | Control loop on core 1, state machine, observation |
| `main/policy.c` | Loads the built-in or uploaded policy, stores uploads in flash |
| `main/policy_net.c` | Policy file parser and MLP forward pass in plain C (also builds on the PC) |
| `main/policy.bin` | The built-in policy, from `make export` in `software_stack` |
| `main/imu.c` | BMI160 driver, calibration, offsets saved in NVS |
| `main/motors.c` | TB6612 PWM and direction, encoder counting |
| `main/net.c` | WiFi access point, HTTP server (page, policy upload), WebSocket |
| `main/log_stream.c` | Copies log output to the browser |
| `main/web/index.html` | The web page |
| `partitions.csv` | Flash layout: app, NVS, uploaded policy |

## Dual core

**Core 1** runs only the control task and its interrupts, which are the loop timer and I2C. A hardware timer triggers the loop at `CONTROL_HZ`.

**Core 0** runs everything else: WiFi, lwIP, the HTTP server, log and telemetry streaming, and the status LED.

The two cores share state through small spinlock-protected copies. The control task never logs to the UART directly, because a UART print at 115200 baud would stall the loop. It queues its messages with `log_deferred()` and core 0 prints them.

The web page shows the loop period, jitter, overruns and inference time. Use them to confirm that WiFi traffic does not disturb the loop.

## Inference backend

The policy runs as plain C float code instead of TFLite Micro. Most of a TFLite Micro run for a small MLP goes to interpreter and per-op overhead. Its float kernels are the unoptimised reference ones, and int8 quantisation would add error and about 300 KB of flash. The boot log and the web page report the measured inference time.
