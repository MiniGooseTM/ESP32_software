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

- It tilts past `FALL_ANGLE_DEG`.
- The IMU fails to respond.
- The browser stops sending commands. After `CMD_TIMEOUT_MS` the command drops to zero, and after `LINK_LOST_STOP_MS` the motors are cut.

**Motor test** drives each wheel directly. Use it with the wheels off the ground to check that +1 turns both wheels forward. Then fix `MOTOR_x_SIGN` and `ENCODER_x_SIGN` in `main/config.h` if needed.

## Loading a trained policy

1. Save the actor weights as a `.npz`. The docstring in `tools/export_policy.py` shows how, including a Stable-Baselines3 example with VecNormalize.
2. Run `python3 tools/export_policy.py policy.npz`. Use the system Python, because the ESP-IDF environment has no numpy. This rewrites `main/policy_weights.h`.
3. Edit `build_observation()` in `main/control.c` so the order, units and signs match the training environment exactly. The observation normalisation comes from the `.npz`, so don't add it here.
4. Set `CONTROL_HZ` in `main/config.h` to the control rate used in training.
5. Rebuild. At boot the log prints the result of the policy self-test, which compares against numpy reference outputs, along with the measured inference time.

## Structure

| File | Role |
|---|---|
| `main/config.h` | All pins, rates, limits and WiFi settings |
| `main/control.c` | Control loop on core 1, state machine, observation |
| `main/policy.c` | MLP forward pass in plain C |
| `main/imu.c` | BMI160 driver, calibration, offsets saved in NVS |
| `main/motors.c` | TB6612 PWM and direction, encoder counting |
| `main/net.c` | WiFi access point, HTTP server, WebSocket |
| `main/log_stream.c` | Copies log output to the browser |
| `main/web/index.html` | The web page |

## Dual core

**Core 1** runs only the control task and its interrupts, which are the loop timer and I2C. A hardware timer triggers the loop at `CONTROL_HZ`.

**Core 0** runs everything else: WiFi, lwIP, the HTTP server, log and telemetry streaming, and the status LED.

The two cores share state through small spinlock-protected copies. The control task never logs to the UART directly, because a UART print at 115200 baud would stall the loop. It queues its messages with `log_deferred()` and core 0 prints them.

The web page shows the loop period, jitter, overruns and inference time. Use them to confirm that WiFi traffic does not disturb the loop.

## Inference backend

The policy runs as plain C float code instead of TFLite Micro. Most of a TFLite Micro run for a small MLP goes to interpreter and per-op overhead. Its float kernels are the unoptimised reference ones, and int8 quantisation would add error and about 300 KB of flash. The boot log and the web page report the measured inference time.
