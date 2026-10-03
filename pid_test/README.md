# PID balance test

Balances the robot with a tilt PID at 200 Hz. Every gain can be tuned live from a web page. The IMU, motor and log drivers come from `robot/main/`, along with the pins, IMU axes, safety limits and WiFi settings in `robot/main/config.h`. The PID defaults are in `main/pid_config.h`, and the loop rate is set in `main/CMakeLists.txt`.

## Build and flash

```sh
source ~/.espressif/tools/activate_idf_v6.1.sh
cd pid_test
idf.py build flash monitor
```

Join the WiFi network `goose` with the password `goosegoose`, then open http://192.168.4.1.

## Controller

```
tilt  = complementary filter of gyro x and the accelerometer, deg, forward positive
sp    = trim - Kv * average wheel speed      (clamped to trim +- sp_limit)
e     = tilt - sp
u     = Kp*e + Ki*integral(e) + Kd*tilt_rate (clamped to +- out_limit)
```

Both wheels get the command `u`. Leaning forward drives the wheels forward. The D term uses the gyro rate directly, so it stays clean and does not jump when the setpoint changes. The integral stops growing while the output is saturated.

## Tuning procedure

1. **Check the axes.** With the robot stopped, lean it forward. The tilt must go positive and the rate must be positive while you lean it. If not, fix `IMU_TILT_GYRO_SIGN` or the accelerometer axes in `robot/main/config.h`.
2. **Check the motors.** Run Start with a small Kp, and hold the robot in your hands. Leaning it forward must spin both wheels forward. If a wheel turns the wrong way, flip its `MOTOR_x_SIGN`.
3. **Find the balance point.** Hold the robot where it balances by itself and press **Set trim here**.
4. **Tune Kp with Ki and Kd at 0.** Raise Kp until the robot catches itself and oscillates steadily, then back off about a third.
5. **Tune Kd.** Raise it until the oscillation damps out. If the motors buzz or jitter, Kd is too high.
6. **Tune Ki, optionally.** A small Ki removes a steady lean.
7. **Stop the drift with Kv.** A pure tilt PID slowly rolls away. A small positive Kv leans the setpoint back as the robot speeds up.
8. **Compensate friction if needed.** If small corrections don't turn the wheels, raise friction until they just do.
9. **Save.** Press **Save to flash** while stopped. Saved gains load on every boot.

Use **Record** to capture every control step, then **Download CSV** to analyse it offline. The robot stops on its own if it tilts past 45°, the IMU fails, or the page stops sending its heartbeat for 5 s. The IMU calibration is shared with the robot firmware, because both keep it in the same flash storage.
