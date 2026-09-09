# Hardware messages

## Arduino to host

USB serial: 115200 baud, about 10 updates per second. Each message ends with a newline.

```text
S,seq,millis,range_mm,left_ticks,right_ticks,line_left,line_center,line_right,estop,fog,imu_ok,ax_mg,ay_mg,az_mg,gz_centideg_s
```

- Range: 20-4000 mm; `-1` means unavailable.
- Line sensors: 0-1023. Flags: 0 or 1.
- Acceleration: milli-g. Yaw rate: hundredths of a degree/second.
- Encoder counters: unsigned 32-bit, rising-edge counts.

## Host to Arduino

```text
D,seq,left_pwm,right_pwm
```

PWM is forward-only, 0-110. Commands expire after 300 ms. Invalid commands, emergency input or unavailable range/IMU disable drive.

## Wi-Fi

UDP 5005 carries peer warnings; UDP 5006 carries command-centre telemetry.

Packets include vehicle ID, timestamp, sequence, zone, fog, warning and state. Telemetry adds distance, speed, PWM, reason and TTC. TTC is `null` when the target is not closing.

Peer warnings expire after one second. Use unique IDs, matching zones and synchronized clocks. Packets are unauthenticated, so use a private demo network. Local obstacle stopping does not depend on Wi-Fi.
