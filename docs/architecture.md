# How it works

## Simulation

```text
Sensors + received V2V packets
              |
       Position and fusion
              |
       Collision prediction
              |
      Speed / braking / yield
              |
          Truck motion

Vehicle telemetry -> Command centre display
```

Each truck keeps its own measurements and warnings. Old data expires. The network adds delay and packet loss.

## Tabletop build

```text
Range / encoders / line sensors / IMU -> Arduino Uno
Arduino Uno <-> USB <-> Pi or laptop controller
Arduino Uno -> motor driver -> motors
Pi <-> Wi-Fi warnings <-> other rovers
Pi -> telemetry -> command centre
Emergency button -> motor power cutoff + Uno input
```

The Uno reads sensors and disables drive if commands stop arriving for 300 ms. The host follows tape, estimates speed and collision risk, and sends motor commands. Faults stop the rover.

The hardware and simulation currently use separate implementations. The hardware replay uses synthetic sensor inputs. See [hardware setup](hardware.md) and [message format](protocol.md).
