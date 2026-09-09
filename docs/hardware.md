# Tabletop hardware

The build uses two motors for differential steering, with tape for route guidance. The circuit is for a tabletop rover; the full-size truck uses different hardware.

## Parts

| Part | Quantity |
|---|---:|
| Raspberry Pi or USB laptop | 1 |
| Arduino Uno R3 | 1 |
| TB6612FNG motor-driver carrier | 1 |
| Low-current encoder gearmotors | 2 |
| HC-SR04 ultrasonic sensor | 1 |
| Analog line sensors | 3 |
| MPU6050 breakout and bidirectional I2C level shifter | 1 each |
| Latching emergency button with two isolated NC contacts | 1 |
| Fog switch and 10 kOhm pull-up | 1 each |
| Motor battery, fuse and master switch | 1 set |
| Regulated Pi supply, wiring and chassis | As needed |

The electronics base is 250 x 205 mm. Four wheels need mechanical coupling on each side if only two motors are used.

## Uno pins

| Pin | Connection |
|---|---|
| D2 / D3 | Left / right encoder pulses |
| D4 | Driver STBY; 10 kOhm pull-down |
| D5 / D6 | Driver PWMA / PWMB |
| D7 / D8 | Driver AIN1 / AIN2 |
| D9 / D10 | Driver BIN1 / BIN2 |
| D11 / D12 | HC-SR04 TRIG / ECHO |
| D13 | Fog switch to ground; external 10 kOhm pull-up |
| A0 / A1 / A2 | Left / centre / right line sensors |
| A3 | NC emergency sense contact to ground |
| A4 / A5 | SDA / SCL through the level shifter |

## Power and wiring

- Motor supply: four NiMH cells (4.8 V nominal), master switch and fuse. The schematic uses a provisional 2 A fuse and a maximum selected motor stall current of 0.8 A per channel; check ratings against the actual parts.
- Pi supply: regulated 5.1 V / 3 A through USB-C. Pi USB powers the Uno and carries serial data. Do not add a second Uno 5 V feed.
- Driver VCC and compatible range/line/encoder sensors use 5 V. Motors use the separate motor supply. Join grounds at the distribution point.
- Emergency contact SW2 disconnects motor power; SW3 opens the A3 sense circuit. Both belong to the same latching actuator and need suitable DC current ratings.
- MPU supply and I2C low side are 3.3 V; Uno I2C is 5 V. Use a compatible breakout and level shifter. MPU AD0 goes to ground (address 0x68); mount +Z upward.
- HC-SR04 ECHO goes to Uno D12, not Pi GPIO. Line and encoder outputs must be compatible with Uno 5 V inputs.
- Driver AO1/AO2 connect to the left motor; BO1/BO2 to the right. Keep the decoupling shown in the [circuit](../assets/circuit/README.md).

## Upload and run

Open `src/hardware/arduino/rover_firmware/rover_firmware.ino` in Arduino IDE. Select Uno R3 with Arduino AVR Boards 1.8.6 or newer, then compile and upload. The sketch uses the bundled Wire library.

After the [Conda setup](../README.md), run from the repo folder:

```sh
conda install -n first-light -c conda-forge pyserial
conda run --no-capture-output -n first-light python src/main.py hardware --port COM3
```

Replace `COM3` with the connected port, such as `/dev/ttyACM0` on Linux. Close Arduino Serial Monitor first. Motors stay disabled until `--arm` is added.

## Calibration

1. Check telemetry with motor power disconnected.
2. Set wheel diameter and rising-edge pulses per wheel revolution in `src/hardware/config.json`.
3. Set the line threshold from tape/background readings. A wide marker activates all three sensors and stops the rover.
4. Check IMU acceleration is near (0,0,1) g on a level surface.
5. Lift the wheels before enabling motors. Check direction and calibrate speed/PWM.
6. Check obstacle stop, line loss, emergency cutoff, disconnected sensors and USB timeout. Measure stopping distance before driving at cruise speed.

The sonar needs a valid echo from 20-4000 mm; no echo causes a stop. Emergency and host faults need reset/restart. The destination stop stays latched.

## Cooperation

The fog switch demonstrates reduced visibility; it does not measure fog. Use unique vehicle IDs and matching zones. `--network` enables peer warnings; `--center LAPTOP_IP` sends telemetry to the command centre:

```sh
conda run --no-capture-output -n first-light python src/main.py command-center
```

The physical build and firmware have not yet been verified on an assembled rover. [Component references](designs.md) · [Serial and Wi-Fi messages](protocol.md)
