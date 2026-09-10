# First Light

---

## Project Information

- Project Title: First Light
- PS ID: SIH26007
- PS Title: Safe and Efficient Operation of Mine Vehicles in Fog and Low-Visibility Conditions in Open Cast Iron Ore Mines.
- Category: Hardware
- Theme: Smart Automation

---

## Problem Statement

Dense fog in Mining complexes reduces visibility, makes dumper movement unsafe and inefficient. Also increases collision risks, delays haul cycles, and causes production losses.

---

## Proposed Solution

First Light is a safety system that uses multiple sensors and V2V communication to detect vehicles, obstacles, fog, and collision risks in real time. It dynamically adjusts vehicle speed, stops vehicles when necessary, and coordinates movement at junctions to enable safer and more efficient mining operations in low visibility.	

---

## Key Features

- Real-time object detection & tracking
- Collision-risk and time-to-collision estimation
- Dynamic speed control based on risk and visibility
- Automatic stop/resume with safe-state handling
- V2V coordination with fault and communication-robustness testing

---

## Technology / Sensors Stack

- LiDAR + Camera — Environment and obstacle perception
- Radar — Object detection and relative motion
- GNSS + IMU + Encoders — Position and vehicle movement
- V2V Communication — Sharing hazards and fog information
- Arduino + Raspberry Pi — Real-time control, telemetry and communication

---

## Architecture 

```
Vehicle
   |
   v
Sensors
   |
   v
Perception
   |
   v
Risk & Decision
   |
   v
Vehicle Control
   |
   +------> V2V Communication
   |
   v
Safe Movement
```

---

## Files

- [Simulation notes](docs/simulation.md)
- [How it works](docs/architecture.md)
- [Hardware setup and pins](docs/hardware.md)
- [Design gallery](assets/designs.html) and [design PDF](assets/first_light_designs_clean.pdf)
- [CAD files](assets/cad/README.md)
- [Circuit diagrams and parts list](assets/circuit/README.md)
- [Presentation](./First_Light_SIH26007.pptx)

Simulation code is in `src/simulation/`; hardware code is in `src/hardware/`. The full-size truck CAD is a concept. The circuit is for the tabletop build. The demo video is shared separately.

---

## Final Presentation

PPT: [Open Final Presentation](./First_Light_SIH26007.pptx)

---

Video Link : [Video](https://www.youtube.com/watch?v=g4CtgDOWm1c)

---
## Setup and run

1. Install [Miniforge](https://github.com/conda-forge/miniforge#install) for your computer. Skip this if Conda is already installed.
2. Clone or download this repo.
3. On Windows, open **Miniforge Prompt** from the Start menu. Change to the downloaded repo folder containing `environment.yml`. On Linux/macOS, use a terminal with Conda available.
4. Create the environment once:

```sh
conda env create -f environment.yml
```

Run the simulation:

```sh
conda run --no-capture-output -n first-light python src/main.py --seed 42 --vehicles 4 --hold
```

This starts the PyBullet window and the Tkinter dashboard on a computer with a graphical desktop. The environment includes Python 3.12, PyBullet, NumPy, Pillow and Tkinter. Use the same run command each time; there is no need to recreate the environment.

Change `42` for a different seed or `4` for a different number of trucks. The seed changes fog, sensor noise and communication conditions. `--hold` keeps the windows open after parking.

### Controls

Click the PyBullet window before using the keys.

| Key | Action |
|---|---|
| 1-6 | Select a truck |
| C | Change camera |
| Space | Pause/resume |
| Escape | End the run |
| Mouse | Orbit, pan and zoom |

The Tkinter window displays fleet information. Keyboard controls are in PyBullet.

### Common setup issues

- **Conda not recognized:** use Miniforge Prompt on Windows.
- **Failed building wheel for PyBullet / NumPy missing:** use the Conda commands above instead of installing the simulation with pip.
- **Environment already exists:** skip creation and run the simulation.
- **Tkinter window missing:** check it with `conda run -n first-light python -m tkinter`.

The optional local `runtime/` folder is not included in GitHub. It runs the simulation without Tkinter. The batch files do not install dependencies; use the Conda run command above for this setup.

### Hardware demo

Run the controller with simulated sensor inputs:

```sh
conda run --no-capture-output -n first-light python src/main.py hardware --mock
```

The physical version uses an Uno, Pi/laptop, motor driver, ultrasonic sensor, encoders, line sensors, IMU and emergency stop. The hardware code has not yet been verified on an assembled rover.

