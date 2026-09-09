# Simulation

Complete the [README setup](../README.md) first. Run commands from the repo folder.

## Run options

Different seed:

```sh
conda run --no-capture-output -n first-light python src/main.py --seed 7 --vehicles 4 --hold
```

Six trucks:

```sh
conda run --no-capture-output -n first-light python src/main.py --seed 42 --vehicles 6 --hold
```

Dense fog:

```sh
conda run --no-capture-output -n first-light python src/main.py --scenario dense-fog --hold
```

Other scenarios: `demo`, `intersection`, `v2v-warning`, `sensor-disagreement` and `fleet-arrival`. These change settings on the same mine map.

The default mission takes about 2-3 simulated minutes. Seeds change fog, clutter, sensor noise and packet delivery; the road layout stays the same.

## Sensors

| Sensor | Role in the simulation |
|---|---|
| Camera | Forward detections; fog reduces range and adds noise |
| LiDAR | Ray casts measure surfaces; fog reduces range and causes missed returns |
| Radar | Estimates position and closing speed; this model does not attenuate it in fog |
| GNSS | Gives noisy position updates |
| Speed | Estimates vehicle speed |
| IMU | Gives orientation for position estimation between GNSS updates |

Camera and radar use simplified measurements, not real image processing or radio-wave simulation. Target identities are supplied by the simulated scene.

## Driving logic

Measurements and received V2V packets are combined by uncertainty and age. The truck calculates time to collision, closest approach and junction conflicts, then slows, stops or yields. Shared fog reports can make it slow before entering fog.

Routes lead to separate parking slots. The command centre displays received telemetry; each truck makes its own driving decisions.

Results are saved in `results/latest.json`. `--headless` runs without windows; `--snapshot results/mine.png` saves an image. Configuration overrides can be supplied with `--config src/simulation/config.example.json`.

The vehicle physics and sensors are simplified. A successful simulation run does not establish real-truck stopping performance.
