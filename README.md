# Depth Analyse

Raspberry Pi 5 based **groove detection system** using continuous rotation servo and ToF sensor.

## Features
- 360° continuous rotation scan using MG996R servo
- VL53L0X Time-of-Flight distance measurement
- Automatic groove detection (depth & angular width)
- CSV data export + Matplotlib graph output

## Hardware Required
- Raspberry Pi 5
- MG996R 360° Continuous Rotation Servo
- VL53L0X / VL53L1X ToF Sensor
- External 5V power supply for servo (recommended)

## Wiring

| Component     | Pin                  | Raspberry Pi Pin |
|---------------|----------------------|------------------|
| Servo Signal  | GPIO 18              | Physical Pin 12  |
| Servo Power   | External 5V          | -                |
| Servo GND     | Common GND           | Pin 6            |
| ToF SDA       | SDA (Pin 3)          | I2C              |
| ToF SCL       | SCL (Pin 5)          | I2C              |
| ToF VCC       | 3.3V                 | Pin 1            |
| ToF GND       | GND                  | Pin 6            |

## Installation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-dev python3-venv libatlas-base-dev python3-lgpio

# Create a venv that can see Raspberry Pi system GPIO packages
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# Install project Python packages
python3 -m pip install -r requirements.txt

# Test only the servo
python3 src/depth_analyse.py --servo-test
```

## Running a Scan

```bash
python3 drop_test_refined.py --mode real --sensor-test
python3 drop_test_refined.py --mode real --backend vl53l4cd --rotation-time 72
```

Use `--backend vl53l0x` if the connected board is a VL53L0X. The default
`--backend auto` tries VL53L0X first, then VL53L4CD, then the clone register
reader.

The refined drop test now starts and stops the MG996R for both the before and
after scans. The defaults use GPIO 18, a 1350 us run pulse, and a 1500 us stop
pulse. Tune them with `--servo-run-us` and `--servo-neutral-us`; set the measured
full-turn time with `--rotation-time`. Use `--no-servo` for a sensor-only run.

## Web control panel

```bash
.venv/bin/python web_app.py
```

Open `http://localhost:5000` on the Pi, or `http://<pi-address>:5000` from
another device on the same network. Each run is saved under a timestamped
`scans/YYYY-MM-DD_HH-MM-SS/` folder with the before, after, comparison,
interactive dashboard, raw data and processed profile.

## Troubleshooting

If you see `ModuleNotFoundError: No module named 'pandas'`, you are probably
running outside the virtual environment. Run:

```bash
cd ~/depth_analyse
source .venv/bin/activate
python3 drop_test_refined.py
```

If you see `ModuleNotFoundError: No module named 'lgpio'`, install Raspberry Pi's
GPIO package and recreate the venv so it can see system packages:

```bash
cd ~/depth_analyse
sudo apt install -y python3-lgpio
rm -rf .venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```
