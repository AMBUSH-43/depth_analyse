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

# Install dependencies
sudo apt install -y python3-pip python3-dev libatlas-base-dev

# Install Python packages
pip3 install -r requirements.txt

# Test only the servo
python3 src/depth_analyse.py --servo-test
```

## Running a Scan

```bash
python3 drop_test_refined.py --mode real --backend vl53l4cd --rotation-time 72
```

The refined drop test now starts and stops the MG996R for both the before and
after scans. The defaults use GPIO 18, a 1350 us run pulse, and a 1500 us stop
pulse. Tune them with `--servo-run-us` and `--servo-neutral-us`; set the measured
full-turn time with `--rotation-time`. Use `--no-servo` for a sensor-only run.
