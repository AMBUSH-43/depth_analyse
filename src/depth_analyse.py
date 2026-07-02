#!/usr/bin/env python3
"""
Depth Analyse - MG996R Continuous Servo (GPIO Busy Fixed)
"""

import time
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
import lgpio
import board
import busio
import adafruit_vl53l0x

# ==================== CONFIG ====================
SERVO_PIN = 18
STEP_ANGLE = 2
TOTAL_ANGLE = 360
SAMPLES_PER_ANGLE = 5
GROOVE_THRESHOLD_MM = 3.0

# MG996R 360-degree servos are continuous-rotation servos:
# 1500 us stops, values below/above 1500 us set direction and speed.
SERVO_NEUTRAL_US = 1500
SERVO_CLOCKWISE_US = 1350
SERVO_COUNTERCLOCKWISE_US = 1650
SERVO_RUN_US = SERVO_CLOCKWISE_US

# Calibrate this by timing one full turn at SERVO_RUN_US with the real load.
# The previous code effectively assumed 0.40 s per 2 degrees: 72 s/rev.
SECONDS_PER_REVOLUTION = 72.0
SERVO_TEST_SECONDS = 3.0

# ==================== HARDWARE SETUP ====================
try:
    h = lgpio.gpiochip_open(4)
except:
    h = lgpio.gpiochip_open(0)

# Free pin first if busy
try:
    lgpio.gpio_free(h, SERVO_PIN)
except:
    pass

lgpio.gpio_claim_output(h, SERVO_PIN)

i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l0x.VL53L0X(i2c)

# ==================== SERVO CONTROL ====================
def set_servo(microseconds):
    if not 500 <= microseconds <= 2500:
        raise ValueError(f"Servo pulse must be 500-2500 us, got {microseconds}")
    duty = (microseconds / 20000.0) * 100
    lgpio.tx_pwm(h, SERVO_PIN, 50, duty)

def stop_servo():
    set_servo(SERVO_NEUTRAL_US)

def servo_test():
    print(">>> Servo test: clockwise")
    set_servo(SERVO_CLOCKWISE_US)
    time.sleep(SERVO_TEST_SECONDS)
    stop_servo()
    time.sleep(0.5)

    print(">>> Servo test: counterclockwise")
    set_servo(SERVO_COUNTERCLOCKWISE_US)
    time.sleep(SERVO_TEST_SECONDS)
    stop_servo()
    print(">>> Servo test finished")

# ==================== FUNCTIONS ====================
def read_tof_median(samples=5):
    readings = []
    for _ in range(samples):
        dist = tof.distance * 1000
        if dist > 0:
            readings.append(dist)
        time.sleep(0.03)
    return np.median(readings) if readings else 0

def continuous_scan():
    data = []
    sample_count = int(TOTAL_ANGLE / STEP_ANGLE)
    sample_interval = SECONDS_PER_REVOLUTION / sample_count

    print(">>> Starting continuous 360° scan...")
    print(f">>> Servo pulse: {SERVO_RUN_US} us | Estimated scan time: {SECONDS_PER_REVOLUTION:.1f} s")
    set_servo(SERVO_RUN_US)
    time.sleep(0.5)

    try:
        for sample_index in range(sample_count):
            angle = sample_index * STEP_ANGLE
            time.sleep(sample_interval)
            distance = read_tof_median(SAMPLES_PER_ANGLE)
            data.append({
                "angle": round(angle, 1),
                "distance_mm": round(distance, 1)
            })
            print(f"Angle: {angle:5.1f}° | Distance: {distance:6.1f} mm")
    except KeyboardInterrupt:
        print("\nScan stopped early.")
    finally:
        stop_servo()

    return data

def calculate_baseline(distances):
    sorted_d = sorted(distances)
    return np.median(sorted_d[:int(len(sorted_d) * 0.4)])

def detect_grooves(data, baseline):
    grooves = []
    current = []
    for p in data:
        depth = p["distance_mm"] - baseline
        if depth > GROOVE_THRESHOLD_MM:
            current.append({**p, "depth_mm": round(depth, 2)})
        elif current:
            grooves.append(current)
            current = []
    if current:
        grooves.append(current)
    return grooves

def plot_results(data, baseline, grooves):
    angles = [d["angle"] for d in data]
    dists = [d["distance_mm"] for d in data]

    plt.figure(figsize=(12, 6))
    plt.plot(angles, dists, 'b-', label="Distance")
    plt.axhline(y=baseline, color='green', linestyle='--', label=f"Baseline")
    
    for i, g in enumerate(grooves):
        g_angles = [p["angle"] for p in g]
        plt.fill_between(g_angles,
                         [baseline] * len(g),
                         [baseline + p["depth_mm"] for p in g],
                         alpha=0.4, color='red', label="Groove" if i == 0 else "")
    
    plt.title("360° Depth Scan")
    plt.xlabel("Angle (°)")
    plt.ylabel("Distance (mm)")
    plt.legend()
    plt.grid(True)
    plt.savefig("depth_scan_graph.png", dpi=150)
    plt.show()

# ==================== MAIN ====================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Depth Analyse 360-degree groove scanner")
    parser.add_argument(
        "--servo-test",
        action="store_true",
        help="Run the servo in both directions without starting a depth scan",
    )
    args = parser.parse_args()

    try:
        if args.servo_test:
            servo_test()
        else:
            scan_data = continuous_scan()

            if scan_data:
                baseline = calculate_baseline([d["distance_mm"] for d in scan_data])
                grooves = detect_grooves(scan_data, baseline)

                with open("depth_scan_data.csv", "w", newline="") as f:
                    writer = csv.DictWriter(f, ["angle", "distance_mm"])
                    writer.writeheader()
                    writer.writerows(scan_data)

                print(f"\nScan finished! Baseline: {baseline:.1f} mm | Grooves: {len(grooves)}")
                plot_results(scan_data, baseline, grooves)
    finally:
        stop_servo()
        lgpio.gpiochip_close(h)
