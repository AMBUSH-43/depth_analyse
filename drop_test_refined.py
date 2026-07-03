#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drop Test - refined Raspberry Pi / dummy scanner.

What this version adds over the earlier script:
  - CLI mode selection: no need to edit MODE inside the file
  - Dummy mode for laptop development
  - Real mode for Raspberry Pi VL53L4CD / VL53L4CDK-style sensors
  - Clone-friendly backend for "UL53LDK" style modules
  - Optional real backend: auto, vl53l4cd, clone, or smbus
  - Safer Pi 5 fan GPIO setup with LGPIOFactory when available
  - Robust outlier filtering, circular smoothing, and scan alignment
  - JSON + CSV export
  - PNG plots
  - Interactive HTML dashboard:
      * 3D cylindrical scan map
      * before vs after profile
      * after-before deviation graph

Run on laptop:
  python3 drop_test_refined.py --mode dummy --auto-demo

Run on Raspberry Pi with the official VL53L4CD driver:
  python3 drop_test_refined.py --mode real --backend vl53l4cd

Run on Raspberry Pi with the fake/clone UL53LDK-style module:
  python3 drop_test_refined.py --mode real --backend clone --distance-reg auto
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import random
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Protocol

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Change this only if you want a file-level default.
# CLI --mode overrides it.
DEFAULT_MODE = "dummy"  # "dummy" or "real"


@dataclass(frozen=True)
class Config:
    mode: str = DEFAULT_MODE
    backend: str = "auto"  # auto, vl53l4cd, clone, smbus
    strict_hardware: bool = False
    auto_demo: bool = False
    strict_identity: bool = False

    target_points: int = 360
    read_interval_s: float = 0.20
    smooth_window: int = 7
    outlier_threshold_mm: float = 50.0
    deviation_threshold_mm: float = 0.5
    max_align_shift_deg: float = 45.0

    # If your base rotates exactly once during the before scan, leave this None.
    # If you have calibrated servo timing, set it and angles will be time based.
    rotation_time_s: float | None = None

    # MG996R continuous-rotation servo (values copied from the working
    # src/depth_analyse.py implementation).
    servo_enabled: bool = True
    servo_pin: int = 18
    servo_run_us: int = 1350
    servo_neutral_us: int = 1500
    servo_frequency_hz: int = 50

    # Direct smbus fallback settings.
    i2c_bus: int = 1
    i2c_addr: int = 0x29
    distance_reg: int | None = None

    # Fan.
    fan_enabled: bool = True
    fan_pin: int = 21
    fan_on_temp_c: float = 55.0
    fan_off_temp_c: float = 45.0

    # Geometry for the 3D visualization.
    sensor_to_axis_mm: float = 260.0
    display_height_mm: float = 35.0
    display_layers: int = 28

    # Output files.
    log_file: str = "drop_shape_log.txt"
    graph_original: str = "original_before.png"
    graph_after: str = "after_drop.png"
    graph_compare: str = "comparison.png"
    data_json: str = "scan_data.json"
    raw_csv: str = "scan_raw.csv"
    profile_csv: str = "scan_profile.csv"
    dashboard_html: str = "drop_scan_dashboard.html"


class KeyReader:
    def __init__(self) -> None:
        self.is_windows = platform.system() == "Windows"
        if self.is_windows:
            import msvcrt

            self._msvcrt = msvcrt
        else:
            import select

            self._select = select

    def has_key(self, timeout_s: float = 0.0) -> bool:
        if self.is_windows:
            return bool(self._msvcrt.kbhit())
        return bool(self._select.select([sys.stdin], [], [], timeout_s)[0])

    def read_key(self) -> str:
        if self.is_windows:
            try:
                return self._msvcrt.getch().decode(errors="ignore").lower()
            except Exception:
                return ""
        return sys.stdin.read(1).strip().lower()

    def wait_for(self, target_key: str, message: str) -> None:
        print(message)
        while True:
            if self.has_key(0.1):
                if self.read_key() == target_key.lower():
                    return
            time.sleep(0.05)


class DistanceSensor(Protocol):
    def open(self) -> None:
        ...

    def read_mm(self, angle_deg: float, is_after: bool) -> float | None:
        ...

    def close(self) -> None:
        ...


class Servo(Protocol):
    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def close(self) -> None:
        ...


class MockServo:
    def start(self) -> None:
        print("-> Mock servo running")

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass


class RealServo:
    """MG996R continuous servo control using the project's working lgpio method."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.lgpio = None
        self.handle = None
        self.closed = False

        if not 500 <= cfg.servo_run_us <= 2500:
            raise ValueError("--servo-run-us must be between 500 and 2500")
        if not 500 <= cfg.servo_neutral_us <= 2500:
            raise ValueError("--servo-neutral-us must be between 500 and 2500")

        import lgpio

        self.lgpio = lgpio
        last_error: Exception | None = None
        for chip in (4, 0):
            try:
                self.handle = lgpio.gpiochip_open(chip)
                break
            except Exception as exc:
                last_error = exc
        if self.handle is None:
            raise RuntimeError(f"Could not open GPIO chip for servo: {last_error}")

        try:
            try:
                lgpio.gpio_free(self.handle, cfg.servo_pin)
            except Exception:
                pass
            lgpio.gpio_claim_output(self.handle, cfg.servo_pin)
            self.stop()
        except Exception:
            lgpio.gpiochip_close(self.handle)
            self.handle = None
            raise

        print(
            f"-> Servo ready on GPIO {cfg.servo_pin} "
            f"(run {cfg.servo_run_us} us, stop {cfg.servo_neutral_us} us)"
        )

    def _set_pulse(self, microseconds: int) -> None:
        if self.closed or self.lgpio is None or self.handle is None:
            raise RuntimeError("Servo is closed")
        duty = (microseconds / 20000.0) * 100.0
        self.lgpio.tx_pwm(
            self.handle,
            self.cfg.servo_pin,
            self.cfg.servo_frequency_hz,
            duty,
        )

    def start(self) -> None:
        self._set_pulse(self.cfg.servo_run_us)
        print(f"-> Servo running at {self.cfg.servo_run_us} us")

    def stop(self) -> None:
        if not self.closed:
            self._set_pulse(self.cfg.servo_neutral_us)

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.stop()
        finally:
            self.closed = True
            if self.lgpio is not None and self.handle is not None:
                try:
                    self.lgpio.gpio_free(self.handle, self.cfg.servo_pin)
                except Exception:
                    pass
                self.lgpio.gpiochip_close(self.handle)
                self.handle = None


def build_servo(cfg: Config) -> Servo:
    if cfg.mode != "real" or not cfg.servo_enabled:
        return MockServo()
    return RealServo(cfg)


class DummyVL53L4CD:
    def open(self) -> None:
        print("-> Dummy VL53L4CD ready")

    def read_mm(self, angle_deg: float, is_after: bool) -> float:
        angle = angle_deg % 360.0
        theta = math.radians(angle)

        distance = 205.0
        distance += 20.0 * math.sin(4.0 * theta)
        distance += 9.0 * math.cos(7.0 * theta)

        # Existing surface features.
        distance += gaussian_angle(angle, 72.0, 5.0, 12.0)
        distance += gaussian_angle(angle, 188.0, 9.0, 8.0)
        distance += gaussian_angle(angle, 306.0, 4.5, 16.0)

        if is_after:
            # Positive change means the sensor sees the surface farther away:
            # this is a deeper dent/slot for a fixed sensor.
            distance += gaussian_angle(angle, 118.0, 12.0, 34.0)
            distance += gaussian_angle(angle, 248.0, 9.0, 22.0)
            distance += gaussian_angle(angle, 190.0, 10.0, -6.0)

        distance += random.gauss(0.0, 3.0 if not is_after else 4.0)

        if random.random() < 0.01:
            distance += random.choice((-35.0, 45.0))

        return float(max(40.0, min(900.0, distance)))

    def close(self) -> None:
        pass


class AdafruitVL53L4CD:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.sensor = None

    def open(self) -> None:
        import board
        import busio
        import adafruit_vl53l4cd

        print("-> Initializing real VL53L4CD via Adafruit driver")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl53l4cd.VL53L4CD(i2c)

        try:
            model_id, module_type = self.sensor.model_info
            print(f"-> Sensor model_id=0x{model_id:02X}, module_type=0x{module_type:02X}")
            if model_id != 0xEB or module_type != 0xCC:
                message = (
                    "Sensor identity is not genuine VL53L4CD "
                    "(expected model_id=0xEB, module_type=0xCC)."
                )
                if self.cfg.strict_identity:
                    raise RuntimeError(message)
                print(f"[WARN] {message} Continuing because clone mode is allowed.")
        except Exception as exc:
            if self.cfg.strict_identity:
                raise
            print(f"[WARN] Could not verify VL53L4CD identity: {exc}. Continuing.")

        try:
            self.sensor.timing_budget = 20
            self.sensor.inter_measurement = 0
        except Exception:
            pass

        self.sensor.start_ranging()
        print("-> Real VL53L4CD ranging started")

    def read_mm(self, angle_deg: float, is_after: bool) -> float | None:
        del angle_deg, is_after
        if self.sensor is None:
            raise RuntimeError("Sensor is not open")

        deadline = time.perf_counter() + 0.15
        while not self.sensor.data_ready:
            if time.perf_counter() >= deadline:
                return None
            time.sleep(0.001)

        self.sensor.clear_interrupt()
        if getattr(self.sensor, "range_status", 0) != 0:
            return None

        # Adafruit's VL53L4CD distance property is in centimeters.
        return sanitize_distance(float(self.sensor.distance) * 10.0)

    def close(self) -> None:
        if self.sensor is not None:
            try:
                self.sensor.stop_ranging()
            except Exception:
                pass


class SmbusRegisterVL53:
    """
    Minimal direct-register reader for fake/clone "UL53LDK" style modules.

    This intentionally preserves the older pure-smbus approach because many
    fake modules expose a simple distance register even when they do not behave
    like a genuine ST VL53L4CD.
    """

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.bus = None

    def open(self) -> None:
        import smbus

        self.bus = smbus.SMBus(self.cfg.i2c_bus)
        print(f"-> smbus opened on /dev/i2c-{self.cfg.i2c_bus}")
        if self.cfg.distance_reg is None:
            print("-> clone register auto mode: trying 0x14 then 0x1E on each read")
        else:
            print(f"-> clone distance register: 0x{self.cfg.distance_reg:02X}")

    def read_mm(self, angle_deg: float, is_after: bool) -> float | None:
        del angle_deg, is_after
        if self.bus is None:
            raise RuntimeError("I2C bus is not open")

        registers = [0x14, 0x1E] if self.cfg.distance_reg is None else [self.cfg.distance_reg]
        last_error = None
        for register in registers:
            try:
                data = self.bus.read_i2c_block_data(self.cfg.i2c_addr, register, 2)
                dist = (data[0] << 8) | data[1]
                valid = sanitize_distance(dist)
                if valid is not None:
                    return valid
            except Exception as exc:
                last_error = exc

        if last_error is not None:
            print(f"\n[WARN] I2C read error: {last_error}")
        return None

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass


def sanitize_distance(value: object) -> float | None:
    if value is None:
        return None
    try:
        dist = float(value)
    except Exception:
        return None
    if dist == 8191 or dist < 20 or dist > 2000:
        return None
    return dist


class MockFan:
    def __init__(self) -> None:
        self.is_on = False

    def update(self) -> str:
        return "MOCK/OFF"

    def off(self) -> None:
        self.is_on = False


class RealFan:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.fan = None
        self.cpu = None

        if not cfg.fan_enabled:
            return

        try:
            from gpiozero import CPUTemperature, Device, OutputDevice

            try:
                from gpiozero.pins.lgpio import LGPIOFactory

                Device.pin_factory = LGPIOFactory()
            except Exception as exc:
                print(f"[WARN] LGPIOFactory unavailable, trying gpiozero default: {exc}")

            self.fan = OutputDevice(cfg.fan_pin, active_high=True, initial_value=False)
            self.cpu = CPUTemperature()
        except Exception as exc:
            print(f"[WARN] Fan disabled: {exc}")

    def update(self) -> str:
        if self.fan is None or self.cpu is None:
            return "OFF"
        temp_c = float(self.cpu.temperature)
        if temp_c >= self.cfg.fan_on_temp_c:
            self.fan.on()
            return f"ON ({temp_c:.1f} C)"
        if temp_c <= self.cfg.fan_off_temp_c:
            self.fan.off()
            return f"OFF ({temp_c:.1f} C)"
        return f"unchanged ({temp_c:.1f} C)"

    def off(self) -> None:
        if self.fan is not None:
            self.fan.off()


def gaussian_angle(angle: float, center: float, sigma: float, amplitude: float) -> float:
    delta = ((angle - center + 180.0) % 360.0) - 180.0
    return amplitude * math.exp(-0.5 * (delta / sigma) ** 2)


def build_sensor(cfg: Config) -> DistanceSensor:
    if cfg.mode == "dummy":
        sensor = DummyVL53L4CD()
        sensor.open()
        return sensor

    backends = ["vl53l4cd", "clone"] if cfg.backend == "auto" else [cfg.backend]
    last_error: Exception | None = None

    for backend in backends:
        normalized_backend = "vl53l4cd" if backend == "adafruit" else backend
        sensor: DistanceSensor
        if normalized_backend == "vl53l4cd":
            sensor = AdafruitVL53L4CD(cfg)
        elif normalized_backend in {"clone", "smbus"}:
            sensor = SmbusRegisterVL53(cfg)
        else:
            raise RuntimeError(f"Unsupported backend: {backend}")

        try:
            sensor.open()
            return sensor
        except Exception as exc:
            last_error = exc
            print(f"[WARN] {backend} backend failed: {exc}")

    if cfg.strict_hardware:
        raise RuntimeError(f"All hardware backends failed: {last_error}")

    print("-> Falling back to dummy mode")
    dummy = DummyVL53L4CD()
    dummy.open()
    return dummy


def angle_for_sample(index: int, elapsed_s: float, cfg: Config) -> float:
    if cfg.rotation_time_s and cfg.rotation_time_s > 0:
        return (elapsed_s % cfg.rotation_time_s) * 360.0 / cfg.rotation_time_s
    return (index * 360.0 / cfg.target_points) % 360.0


def run_scan(
    sensor: DistanceSensor,
    servo: Servo,
    cfg: Config,
    keys: KeyReader,
    name: str,
    is_after: bool,
    duration_s: float | None,
    auto: bool,
) -> pd.DataFrame:
    rows: list[dict[str, float | str]] = []
    start = time.perf_counter()
    index = 0

    if duration_s is None:
        print(f"\n{name.upper()} scan started - target {cfg.target_points} points")
    else:
        print(f"\n{name.upper()} scan started - running {duration_s:.2f} s")

    servo.start()
    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start

            if duration_s is not None and elapsed >= duration_s:
                break
            if duration_s is None and len(rows) >= cfg.target_points:
                break

            angle = angle_for_sample(index, elapsed, cfg)
            dist = sensor.read_mm(angle, is_after=is_after)

            if dist is not None:
                rows.append(
                    {
                        "scan": name,
                        "timestamp_s": elapsed,
                        "angle_deg": angle,
                        "distance_mm": dist,
                    }
                )
                print(
                    f"\r  {len(rows):3d} pts | {dist:7.2f} mm | {angle:6.1f} deg",
                    end="",
                    flush=True,
                )

            if not auto and keys.has_key(0.0):
                keys.read_key()
                print("\nManual stop")
                break

            index += 1
            time.sleep(cfg.read_interval_s)
    finally:
        servo.stop()
        print("\n-> Servo stopped")

    total = time.perf_counter() - start
    print(f"\n{name.upper()} finished - {len(rows)} points in {total:.2f} s")
    return pd.DataFrame(rows)


def robust_filter_scan(df: pd.DataFrame, cfg: Config) -> pd.DataFrame:
    if df.empty:
        return df
    values = df["distance_mm"].to_numpy(dtype=float)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_sigma = 1.4826 * mad if mad > 0 else float(np.std(values))
    limit = max(cfg.outlier_threshold_mm, 3.5 * robust_sigma)
    keep = np.abs(values - median) <= limit
    return df.loc[keep].reset_index(drop=True)


def circular_smooth(values: np.ndarray, window: int) -> np.ndarray:
    if len(values) < 3 or window <= 1:
        return values
    if window % 2 == 0:
        window += 1
    pad = window // 2
    extended = np.concatenate([values[-pad:], values, values[:pad]])
    kernel = np.ones(window, dtype=float) / window
    smoothed = np.convolve(extended, kernel, mode="same")
    return smoothed[pad : pad + len(values)]


def interpolate_to_grid(df: pd.DataFrame, cfg: Config) -> np.ndarray:
    filtered = robust_filter_scan(df, cfg)
    if filtered.empty:
        raise RuntimeError("No valid scan points after filtering")

    angles = filtered["angle_deg"].to_numpy(dtype=float) % 360.0
    values = filtered["distance_mm"].to_numpy(dtype=float)
    order = np.argsort(angles)
    angles = angles[order]
    values = values[order]

    unique_angles, unique_idx = np.unique(np.round(angles, 6), return_index=True)
    values = values[unique_idx]
    angles = unique_angles

    grid = np.linspace(0.0, 360.0, cfg.target_points, endpoint=False)
    x_ext = np.concatenate([angles - 360.0, angles, angles + 360.0])
    y_ext = np.concatenate([values, values, values])
    interpolated = np.interp(grid, x_ext, y_ext)
    return circular_smooth(interpolated, cfg.smooth_window)


def align_after_to_before(
    before: np.ndarray,
    after: np.ndarray,
    max_shift_deg: float,
) -> tuple[np.ndarray, int, float]:
    n = min(len(before), len(after))
    before = before[:n]
    after = after[:n]
    a = before - before.mean()
    b = after - after.mean()

    best_shift = 0
    best_score = -float("inf")
    max_shift = max(0, min(n // 2, int(round(abs(max_shift_deg) * n / 360.0))))
    candidate_shifts = range(-max_shift, max_shift + 1)

    for shift in candidate_shifts:
        score = float(np.dot(a, np.roll(b, shift)))
        if score > best_score:
            best_score = score
            best_shift = shift % n

    aligned = np.roll(after, best_shift)
    shift_deg = best_shift * 360.0 / n
    if shift_deg > 180.0:
        shift_deg -= 360.0
    return aligned, best_shift, shift_deg


def build_profile(before_raw: pd.DataFrame, after_raw: pd.DataFrame, cfg: Config) -> tuple[pd.DataFrame, float]:
    before = interpolate_to_grid(before_raw, cfg)
    after = interpolate_to_grid(after_raw, cfg)
    after_aligned, _, shift_deg = align_after_to_before(before, after, cfg.max_align_shift_deg)
    deviation = after_aligned - before
    angles = np.linspace(0.0, 360.0, len(before), endpoint=False)

    profile = pd.DataFrame(
        {
            "point": np.arange(1, len(before) + 1),
            "angle_deg": angles,
            "before_mm": before,
            "after_aligned_mm": after_aligned,
            "deviation_mm": deviation,
        }
    )
    return profile, shift_deg


def plot_shape(points: np.ndarray, title: str, filename: str) -> None:
    if len(points) < 3:
        return

    theta = np.linspace(0.0, 2.0 * np.pi, len(points), endpoint=False)
    x = points * np.cos(theta)
    y = points * np.sin(theta)

    plt.figure(figsize=(8, 8))
    plt.plot(x, y, color="#2563eb", lw=2)
    plt.fill(x, y, color="#93c5fd", alpha=0.35)
    plt.title(title)
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()
    print(f"Saved: {Path(filename).resolve()}")


def plot_comparison(profile: pd.DataFrame, cfg: Config) -> None:
    x = profile["angle_deg"]
    deviation = profile["deviation_mm"]

    plt.figure(figsize=(13, 9))

    plt.subplot(2, 1, 1)
    plt.plot(x, profile["before_mm"], color="#2563eb", lw=2, label="Before")
    plt.plot(x, profile["after_aligned_mm"], color="#dc2626", lw=2, ls="--", label="After aligned")
    plt.ylabel("Distance (mm)")
    plt.xlim(0, 360)
    plt.grid(alpha=0.3)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(x, deviation, color="#16a34a", lw=2)
    plt.fill_between(x, 0, deviation, color="#86efac", alpha=0.5)
    plt.axhline(0, color="black", ls="--", alpha=0.65)
    plt.axhline(cfg.deviation_threshold_mm, color="#dc2626", ls="--", alpha=0.8)
    plt.axhline(-cfg.deviation_threshold_mm, color="#f59e0b", ls="--", alpha=0.8)
    plt.title(f"Change detection - avg {deviation.mean():+.2f} mm")
    plt.xlabel("Angle (deg)")
    plt.ylabel("After - before (mm)")
    plt.xlim(0, 360)
    plt.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(cfg.graph_compare, dpi=200)
    plt.close()
    print(f"Saved: {Path(cfg.graph_compare).resolve()}")


def make_3d_cloud(profile: pd.DataFrame, cfg: Config) -> pd.DataFrame:
    rows = []
    layers = max(2, cfg.display_layers)

    for layer in range(layers):
        z = cfg.display_height_mm * layer / (layers - 1)
        axial_texture = 0.35 * math.sin(layer * 0.55)

        for _, row in profile.iterrows():
            theta = math.radians(float(row["angle_deg"]))
            # Fixed sensor outside object: larger distance means smaller surface radius.
            radius = cfg.sensor_to_axis_mm - float(row["after_aligned_mm"]) + axial_texture
            rows.append(
                {
                    "x": radius * math.cos(theta),
                    "y": radius * math.sin(theta),
                    "z": z,
                    "angle_deg": row["angle_deg"],
                    "radius_mm": radius,
                    "deviation_mm": row["deviation_mm"],
                }
            )

    return pd.DataFrame(rows)


def save_dashboard(profile: pd.DataFrame, cfg: Config, shift_deg: float) -> None:
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except Exception as exc:
        print(f"[WARN] Plotly unavailable, HTML dashboard skipped: {exc}")
        return

    cloud = make_3d_cloud(profile, cfg)
    fig = make_subplots(
        rows=2,
        cols=2,
        specs=[
            [{"type": "scene", "rowspan": 2}, {"type": "xy"}],
            [None, {"type": "xy"}],
        ],
        column_widths=[0.56, 0.44],
        row_heights=[0.52, 0.48],
        horizontal_spacing=0.08,
        vertical_spacing=0.12,
        subplot_titles=(
            "3D scan map",
            "Before vs after aligned",
            "Deviation: after - before",
        ),
    )

    fig.add_trace(
        go.Scatter3d(
            x=cloud["x"],
            y=cloud["y"],
            z=cloud["z"],
            mode="markers",
            marker={
                "size": 2.5,
                "color": cloud["deviation_mm"],
                "colorscale": "RdBu",
                "reversescale": True,
                "opacity": 0.84,
                "colorbar": {"title": "Deviation mm", "x": 0.50},
            },
            customdata=np.column_stack([cloud["angle_deg"], cloud["deviation_mm"]]),
            hovertemplate=(
                "Angle=%{customdata[0]:.1f} deg<br>"
                "Z=%{z:.2f} mm<br>"
                "Deviation=%{customdata[1]:+.2f} mm<extra></extra>"
            ),
            name="3D map",
        ),
        row=1,
        col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=profile["angle_deg"],
            y=profile["before_mm"],
            mode="lines",
            line={"width": 2, "color": "#2563eb"},
            name="Before",
        ),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(
            x=profile["angle_deg"],
            y=profile["after_aligned_mm"],
            mode="lines",
            line={"width": 2, "color": "#dc2626", "dash": "dash"},
            name="After aligned",
        ),
        row=1,
        col=2,
    )

    colors = np.where(profile["deviation_mm"] >= 0, "#dc2626", "#f59e0b")
    fig.add_trace(
        go.Bar(
            x=profile["angle_deg"],
            y=profile["deviation_mm"],
            marker={"color": colors},
            name="Deviation",
            hovertemplate="Angle=%{x:.1f} deg<br>Deviation=%{y:+.2f} mm<extra></extra>",
        ),
        row=2,
        col=2,
    )

    for y, color, dash in (
        (0.0, "#111827", "solid"),
        (cfg.deviation_threshold_mm, "#dc2626", "dash"),
        (-cfg.deviation_threshold_mm, "#f59e0b", "dash"),
    ):
        fig.add_trace(
            go.Scatter(
                x=[0, 360],
                y=[y, y],
                mode="lines",
                line={"width": 1, "color": color, "dash": dash},
                hoverinfo="skip",
                showlegend=False,
            ),
            row=2,
            col=2,
        )

    fig.update_layout(
        title={
            "text": f"Drop Test Dashboard - alignment shift {shift_deg:+.1f} deg",
            "x": 0.5,
            "xanchor": "center",
        },
        template="plotly_white",
        height=900,
        margin={"l": 20, "r": 28, "t": 86, "b": 36},
        legend={"orientation": "h", "y": 1.02, "x": 1.0, "xanchor": "right"},
    )
    fig.update_scenes(xaxis_title="X mm", yaxis_title="Y mm", zaxis_title="Z mm", aspectmode="data")
    fig.update_xaxes(title_text="Angle deg", range=[0, 360], row=1, col=2)
    fig.update_yaxes(title_text="Distance mm", row=1, col=2)
    fig.update_xaxes(title_text="Angle deg", range=[0, 360], row=2, col=2)
    fig.update_yaxes(title_text="Deviation mm", row=2, col=2)

    fig.write_html(cfg.dashboard_html, include_plotlyjs="cdn", full_html=True)
    print(f"Saved: {Path(cfg.dashboard_html).resolve()}")


def save_data(before_raw: pd.DataFrame, after_raw: pd.DataFrame, profile: pd.DataFrame, cfg: Config, shift_deg: float) -> None:
    raw = pd.concat([before_raw, after_raw], ignore_index=True)
    raw.to_csv(cfg.raw_csv, index=False)
    profile.to_csv(cfg.profile_csv, index=False)

    payload = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "mode": cfg.mode,
        "backend": cfg.backend,
        "target_points": cfg.target_points,
        "alignment_shift_deg": shift_deg,
        "avg_change_mm": float(profile["deviation_mm"].mean()),
        "max_deeper_mm": float(profile["deviation_mm"].max()),
        "max_shallower_mm": float(profile["deviation_mm"].min()),
        "profile": profile.to_dict(orient="list"),
    }
    Path(cfg.data_json).write_text(json.dumps(payload, indent=2), encoding="utf-8")

    with open(cfg.log_file, "a", encoding="utf-8") as log:
        log.write(f"\n=== {datetime.now():%Y-%m-%d %H:%M:%S} | MODE={cfg.mode.upper()} ===\n")
        log.write(f"Raw CSV: {Path(cfg.raw_csv).resolve()}\n")
        log.write(f"Profile CSV: {Path(cfg.profile_csv).resolve()}\n")
        log.write(f"JSON: {Path(cfg.data_json).resolve()}\n")
        log.write(f"Dashboard: {Path(cfg.dashboard_html).resolve()}\n")
        log.write(f"Alignment shift: {shift_deg:+.2f} deg\n")
        log.write(f"Avg change: {profile['deviation_mm'].mean():+.3f} mm\n")
        log.write(f"Max deeper: {profile['deviation_mm'].max():+.3f} mm\n")
        log.write(f"Max shallower: {profile['deviation_mm'].min():+.3f} mm\n")

    print(f"Saved: {Path(cfg.raw_csv).resolve()}")
    print(f"Saved: {Path(cfg.profile_csv).resolve()}")
    print(f"Saved: {Path(cfg.data_json).resolve()}")
    print(f"Log:   {Path(cfg.log_file).resolve()}")


def print_summary(profile: pd.DataFrame, shift_deg: float) -> None:
    deviation = profile["deviation_mm"]
    print("\n" + "-" * 60)
    print("RESULTS")
    print("-" * 60)
    print(f"Points compared       : {len(profile)}")
    print(f"Alignment correction  : {shift_deg:+.2f} deg")
    print(f"Avg change            : {deviation.mean():+7.2f} mm")
    print(f"Max deeper/farther    : {deviation.max():+7.2f} mm")
    print(f"Max shallower/closer  : {deviation.min():+7.2f} mm")
    print(f"Peak absolute change  : {deviation.abs().max():+7.2f} mm")


def parse_args() -> Config:
    parser = argparse.ArgumentParser(description="Refined drop test scanner")
    parser.add_argument("--mode", choices=["dummy", "real"], default=DEFAULT_MODE)
    parser.add_argument(
        "--backend",
        choices=["auto", "vl53l4cd", "adafruit", "clone", "smbus"],
        default="auto",
        help="auto tries official VL53L4CD first, then clone smbus. 'clone' is best for UL53LDK-style fake modules.",
    )
    parser.add_argument("--strict-hardware", action="store_true")
    parser.add_argument("--strict-identity", action="store_true", help="Reject non-genuine VL53L4CD identity values")
    parser.add_argument("--auto-demo", action="store_true", help="Run without keypress prompts")
    parser.add_argument("--target-points", type=int, default=360)
    parser.add_argument("--read-interval", type=float, default=0.20)
    parser.add_argument("--smooth-window", type=int, default=7)
    parser.add_argument("--outlier-threshold", type=float, default=50.0)
    parser.add_argument("--rotation-time", type=float, default=0.0, help="Seconds per 360 deg; 0 uses point index")
    parser.add_argument("--no-servo", action="store_true", help="Disable servo output in real mode")
    parser.add_argument("--servo-pin", type=int, default=18, help="Servo signal BCM GPIO (default: 18)")
    parser.add_argument("--servo-run-us", type=int, default=1350, help="Continuous rotation pulse in microseconds")
    parser.add_argument("--servo-neutral-us", type=int, default=1500, help="Servo stop pulse in microseconds")
    parser.add_argument("--max-align-shift", type=float, default=45.0, help="Maximum before/after circular alignment shift in degrees")
    parser.add_argument("--distance-reg", default="auto", help="Clone/smbus distance register: auto, 0x14, or 0x1E")
    parser.add_argument("--no-fan", action="store_true")
    parser.add_argument("--dashboard", default="drop_scan_dashboard.html")
    args = parser.parse_args()

    return Config(
        mode=args.mode,
        backend=args.backend,
        strict_hardware=args.strict_hardware,
        strict_identity=args.strict_identity,
        auto_demo=args.auto_demo,
        target_points=args.target_points,
        read_interval_s=args.read_interval,
        smooth_window=args.smooth_window,
        outlier_threshold_mm=args.outlier_threshold,
        max_align_shift_deg=args.max_align_shift,
        rotation_time_s=args.rotation_time if args.rotation_time > 0 else None,
        servo_enabled=not args.no_servo,
        servo_pin=args.servo_pin,
        servo_run_us=args.servo_run_us,
        servo_neutral_us=args.servo_neutral_us,
        distance_reg=None if str(args.distance_reg).lower() == "auto" else int(str(args.distance_reg), 16),
        fan_enabled=not args.no_fan,
        dashboard_html=args.dashboard,
    )


def main() -> int:
    cfg = parse_args()
    keys = KeyReader()
    fan = RealFan(cfg) if cfg.mode == "real" else MockFan()
    sensor = build_sensor(cfg)
    servo = build_servo(cfg)

    print("\n" + "=" * 78)
    print("   DROP TEST - REFINED REAL PI / DUMMY VERSION")
    print("=" * 78)
    print(f"Mode: {cfg.mode.upper()} | backend: {cfg.backend} | target: {cfg.target_points} points")
    print(f"Fan: {fan.update()}")

    before_raw = pd.DataFrame()
    after_raw = pd.DataFrame()

    try:
        if cfg.mode == "dummy":
            # build_sensor already opens dummy. This call is harmless but avoids double open.
            pass

        if cfg.mode == "real":
            # build_sensor opened hardware already.
            pass

        if cfg.auto_demo:
            before_raw = run_scan(sensor, servo, cfg, keys, "before", False, None, auto=True)
            duration = max(0.1, float(before_raw["timestamp_s"].max()) if not before_raw.empty else cfg.target_points * cfg.read_interval_s)
            after_raw = run_scan(sensor, servo, cfg, keys, "after", True, duration, auto=True)
        else:
            keys.wait_for("1", "\nPress 1 to start BEFORE scan")
            before_raw = run_scan(sensor, servo, cfg, keys, "before", False, None, auto=False)

            duration = max(0.1, float(before_raw["timestamp_s"].max()) if not before_raw.empty else 0.0)
            print("\nDo the drop now.")
            keys.wait_for("2", f"Press 2 to start AFTER scan ({duration:.2f} s)")
            after_raw = run_scan(sensor, servo, cfg, keys, "after", True, duration, auto=False)

        profile, shift_deg = build_profile(before_raw, after_raw, cfg)
        print_summary(profile, shift_deg)

        plot_shape(profile["before_mm"].to_numpy(), "Before Drop Shape", cfg.graph_original)
        plot_shape(profile["after_aligned_mm"].to_numpy(), "After Drop Shape", cfg.graph_after)
        plot_comparison(profile, cfg)
        save_dashboard(profile, cfg, shift_deg)
        save_data(before_raw, after_raw, profile, cfg, shift_deg)

    except KeyboardInterrupt:
        print("\nStopped by user")
        return 130
    except Exception as exc:
        print(f"\nError: {exc}")
        return 1
    finally:
        servo.close()
        fan.off()
        sensor.close()
        print("Fan OFF")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

