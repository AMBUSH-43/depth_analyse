#!/usr/bin/env python3
"""Browser control panel for the Depth Analyse before/after scanner."""

from __future__ import annotations

import json
import threading
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import pandas as pd
from flask import Flask, jsonify, render_template, request, send_from_directory

from drop_test_refined import (
    Config,
    KeyReader,
    build_profile,
    build_sensor,
    build_servo,
    plot_comparison,
    plot_shape,
    print_summary,
    run_scan,
    save_dashboard,
    save_data,
)

ROOT = Path(__file__).resolve().parent
SCAN_ROOT = ROOT / "scans"
SCAN_ROOT.mkdir(exist_ok=True)

app = Flask(__name__)
lock = threading.Lock()
state: dict[str, Any] = {
    "stage": "idle",
    "message": "Ready for a new scan",
    "progress": 0,
    "points": 0,
    "live": [],
    "run_id": None,
    "error": None,
    "result": None,
    "before": None,
    "config": None,
}


def public_state() -> dict[str, Any]:
    with lock:
        return {k: v for k, v in state.items() if k not in {"before", "config"}}


def update(**values: Any) -> None:
    with lock:
        state.update(values)


def config_from_payload(payload: dict[str, Any], run_dir: Path) -> Config:
    mode = str(payload.get("mode", "dummy"))
    backend = str(payload.get("backend", "auto"))
    points = max(12, min(1440, int(payload.get("target_points", 360))))
    interval = max(0.01, min(5.0, float(payload.get("read_interval_s", 0.2))))
    rotation = float(payload.get("rotation_time_s", 0) or 0)
    return Config(
        mode=mode if mode in {"dummy", "real"} else "dummy",
        backend=backend if backend in {"auto", "vl53l0x", "vl53l4cd", "adafruit", "clone", "smbus"} else "auto",
        strict_hardware=bool(payload.get("strict_hardware", mode == "real")),
        target_points=points,
        read_interval_s=interval,
        rotation_time_s=rotation if rotation > 0 else None,
        servo_enabled=bool(payload.get("servo_enabled", True)),
        servo_pin=int(payload.get("servo_pin", 18)),
        servo_run_us=int(payload.get("servo_run_us", 1350)),
        servo_neutral_us=int(payload.get("servo_neutral_us", 1500)),
        fan_enabled=False,
        log_file=str(run_dir / "drop_shape_log.txt"),
        graph_original=str(run_dir / "before.png"),
        graph_after=str(run_dir / "after.png"),
        graph_compare=str(run_dir / "comparison.png"),
        data_json=str(run_dir / "scan_data.json"),
        raw_csv=str(run_dir / "scan_raw.csv"),
        profile_csv=str(run_dir / "scan_profile.csv"),
        dashboard_html=str(run_dir / "interactive_3d.html"),
    )


def progress_callback(reading: dict[str, float | str]) -> None:
    with lock:
        live = list(state["live"])
        live.append(reading)
        target = state["config"].target_points
        state.update(
            live=live[-500:],
            points=len(live),
            progress=min(100, round(len(live) * 100 / target)),
            message=f"Reading {len(live)} of {target}",
        )


def scan_worker(phase: str) -> None:
    sensor = servo = None
    try:
        with lock:
            cfg: Config = state["config"]
            before: pd.DataFrame | None = state["before"]
        update(stage=f"scanning_{phase}", message=f"{phase.title()} scan in progress", live=[], points=0, progress=0, error=None)
        sensor = build_sensor(cfg)
        servo = build_servo(cfg)
        frame = run_scan(sensor, servo, cfg, KeyReader(), phase, phase == "after", None, True, progress_callback)
        run_dir = Path(cfg.data_json).parent

        if phase == "before":
            frame.to_csv(run_dir / "before_raw.csv", index=False)
            update(
                stage="awaiting_drop",
                message="Before scan saved. Perform the drop, then start the after scan.",
                progress=100,
                before=frame,
            )
            return

        if before is None or before.empty:
            raise RuntimeError("Before scan data is missing")
        frame.to_csv(run_dir / "after_raw.csv", index=False)
        update(stage="processing", message="Building comparison and visualisations", progress=100)
        profile, shift_deg = build_profile(before, frame, cfg)
        print_summary(profile, shift_deg)
        plot_shape(profile["before_mm"].to_numpy(), "Before Drop Shape", cfg.graph_original)
        plot_shape(profile["after_aligned_mm"].to_numpy(), "After Drop Shape", cfg.graph_after)
        plot_comparison(profile, cfg)
        save_dashboard(profile, cfg, shift_deg)
        save_data(before, frame, profile, cfg, shift_deg)
        result = {
            "run_id": state["run_id"],
            "alignment_shift_deg": round(float(shift_deg), 3),
            "avg_change_mm": round(float(profile["deviation_mm"].mean()), 3),
            "max_deeper_mm": round(float(profile["deviation_mm"].max()), 3),
            "max_shallower_mm": round(float(profile["deviation_mm"].min()), 3),
            "files": ["before.png", "after.png", "comparison.png", "interactive_3d.html", "scan_data.json", "scan_raw.csv", "scan_profile.csv"],
        }
        update(stage="complete", message="Analysis complete", result=result, progress=100)
    except Exception as exc:
        update(stage="error", message="Scan failed", error=str(exc))
    finally:
        if servo is not None:
            servo.close()
        if sensor is not None:
            sensor.close()


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/status")
def status():
    return jsonify(public_state())


@app.post("/api/before")
def start_before():
    payload = request.get_json(silent=True) or {}
    with lock:
        if str(state["stage"]).startswith("scanning") or state["stage"] == "processing":
            return jsonify({"error": "A scan is already running"}), 409
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")[:-3]
    run_dir = SCAN_ROOT / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    cfg = config_from_payload(payload, run_dir)
    (run_dir / "settings.json").write_text(json.dumps(asdict(cfg), indent=2), encoding="utf-8")
    update(run_id=run_id, config=cfg, before=None, result=None, error=None)
    threading.Thread(target=scan_worker, args=("before",), daemon=True).start()
    return jsonify({"ok": True, "run_id": run_id})


@app.post("/api/after")
def start_after():
    with lock:
        if state["stage"] != "awaiting_drop":
            return jsonify({"error": "Complete a before scan first"}), 409
    threading.Thread(target=scan_worker, args=("after",), daemon=True).start()
    return jsonify({"ok": True})


@app.get("/api/runs")
def runs():
    items = []
    for folder in sorted(SCAN_ROOT.iterdir(), reverse=True):
        if not folder.is_dir():
            continue
        data = {}
        result_file = folder / "scan_data.json"
        if result_file.exists():
            try:
                data = json.loads(result_file.read_text(encoding="utf-8"))
            except Exception:
                pass
        items.append({"run_id": folder.name, "complete": result_file.exists(), "summary": data})
    return jsonify(items[:50])


@app.get("/runs/<run_id>/<path:filename>")
def run_file(run_id: str, filename: str):
    return send_from_directory(SCAN_ROOT / run_id, filename)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
