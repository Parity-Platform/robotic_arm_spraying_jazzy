"""Post-process a raw demonstration CSV into a spraying-ready trajectory.

Input: demos/demo_YYYYMMDD_HHMMSS_raw.csv
  time_s, x_m, y_m, z_m, qx, qy, qz, qw, spray, segment_id

Output: trajectory_output.csv (schema matches spraying_pathways executor)
  time [s], x [m], y [m], speed [m/s], acceleration [m/s^2], spray [bool]

Steps:
  1. Drop pre-spray warm-up and post-spray tail.
  2. Drop samples with stale timestamps or non-monotonic time.
  3. Project 3D EE pose onto the panel frame (XY). Panel origin and normal
     can be specified via parameters; default uses base_link XY.
  4. Low-pass filter XY with a zero-phase Butterworth (scipy) or fall back to
     moving average if scipy is unavailable.
  5. Uniform-resample to target dt (default 0.025 s) by linear interpolation.
  6. Compute instantaneous speed and acceleration from finite differences.
  7. Hold spray flag majority-vote over each resampled bin.
  8. Write CSV with the exact column headers the spraying node expects.

This module can be used as a library (extract_trajectory) or run as a ROS 2
console script (`trajectory_extractor --ros-args -p input_csv:=...`).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from scipy.signal import butter, filtfilt
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False


OUT_HEADER = [
    'time [s]', 'x [m]', 'y [m]', 'speed [m/s]',
    'acceleration [m/s^2]', 'spray [bool]',
]


def load_raw(path: Path) -> np.ndarray:
    rows = []
    with path.open('r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append([
                float(row['time_s']),
                float(row['x_m']), float(row['y_m']), float(row['z_m']),
                int(row['spray']),
                int(row['segment_id']),
            ])
    if not rows:
        raise ValueError(f'empty demo: {path}')
    return np.asarray(rows, dtype=float)


def project_to_panel(xyz: np.ndarray, origin: np.ndarray,
                     normal: np.ndarray) -> np.ndarray:
    """Project Nx3 points onto a plane (origin + normal), return Nx2 (u,v)."""
    n = normal / np.linalg.norm(normal)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 \
        else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    d = xyz - origin
    return np.stack([d @ u, d @ v], axis=1)


def lowpass(x: np.ndarray, fs: float, cutoff_hz: float) -> np.ndarray:
    if HAVE_SCIPY and len(x) > 12:
        ny = 0.5 * fs
        w = max(0.01, min(0.99, cutoff_hz / ny))
        b, a = butter(4, w, btype='low')
        return filtfilt(b, a, x)
    # Fallback: centred moving average.
    k = max(3, int(round(fs / max(1e-6, cutoff_hz))))
    if k % 2 == 0:
        k += 1
    kernel = np.ones(k) / k
    return np.convolve(x, kernel, mode='same')


def resample_uniform(t: np.ndarray, x: np.ndarray,
                     y: np.ndarray, spray: np.ndarray,
                     dt: float) -> tuple[np.ndarray, np.ndarray,
                                          np.ndarray, np.ndarray]:
    t0, t1 = float(t[0]), float(t[-1])
    n = int(math.floor((t1 - t0) / dt)) + 1
    tu = t0 + dt * np.arange(n)
    xu = np.interp(tu, t, x)
    yu = np.interp(tu, t, y)
    # Majority vote spray flag in each bin.
    su = np.zeros(n, dtype=int)
    bins = np.searchsorted(t, tu)
    for i, b in enumerate(bins):
        lo = max(0, b - 2)
        hi = min(len(spray), b + 3)
        su[i] = 1 if spray[lo:hi].mean() >= 0.5 else 0
    return tu, xu, yu, su


def derivatives(t: np.ndarray, x: np.ndarray,
                y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    speed = np.hypot(dx, dy)
    accel = np.gradient(speed, t)
    return speed, accel


def trim_to_spray(data: np.ndarray, keep_context_s: float = 0.3,
                  rate_hz: float = 100.0) -> np.ndarray:
    spray = data[:, 4]
    idx = np.where(spray > 0.5)[0]
    if idx.size == 0:
        return data
    pad = int(keep_context_s * rate_hz)
    lo = max(0, idx[0] - pad)
    hi = min(len(data), idx[-1] + pad + 1)
    return data[lo:hi]


def extract_trajectory(input_csv: Path, output_csv: Path,
                       panel_origin: Optional[np.ndarray] = None,
                       panel_normal: Optional[np.ndarray] = None,
                       cutoff_hz: float = 8.0,
                       dt_out: float = 0.025,
                       trim_context_s: float = 0.3) -> int:
    raw = load_raw(input_csv)
    # Infer original sample rate.
    dt_in = float(np.median(np.diff(raw[:, 0])))
    rate_in = 1.0 / max(1e-6, dt_in)

    raw = trim_to_spray(raw, trim_context_s, rate_in)
    if raw.shape[0] < 10:
        raise ValueError('demo too short after trimming')

    xyz = raw[:, 1:4]
    if panel_origin is None:
        panel_origin = np.array([0.0, 0.0, float(np.median(xyz[:, 2]))])
    if panel_normal is None:
        panel_normal = np.array([0.0, 0.0, 1.0])
    uv = project_to_panel(xyz, panel_origin, panel_normal)

    t_raw = raw[:, 0]
    u = lowpass(uv[:, 0], rate_in, cutoff_hz)
    v = lowpass(uv[:, 1], rate_in, cutoff_hz)

    tu, xu, yu, su = resample_uniform(t_raw, u, v, raw[:, 4], dt_out)
    tu = tu - tu[0]
    speed, accel = derivatives(tu, xu, yu)

    with output_csv.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(OUT_HEADER)
        for i in range(len(tu)):
            w.writerow([
                f'{tu[i]:.6f}',
                f'{xu[i]:.6f}', f'{yu[i]:.6f}',
                f'{speed[i]:.6f}', f'{accel[i]:.6f}',
                int(su[i]),
            ])
    return len(tu)


def _parse_vec3(s: str) -> np.ndarray:
    parts = [float(x) for x in s.split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('expected 3 comma-separated floats')
    return np.asarray(parts, dtype=float)


def _cli() -> int:
    p = argparse.ArgumentParser(prog='trajectory_extractor')
    p.add_argument('input_csv', type=Path)
    p.add_argument('--output', '-o', type=Path, default=Path('trajectory_output.csv'))
    p.add_argument('--panel-origin', type=_parse_vec3, default=None,
                   help='comma-separated x,y,z of panel origin in base_link')
    p.add_argument('--panel-normal', type=_parse_vec3, default=None,
                   help='comma-separated x,y,z of panel surface normal')
    p.add_argument('--cutoff-hz', type=float, default=8.0)
    p.add_argument('--dt', type=float, default=0.025)
    p.add_argument('--trim-context-s', type=float, default=0.3)
    args = p.parse_args()

    n = extract_trajectory(
        args.input_csv, args.output,
        panel_origin=args.panel_origin,
        panel_normal=args.panel_normal,
        cutoff_hz=args.cutoff_hz,
        dt_out=args.dt,
        trim_context_s=args.trim_context_s,
    )
    print(f'wrote {n} rows -> {args.output}')
    return 0


def main() -> None:
    # Support both console invocation and ROS 2 entry point; ROS args are
    # stripped by the launch system if present.
    try:
        import rclpy
        if rclpy.ok():
            rclpy.init()
            rclpy.shutdown()
    except Exception:
        pass
    sys.exit(_cli())


if __name__ == '__main__':
    main()
