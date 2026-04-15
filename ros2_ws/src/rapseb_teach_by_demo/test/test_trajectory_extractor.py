"""Unit tests for the trajectory extractor.

Generates a synthetic raw demonstration, runs the extractor, and checks that
the output conforms to the schema used by the spraying executor.
"""
from __future__ import annotations

import csv
import math
import tempfile
from pathlib import Path

import numpy as np
import pytest

from rapseb_teach_by_demo.trajectory_extractor import (
    extract_trajectory, OUT_HEADER,
)


def _write_raw(path: Path, duration: float = 4.0, rate: float = 100.0) -> None:
    n = int(duration * rate)
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'time_s', 'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw', 'spray', 'segment_id',
        ])
        for i in range(n):
            t = i / rate
            # Zig-zag in XY, stationary Z, spray on in the middle half.
            x = 0.02 * math.sin(2 * math.pi * 0.5 * t) + 0.5
            y = 0.002 * t + 0.3
            z = 1.0
            spray = 1 if 0.25 * duration <= t <= 0.75 * duration else 0
            w.writerow([
                f'{t:.6f}', f'{x:.6f}', f'{y:.6f}', f'{z:.6f}',
                '0', '0', '0', '1', spray, 0,
            ])


def test_extract_synthetic_demo():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        raw = tmp / 'demo_raw.csv'
        out = tmp / 'trajectory_output.csv'
        _write_raw(raw)

        n = extract_trajectory(raw, out, dt_out=0.025, cutoff_hz=8.0)
        assert n > 50

        with out.open('r') as f:
            reader = csv.reader(f)
            header = next(reader)
            assert header == OUT_HEADER
            rows = [row for row in reader]
        assert len(rows) == n

        t = np.array([float(r[0]) for r in rows])
        x = np.array([float(r[1]) for r in rows])
        y = np.array([float(r[2]) for r in rows])
        speed = np.array([float(r[3]) for r in rows])
        spray = np.array([int(r[5]) for r in rows])

        # Time is monotonically increasing and starts at zero.
        assert t[0] == pytest.approx(0.0, abs=1e-6)
        assert (np.diff(t) > 0).all()

        # Speed is non-negative and realistic for the synthetic motion.
        assert (speed >= 0).all()
        assert speed.max() < 1.0

        # Spray flag dominates after trim_to_spray keeps only the active
        # window plus a small context pad on either side.
        frac = float(spray.sum()) / len(spray)
        assert 0.5 < frac < 0.95

        # The oscillation on the raw X axis survives projection onto the
        # panel (it ends up on the projected y axis for the default panel
        # normal [0,0,1]). Combined planar path length must be non-trivial.
        path_len = float(np.hypot(np.diff(x), np.diff(y)).sum())
        assert path_len > 0.05
