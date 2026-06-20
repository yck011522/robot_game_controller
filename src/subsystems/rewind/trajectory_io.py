"""Compact gzip JSON persistence for recorded gameplay joint trajectories."""

from __future__ import annotations

import gzip
import json
import os
from pathlib import Path

from compas_fab.robots import JointTrajectory


TRAJECTORY_SCHEMA_VERSION = 1


def write_joint_trajectory_json_gz(
    trajectory: JointTrajectory, output_path: str | Path
) -> int:
    """Atomically write timestamp-plus-six-joint samples as gzip JSON.

    The trajectory timestamp is relative to play entry in seconds and joint
    targets are stored in radians. The returned integer is the sample count.
    This function is called once after each completed batch-game rewind.
    """

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    samples = [
        [
            float(point.time_from_start.seconds),
            *[float(value) for value in point.joint_values[:6]],
        ]
        for point in trajectory.points
    ]
    payload = {
        "schema_version": TRAJECTORY_SCHEMA_VERSION,
        "time_unit": "seconds_from_play_start",
        "joint_unit": "radians",
        "joint_names": list(trajectory.joint_names[:6]),
        "samples": samples,
    }
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    compressed = gzip.compress(encoded, compresslevel=6, mtime=0)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(compressed)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    return len(samples)
