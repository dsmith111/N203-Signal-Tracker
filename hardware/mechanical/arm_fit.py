"""Forearm cross-sections that shape the bracer's arm brace.

Each station is ``(distance from the wrist end, arm width, arm height)`` in
millimetres.  ``REFERENCE_ARM_STATIONS`` is a nominal adult forearm, not a
measurement of anyone in particular.  The brace only fits an arm close to the
stations it was generated from.

To fit your own arm, copy ``arm_fit.example.json`` to ``arm_fit.local.json``
in this directory and put your measurements in it.  That file is ignored by
git.  When it exists, every bracer generator uses it and writes to
``models/local/`` instead of ``models/``, so the preview images kept in the
repository always show the reference arm.  Pass ``--reference-arm`` on the
command line, or set ``N203_REFERENCE_ARM=1``, to ignore the local file.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path


REFERENCE_ARM_STATIONS = (
    (0.00, 58.00, 44.00),
    (85.00, 80.00, 72.00),
    (145.00, 95.00, 86.00),
)

LOCAL_FILE = Path(__file__).resolve().with_name("arm_fit.local.json")


def _reference_requested() -> bool:
    return "--reference-arm" in sys.argv or os.environ.get("N203_REFERENCE_ARM", "0") not in ("", "0")


def _load_local() -> tuple[tuple[float, float, float], ...]:
    data = json.loads(LOCAL_FILE.read_text(encoding="utf-8"))
    stations = tuple(tuple(float(value) for value in station) for station in data["stations_mm"])
    distances = [station[0] for station in stations]
    if (
        len(stations) < 2
        or any(len(station) != 3 for station in stations)
        or distances[0] != 0.0
        or any(b <= a for a, b in zip(distances, distances[1:]))
        or any(value <= 0.0 for station in stations for value in station[1:])
    ):
        raise ValueError(
            f"{LOCAL_FILE.name}: need at least two [distance, width, height] stations, "
            "starting at distance 0 and increasing, with positive sizes"
        )
    return stations


USING_LOCAL_ARM = LOCAL_FILE.is_file() and not _reference_requested()
ARM_STATIONS = _load_local() if USING_LOCAL_ARM else REFERENCE_ARM_STATIONS


def output_dir(here: Path) -> Path:
    """Where a bracer generator in directory ``here`` writes by default."""
    return here / "models" / "local" if USING_LOCAL_ARM else here / "models"


def describe() -> str:
    source = f"{LOCAL_FILE.name} (output goes to models/local/)" if USING_LOCAL_ARM else "the reference arm"
    sizes = ", ".join(f"{width:g} x {height:g} mm at {distance:g}" for distance, width, height in ARM_STATIONS)
    return f"Arm fit from {source}: {sizes}"
