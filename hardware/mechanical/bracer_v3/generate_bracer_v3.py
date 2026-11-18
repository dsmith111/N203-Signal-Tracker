"""Generate and verify the N203 right-arm bracer, version 3.

Outputs, under ``models/``:

* ``n203_v3_assembly.step``           named assembly in the bracer frame
* ``print/n203_v3_<part>.step|.stl``  each printed part in build orientation
* ``n203_v3_manifest.json``           dimensions, bill of materials, checks

The script refuses to finish if a check fails.  Version 1 looked plausible in
a picture but could not be assembled; version 2 assembled in CAD but did not
survive a loosely tuned printer: thin walls, small posts that broke off,
self-tapped holes that closed up, and four display supports that started
printing in mid-air.  The "robust" and "fasteners" groups below exist so that
those faults cannot come back unnoticed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
import vtk
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

import v3_geometry as g


Vec = cq.Vector
BED = (220.0, 220.0, 250.0)  # Ender 3 Pro build volume
MAX_BRIDGE = 30.0  # longest unsupported span accepted without supports
STL_DEFLECTION = 0.02  # absolute chord error, mm
STL_ANGLE = 0.10
TOUCH_VOLUME = 0.02  # mm^3 treated as numerical noise
EPS_INTERFERENCE = 0.05


def bounds(shape: cq.Shape) -> dict[str, float]:
    bb = shape.BoundingBox()
    return {
        "min_x": round(bb.xmin, 3),
        "max_x": round(bb.xmax, 3),
        "min_y": round(bb.ymin, 3),
        "max_y": round(bb.ymax, 3),
        "min_z": round(bb.zmin, 3),
        "max_z": round(bb.zmax, 3),
        "size_x": round(bb.xlen, 3),
        "size_y": round(bb.ylen, 3),
        "size_z": round(bb.zlen, 3),
    }


def boxes_apart(a: cq.Shape, b: cq.Shape, margin: float = 0.0) -> bool:
    ba, bb = a.BoundingBox(), b.BoundingBox()
    return (
        ba.xmax + margin < bb.xmin
        or bb.xmax + margin < ba.xmin
        or ba.ymax + margin < bb.ymin
        or bb.ymax + margin < ba.ymin
        or ba.zmax + margin < bb.zmin
        or bb.zmax + margin < ba.zmin
    )


def overlap(a: cq.Shape, b: cq.Shape) -> float:
    if boxes_apart(a, b):
        return 0.0
    try:
        return abs(a.intersect(b).Volume())
    except Exception:
        return float("nan")


class Report:
    def __init__(self) -> None:
        self.rows: list[dict[str, object]] = []

    def add(self, group: str, name: str, ok: bool, detail: str) -> None:
        self.rows.append({"group": group, "check": name, "ok": bool(ok), "detail": detail})
        if not ok:
            print(f"  FAIL [{group}] {name}: {detail}")

    @property
    def failures(self) -> list[dict[str, object]]:
        return [row for row in self.rows if not row["ok"]]

    def summary(self) -> dict[str, dict[str, int]]:
        result: dict[str, dict[str, int]] = {}
        for row in self.rows:
            entry = result.setdefault(str(row["group"]), {"passed": 0, "failed": 0})
            entry["passed" if row["ok"] else "failed"] += 1
        return result


# --------------------------------------------------------------------------
# Scene in the tray frame
# --------------------------------------------------------------------------
def build_scene() -> dict[str, object]:
    printed = dict(g.printed_parts_tray_frame())
    printed["saddle"] = g.tray_frame().unplace(g.make_saddle())
    components = {item.name: item for item in g.component_items()}
    fasteners = {item.name: item for item in g.fastener_items()}
    return {
        "printed": printed,
        "components": components,
        "fasteners": fasteners,
        "keepouts": g.connector_keepouts(),
        "usb_plugs": g.usb_plug_keepouts(),
        "routes": g.wire_routes(),
        "nut_sites": g.side_nut_sites(),
    }


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------
def check_printed_parts(report: Report, plates: dict[str, cq.Shape]) -> None:
    for name, shape in plates.items():
        solids = len(shape.Solids())
        report.add("printable", f"{name} is one solid", solids == 1, f"{solids} solid(s)")
        report.add("printable", f"{name} is a valid B-rep", shape.isValid(), "OpenCascade validity")
        bb = shape.BoundingBox()
        fits = bb.xlen <= BED[0] and bb.ylen <= BED[1] and bb.zlen <= BED[2]
        report.add(
            "printable",
            f"{name} fits the 220 x 220 x 250 bed",
            fits,
            f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm in build orientation",
        )


def absolute_mesh(shape: cq.Shape, deflection: float = 0.03) -> tuple[np.ndarray, np.ndarray]:
    """Triangulate with an absolute chord error.

    CadQuery's own tessellate() scales its tolerance by feature size, which
    turns the long, gently curved arm surfaces into a few flat facets.
    """
    meshed = shape.copy()
    BRepMesh_IncrementalMesh(meshed.wrapped, deflection, False, 0.12, True)
    vertices, triangles = meshed.tessellate(1.0e3, 1.0e3)
    return (
        np.array([(v.x, v.y, v.z) for v in vertices], dtype=float),
        np.array(triangles, dtype=int),
    )


def check_overhangs(report: Report, plates: dict[str, cq.Shape]) -> dict[str, dict[str, float]]:
    """Find surfaces that face the bed more steeply than 45 degrees.

    Anything flagged must be a short bridge (a slot roof, an opening lintel,
    a blind-hole end).  A large flagged patch would need support material.
    """
    facts: dict[str, dict[str, float]] = {}
    for name, shape in plates.items():
        points, triangles = absolute_mesh(shape)
        a, b, c = (points[triangles[:, i]] for i in range(3))
        normal = np.cross(b - a, c - a)
        area = np.linalg.norm(normal, axis=1) / 2.0
        keep = area > 1.0e-9
        a, b, c, area = a[keep], b[keep], c[keep], area[keep]
        normal = normal[keep] / (2.0 * area[:, None])
        lowest = np.minimum.reduce([a[:, 2], b[:, 2], c[:, 2]])
        steep = (normal[:, 2] < -0.72) & (lowest > 0.25)
        worst_span, total = 0.0, float(area[steep].sum())
        if steep.any():
            corners = np.concatenate([a[steep], b[steep], c[steep]])
            owner = np.tile(np.arange(int(steep.sum())), 3)
            cells: dict[tuple[int, int, int], set[int]] = {}
            for key, index in zip(map(tuple, np.floor(corners / 2.5).astype(int)), owner):
                cells.setdefault(key, set()).add(int(index))
            parent = list(range(int(steep.sum())))

            def find(i: int) -> int:
                while parent[i] != i:
                    parent[i] = parent[parent[i]]
                    i = parent[i]
                return i

            for key, members in cells.items():
                neighbours: set[int] = set()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            neighbours |= cells.get((key[0] + dx, key[1] + dy, key[2] + dz), set())
                first = next(iter(members))
                for other in neighbours:
                    ra, rb = find(first), find(other)
                    if ra != rb:
                        parent[ra] = rb
            groups: dict[int, list[int]] = {}
            for index in range(int(steep.sum())):
                groups.setdefault(find(index), []).append(index)
            sa, sb, sc, sarea = a[steep], b[steep], c[steep], area[steep]
            for members in groups.values():
                if sarea[members].sum() < 4.0:
                    continue
                patch = np.concatenate([sa[members], sb[members], sc[members]])
                extent = patch.max(axis=0) - patch.min(axis=0)
                worst_span = max(worst_span, float(max(extent[0], extent[1])))
        facts[name] = {"steep_overhang_mm2": round(total, 1), "longest_bridge_mm": round(worst_span, 1)}
        report.add(
            "printable",
            f"{name} needs no support material",
            worst_span <= MAX_BRIDGE,
            f"{total:.0f} mm^2 beyond 45 deg, all in bridges up to {worst_span:.1f} mm"
            if total > 0.5
            else "no surface beyond 45 deg",
        )
    return facts


# --------------------------------------------------------------------------
# Robustness on a loosely tuned printer
# --------------------------------------------------------------------------
def welded_mesh(shape: cq.Shape) -> tuple[np.ndarray, np.ndarray]:
    """absolute_mesh() with the per-face duplicate vertices merged."""
    points, triangles = absolute_mesh(shape)
    keys = np.round(points / 1.0e-3).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    triangles = inverse.reshape(-1)[triangles]
    keep = (
        (triangles[:, 0] != triangles[:, 1])
        & (triangles[:, 1] != triangles[:, 2])
        & (triangles[:, 0] != triangles[:, 2])
    )
    return points[first], triangles[keep]


def cluster(points: np.ndarray, cell: float) -> list[np.ndarray]:
    """Group points whose grid cells touch; returns index arrays."""
    if len(points) == 0:
        return []
    cells: dict[tuple[int, int, int], list[int]] = {}
    for index, key in enumerate(map(tuple, np.floor(points / cell).astype(int))):
        cells.setdefault(key, []).append(index)
    seen: set[tuple[int, int, int]] = set()
    groups: list[np.ndarray] = []
    for start in cells:
        if start in seen:
            continue
        seen.add(start)
        stack, members = [start], []
        while stack:
            key = stack.pop()
            members.extend(cells[key])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        other = (key[0] + dx, key[1] + dy, key[2] + dz)
                        if other in cells and other not in seen:
                            seen.add(other)
                            stack.append(other)
        groups.append(np.array(members, dtype=int))
    return groups


def thin_walls(points: np.ndarray, triangles: np.ndarray, limit: float) -> list[dict[str, object]]:
    """Patches where two opposite faces are closer together than ``limit``.

    Rays are cast inward from points spread over the whole surface.  Only a
    hit on a face that looks back at the start (within about 37 degrees of
    parallel) counts, so a sharp edge of a thick body is not a thin wall.
    """
    a, b, c = (points[triangles[:, i]] for i in range(3))
    cross = np.cross(b - a, c - a)
    double_area = np.linalg.norm(cross, axis=1)
    normals = cross / np.maximum(double_area, 1.0e-12)[:, None]

    mesh = vtk.vtkPolyData()
    vtk_points = vtk.vtkPoints()
    vtk_points.SetData(numpy_to_vtk(points, deep=True))
    mesh.SetPoints(vtk_points)
    cells = vtk.vtkCellArray()
    cells.SetData(3, numpy_to_vtkIdTypeArray(triangles.astype(np.int64).ravel(), deep=True))
    mesh.SetPolys(cells)
    tree = vtk.vtkOBBTree()
    tree.SetDataSet(mesh)
    tree.BuildLocator()

    rng = np.random.default_rng(203)
    counts = np.maximum(1, np.ceil(double_area / 2.0 / 1.1).astype(int))
    owner = np.repeat(np.arange(len(triangles)), counts)
    r1, r2 = np.sqrt(rng.random(len(owner))), rng.random(len(owner))
    samples = (
        (1.0 - r1)[:, None] * a[owner]
        + (r1 * (1.0 - r2))[:, None] * b[owner]
        + (r1 * r2)[:, None] * c[owner]
    )
    weight = (double_area / 2.0 / counts)[owner]

    hits = vtk.vtkPoints()
    hit_cells = vtk.vtkIdList()
    thin_points, thin_weight, thin_value = [], [], []
    for index in range(len(owner)):
        normal = normals[owner[index]]
        start = samples[index] - normal * 0.02
        end = samples[index] - normal * limit
        if tree.IntersectWithLine(start, end, hits, hit_cells) == 0:
            continue
        best, best_cell = None, -1
        for k in range(hits.GetNumberOfPoints()):
            distance = float(np.linalg.norm(np.array(hits.GetPoint(k)) - samples[index]))
            if distance > 0.05 and (best is None or distance < best):
                best, best_cell = distance, hit_cells.GetId(k)
        if best is None or float(np.dot(normals[best_cell], normal)) > -0.8:
            continue
        thin_points.append(samples[index])
        thin_weight.append(weight[index])
        thin_value.append(best)

    patches = []
    thin_array = np.array(thin_points).reshape(-1, 3)
    for members in cluster(thin_array, 2.0):
        area = float(np.array(thin_weight)[members].sum())
        if area < 6.0:
            continue
        centre = thin_array[members].mean(axis=0)
        patches.append(
            {
                "area_mm2": round(area, 1),
                "thinnest_mm": round(float(np.array(thin_value)[members].min()), 2),
                "at": [round(float(v), 1) for v in centre],
            }
        )
    return sorted(patches, key=lambda patch: -float(patch["area_mm2"]))


def midair_starts(points: np.ndarray, triangles: np.ndarray) -> list[list[float]]:
    """Downward-facing low points that are not on the bed.

    A printer builds upward, so a feature whose lowest edge hangs below the
    place where it joins the rest of the part starts on nothing.
    """
    z = points[:, 2]
    edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    parent = list(range(len(points)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    level = np.abs(z[edges[:, 0]] - z[edges[:, 1]]) < 2.0e-3
    for first, second in edges[level]:
        ra, rb = find(int(first)), find(int(second))
        if ra != rb:
            parent[ra] = rb
    roots = np.array([find(i) for i in range(len(points))])
    has_lower = np.zeros(len(points), dtype=bool)
    has_lower[edges[z[edges[:, 1]] < z[edges[:, 0]] - 2.0e-3, 0]] = True
    has_lower[edges[z[edges[:, 0]] < z[edges[:, 1]] - 2.0e-3, 1]] = True
    supported = np.zeros(len(points), dtype=bool)
    np.logical_or.at(supported, roots, has_lower)

    a, b, c = (points[triangles[:, i]] for i in range(3))
    down = np.cross(b - a, c - a)[:, 2] / 2.0  # signed area seen from above
    facing = np.zeros(len(points))
    for corner in range(3):
        np.add.at(facing, roots[triangles[:, corner]], down)

    found: dict[int, list[int]] = {}
    for index in range(len(points)):
        root = int(roots[index])
        if z[index] > 0.3 and not supported[root] and facing[root] < -1.0e-6:
            found.setdefault(root, []).append(index)
    return [[round(float(v), 1) for v in points[members].mean(axis=0)] for members in found.values()]


def small_islands(shape: cq.Shape, minimum: float) -> tuple[list[dict[str, object]], float]:
    """Free-standing cross-sections smaller than ``minimum`` mm^2.

    The part is sliced every millimetre.  An island counts only if it is
    still small 2 mm lower down, so the tip of a thick sloping body is not
    mistaken for a post.
    """
    bb = shape.BoundingBox()
    levels = np.arange(0.37, bb.zmax - 0.05, 1.0)
    slices: list[list[tuple[float, cq.Face, cq.Vector]]] = []
    for level in levels:
        # A square: makePlane's two sizes do not map to x and y in that order.
        plane = cq.Face.makePlane(
            max(bb.xlen, bb.ylen) + 40.0,
            max(bb.xlen, bb.ylen) + 40.0,
            basePnt=Vec(bb.center.x, bb.center.y, float(level)),
            dir=Vec(0.0, 0.0, 1.0),
        )
        faces = shape.intersect(plane).Faces()
        slices.append([(face.Area(), face, face.Center()) for face in faces])
    flagged, smallest = [], 1.0e9
    for k, islands in enumerate(slices):
        for area, _, centre in islands:
            if len(islands) > 1:
                smallest = min(smallest, area)
            if area >= minimum:
                continue
            below_ok = False
            if k >= 2:
                # Is the material 2 mm straight below part of a big cross-section?
                probe = cq.Vertex.makeVertex(centre.x, centre.y, float(levels[k - 2]))
                for other_area, other_face, _ in slices[k - 2]:
                    if other_area >= minimum and other_face.distance(probe) < 1.0e-4:
                        below_ok = True
                        break
            if not below_ok:
                flagged.append(
                    {"area_mm2": round(area, 1), "at": [round(centre.x, 1), round(centre.y, 1), round(float(levels[k]), 1)]}
                )
    return flagged, (smallest if smallest < 1.0e8 else 0.0)


def check_robustness(report: Report, plates: dict[str, cq.Shape]) -> dict[str, dict[str, object]]:
    facts: dict[str, dict[str, object]] = {}
    for name, shape in plates.items():
        points, triangles = welded_mesh(shape)

        patches = thin_walls(points, triangles, g.MIN_WALL - 0.05)
        report.add(
            "robust",
            f"{name} has no wall thinner than {g.MIN_WALL:.1f} mm",
            not patches,
            "every pair of opposite faces is at least that far apart"
            if not patches
            else "; ".join(f"{p['thinnest_mm']} mm over {p['area_mm2']} mm^2 at {p['at']}" for p in patches[:4]),
        )

        starts = midair_starts(points, triangles)
        report.add(
            "robust",
            f"{name} has no feature that starts printing in mid-air",
            not starts,
            "every downward-facing low point is on the bed" if not starts else f"{len(starts)} hanging start(s), e.g. at {starts[:4]}",
        )

        flagged, smallest = small_islands(shape, g.MIN_ISLAND_AREA)
        report.add(
            "robust",
            f"{name} has no free-standing feature under {g.MIN_ISLAND_AREA:.0f} mm^2",
            not flagged,
            f"smallest separate cross-section {smallest:.0f} mm^2"
            if not flagged
            else f"{len(flagged)} slice(s), e.g. " + "; ".join(f"{f['area_mm2']} mm^2 at {f['at']}" for f in flagged[:4]),
        )
        facts[name] = {
            "thin_wall_patches": patches,
            "midair_starts": starts,
            "smallest_island_mm2": round(smallest, 1),
        }
    return facts


def check_interference(report: Report, scene: dict[str, object]) -> None:
    printed: dict[str, cq.Shape] = scene["printed"]  # type: ignore[assignment]
    components = {name: item.shape for name, item in scene["components"].items()}  # type: ignore[union-attr]
    bodies = {**printed, **components}
    names = list(bodies)
    worst = 0.0
    for i, first in enumerate(names):
        for second in names[i + 1 :]:
            volume = overlap(bodies[first], bodies[second])
            worst = max(worst, volume if volume == volume else 1e9)
            if not volume <= EPS_INTERFERENCE:
                report.add("interference", f"{first} vs {second}", False, f"{volume:.3f} mm^3 overlap")
    report.add(
        "interference",
        f"{len(names)} bodies, {len(names) * (len(names) - 1) // 2} pairs clear",
        worst <= EPS_INTERFERENCE,
        f"largest overlap {worst:.4f} mm^3",
    )

    # Fasteners must pass cleanly through board holes and clear every other board.
    fasteners = {name: item.shape for name, item in scene["fasteners"].items()}  # type: ignore[union-attr]
    worst = 0.0
    for fastener_name, fastener in fasteners.items():
        for component_name, component in components.items():
            volume = overlap(fastener, component)
            worst = max(worst, volume if volume == volume else 1e9)
            if not volume <= EPS_INTERFERENCE:
                report.add(
                    "interference",
                    f"{fastener_name} vs {component_name}",
                    False,
                    f"{volume:.3f} mm^3 overlap",
                )
    report.add(
        "interference",
        f"{len(fasteners)} fasteners clear of all boards",
        worst <= EPS_INTERFERENCE,
        f"largest overlap {worst:.4f} mm^3",
    )


# Each entry: frame the play is measured in, and the allowed free play (mm)
# along -x, +x, -y, +y, -z, +z of that frame before something must stop it.
T_AXES = g.Frame(origin=(0.0, 0.0, 0.0), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, 0.0, 1.0))
CAPTIVITY = {
    # purchased parts (boards have 0.5 to 0.6 mm of clearance per side)
    "display_msp3520": (g.D_FRAME, (0.2, 0.2, 0.2, 0.2, 0.1, 0.1)),
    "esp32_s3": (T_AXES, (0.6, 0.6, 0.6, 0.6, 0.1, 0.1)),
    "esp32_c3": (T_AXES, (0.6, 0.6, 0.6, 0.6, g.C3_STUB_GAP + 0.1, 0.1)),
    "rd03d_radar": (g.W_FRAME, (0.6, 0.6, 0.6, 1.3, 0.2, 0.8)),
    "bq24074_charger": (T_AXES, (0.7, 0.7, 0.6, 0.7, 0.1, 0.7)),
    "lipo_battery": (T_AXES, (1.6, 1.6, 1.6, 1.6, 0.1, g.BATTERY_FOAM_GAP + 0.1)),
    "pololu_regulator": (T_AXES, (0.6, 0.6, 0.6, 0.6, 0.1, 0.5)),
    # printed parts (bolts have 0.4 mm of play in their clearance holes)
    "tray": (T_AXES, (0.5, 0.5, 0.5, 0.5, 0.1, 0.1)),
    "bezel": (g.D_FRAME, (0.5, 0.5, 0.5, 0.5, 0.1, 0.1)),
    "power_cover": (T_AXES, (0.5, 0.5, 0.5, 0.5, 0.1, 0.1)),
    "s3_keeper": (T_AXES, (0.5, 0.5, 0.6, 0.6, 0.1, 0.1)),
    "c3_keeper": (T_AXES, (0.5, 0.5, 0.6, 0.6, 0.1, 0.2)),
}
DIRECTION_LABELS = ("-x", "+x", "-y", "+y", "-z", "+z")


def check_captivity(report: Report, scene: dict[str, object]) -> dict[str, dict[str, str]]:
    """Nudge every part along six directions; something solid must stop it.

    A part that can be moved without touching a printed part, a fastener, or a
    neighbouring board is exactly the "floating in space" failure.
    """
    printed: dict[str, cq.Shape] = scene["printed"]  # type: ignore[assignment]
    components = {name: item.shape for name, item in scene["components"].items()}  # type: ignore[union-attr]
    fasteners = {name: item.shape for name, item in scene["fasteners"].items()}  # type: ignore[union-attr]
    everything = {**printed, **components, **fasteners}
    results: dict[str, dict[str, str]] = {}

    for name, (frame, plays) in CAPTIVITY.items():
        body = everything[name]
        stops: dict[str, str] = {}
        for index, label in enumerate(DIRECTION_LABELS):
            unit = [0.0, 0.0, 0.0]
            unit[index // 2] = -1.0 if index % 2 == 0 else 1.0
            step = plays[index] + 0.35
            delta = Vec(*frame.direction(*(component * step for component in unit)))
            moved = body.translate(delta)
            stopper = None
            for other_name, other in everything.items():
                if other_name == name:
                    continue
                volume = overlap(moved, other)
                if volume == volume and volume > TOUCH_VOLUME:
                    stopper = other_name
                    break
            stops[label] = stopper or "NOTHING"
            report.add(
                "captive",
                f"{name} cannot move {label}",
                stopper is not None,
                f"stopped by {stopper} within {step:.2f} mm" if stopper else f"free for {step:.2f} mm",
            )
        results[name] = stops
    return results


def check_fasteners(report: Report, scene: dict[str, object]) -> None:
    """Every bolt runs in a clear hole and every nut has a slot it can reach."""
    printed: dict[str, cq.Shape] = scene["printed"]  # type: ignore[assignment]
    fasteners = {name: item.shape for name, item in scene["fasteners"].items()}  # type: ignore[union-attr]
    worst = 0.0
    for fastener_name, fastener in fasteners.items():
        for part_name, part in printed.items():
            volume = overlap(fastener, part)
            worst = max(worst, volume if volume == volume else 1e9)
            if not volume <= EPS_INTERFERENCE:
                report.add("fasteners", f"{fastener_name} vs {part_name}", False, f"{volume:.3f} mm^3 of plastic in the way")
    report.add(
        "fasteners",
        f"{len(fasteners)} bolts and nuts sit in clear holes, slots, and pockets",
        worst <= EPS_INTERFERENCE,
        f"largest overlap with a printed part {worst:.4f} mm^3",
    )
    corners = g.M3_NUT_AF / math.cos(math.radians(30.0))
    report.add(
        "fasteners",
        "no nut can spin in its slot or pocket",
        g.M3_NUT_AF < g.NUT_SLOT_W < corners and g.M3_NUT_AF < g.NUT_POCKET_AF < corners,
        f"slot {g.NUT_SLOT_W:.1f} mm, pocket {g.NUT_POCKET_AF:.1f} mm, nut {g.M3_NUT_AF:.1f} mm flats / {corners:.2f} mm corners",
    )

    tray = printed["tray"]
    for site in scene["nut_sites"]:  # type: ignore[union-attr]
        entry = Vec(*site.direction(1.0, 0.0, 0.0))
        problem = None
        for distance in (1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0):
            if overlap(site.nut.translate(entry * distance), tray) > TOUCH_VOLUME:
                problem = f"blocked {distance:.0f} mm out of the slot"
                break
        report.add("fasteners", f"{site.name} slides into its slot", problem is None, problem or "12 mm of clear approach")
        loose = []
        slack = g.NUT_SLOT_H - g.M3_NUT_T + 0.3
        for label, local in (
            ("deeper", (-0.6, 0.0, 0.0)),
            ("sideways -", (0.0, -0.7, 0.0)),
            ("sideways +", (0.0, 0.7, 0.0)),
            ("toward the head", (0.0, 0.0, 0.3)),
            ("away from the head", (0.0, 0.0, -slack)),
        ):
            if overlap(site.nut.translate(Vec(*site.direction(*local))), tray) <= TOUCH_VOLUME:
                loose.append(label)
        report.add(
            "fasteners",
            f"{site.name} is trapped on every side but its entry",
            not loose,
            "slot walls, roof, and floor all stop it" if not loose else f"free to move {', '.join(loose)}",
        )


def check_wiring_space(report: Report, scene: dict[str, object]) -> None:
    printed: dict[str, cq.Shape] = scene["printed"]  # type: ignore[assignment]
    components = {name: item.shape for name, item in scene["components"].items()}  # type: ignore[union-attr]
    keepouts = scene["keepouts"]  # type: ignore[assignment]
    owner = {
        "keepout_display_header_jumpers": "display_msp3520",
        "keepout_s3_minus_y_row_jumpers": "esp32_s3",
        "keepout_s3_plus_y_row_jumpers": "esp32_s3",
        "keepout_c3_minus_y_row_jumpers": "esp32_c3",
        "keepout_c3_plus_y_row_jumpers": "esp32_c3",
        "keepout_radar_plug": "rd03d_radar",
        "keepout_charger_battery_jst_plug": "bq24074_charger",
        "keepout_charger_load_jst_plug": "bq24074_charger",
    }
    for item in keepouts:  # type: ignore[union-attr]
        worst_name, worst = "", 0.0
        for other_name, other in {**printed, **components}.items():
            if other_name == owner.get(item.name):
                continue
            volume = overlap(item.shape, other)
            if volume != volume or volume > worst:
                worst_name, worst = other_name, volume if volume == volume else 1e9
        report.add(
            "wiring",
            f"{item.name} is clear",
            worst <= EPS_INTERFERENCE,
            "no solid in the connector space" if worst <= EPS_INTERFERENCE else f"{worst:.2f} mm^3 inside {worst_name}",
        )
    items = list(keepouts)  # type: ignore[arg-type]
    for i, first in enumerate(items):
        for second in items[i + 1 :]:
            volume = overlap(first.shape, second.shape)
            if volume > EPS_INTERFERENCE:
                report.add("wiring", f"{first.name} vs {second.name}", False, f"{volume:.2f} mm^3 shared")

    for plug in scene["usb_plugs"]:  # type: ignore[union-attr]
        worst_name, worst = "", 0.0
        for other_name, other in printed.items():
            volume = overlap(plug.shape, other)
            if volume != volume or volume > worst:
                worst_name, worst = other_name, volume if volume == volume else 1e9
        report.add(
            "wiring",
            f"{plug.name} can reach its socket",
            worst <= EPS_INTERFERENCE,
            "25 mm overmould passes the wall opening" if worst <= EPS_INTERFERENCE else f"hits {worst_name} ({worst:.2f} mm^3)",
        )

    for route in scene["routes"]:  # type: ignore[union-attr]
        shape = route.shape
        worst_name, worst = "", 0.0
        for other_name, other in printed.items():
            volume = overlap(shape, other)
            if volume != volume or volume > worst:
                worst_name, worst = other_name, volume if volume == volume else 1e9
        report.add(
            "wiring",
            f"route {route.name} has a free path",
            worst <= 0.5,
            f"{route.length:.0f} mm, {route.diameter:.1f} mm bundle" if worst <= 0.5 else f"passes through {worst_name} ({worst:.1f} mm^3)",
        )
        hit = [
            other_name
            for other_name, other in components.items()
            if overlap(shape, other) > 0.5
        ]
        report.add(
            "wiring",
            f"route {route.name} avoids the boards",
            not hit,
            "clear" if not hit else f"passes through {', '.join(hit)}",
        )


def check_insertion(report: Report, scene: dict[str, object]) -> None:
    """Each board must have a collision-free way into its seat.

    Fitting once installed is not enough: a board that can only reach its
    cradle by passing through a wall cannot be assembled.  Each part is swept
    out of its seat along the direction a hand would move it, lids off, past
    the tray and past the boards that are installed before it.
    """
    printed: dict[str, cq.Shape] = scene["printed"]  # type: ignore[assignment]
    components = {name: item.shape for name, item in scene["components"].items()}  # type: ignore[union-attr]
    keepouts = {item.name: item.shape for item in scene["keepouts"]}  # type: ignore[union-attr]
    tray = printed["tray"]
    up = (0.0, 0.0, 1.0)
    along_wall = (g.SIN_D, 0.0, g.COS_D)  # straight out through the display opening

    def sweep(body: cq.Shape, direction, travel: float, obstacles: dict[str, cq.Shape], start: float = 1.0):
        steps = [start + (travel - start) * i / 11.0 for i in range(12)]
        for distance in steps:
            moved = body.translate(Vec(*(component * distance for component in direction)))
            for name, obstacle in obstacles.items():
                if overlap(moved, obstacle) > TOUCH_VOLUME:
                    return f"hits {name} after {distance:.1f} mm"
        return None

    def record(label: str, problem: str | None, how: str) -> None:
        report.add("assembly", label, problem is None, how if problem is None else problem)

    # Order of assembly: radar, S3, C3, power-bay parts, then the display.
    radar = cq.Compound.makeCompound([components["rd03d_radar"], keepouts["keepout_radar_plug"]])
    record(
        "radar slides down its slot with the lead plugged in",
        sweep(radar, g.W_FRAME.direction(0.0, 1.0, 0.0), 70.0, {"tray": tray}),
        "70 mm up the back wall, clear",
    )

    def tilt_out(body: cq.Shape, pivot: Vec, axis: Vec, outward: Vec, pull: float, lift_out, obstacles: dict[str, cq.Shape]):
        """Tilt a board up about its USB end, back it out, then lift it clear.

        ``axis`` is chosen so a positive angle lifts the far end; ``outward``
        is the in-plane direction from the USB end toward the far end.  At
        each angle the board may also shift a little along ``outward`` and
        upward, which is what a hand does to clear the wall opening.  Once
        tilted it is drawn ``pull`` mm along its own length, so its USB end
        leaves the wall, and then lifted out through the display opening.
        """
        pose = body
        for angle in (3.0, 6.0, 10.0, 15.0, 20.0, 25.0):
            tilted = body.rotate(pivot, pivot + axis, angle)
            found = None
            for lift in (0.0, 0.15, 0.3, 0.6, 1.0, 1.5):
                for slide in (0.0, 0.15, 0.3, 0.5, 0.8, 1.2):
                    candidate = tilted.translate(outward * slide + Vec(0.0, 0.0, lift))
                    if all(overlap(candidate, shape) <= TOUCH_VOLUME for shape in obstacles.values()):
                        found = candidate
                        break
                if found is not None:
                    break
            if found is None:
                return f"no clear pose at {angle:.0f} deg of tilt"
            pose = found
        radians = math.radians(25.0)
        direction = outward * math.cos(radians) + Vec(0.0, 0.0, math.sin(radians))
        problem = sweep(pose, (direction.x, direction.y, direction.z), pull, obstacles, start=1.0)
        if problem is not None:
            return problem
        return sweep(pose.translate(direction * pull), lift_out, 70.0, obstacles)

    # Both MCU boards have USB shells that reach into or under the wall, so
    # they go in USB end first at an angle.
    record(
        "ESP32-S3 tilts into its cradle",
        tilt_out(
            components["esp32_s3"],
            Vec(g.S3_U_USB_END, g.S3_VC, g.BOARD_BOTTOM_W),
            Vec(0.0, 1.0, 0.0),
            Vec(-1.0, 0.0, 0.0),
            14.0,
            up,
            {"tray": tray, "rd03d_radar": components["rd03d_radar"]},
        ),
        "tilt 25 deg about the USB end, back out 14 mm, lift straight out",
    )
    record(
        "ESP32-C3 tilts into its cradle",
        tilt_out(
            components["esp32_c3"],
            Vec(g.C3_U_USB_END, g.C3_VC, g.BOARD_BOTTOM_W),
            Vec(0.0, -1.0, 0.0),
            Vec(1.0, 0.0, 0.0),
            6.0,
            along_wall,
            {
                "tray": tray,
                "rd03d_radar": components["rd03d_radar"],
                "esp32_s3": components["esp32_s3"],
                "s3_keeper": printed["s3_keeper"],
            },
        ),
        "tilt 25 deg about the USB end, back out 6 mm, lift through the display opening",
    )
    for name, label in (
        ("bq24074_charger", "charger drops onto its rails"),
        ("lipo_battery", "battery drops into its pocket"),
        ("pololu_regulator", "regulator drops into its pocket"),
    ):
        record(label, sweep(components[name], up, 40.0, {"tray": tray}), "40 mm straight up, clear")

    # S3 keeper: draw the tongue out of the wall mortise, lift the far end,
    # and take it out between the two display gussets.
    keeper = printed["s3_keeper"]
    problem = sweep(keeper, (-1.0, 0.0, 0.0), 3.0, {"tray": tray}, start=0.5)
    if problem is None:
        problem = tilt_out(
            keeper.translate(Vec(-3.0, 0.0, 0.0)),
            Vec(g.U_IN_END + g.S3_MORTISE_DEPTH - 3.5, g.S3_VC, g.BOARD_TOP_W),
            Vec(0.0, 1.0, 0.0),
            Vec(-1.0, 0.0, 0.0),
            14.0,
            up,
            {"tray": tray},
        )
    record("S3 keeper bar comes out of its wall mortise", problem, "3 mm back, tilt 25 deg, 14 mm along its length, then straight up")

    keeper = printed["c3_keeper"]
    problem = sweep(keeper, (1.0, 0.0, 0.0), 3.0, {"tray": tray}, start=0.5)
    if problem is None:
        problem = sweep(keeper.translate(Vec(3.0, 0.0, 0.0)), along_wall, 70.0, {"tray": tray})
    record("C3 keeper tongue slides out from under its shelf", problem, "3 mm back, then out through the display opening")

    display = cq.Compound.makeCompound(
        [components["display_msp3520"], keepouts["keepout_display_header_jumpers"]]
    )
    installed_first = {
        "tray": tray,
        **{k: components[k] for k in ("esp32_s3", "esp32_c3", "rd03d_radar")},
        **{k: v for k, v in keepouts.items() if "s3" in k or "c3" in k},
    }
    record(
        "display seats on its corbels with its jumpers attached",
        sweep(display, g.D_FRAME.direction(0.0, 0.0, 1.0), 60.0, installed_first),
        "60 mm along the screen normal, clear of the tray, boards, and their jumpers",
    )


def check_saddle(report: Report) -> dict[str, object]:
    frame = g.tray_frame()
    inner = g._inner_loft()
    bolt_reach = 12.0  # M3x12 measured from under the head
    shallowest = 1e9
    for index, (u, v, tilted) in enumerate(g.TRAY_SCREWS, start=1):
        screw_frame = g.tray_screw_frame(u, v, tilted)
        seat = -g.tray_screw_deck_drop(tilted) - g.SADDLE_NUT_SEAT
        low = min(seat - g.M3_NUT_T, -bolt_reach)

        def buried(extra: float) -> bool:
            probe = frame.place(screw_frame.place(g.hex_prism(g.M3_NUT_AF, low - extra, seat)))
            return overlap(probe, inner) <= TOUCH_VOLUME

        recess = 0.0
        for extra in (g.SADDLE_NUT_RECESS, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0):
            if not buried(extra):
                break
            recess = extra
        shallowest = min(shallowest, recess)
        report.add(
            "saddle",
            f"nut pocket {index} keeps the nut and bolt tip off the arm",
            recess >= g.SADDLE_NUT_RECESS,
            f"recessed at least {recess:.1f} mm below the cavity surface; {g.SADDLE_NUT_SEAT:.1f} mm of deck above the nut",
        )

    saddle = g.make_saddle()
    slot_count = 0
    for station in g.STRAP_STATIONS:
        for side in (-1.0, 1.0):
            y = side * g._shell_outer_half_width(station, g.STRAP_SLOT_CENTER_Z, 0.0)
            probe = g.box3(
                station - g.STRAP_WIDTH / 2.0,
                station + g.STRAP_WIDTH / 2.0,
                min(y - 9.0, y + 9.0),
                max(y - 9.0, y + 9.0),
                g.STRAP_SLOT_CENTER_Z - 1.5,
                g.STRAP_SLOT_CENTER_Z + 1.5,
            )
            blocked = overlap(probe, saddle)
            ok = blocked <= TOUCH_VOLUME
            slot_count += int(ok)
            report.add(
                "saddle",
                f"strap slot at x={station:.0f} mm, {'+' if side > 0 else '-'}Y is an open through-hole",
                ok,
                f"a {g.STRAP_WIDTH:.0f} x 3 mm strap passes the wall" if ok else f"{blocked:.1f} mm^3 of plastic in the way",
            )

    deck_gaps = []
    for step in range(15):
        x = 0.5 + (g.BRACER_LENGTH - 1.0) * step / 14.0
        deck_z = x * g.TAN_D + g.deck_offset()
        deck_gaps.append(round(deck_z - g.ridge_height(x), 2))
    report.add(
        "saddle",
        "deck plane clears the shell ridge along the whole brace",
        min(deck_gaps) >= g.DECK_MIN_GAP - 0.05,
        f"gap {min(deck_gaps):.2f} to {max(deck_gaps):.2f} mm (solid fill)",
    )
    tray_end_x = frame.point(g.TRAY_L, 0.0, 0.0)[0]
    report.add(
        "saddle",
        "tray sits inside the length of the brace",
        frame.origin[0] >= 0.0 and tray_end_x <= g.BRACER_LENGTH,
        f"tray spans x = {frame.origin[0]:.1f} to {tray_end_x:.1f} mm of {g.BRACER_LENGTH:.0f}",
    )
    return {"deck_gap_mm": deck_gaps, "strap_slots_open": slot_count, "shallowest_nut_recess_mm": shallowest}


def check_function(report: Report, scene: dict[str, object]) -> None:
    printed: dict[str, cq.Shape] = scene["printed"]  # type: ignore[assignment]
    # The screen must be fully visible and the radar unobstructed.
    aa_half_l, aa_half_w = 73.44 / 2.0, 48.96 / 2.0
    view = g.D_FRAME.place(
        g.box3(2.33 - aa_half_l, 2.33 + aa_half_l, -aa_half_w, aa_half_w, g.PCB_FRONT_R + g.DISPLAY_FRONT_STACK, 80.0)
    )
    blocked = max(overlap(view, shape) for shape in printed.values())
    report.add("function", "active display area is unobstructed", blocked <= TOUCH_VOLUME, "73.44 x 48.96 mm visible along the screen normal")

    half = g.RADAR_WID / 2.0 - g.RADAR_EDGE_GRIP
    beam = g.W_FRAME.place(
        g.box3(-half, half, g.RADAR_BOTTOM_Y + g.RADAR_EDGE_GRIP, g.RADAR_TOP_Y - g.RADAR_EDGE_GRIP, -120.0, g.RADAR_LIP_T)
    )
    blocked = max(overlap(beam, shape) for shape in printed.values())
    report.add("function", "radar antenna face looks out through an open window", blocked <= TOUCH_VOLUME, "no plastic in front of the antenna")

    # Header on the viewer's right, screen facing the elbow.
    # Read from the thumb side, the viewer's right is toward the elbow (+X).
    header = g.tray_frame().point(*g.D_FRAME.point(g.DISPLAY_HEADER_P, 0.0, g.PCB_BACK_R))
    centre = g.tray_frame().point(*g.D_FRAME.point(0.0, 0.0, g.PCB_BACK_R))
    report.add(
        "function",
        "display header is on the viewer's right (elbow end)",
        header[0] - centre[0] > 40.0,
        f"header row {header[0] - centre[0]:+.1f} mm along X from the screen centre",
    )
    normal = g.tray_frame().direction(*g.D_FRAME.direction(0.0, 0.0, 1.0))
    tilt = math.degrees(math.atan2(-normal[1], normal[2]))
    report.add(
        "function",
        "screen faces the thumb side of the forearm",
        normal[1] < -0.3 and normal[2] > 0.3 and abs(normal[0]) < 0.2,
        f"normal leans {tilt:.1f} deg from dorsal toward -Y",
    )
    long_axis = g.tray_frame().direction(*g.D_FRAME.direction(1.0, 0.0, 0.0))
    report.add(
        "function",
        "screen is landscape along the forearm",
        abs(long_axis[0]) > 0.95,
        f"98 mm axis direction ({long_axis[0]:+.2f}, {long_axis[1]:+.2f}, {long_axis[2]:+.2f})",
    )
    radar_normal = g.tray_frame().direction(*g.W_FRAME.direction(0.0, 0.0, -1.0))
    screen_back = [-component for component in normal]
    report.add(
        "function",
        "radar looks out of the back of the screen, square to the forearm",
        radar_normal[1] > 0.999 and screen_back[1] > 0.3,
        f"boresight ({radar_normal[0]:+.3f}, {radar_normal[1]:+.3f}, {radar_normal[2]:+.3f})",
    )
    radar_up = g.tray_frame().direction(*g.W_FRAME.direction(0.0, 1.0, 0.0))
    report.add(
        "function",
        "radar long axis is upright (dorsal), so azimuth is measured along the forearm",
        radar_up[2] > 0.999,
        f"long axis ({radar_up[0]:+.3f}, {radar_up[1]:+.3f}, {radar_up[2]:+.3f})",
    )

    # A straight screwdriver must reach every screw with the lids off.
    components = {name: item.shape for name, item in scene["components"].items()}  # type: ignore[union-attr]
    obstacles = {"tray": printed["tray"], **{k: v for k, v in components.items() if k not in ("display_msp3520",)}}
    for index, (u, v, tilted) in enumerate(g.TRAY_SCREWS, start=1):
        driver = g.tray_screw_frame(u, v, tilted).place(g.vcyl(5.0, 0.0, 0.0, g.M3_HEAD_H, 90.0))
        hit = [name for name, shape in obstacles.items() if overlap(driver, shape) > TOUCH_VOLUME]
        report.add(
            "function",
            f"tray bolt {index} has straight driver access",
            not hit,
            "5 mm driver clear through the display opening or bay" if not hit else f"blocked by {', '.join(hit)}",
        )


# --------------------------------------------------------------------------
# Export
# --------------------------------------------------------------------------
def export_step(shape: cq.Shape, path: Path) -> None:
    cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path))
    if cq.importers.importStep(str(path)).val().isNull():
        raise RuntimeError(f"STEP validation failed: {path}")


def export_stl(shape: cq.Shape, path: Path) -> None:
    # Mesh a copy so tessellation does not inflate later bounding boxes, and
    # use an absolute chord error so the arm-fit surfaces stay smooth.
    shape.copy().exportStl(
        str(path),
        tolerance=STL_DEFLECTION,
        angularTolerance=STL_ANGLE,
        relative=False,
    )


def build_assembly(scene: dict[str, object]) -> cq.Assembly:
    assembly = cq.Assembly(name="n203_v3_right_arm_bracer")
    printed = cq.Assembly(name="printed_parts")
    printed.add(g.make_saddle(), name="saddle", color=cq.Color(*g.PRINTED_COLORS["saddle"]))
    for name, shape in g.printed_parts_tray_frame().items():
        printed.add(g.to_bracer(shape), name=name, color=cq.Color(*g.PRINTED_COLORS[name]))
    assembly.add(printed, name="printed_parts")

    boards = cq.Assembly(name="purchased_components")
    for item in scene["components"].values():  # type: ignore[union-attr]
        boards.add(g.to_bracer(item.shape), name=item.name, color=cq.Color(*item.color))
    assembly.add(boards, name="purchased_components")

    screws = cq.Assembly(name="fasteners")
    for fastener in scene["fasteners"].values():  # type: ignore[union-attr]
        screws.add(g.to_bracer(fastener.shape), name=fastener.name, color=cq.Color(0.78, 0.80, 0.84))
    assembly.add(screws, name="fasteners")

    guides = cq.Assembly(name="wiring_guides_hide_or_delete")
    for item in scene["keepouts"]:  # type: ignore[union-attr]
        guides.add(g.to_bracer(item.shape), name=item.name, color=cq.Color(*item.color, 0.45))
    for route in scene["routes"]:  # type: ignore[union-attr]
        guides.add(g.to_bracer(route.shape), name=f"route_{route.name}", color=cq.Color(*route.color))
    assembly.add(guides, name="wiring_guides_hide_or_delete")
    return assembly


def bill_of_materials(scene: dict[str, object]) -> dict[str, object]:
    counts: dict[str, dict[str, object]] = {}
    for fastener in scene["fasteners"].values():  # type: ignore[union-attr]
        entry = counts.setdefault(fastener.spec, {"quantity": 0, "used_for": []})
        entry["quantity"] = int(entry["quantity"]) + 1
        if fastener.joins not in entry["used_for"]:
            entry["used_for"].append(fastener.joins)  # type: ignore[union-attr]
    return {
        "fasteners": counts,
        "other": {
            "20 mm hook-and-loop cinch strap": 2,
            "M3 washer, optional, under each bolt head": 15,
            "soft foam strip, about 2 mm, for the battery pad and the charger pillars": 1,
            "small zip ties for the three tunnels in the ESP32-S3 rail": 3,
            "female-female jumper wires, 15-20 cm": "about 20",
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=g.OUTPUT_DIR)
    parser.add_argument("--reference-arm", action="store_true", help="ignore arm_fit.local.json and use the reference arm")
    parser.add_argument("--checks-only", action="store_true", help="run the checks without exporting files")
    args = parser.parse_args()
    print(g.arm_fit.describe())

    scene = build_scene()
    plates = g.print_oriented_parts()
    report = Report()

    print("Checking printed parts ...")
    check_printed_parts(report, plates)
    overhang_facts = check_overhangs(report, plates)
    print("Checking robustness on a loose printer ...")
    robust_facts = check_robustness(report, plates)
    print("Checking interference ...")
    check_interference(report, scene)
    print("Checking bolts and nuts ...")
    check_fasteners(report, scene)
    print("Checking that every part is held ...")
    captivity = check_captivity(report, scene)
    print("Checking wiring space ...")
    check_wiring_space(report, scene)
    print("Checking assembly paths ...")
    check_insertion(report, scene)
    print("Checking saddle ...")
    saddle_facts = check_saddle(report)
    print("Checking function ...")
    check_function(report, scene)

    summary = report.summary()
    for group, counts in summary.items():
        print(f"  {group:13s} {counts['passed']:3d} passed, {counts['failed']:3d} failed")

    if not args.checks_only:
        out = args.output
        print_dir = out / "print"
        print_dir.mkdir(parents=True, exist_ok=True)
        for name, shape in plates.items():
            export_step(shape, print_dir / f"n203_v3_{name}.step")
            export_stl(shape, print_dir / f"n203_v3_{name}.stl")

        assembly_path = out / "n203_v3_assembly.step"
        build_assembly(scene).export(str(assembly_path))
        if cq.importers.importStep(str(assembly_path)).val().isNull():
            raise RuntimeError("Assembly STEP validation failed")

        frame = g.tray_frame()
        assembled = [g.make_saddle()] + [g.to_bracer(s) for s in g.printed_parts_tray_frame().values()]
        assembled += [g.to_bracer(item.shape) for item in scene["components"].values()]  # type: ignore[union-attr]
        manifest = {
            "units": "mm",
            "status": "version 3, drawn for a loosely tuned 0.4 mm printer; all automated checks passed" if not report.failures else "CHECKS FAILED",
            "coordinate_system": {
                "x": "wrist toward elbow",
                "y": "the little-finger side of a right arm held palm-down; the thumb side, which the screen faces, is -Y",
                "z": "away from the dorsal forearm surface",
            },
            "arm_stations": [
                {"distance": x, "width": width, "height": height} for x, width, height in g.ARM_STATIONS
            ],
            "deck": {
                "tilt_degrees": g.DECK_TILT_DEG,
                "tray_origin_in_bracer_frame": [round(value, 3) for value in frame.origin],
                **saddle_facts,
            },
            "screen": {
                "orientation": "landscape, 98 mm axis along the forearm, facing the thumb side, header toward the elbow (viewer's right)",
                "tilt_toward_thumb_side_degrees": g.SCREEN_ANGLE_FROM_DECK_DEG,
            },
            "printed_parts": {
                name: {
                    "files": [f"print/n203_v3_{name}.step", f"print/n203_v3_{name}.stl"],
                    "build_size_mm": [round(v, 1) for v in (shape.BoundingBox().xlen, shape.BoundingBox().ylen, shape.BoundingBox().zlen)],
                    "volume_cm3": round(shape.Volume() / 1000.0, 1),
                    **overhang_facts.get(name, {}),
                    **robust_facts.get(name, {}),
                }
                for name, shape in plates.items()
            },
            "components": {
                item.name: {
                    "source_model": item.source,
                    "mounting": item.note,
                    "held_by": captivity.get(item.name, {}),
                    "bounds_mm": bounds(g.to_bracer(item.shape)),
                }
                for item in scene["components"].values()  # type: ignore[union-attr]
            },
            "bill_of_materials": bill_of_materials(scene),
            "wire_routes": [
                {"name": route.name, "description": route.description, "length_mm": round(route.length, 0)}
                for route in scene["routes"]  # type: ignore[union-attr]
            ],
            "assembly_bounds_mm": bounds(cq.Compound.makeCompound(assembled)),
            "checks": {"summary": summary, "results": report.rows},
        }
        manifest_path = out / "n203_v3_manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {assembly_path}")
        print(f"Wrote {manifest_path}")

    if report.failures:
        print(f"{len(report.failures)} check(s) failed")
        sys.exit(1)
    print(f"All {len(report.rows)} checks passed")


if __name__ == "__main__":
    main()
