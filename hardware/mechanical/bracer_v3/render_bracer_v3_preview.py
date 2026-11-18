"""Render shaded preview sheets for the N203 proof-of-concept bracer.

Uses VTK off-screen rendering (installed with CadQuery) so the sheets show
real shaded solids, cut-aways, and an exploded view instead of wireframes.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
import vtk
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from PIL import Image, ImageDraw, ImageFont
from vtk.util.numpy_support import vtk_to_numpy

import v3_geometry as g


Vec = cq.Vector
BACKGROUND = (1.0, 1.0, 1.0)


@dataclass
class Drawable:
    shape: cq.Shape
    color: tuple[float, float, float]
    opacity: float = 1.0
    edges: bool = True


def _polydata(shape: cq.Shape, deflection: float = 0.05, angular: float = 0.20) -> vtk.vtkPolyData:
    # Absolute chord error: tessellate() alone scales its tolerance by size
    # and facets the long arm surfaces.
    shape = shape.copy()
    BRepMesh_IncrementalMesh(shape.wrapped, deflection, False, angular, True)
    vertices, triangles = shape.tessellate(1.0e3, 1.0e3)
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len(vertices))
    for index, vertex in enumerate(vertices):
        points.SetPoint(index, vertex.x, vertex.y, vertex.z)
    cells = vtk.vtkCellArray()
    for a, b, c in triangles:
        cells.InsertNextCell(3, (a, b, c))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    return poly


def render(
    drawables: list[Drawable],
    *,
    direction: tuple[float, float, float],
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
    size: tuple[int, int] = (1100, 820),
    zoom: float = 1.0,
) -> Image.Image:
    """Orthographic render looking from ``direction`` toward the scene centre."""
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(*BACKGROUND)
    for drawable in drawables:
        poly = _polydata(drawable.shape)
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(poly)
        normals.SetFeatureAngle(35.0)
        normals.SplittingOn()
        normals.ConsistencyOn()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normals.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        prop = actor.GetProperty()
        prop.SetColor(*drawable.color)
        prop.SetOpacity(drawable.opacity)
        prop.SetAmbient(0.32)
        prop.SetDiffuse(0.72)
        prop.SetSpecular(0.12)
        prop.SetSpecularPower(18.0)
        renderer.AddActor(actor)
        if drawable.edges:
            feature = vtk.vtkFeatureEdges()
            feature.SetInputData(poly)
            feature.BoundaryEdgesOn()
            feature.FeatureEdgesOn()
            feature.ManifoldEdgesOff()
            feature.NonManifoldEdgesOff()
            feature.SetFeatureAngle(28.0)
            edge_mapper = vtk.vtkPolyDataMapper()
            edge_mapper.SetInputConnection(feature.GetOutputPort())
            edge_mapper.ScalarVisibilityOff()
            edge_actor = vtk.vtkActor()
            edge_actor.SetMapper(edge_mapper)
            edge_actor.GetProperty().SetColor(0.05, 0.05, 0.06)
            edge_actor.GetProperty().SetLineWidth(1.2)
            edge_actor.GetProperty().SetOpacity(min(1.0, drawable.opacity + 0.25))
            renderer.AddActor(edge_actor)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetMultiSamples(8)
    window.AddRenderer(renderer)
    window.SetSize(*size)
    if any(drawable.opacity < 1.0 for drawable in drawables):
        renderer.SetUseDepthPeeling(1)
        renderer.SetMaximumNumberOfPeels(12)
        renderer.SetOcclusionRatio(0.02)
        window.SetAlphaBitPlanes(1)
        window.SetMultiSamples(0)

    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.SetFocalPoint(0.0, 0.0, 0.0)
    camera.SetPosition(*direction)
    camera.SetViewUp(*up)
    renderer.ResetCamera()
    # Frame the scene tightly: project the bounding-box corners onto the view
    # plane and size the parallel projection to fit them.
    camera.OrthogonalizeViewUp()
    view_up = Vec(*camera.GetViewUp())
    forward = Vec(*camera.GetDirectionOfProjection())
    right = forward.cross(view_up)
    bounds = renderer.ComputeVisiblePropBounds()
    corners = [
        Vec(x, y, z)
        for x in bounds[0:2]
        for y in bounds[2:4]
        for z in bounds[4:6]
    ]
    horizontal = [corner.dot(right) for corner in corners]
    vertical = [corner.dot(view_up) for corner in corners]
    depth = [corner.dot(forward) for corner in corners]
    centre = (
        right * ((min(horizontal) + max(horizontal)) / 2.0)
        + view_up * ((min(vertical) + max(vertical)) / 2.0)
        + forward * ((min(depth) + max(depth)) / 2.0)
    )
    reach = max(depth) - min(depth) + 200.0
    camera.SetFocalPoint(centre.x, centre.y, centre.z)
    eye = centre - forward * reach
    camera.SetPosition(eye.x, eye.y, eye.z)
    half_h = (max(vertical) - min(vertical)) / 2.0
    half_w = (max(horizontal) - min(horizontal)) / 2.0
    camera.SetParallelScale(max(half_h, half_w * size[1] / size[0]) * 1.06 / zoom)
    renderer.ResetCameraClippingRange()

    light = vtk.vtkLight()
    light.SetLightTypeToCameraLight()
    light.SetPosition(-0.6, 0.9, 1.0)
    light.SetIntensity(0.45)
    renderer.AddLight(light)

    window.Render()
    grabber = vtk.vtkWindowToImageFilter()
    grabber.SetInput(window)
    grabber.ReadFrontBufferOff()
    grabber.Update()
    image = grabber.GetOutput()
    width, height, _ = image.GetDimensions()
    array = vtk_to_numpy(image.GetPointData().GetScalars()).reshape(height, width, -1)
    return Image.fromarray(array[::-1, :, :3].copy(), "RGB")


def _font(size: int) -> ImageFont.ImageFont:
    for name in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def sheet(
    title: str,
    panels: list[tuple[str, Image.Image]],
    columns: int,
    path: Path,
) -> None:
    title_font, label_font = _font(30), _font(21)
    panel_w = max(image.width for _, image in panels)
    panel_h = max(image.height for _, image in panels)
    rows = (len(panels) + columns - 1) // columns
    label_h, title_h, pad = 40, 64, 14
    canvas = Image.new(
        "RGB",
        (columns * (panel_w + pad) + pad, title_h + rows * (panel_h + label_h + pad) + pad),
        "white",
    )
    draw = ImageDraw.Draw(canvas)
    draw.text((pad + 4, 16), title, fill=(15, 15, 20), font=title_font)
    for index, (label, image) in enumerate(panels):
        column, row = index % columns, index // columns
        x = pad + column * (panel_w + pad)
        y = title_h + row * (panel_h + label_h + pad)
        draw.rectangle((x, y, x + panel_w - 1, y + label_h + panel_h - 1), outline=(205, 208, 214))
        draw.text((x + 10, y + 8), label, fill=(35, 38, 46), font=label_font)
        canvas.paste(image, (x + (panel_w - image.width) // 2, y + label_h))
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path)
    print(f"Wrote {path}")


# --------------------------------------------------------------------------
# Scene assembly
# --------------------------------------------------------------------------
def assembly_drawables(
    *,
    frame: str = "bracer",
    hide: tuple[str, ...] = (),
    opacity: dict[str, float] | None = None,
    keepouts: bool = False,
    wires: bool = False,
    fasteners: bool = True,
    clip: cq.Shape | None = None,
    offsets: dict[str, tuple[float, float, float]] | None = None,
) -> list[Drawable]:
    """Collect drawables; ``frame`` is 'bracer' or 'tray'."""
    opacity = opacity or {}
    offsets = offsets or {}

    def finish(name: str, shape: cq.Shape) -> cq.Shape | None:
        if name in offsets:
            shape = shape.translate(Vec(*offsets[name]))
        if frame == "bracer":
            shape = g.to_bracer(shape)
        if clip is not None:
            shape = shape.cut(clip)
            if not shape.Solids():
                return None
        return shape

    drawables: list[Drawable] = []

    def add(name: str, shape: cq.Shape, color, edges: bool = True) -> None:
        if name in hide:
            return
        final = finish(name, shape)
        if final is not None:
            drawables.append(Drawable(final, color, opacity.get(name, 1.0), edges))

    if "saddle" not in hide:
        saddle = g.make_saddle()
        if frame == "tray":
            saddle = g.tray_frame().unplace(saddle)
        if "saddle" in offsets:
            delta = Vec(*offsets["saddle"])
            saddle = saddle.translate(
                Vec(*g.tray_frame().direction(delta.x, delta.y, delta.z)) if frame == "bracer" else delta
            )
        if clip is not None:
            saddle = saddle.cut(clip)
        drawables.append(
            Drawable(saddle, g.PRINTED_COLORS["saddle"], opacity.get("saddle", 1.0))
        )

    for name, shape in g.printed_parts_tray_frame().items():
        add(name, shape, g.PRINTED_COLORS[name])
    for item in g.component_items():
        add(item.name, item.shape, item.color)
    if fasteners:
        for fastener in g.fastener_items():
            group = fastener.name.rsplit("_", 1)[0] if fastener.name[-1].isdigit() else fastener.name
            if group in hide or "fasteners" in hide:
                continue
            shape = fastener.shape
            owner = {
                "tray_bolt": "tray",
                "tray_nut": "saddle",
                "display_bolt": "bezel",
                "cover_bolt": "power_cover",
                "s3_keeper_bolt": "s3_keeper",
                "c3_keeper_bolt": "c3_keeper",
            }.get(group, "")
            if owner in hide:
                continue
            if group in offsets:
                shape = shape.translate(Vec(*offsets[group]))
            elif owner in offsets:
                shape = shape.translate(Vec(*offsets[owner]))
            if frame == "bracer":
                shape = g.to_bracer(shape)
            if clip is not None:
                shape = shape.cut(clip)
                if not shape.Solids():
                    continue
            drawables.append(Drawable(shape, (0.78, 0.80, 0.84)))
    if keepouts:
        for item in g.connector_keepouts():
            add(item.name, item.shape, item.color, edges=True)
            drawables[-1].opacity = 0.55
    if wires:
        for route in g.wire_routes():
            add(route.name, route.shape, route.color, edges=False)
    return drawables


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=g.OUTPUT_DIR)
    parser.add_argument("--reference-arm", action="store_true", help="ignore arm_fit.local.json and use the reference arm")
    args = parser.parse_args()
    out = args.output

    # ---- sheet 1: complete assembly --------------------------------------
    full = assembly_drawables()
    sheet(
        "N203 right-arm bracer v3 - assembled (bracer frame: X wrist to elbow, -Y thumb side, Z dorsal)",
        [
            ("Wearer's view: from the thumb side, looking at the screen (hand to the left)", render(full, direction=(0.12, -1.0, 0.95))),
            ("Wrist end: C3 USB (hand toward the viewer)", render(full, direction=(-1.0, -0.55, 0.55))),
            ("Back of the screen: radar window; elbow end: S3 USB ports", render(full, direction=(0.75, 1.0, 0.5))),
            ("Underside: arm cavity, deck block, strap slots", render(full, direction=(0.25, -0.55, -1.0), up=(0.0, 0.0, -1.0))),
        ],
        2,
        out / "n203_v3_assembly.png",
    )

    # ---- sheet 2: how it goes together ------------------------------------
    cut_x = g.tray_frame().point(g.S3_UC, 0.0, 0.0)[0]
    section_bracer = g.box3(-60.0, cut_x, -200.0, 200.0, -120.0, 220.0)
    section = assembly_drawables(clip=section_bracer, keepouts=True)
    exploded = assembly_drawables(
        offsets={
            "saddle": (0.0, 0.0, -42.0),
            "bezel": (0.0, -38.0 * g.SIN_A, 38.0 * g.COS_A),
            "display_msp3520": (0.0, -18.0 * g.SIN_A, 18.0 * g.COS_A),
            "power_cover": (0.0, 0.0, 34.0),
            "s3_keeper": (0.0, 0.0, 16.0),
            "c3_keeper": (0.0, 0.0, 16.0),
            "tray_bolt": (0.0, 0.0, 22.0),
        }
    )
    open_tray = assembly_drawables(
        frame="tray",
        hide=("saddle", "bezel", "display_msp3520", "power_cover", "display_bolt", "cover_bolt", "tray_bolt"),
    )
    wiring = assembly_drawables(
        frame="tray",
        hide=("saddle", "bezel", "display_msp3520", "power_cover", "display_bolt", "cover_bolt"),
        keepouts=True,
        wires=True,
    )
    sheet(
        "N203 bracer v3 - assembly, section, and wiring",
        [
            ("Exploded: saddle, tray, boards, keepers, display, bezel, cover", render(exploded, direction=(0.55, -1.0, 0.75))),
            ("Cross-section through the S3, seen from the wrist (yellow = jumper/plug space)", render(section, direction=(-1.0, 0.0, 0.0))),
            ("Tray populated, lids off (looking down on the deck)", render(open_tray, direction=(0.0, 0.0, 1.0), up=(-1.0, 0.0, 0.0), zoom=1.1)),
            ("Wire routes and jumper keep-outs, lids off", render(wiring, direction=(0.35, -0.8, 1.0))),
        ],
        2,
        out / "n203_v3_internals.png",
    )

    # ---- sheet 3: printable parts ------------------------------------------
    plates = g.print_oriented_parts()
    panels = []
    for name in ("saddle", "tray", "bezel", "power_cover", "s3_keeper", "c3_keeper", "test_coupon"):
        shape = plates[name]
        bb = shape.BoundingBox()
        label = f"{name}: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm as printed"
        panels.append(
            (
                label,
                render([Drawable(shape, (0.55, 0.60, 0.68))], direction=(0.8, -1.0, 0.85), size=(760, 560)),
            )
        )
    sheet("N203 bracer v3 - printed parts in build orientation (bed is the bottom face)", panels, 3, out / "n203_v3_print_parts.png")


if __name__ == "__main__":
    main()
