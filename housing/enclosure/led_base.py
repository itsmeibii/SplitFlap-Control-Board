"""
LED base: holds 2x HUB75 64x32 P2.5 panels (160x80) under the row of 6 split-flap
hoods, plus the control board + Pi 4. Also the Chainlink Driver carrier that bolts
to the back walls of the hoods.

Run:  ~/venvs/cad/bin/python led_base.py
Outputs: base_seg_{A,B,C}, driver_carrier (.step/.stl)
         row_assembly.step  (everything placed, with reference boxes for the
                             panels / driver PCB / Pi; upload this one to Fusion)

Same frame as splitflap_hood.py, with hood 0 at the origin: X along the row,
Y front->back (front plate face Y=0), Z up. The hoods sit on Z=0 and the base is below.

  - Length = 6 hoods + 2 end caps (flush with the hood row), depth = hood depth.
  - Front: a lip frames the panels with 2mm overlap (same look as the hoods).
    Behind the panels, a strip along the top and one along the bottom carry slots;
    M3 screws go through the slots into the panels' own back holes and pull them
    back against the strips.
  - Printed in 3 segments (bed 256). Joints line up with the hood joints between
    hoods 1|2 and 3|4. Internal ring flanges are bolted with M4x10 (nut traps).
  - Hood -> base: 2 front pegs per hood drop into holes. 2 M4x10 per hood go up
    from inside the base, through the top plate, into nuts trapped in the hood's back tabs.
  - Back is open. Segment C has the Pi 4 on M2.5 standoffs, USB/Ethernet window in
    the right end wall, USB-C/HDMI reachable from the open back.
"""
import math
import os

import cadquery as cq

import splitflap_hood as H

OUT = os.path.dirname(os.path.abspath(__file__))
box, hex_x = H.box, H.hex_x

# ---------------------------------------------------------------- row
N_MOD = 6
PITCH = H.WIDTH
XL = H.X_OUT0 - H.WALL                              # outer face of left end cap
XR = (N_MOD - 1) * PITCH + H.X_OUT1 + H.WALL        # outer face of right end cap
XC = (XL + XR) / 2
YF, YB = H.Y_FRONT, H.Y_BACK_OUT

# ---------------------------------------------------------------- panels (MEASURE PANEL_T + PANEL_HOLE_INSET!)
N_PANELS = 2
PANEL_W, PANEL_H = 160.0, 80.0
PANEL_T = 14.0            # front face to the back surface the screw holes are in
PANEL_HOLE_INSET = 10.0   # panel top (or bottom) edge to the centre of its screw holes
PANEL_CLR = 0.3
LIP_OVERLAP = 2.0
STRIP_T = 3.0             # mounting strips behind the panels
PANEL_SCREW_SLOT = 3.4    # M3 (the panels' holes are M3)

# ---------------------------------------------------------------- base shell
WALL = 2.5
T_TOP = 3.0
T_FLOOR = 3.0
PANEL_Z1 = -T_TOP - PANEL_CLR
PANEL_Z0 = PANEL_Z1 - PANEL_H
FLOOR_TOP = PANEL_Z0 - PANEL_CLR
ZB = FLOOR_TOP - T_FLOOR                             # base bottom
PX0 = XC - N_PANELS * PANEL_W / 2
PX1 = XC + N_PANELS * PANEL_W / 2
PANEL_BACK = PANEL_T + PANEL_CLR
HOLE_ROWS = (PANEL_Z1 - PANEL_HOLE_INSET, PANEL_Z0 + PANEL_HOLE_INSET)

# segments: joints on hood joints
JOINTS = [XL, 2 * PITCH + H.X_OUT0, 4 * PITCH + H.X_OUT0, XR]
SEG_NAMES = "ABC"
FLANGE_Y0 = 20.0          # ring flanges start behind the panel strips
FLANGE_BORDER = 12.0
FLANGE_T_CLR, FLANGE_T_NUT = 3.0, 5.0       # grip 8, nut in the far end
HOOD_SCREW_BOSS = 3.0     # under the top plate: grip 3 + 3 + tab 4 = 10

# top-plate cable openings (hood-local X, Y), under the hood back zone
CABLE_OPEN = ((16.0, 66.0), (58.0, 71.0))

# Pi 4 in segment C, ports facing the open back (USB-C/HDMI) and the right end (USB/Ethernet)
PI_W, PI_D = 85.0, 56.0
PI_X0 = XR - WALL - 8.0 - PI_W
PI_Y1 = YB - 3.0
PI_STANDOFF_H = 5.0
PI_HOLES = [(PI_X0 + x, PI_Y1 - y) for x in (3.5, 61.5) for y in (3.5, 52.5)]

# ---------------------------------------------------------------- Chainlink Driver
DRV_L, DRV_W = 198.12, 30.48
DRV_HOLES = [(3.302, 3.302), (194.818, 3.302), (3.302, 27.178), (99.06, 27.178), (194.818, 27.178)]
CARRIER_T = 4.0
CARRIER_Z = (4.5, 39.0)
DRV_STANDOFF = 5.0
DRV_PARTS_H = 12.0         # tallest part above the PCB (IDC box header / JST + plug)
DRV_TOP_Z = 36.5           # PCB top edge (board y=0 edge, JST motor connectors) faces up
# (hood index, hood-local x) of the back-wall holes used: the ones just past each
# end of the PCB, so the screws and nuts stay clear of the board
CARRIER_MOUNTS = [(1, 62.0), (4, 20.0)]
CARRIER_X = (CARRIER_MOUNTS[0][0] * PITCH + CARRIER_MOUNTS[0][1] - 8.0,
             CARRIER_MOUNTS[1][0] * PITCH + CARRIER_MOUNTS[1][1] + 8.0)
CARRIER_NUT_DEPTH = 1.5    # shallow pocket so the nut can't spin; grip 2 + 4 = 6


def cyl_z(x, y, d, z0, z1):
    return cq.Workplane("XY").center(x, y).circle(d / 2).extrude(z1 - z0).translate((0, 0, z0))


def hex_z(x, y, flats, z0, z1):
    return cq.Workplane("XY").center(x, y).polygon(6, flats / math.cos(math.pi / 6)).extrude(z1 - z0).translate((0, 0, z0))


def cyl_y(x, z, d, y0, y1):
    return cq.Workplane("XZ").center(x, z).circle(d / 2).extrude(-(y1 - y0)).translate((0, y0, 0))


def hex_y(x, z, flats, y0, y1):
    return cq.Workplane("XZ").center(x, z).polygon(6, flats / math.cos(math.pi / 6)).extrude(-(y1 - y0)).translate((0, y0, 0))


def slot_y(xc, z, length, w, y0, y1):
    return cq.Workplane("XZ").center(xc, z).slot2D(length, w).extrude(-(y1 - y0)).translate((0, y0, 0))


def flange_bolts():
    return [(y, z) for y in (FLANGE_Y0 + FLANGE_BORDER / 2, YB - FLANGE_BORDER / 2)
            for z in (-T_TOP - FLANGE_BORDER / 2, FLOOR_TOP + FLANGE_BORDER / 2)]


def build_base_full():
    b = box(XL, XR, YF, YB, ZB, 0)
    b = b.cut(box(XL + WALL, XR - WALL, H.Y_LIP_IN, YB + 1, FLOOR_TOP, -T_TOP))
    # panel window
    b = b.cut(box(PX0 + LIP_OVERLAP, PX1 - LIP_OVERLAP, YF - 1, H.Y_LIP_IN + 0.01, PANEL_Z0 + LIP_OVERLAP, PANEL_Z1 - LIP_OVERLAP))

    # panel mounting strips: one hangs from the top plate, one stands on the floor.
    # A long slot per panel per row, so the exact X of the panel's holes doesn't matter.
    ys = (PANEL_BACK, PANEL_BACK + STRIP_T)
    top_row, bot_row = HOLE_ROWS
    b = b.union(box(PX0, PX1, *ys, top_row - 6, -T_TOP + 0.1))
    b = b.union(box(PX0, PX1, *ys, FLOOR_TOP - 0.1, bot_row + 6))
    for k in range(N_PANELS):
        cx = PX0 + (k + 0.5) * PANEL_W
        for z in HOLE_ROWS:
            b = b.cut(slot_y(cx, z, PANEL_W - 12, PANEL_SCREW_SLOT, ys[0] - 1, ys[1] + 1))

    screws, pegs = H.base_points()
    for k in range(N_MOD):
        ox = k * PITCH
        for x, y in screws:
            b = b.union(cyl_z(ox + x, y, 10.0, -T_TOP - HOOD_SCREW_BOSS, -T_TOP + 0.1))
            b = b.cut(cyl_z(ox + x, y, H.BOLT_CLEAR, -T_TOP - HOOD_SCREW_BOSS - 1, 1))
        for x, y in pegs:
            b = b.cut(cyl_z(ox + x, y, H.PEG_D + 0.4, -T_TOP - 1, 1))
        (cx0, cx1), (cy0, cy1) = CABLE_OPEN
        b = b.cut(box(ox + cx0, ox + cx1, cy0, cy1, -T_TOP - 1, 1))

    # Pi standoffs (M2.5 self-tap)
    for x, y in PI_HOLES:
        b = b.union(cyl_z(x, y, 6.0, FLOOR_TOP - 0.1, FLOOR_TOP + PI_STANDOFF_H))
        b = b.cut(cyl_z(x, y, 2.2, FLOOR_TOP - 1, FLOOR_TOP + PI_STANDOFF_H + 1))
    # right end wall window for USB/Ethernet
    pi_top = FLOOR_TOP + PI_STANDOFF_H + 1.6
    b = b.cut(box(XR - WALL - 1, XR + 1, PI_Y1 - PI_D + 2, PI_Y1 - 2, pi_top - 2.5, pi_top + 18.5))
    return b


def build_segment(i, full=None):
    x0, x1 = JOINTS[i], JOINTS[i + 1]
    seg = (full or build_base_full()).intersect(box(x0, x1, YF - 1, YB + 1, ZB - 1, 1))
    ring_out = (FLANGE_Y0, YB, ZB, 0)
    ring_in = (FLANGE_Y0 + FLANGE_BORDER, YB - FLANGE_BORDER, FLOOR_TOP + FLANGE_BORDER, -T_TOP - FLANGE_BORDER)
    if i > 0:      # left flange: clearance holes
        fl = box(x0, x0 + FLANGE_T_CLR, *ring_out[:2], *ring_out[2:])
        fl = fl.cut(box(x0 - 1, x0 + FLANGE_T_CLR + 1, *ring_in[:2], *ring_in[2:]))
        seg = seg.union(fl)
        for y, z in flange_bolts():
            seg = seg.cut(H.cyl_x(y, z, H.BOLT_CLEAR, x0 - 1, x0 + FLANGE_T_CLR + 1))
    if i < len(JOINTS) - 2:  # right flange: nut traps on its inner face
        fl = box(x1 - FLANGE_T_NUT, x1, *ring_out[:2], *ring_out[2:])
        fl = fl.cut(box(x1 - FLANGE_T_NUT - 1, x1 + 1, *ring_in[:2], *ring_in[2:]))
        seg = seg.union(fl)
        for y, z in flange_bolts():
            seg = seg.cut(H.cyl_x(y, z, H.BOLT_CLEAR, x1 - FLANGE_T_NUT - 1, x1 + 1))
            seg = seg.cut(hex_x(y, z, H.NUT_FLATS, x1 - FLANGE_T_NUT - 0.1, x1 - FLANGE_T_NUT + H.NUT_DEPTH))
    return seg


def driver_frame():
    """PCB placement: returns (x0 of board, y of PCB back face, z of board y=0 edge)."""
    x0 = XC - DRV_L / 2
    y_back = H.Y_BACK_IN - CARRIER_T - DRV_STANDOFF
    return x0, y_back, DRV_TOP_Z


def build_driver_carrier():
    y1 = H.Y_BACK_IN
    y0 = y1 - CARRIER_T
    c = box(*CARRIER_X, y0, y1, *CARRIER_Z)
    # hood attachment: M4x10 from outside the hood back, nut in a shallow pocket on the front
    for k, hx in CARRIER_MOUNTS:
        for hhx, z in H.CARRIER_HOLES:
            if hhx != hx:
                continue
            x = k * PITCH + hx
            c = c.cut(cyl_y(x, z, H.BOLT_CLEAR, y0 - 1, y1 + 1))
            c = c.cut(hex_y(x, z, H.NUT_FLATS, y0 - 0.1, y0 + CARRIER_NUT_DEPTH))
    # PCB standoffs (M3 self-tap, 2.7mm pilot; the board's holes are M3)
    bx, by, bz = driver_frame()
    for hx, hy in DRV_HOLES:
        x, z = bx + hx, bz - hy
        c = c.union(cyl_y(x, z, 7.0, y0 - DRV_STANDOFF, y0 + 0.1))
        c = c.cut(cyl_y(x, z, 2.7, y0 - DRV_STANDOFF - 1, y1 - 1))
    return c


def ref_boxes():
    bx, by, bz = driver_frame()
    return {
        "REF_panels": box(PX0, PX1, 0, PANEL_T, PANEL_Z0, PANEL_Z1),
        "REF_driver_pcb": box(bx, bx + DRV_L, by - 1.6, by, bz - DRV_W, bz),
        "REF_driver_parts": box(bx, bx + DRV_L, by - 1.6 - DRV_PARTS_H, by - 1.6, bz - DRV_W, bz),
        "REF_pi4": box(PI_X0, PI_X0 + PI_W, PI_Y1 - PI_D, PI_Y1, FLOOR_TOP + PI_STANDOFF_H, FLOOR_TOP + PI_STANDOFF_H + 18),
    }


def export(shape, name):
    cq.exporters.export(shape, os.path.join(OUT, name + ".step"))
    cq.exporters.export(shape, os.path.join(OUT, name + ".stl"), tolerance=0.05, angularTolerance=0.1)


def build_all():
    parts = {}
    hood = H.build_hood()
    for k in range(N_MOD):
        parts[f"hood_{k + 1}"] = (hood.translate((k * PITCH, 0, 0)), (0.36, 0.56, 0.84))
    parts["endcap_left"] = (H.build_endcap_left(), (0.3, 0.68, 0.42))
    parts["endcap_right"] = (H.build_endcap_right().translate(((N_MOD - 1) * PITCH, 0, 0)), (0.3, 0.68, 0.42))
    full = build_base_full()
    for i, n in enumerate(SEG_NAMES):
        parts[f"base_seg_{n}"] = (build_segment(i, full), (0.55, 0.55, 0.58))
    parts["driver_carrier"] = (build_driver_carrier(), (0.95, 0.6, 0.2))
    for n, s in ref_boxes().items():
        parts[n] = (s, (0.85, 0.2, 0.2) if "parts" in n else (0.1, 0.45, 0.2))
    return parts


if __name__ == "__main__":
    print(f"row {XR - XL:.1f} long, base {ZB:.1f}..0 tall ({-ZB:.1f}), panels X {PX0:.2f}..{PX1:.2f}")
    print("segments:", [f"{JOINTS[i + 1] - JOINTS[i]:.1f}" for i in range(3)],
          f"carrier {CARRIER_X[1] - CARRIER_X[0]:.1f}")
    parts = build_all()
    asm = cq.Assembly(name="splitflap_row")
    for n, (s, col) in parts.items():
        asm.add(s, name=n, color=cq.Color(*col))
    asm.export(os.path.join(OUT, "row_assembly.step"))
    for n in SEG_NAMES:
        export(parts[f"base_seg_{n}"][0], f"base_seg_{n}")
    export(parts["driver_carrier"][0], "driver_carrier")
    print("done")
