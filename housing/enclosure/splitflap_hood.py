"""
Parametric hood/enclosure for one 52-flap scottbez1 split-flap module.

Run:  ~/venvs/cad/bin/python splitflap_hood.py
Outputs (next to this file): hood.step/.stl, endcap_left.step/.stl, endcap_right.step/.stl

Coordinate frame of every output (Fusion "Z up"):
  X = width, left -> right as seen from the front. Front plate spans X 0..82.6
  Y = depth, front -> back. Front plate FRONT face is Y=0, plate occupies Y 0..3
  Z = up. Bottom edge of the front plate is Z=0, top is Z=143.53
So in Fusion: insert the STEP, then Align / As-built joint the lip's inner face to
the front face of your plate (or just move the plate to this frame).

Shape:
  - Front: a picture-frame lip that overlaps the plate edges by LIP_OVERLAP and
    a rail behind the plate on each side, so the plate sits in a snug slot.
  - Top: flat, then a quarter-ellipse curve down to a vertical back wall. The back
    clears the full flap swing envelope (radius 51.4mm around the spool axis).
  - Bottom: open. Two back tabs (M4 from below, nut trapped) and two front pegs locate it on the LED base.
  - Back-bottom corner: side walls notched so the Chainlink Driver carrier can span several hoods.
  - Sides: flat walls so modules butt together. M4x10 bolts join neighbours:
    left wall = clearance holes, right wall = hex nut traps (or heat-set inserts).
  - Motor: the 28BYJ-48 sticks ~9mm out past the plate's left edge, so both side
    walls have a slot open at the bottom. A module's motor passes through its own
    left wall and into the neighbour's right wall slot. The slot being open at the
    bottom lets the hood drop straight down over the module.
"""
import math
import os

import cadquery as cq

OUT = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- module (measured)
P_W = 82.6          # front plate width
P_H = 143.527       # front plate height
P_T = 3.0           # front plate thickness
SPOOL_Y = 31.52     # spool axis behind the plate front face
SPOOL_Z = 79.64     # spool axis above the plate bottom
FLAP_ENVELOPE_R = 51.39   # sqrt(43^2 + 28.14^2): flap tip "exclusion radius"
MODULE_REAR_Y = 53.3      # back edge of the MDF enclosure
# motor body that sticks out past the plate's LEFT edge (X<0)
MOTOR_Y = (17.52, 45.52)
MOTOR_Z = (53.64, 85.64)
MOTOR_PROTRUDE = 8.95

# ---------------------------------------------------------------- hood parameters
WALL = 2.0               # side + back wall thickness
TOP_T = 2.0              # roof thickness
LIP_T = 2.0              # front lip thickness
LIP_OVERLAP = 2.0        # how far the lip covers the plate edge (front bolt heads sit 2.36 in)
LIP_BOTTOM = True        # frame the bottom edge of the plate as well
SIDE_CLR = 0.25          # plate edge to side wall, per side
TOP_CLR = 0.4            # plate top to roof
SLOT_CLR = 0.15          # plate face to lip / rail (each side)
RAIL_DEPTH = 2.0         # rail thickness in Y (behind the plate)
RAIL_REACH = 2.5         # how far the rail sticks in from the wall
FLAP_CLR = 2.5           # extra room beyond the flap envelope
# Inner roof-to-back curve is a quarter ellipse: BACK_RUN_Y deep, BACK_DROP_Z tall.
# BACK_RUN_Y is capped at ~32 by the module's top-rear corner (Y 53.3, Z 143.5).
BACK_RUN_Y = 31.0
BACK_DROP_Z = 60.0
MOTOR_SLOT_CLR = 2.0

# Joining hardware: every structural joint uses M4x10 + M4 nut. Grip lengths are
# sized so a 10mm screw fully engages the nut without poking into anything.
FASTENER = "nut"         # "nut" (M4 hex nut trap) or "insert" (M4 heat-set insert)
BOLT_CLEAR = 4.5         # M4 clearance hole
NUT_FLATS = 7.2          # M4 nut is 7.0 across flats
NUT_DEPTH = 3.4          # M4 nut is 3.2 thick
INSERT_D = 5.6           # hole for a 5.6-5.8mm OD M4 heat-set insert
INSERT_L = 6.0
BOSS_D = 11.0
HEAD_BOSS_T = 1.5        # left wall: stiffening washer boss
NUT_BOSS_T = 4.0         # right wall: nut/insert boss (insert uses INSERT_L+1); grip 9.5
BOLTS = [(56.0, 14.0), (72.0, 62.0), (60.0, 112.0)]   # (Y, Z) on the side walls

# Chainlink Driver bay: a notch in the back-bottom corner of both side walls so the
# 198mm driver (on its carrier, against the back walls) can run across several hoods.
# Below Z~40 behind Y~62 is outside the flap swing, so nothing hits it.
DRIVER_NOTCH_Y0 = 62.0
DRIVER_NOTCH_Z1 = 45.0
# M4 holes through the back wall for the driver carrier (every hood has them; the
# carrier uses the ones just past each end of the driver PCB)
CARRIER_HOLES = [(x, z) for x in (20.0, 62.0) for z in (12.0, 30.0)]

# Base attachment: back tabs take an M4 screw from below (through the base top
# plate) into a nut trapped in the tab; front tabs carry pegs that drop into the base.
BASE_TABS = True
TAB = 12.0
TAB_T = 4.0
FRONT_TAB_Y = (5.5, 10.5)
FRONT_TAB_W = 5.0
PEG_D = 3.0
PEG_L = 2.5

# ---------------------------------------------------------------- derived
X_IN0, X_IN1 = -SIDE_CLR, P_W + SIDE_CLR
X_OUT0, X_OUT1 = X_IN0 - WALL, X_IN1 + WALL
WIDTH = X_OUT1 - X_OUT0            # also the module pitch
Y_LIP_IN = -SLOT_CLR
Y_FRONT = Y_LIP_IN - LIP_T
Z_TOP_IN = P_H + TOP_CLR
Z_TOP_OUT = Z_TOP_IN + TOP_T
Y_BACK_IN = SPOOL_Y + FLAP_ENVELOPE_R + FLAP_CLR
Y_BACK_OUT = Y_BACK_IN + WALL
RAIL_Y0 = P_T + SLOT_CLR


def side_profile(y_front, y_back, z_top, ry, rz, z_bot):
    """Closed YZ outline: front edge, flat roof, quarter ellipse (ry x rz) down to a vertical back."""
    cy, cz = y_back - ry, z_top - rz
    pts = [(cy + ry * math.sin(t), cz + rz * math.cos(t))
           for t in (math.radians(a) for a in range(0, 91, 5))]
    return (
        cq.Workplane("YZ")
        .moveTo(y_front, z_bot)
        .lineTo(y_front, z_top)
        .lineTo(*pts[0])
        .spline(pts[1:], tangents=[(1, 0), (0, -1)], includeCurrent=True)
        .lineTo(y_back, z_bot)
        .close()
    )


def profile_solid(x0, x1, **kw):
    return side_profile(**kw).extrude(x1 - x0).translate((x0, 0, 0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_x(y, z, d, x0, x1):
    return cq.Workplane("YZ").center(y, z).circle(d / 2).extrude(x1 - x0).translate((x0, 0, 0))


def hex_x(y, z, flats, x0, x1):
    return cq.Workplane("YZ").center(y, z).polygon(6, flats / math.cos(math.pi / 6)).extrude(x1 - x0).translate((x0, 0, 0))


def motor_slot(x0, x1):
    return box(x0, x1, MOTOR_Y[0] - MOTOR_SLOT_CLR, MOTOR_Y[1] + MOTOR_SLOT_CLR, -1, MOTOR_Z[1] + MOTOR_SLOT_CLR)


def build_hood():
    outer = profile_solid(X_OUT0, X_OUT1, y_front=Y_FRONT, y_back=Y_BACK_OUT, z_top=Z_TOP_OUT, ry=BACK_RUN_Y + WALL, rz=BACK_DROP_Z + WALL, z_bot=0)
    inner = profile_solid(X_IN0, X_IN1, y_front=Y_LIP_IN, y_back=Y_BACK_IN, z_top=Z_TOP_IN, ry=BACK_RUN_Y, rz=BACK_DROP_Z, z_bot=-1)
    hood = outer.cut(inner)

    # window in the lip
    zb = LIP_OVERLAP if LIP_BOTTOM else -1
    hood = hood.cut(box(LIP_OVERLAP, P_W - LIP_OVERLAP, Y_FRONT - 1, Y_LIP_IN + 0.01, zb, P_H - LIP_OVERLAP))

    # rails behind the plate (vertical, both sides)
    for x0, x1 in ((X_IN0, X_IN0 + RAIL_REACH), (X_IN1 - RAIL_REACH, X_IN1)):
        hood = hood.union(box(x0, x1, RAIL_Y0, RAIL_Y0 + RAIL_DEPTH, 0, Z_TOP_IN))

    # bosses + holes for joining neighbours
    for y, z in BOLTS:
        hood = hood.union(cyl_x(y, z, BOSS_D, X_IN0 - 0.1, X_IN0 + HEAD_BOSS_T))
        boss_t = NUT_BOSS_T if FASTENER == "nut" else INSERT_L + 1
        hood = hood.union(cyl_x(y, z, BOSS_D, X_IN1 - boss_t, X_IN1 + 0.1))
    for y, z in BOLTS:
        hood = hood.cut(cyl_x(y, z, BOLT_CLEAR, X_OUT0 - 1, X_OUT1 + 1))
        if FASTENER == "nut":
            hood = hood.cut(hex_x(y, z, NUT_FLATS, X_IN1 - NUT_BOSS_T - 0.1, X_IN1 - NUT_BOSS_T + NUT_DEPTH))
        else:
            hood = hood.cut(cyl_x(y, z, INSERT_D, X_IN1 - INSERT_L - 1.1, X_OUT1 + 1))

    # motor slots, open at the bottom, through both side walls
    hood = hood.cut(motor_slot(X_OUT0 - 1, X_IN0 + 0.01))
    hood = hood.cut(motor_slot(X_IN1 - 0.01, X_OUT1 + 1))

    # driver notch through both side walls (open at the bottom)
    for x0, x1 in ((X_OUT0 - 1, X_IN0 + 0.01), (X_IN1 - 0.01, X_OUT1 + 1)):
        hood = hood.cut(box(x0, x1, DRIVER_NOTCH_Y0, Y_BACK_IN + 0.01, -1, DRIVER_NOTCH_Z1))
    for x, z in CARRIER_HOLES:
        hood = hood.cut(cq.Workplane("XZ").center(x, z).circle(BOLT_CLEAR / 2).extrude(-10).translate((0, Y_BACK_IN - 2, 0)))

    if BASE_TABS:
        for x0, x1 in ((X_IN0, X_IN0 + TAB), (X_IN1 - TAB, X_IN1)):
            cx, cy = (x0 + x1) / 2, Y_BACK_IN - TAB / 2
            tab = box(x0, x1, Y_BACK_IN - TAB, Y_BACK_IN + 0.1, 0, TAB_T)
            tab = tab.cut(cq.Workplane("XY").center(cx, cy).circle(BOLT_CLEAR / 2).extrude(10).translate((0, 0, -1)))
            if FASTENER == "nut":
                tab = tab.cut(cq.Workplane("XY").center(cx, cy).polygon(6, NUT_FLATS / math.cos(math.pi / 6)).extrude(NUT_DEPTH + 0.1).translate((0, 0, TAB_T - NUT_DEPTH)))
            else:
                tab = tab.cut(cq.Workplane("XY").center(cx, cy).circle(INSERT_D / 2).extrude(TAB_T + 1).translate((0, 0, -0.5)))
            hood = hood.union(tab)
        for x0, x1 in ((X_IN0, X_IN0 + FRONT_TAB_W), (X_IN1 - FRONT_TAB_W, X_IN1)):
            hood = hood.union(box(x0, x1, FRONT_TAB_Y[0], FRONT_TAB_Y[1], 0, 3.0))
            hood = hood.union(cq.Workplane("XY").center(*front_peg_xy(x0, x1)).circle(PEG_D / 2).extrude(PEG_L).translate((0, 0, -PEG_L)))
    return hood


def front_peg_xy(x0, x1):
    return ((x0 + x1) / 2, sum(FRONT_TAB_Y) / 2)


def base_points():
    """Hood-local (x, y) of the 2 back screws and 2 front pegs, for the base to match."""
    screws = [((x0 + x1) / 2, Y_BACK_IN - TAB / 2) for x0, x1 in ((X_IN0, X_IN0 + TAB), (X_IN1 - TAB, X_IN1))]
    pegs = [front_peg_xy(x0, x1) for x0, x1 in ((X_IN0, X_IN0 + FRONT_TAB_W), (X_IN1 - FRONT_TAB_W, X_IN1))]
    return screws, pegs


def side_plate(x0, x1):
    return profile_solid(x0, x1, y_front=Y_FRONT, y_back=Y_BACK_OUT, z_top=Z_TOP_OUT, ry=BACK_RUN_Y + WALL, rz=BACK_DROP_Z + WALL, z_bot=0)


def build_endcap_left():
    """Bolts to the left wall of the leftmost hood, covers its protruding motor.
    Bolts go in from inside the hood; nuts sit in hex pockets on the cap's outside."""
    x1 = X_OUT0
    x0 = x1 - WALL
    cap = side_plate(x0, x1)
    bulge = MOTOR_PROTRUDE - SIDE_CLR - WALL + 1.5      # room past the wall + clearance
    shell_y = (MOTOR_Y[0] - MOTOR_SLOT_CLR - WALL, MOTOR_Y[1] + MOTOR_SLOT_CLR + WALL)
    cap = cap.union(box(x0 - bulge, x1, shell_y[0], shell_y[1], 0, MOTOR_Z[1] + MOTOR_SLOT_CLR + WALL))
    cap = cap.cut(box(x0 - bulge + WALL, x1 + 1, shell_y[0] + WALL, shell_y[1] - WALL, -1, MOTOR_Z[1] + MOTOR_SLOT_CLR))
    nb = x0 - NUT_BOSS_T          # outer face of the nut boss; grip = 1.5 + 2 + 2 + 4 = 9.5
    for y, z in BOLTS:
        cap = cap.union(cyl_x(y, z, BOSS_D, nb, x0 + 0.1))
        cap = cap.cut(cyl_x(y, z, BOLT_CLEAR, nb - 1, x1 + 1))
        cap = cap.cut(hex_x(y, z, NUT_FLATS, nb - 0.1, nb + NUT_DEPTH))
    return cap


def build_endcap_right():
    """Flat cover for the right wall of the rightmost hood (hides the motor slot).
    Bolts go in from outside into the hood's own nut traps. Washer pads on the outside
    make the grip 2 + 2 + 2 + 4 = 10 so a 10mm screw reaches through the nut without
    poking into the module."""
    x0 = X_OUT1
    cap = side_plate(x0, x0 + WALL)
    for y, z in BOLTS:
        cap = cap.union(cyl_x(y, z, BOSS_D, x0 + WALL - 0.1, x0 + 2 * WALL))
        cap = cap.cut(cyl_x(y, z, BOLT_CLEAR, x0 - 1, x0 + 2 * WALL + 1))
    return cap


def export(shape, name):
    cq.exporters.export(shape, os.path.join(OUT, name + ".step"))
    cq.exporters.export(shape, os.path.join(OUT, name + ".stl"), tolerance=0.05, angularTolerance=0.1)


if __name__ == "__main__":
    print(f"hood outer: W {WIDTH:.2f} (module pitch)  D {Y_BACK_OUT - Y_FRONT:.2f}  H {Z_TOP_OUT:.2f}")
    print(f"6 modules wide: {6 * WIDTH:.1f} mm")
    export(build_hood(), "hood")
    export(build_endcap_left(), "endcap_left")
    export(build_endcap_right(), "endcap_right")
    print("done")
