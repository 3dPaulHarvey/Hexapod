import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ----------------------------
# Constants (same as your sketch)
# ----------------------------
A = 200.0  # femur
B = 300.0  # tibia
L = 62.0   # hip offset from body origin

# ----------------------------
# IK helpers
# ----------------------------
def knee_from_hip_ankle_2d(hip_yz, ankle_yz, A, B, prefer_higher_z=True):
    """
    Solve knee point in the leg plane (Y,Z) via circle-circle intersection.
    Returns knee_yz or None if unreachable.
    """
    (y0, z0) = hip_yz
    (y1, z1) = ankle_yz

    dx = y1 - y0
    dz = z1 - z0
    d = math.hypot(dx, dz)

    # Reachability
    if d == 0 or d > (A + B) or d < abs(A - B):
        return None

    a = (A*A - B*B + d*d) / (2*d)
    h2 = A*A - a*a
    if h2 < 0:
        return None
    h = math.sqrt(max(0.0, h2))

    ym = y0 + a * dx / d
    zm = z0 + a * dz / d

    rx = -dz * (h / d)
    rz =  dx * (h / d)

    knee1 = (ym + rx, zm + rz)
    knee2 = (ym - rx, zm - rz)

    return max((knee1, knee2), key=lambda p: p[1]) if prefer_higher_z else min((knee1, knee2), key=lambda p: p[1])

def ik_angles_deg(x, y, z):
    """
    Match your Arduino math (including the -z usage and theta3 flip).
    Returns (hip, knee, elbow) in degrees (not normalized).
    """
    hip = math.atan2(y, x)

    R = math.sqrt(x*x + y*y)
    D = R - L
    C = math.sqrt(D*D + (-z)*(-z))

    # Clamp acos inputs to [-1, 1] to avoid NaNs from float error
    def clamp(v): return max(-1.0, min(1.0, v))

    knee = math.atan2(D, -z) + math.acos(clamp((A*A + C*C - B*B) / (2*A*C)))
    elbow = math.acos(clamp((A*A + B*B - C*C) / (2*A*B)))

    hip = hip * 180.0 / math.pi
    knee = knee * 180.0 / math.pi
    elbow = elbow * 180.0 / math.pi
    elbow = 180.0 - elbow

    return hip, knee, elbow

# ----------------------------
# Figure + axes layout
# ----------------------------
fig = plt.figure()
fig.canvas.manager.set_window_title("Hexapod IK – Sliders")

ax_side = fig.add_axes([0.08, 0.33, 0.38, 0.60])  # [left, bottom, width, height]
ax_top  = fig.add_axes([0.54, 0.33, 0.38, 0.60])

# Slider axes
ax_sx = fig.add_axes([0.10, 0.22, 0.80, 0.03])
ax_sy = fig.add_axes([0.10, 0.16, 0.80, 0.03])
ax_sz = fig.add_axes([0.10, 0.10, 0.80, 0.03])

# Readout text
ax_txt = fig.add_axes([0.10, 0.01, 0.80, 0.08])
ax_txt.axis("off")
txt = ax_txt.text(0.0, 0.5, "", va="center", fontsize=10)

# ----------------------------
# Sliders
# ----------------------------
sx = Slider(ax_sx, "x", -200.0, 200.0, valinit=0.0, valstep=1.0)
sy = Slider(ax_sy, "y",    0.0, 300.0, valinit=100.0, valstep=1.0)
sz = Slider(ax_sz, "z",    0.0, 250.0, valinit=100.0, valstep=1.0)

# ----------------------------
# Plot objects (lines + points)
# ----------------------------
# side view: hip->knee->ankle in (Y_radial, Z)
side_line1, = ax_side.plot([], [], marker="o")  # hip->knee
side_line2, = ax_side.plot([], [], marker="o")  # knee->ankle
side_unreach, = ax_side.plot([], [], linestyle="--")  # hip->ankle (unreachable)

# top view: hip->knee->ankle in (X, Y)
top_line1, = ax_top.plot([], [], marker="o")    # hip->knee
top_line2, = ax_top.plot([], [], marker="o")    # knee->ankle
top_unreach, = ax_top.plot([], [], linestyle="--")

# ----------------------------
# Static axes formatting
# ----------------------------
ax_side.set_title("Side view (Y–Z leg plane)")
ax_side.set_xlabel("Y (radial)")
ax_side.set_ylabel("Z")
ax_side.grid(True)
ax_side.set_aspect("equal", adjustable="box")

ax_top.set_title("Top view (X–Y)")
ax_top.set_xlabel("X")
ax_top.set_ylabel("Y")
ax_top.grid(True)
ax_top.set_aspect("equal", adjustable="box")

# Set reasonable view limits (tweak if you want)
ax_side.set_xlim(-50, 350)
ax_side.set_ylim(-250, 250)
ax_top.set_xlim(-250, 250)
ax_top.set_ylim(-50, 350)

# ----------------------------
# Update function
# ----------------------------
def update(_=None):
    x = float(sx.val)
    y = float(sy.val)
    z = float(sz.val)

    # Side-view geometry
    R = math.sqrt(x*x + y*y)
    hip_yz = (L, 0.0)
    ankle_yz = (R, -z)  # your convention: use -z in the leg plane
    knee_yz = knee_from_hip_ankle_2d(hip_yz, ankle_yz, A, B, prefer_higher_z=True)

    # Top-view projection using theta1
    theta1 = math.atan2(y, x)
    ux, uy = math.cos(theta1), math.sin(theta1)
    hip_xy = (L * ux, L * uy)
    ankle_xy = (x, y)

    # Update plots
    if knee_yz is None:
        # Unreachable: show dashed hip->ankle
        side_line1.set_data([], [])
        side_line2.set_data([], [])
        side_unreach.set_data([hip_yz[0], ankle_yz[0]], [hip_yz[1], ankle_yz[1]])

        top_line1.set_data([], [])
        top_line2.set_data([], [])
        top_unreach.set_data([hip_xy[0], ankle_xy[0]], [hip_xy[1], ankle_xy[1]])

        txt.set_text(f"XYZ=({x:.1f}, {y:.1f}, {z:.1f})  |  UNREACHABLE for A={A}, B={B}, L={L}")
    else:
        side_unreach.set_data([], [])
        side_line1.set_data([hip_yz[0], knee_yz[0]], [hip_yz[1], knee_yz[1]])
        side_line2.set_data([knee_yz[0], ankle_yz[0]], [knee_yz[1], ankle_yz[1]])

        knee_r = knee_yz[0]  # radial coordinate
        knee_xy = (knee_r * ux, knee_r * uy)

        top_unreach.set_data([], [])
        top_line1.set_data([hip_xy[0], knee_xy[0]], [hip_xy[1], knee_xy[1]])
        top_line2.set_data([knee_xy[0], ankle_xy[0]], [knee_xy[1], ankle_xy[1]])

        hip_deg, knee_deg, elbow_deg = ik_angles_deg(x, y, z)
        txt.set_text(
            f"XYZ=({x:.1f}, {y:.1f}, {z:.1f})   |   Hip={hip_deg:.2f}°   Knee={knee_deg:.2f}°   Elbow={elbow_deg:.2f}°"
        )

    fig.canvas.draw_idle()

# Hook sliders
sx.on_changed(update)
sy.on_changed(update)
sz.on_changed(update)

# Initial draw
update()
plt.show()
