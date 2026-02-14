import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ----------------------------
# Constants (same as Arduino)
# ----------------------------
A = 200.0  # femur
B = 300.0  # tibia
L = 62.0   # coxa length

# ----------------------------
# IK helpers
# ----------------------------
def knee_from_hip_ankle_2d(hip_yz, ankle_yz, A, B, prefer_higher_z=True):
    (y0, z0) = hip_yz
    (y1, z1) = ankle_yz

    dx = y1 - y0
    dz = z1 - z0
    d = math.hypot(dx, dz)

    if d == 0 or d > A + B or d < abs(A - B):
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

    k1 = (ym + rx, zm + rz)
    k2 = (ym - rx, zm - rz)

    return max((k1, k2), key=lambda p: p[1]) if prefer_higher_z else min((k1, k2), key=lambda p: p[1])

def ik_angles_deg(x, y, z):
    hip = math.atan2(y, x)

    R = math.sqrt(x*x + y*y)
    D = R - L
    C = math.sqrt(D*D + (-z)*(-z))

    def clamp(v): return max(-1.0, min(1.0, v))

    knee = math.atan2(D, -z) + math.acos(clamp((A*A + C*C - B*B) / (2*A*C)))
    elbow = math.acos(clamp((A*A + B*B - C*C) / (2*A*B)))

    hip = hip * 180 / math.pi
    knee = knee * 180 / math.pi
    elbow = 180 - elbow * 180 / math.pi

    return hip, knee, elbow

# ----------------------------
# Figure layout
# ----------------------------
fig = plt.figure(figsize=(14, 7))
fig.canvas.manager.set_window_title("Hexapod IK – 2D + 3D (Z up)")

ax_side = fig.add_axes([0.05, 0.32, 0.27, 0.62])
ax_top  = fig.add_axes([0.37, 0.32, 0.27, 0.62])
ax_3d   = fig.add_axes([0.69, 0.20, 0.28, 0.74], projection="3d")

# Sliders
ax_sx = fig.add_axes([0.10, 0.22, 0.55, 0.03])
ax_sy = fig.add_axes([0.10, 0.16, 0.55, 0.03])
ax_sz = fig.add_axes([0.10, 0.10, 0.55, 0.03])

# Text
ax_txt = fig.add_axes([0.10, 0.01, 0.55, 0.07])
ax_txt.axis("off")
txt = ax_txt.text(0, 0.5, "", va="center")

# Sliders
sx = Slider(ax_sx, "x", -200, 200, valinit=0, valstep=1)
sy = Slider(ax_sy, "y",    0, 300, valinit=100, valstep=1)
sz = Slider(ax_sz, "z(down)", 0, 250, valinit=100, valstep=1)  # keep your original meaning

# ----------------------------
# Plot objects
# ----------------------------
side_coxa, = ax_side.plot([], [], lw=3, marker="o")
side_leg,  = ax_side.plot([], [], lw=3, marker="o")

top_coxa, = ax_top.plot([], [], lw=3, marker="o")
top_leg,  = ax_top.plot([], [], lw=3, marker="o")

leg_3d, = ax_3d.plot([], [], [], lw=3, marker="o")

# ----------------------------
# Static formatting
# ----------------------------
ax_side.set_title("Side view (Y–Z leg plane)")
ax_side.set_xlabel("Y (radial)")
ax_side.set_ylabel("Z (up-positive in plot)")
ax_side.set_aspect("equal")
ax_side.grid(True)
ax_side.set_xlim(-50, 350)
ax_side.set_ylim(-250, 250)

ax_top.set_title("Top view (X–Y)")
ax_top.set_xlabel("X")
ax_top.set_ylabel("Y")
ax_top.set_aspect("equal")
ax_top.grid(True)
ax_top.set_xlim(-250, 250)
ax_top.set_ylim(-50, 350)

# 3D: Z UP (standard)
ax_3d.set_title("3D view (X–Y–Z, Z up)")
ax_3d.set_xlabel("X")
ax_3d.set_ylabel("Y")
ax_3d.set_zlabel("Z (up)")
ax_3d.set_xlim(-250, 250)
ax_3d.set_ylim(-250, 250)
ax_3d.set_zlim(-250, 250)
ax_3d.view_init(elev=25, azim=45)

# ----------------------------
# Update
# ----------------------------
def update(_=None):
    x, y, z_down = float(sx.val), float(sy.val), float(sz.val)

    theta1 = math.atan2(y, x)
    ux, uy = math.cos(theta1), math.sin(theta1)

    # Points in 3D with Z UP:
    # body & hip are at Z=0
    body = (0.0, 0.0, 0.0)
    hip  = (L * ux, L * uy, 0.0)

    # Side-view plane still uses your IK convention: ankle_yz uses -z_down
    R = math.sqrt(x*x + y*y)
    hip_yz = (L, 0.0)
    ankle_yz = (R, -z_down)
    knee_yz = knee_from_hip_ankle_2d(hip_yz, ankle_yz, A, B)

    if knee_yz is None:
        return

    # Convert knee/ankle into 3D (Z up)
    # knee_yz[1] is in the same Z axis as ankle_yz (which used -z_down),
    # so it is already "up-positive" for plotting.
    knee  = (knee_yz[0] * ux, knee_yz[0] * uy, knee_yz[1])
    ankle = (x, y, -z_down)

    # Side view: draw coxa (0->L) and hip->knee->ankle
    side_coxa.set_data([0.0, L], [0.0, 0.0])
    side_leg.set_data([L, knee_yz[0], ankle_yz[0]],
                      [0.0, knee_yz[1], ankle_yz[1]])

    # Top view: draw body->hip and hip->knee->ankle
    top_coxa.set_data([0.0, hip[0]], [0.0, hip[1]])
    top_leg.set_data([hip[0], knee[0], ankle[0]],
                     [hip[1], knee[1], ankle[1]])

    # 3D: body->hip->knee->ankle
    leg_3d.set_data([body[0], hip[0], knee[0], ankle[0]],
                    [body[1], hip[1], knee[1], ankle[1]])
    leg_3d.set_3d_properties([body[2], hip[2], knee[2], ankle[2]])

    h, k, e = ik_angles_deg(x, y, z_down)
    txt.set_text(f"XYZ=({x:.1f}, {y:.1f}, {z_down:.1f} down)  |  Hip={h:.2f}°  Knee={k:.2f}°  Elbow={e:.2f}°")

    fig.canvas.draw_idle()

sx.on_changed(update)
sy.on_changed(update)
sz.on_changed(update)
update()

plt.show()
