import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ============================
# Robot parameters (mm)
# ============================
A = 76.0   # femur length
B = 99.0   # tibia length
L = 52.0   # coxa length

# ============================
# Body + mounting geometry (mm)
# ============================
# Your convention:
#   Z up
#   +Y points toward the right leg
#   +X is "backwards" in top view
body_half_width = 60.0      # body center -> each hip mount in +/-Y
hip_x = 0.0                 # hip mounts at x=0 for demo

# Hip mount positions in BODY frame
hipR_body = np.array([hip_x, +body_half_width, 0.0])
hipL_body = np.array([hip_x, -body_half_width, 0.0])

# Middle-leg mount yaw assumptions (adjust to match your physical robot):
# - Right leg "points" +Y
# - Left leg "points"  -Y
mountYawR = -np.pi / 2
mountYawL = +np.pi / 2

# Neutral (standing) foot targets in BODY frame
footR0_body = np.array([+40.0, +120.0, -90.0])
footL0_body = np.array([+40.0, -120.0, -90.0])

# ============================
# Math helpers
# ============================
def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def rotX(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]])

def rotZ(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]])

# ============================
# IK (matches your Arduino structure)
# Returns (theta1, theta2, theta3) in radians
# ============================
def ik_3d(x, y, z):
    R = np.sqrt(x*x + y*y)
    theta1 = np.arctan2(y, x)     # yaw

    D = R - L
    C = np.sqrt(D*D + (-z)*(-z))

    # guard acos domain
    if C < 1e-9:
        b = 0.0
    else:
        b = np.arccos(clamp((A*A + C*C - B*B) / (2*A*C)))

    g = np.arctan2(D, -z)         # matches your code
    theta2 = b + g

    h = np.arccos(clamp((A*A + B*B - C*C) / (2*A*B)))
    theta3 = np.pi - h            # 180 - h (degrees) in your code

    return theta1, theta2, theta3

# ============================
# FK for visualization (one consistent convention)
# NOTE: your servo angle definitions may differ; this is for drawing.
# ============================
def fk_points_from_angles(theta1, theta2, theta3):
    hip = np.array([0.0, 0.0, 0.0])

    # Coxa endpoint (femur joint) along +radial axis before yaw
    coxa_end_plane = np.array([L, 0.0, 0.0])

    # Femur endpoint (knee):
    # theta2 measured from "down" (-Z) toward +radial
    knee_plane = coxa_end_plane + np.array([A*np.sin(theta2), 0.0, -A*np.cos(theta2)])

    # Tibia endpoint (foot):
    # assume tibia absolute = theta2 - theta3
    tib_abs = theta2 - theta3
    foot_plane = knee_plane + np.array([B*np.sin(tib_abs), 0.0, -B*np.cos(tib_abs)])

    # Yaw rotate about Z
    Rz = rotZ(theta1)
    return (Rz @ hip, Rz @ coxa_end_plane, Rz @ knee_plane, Rz @ foot_plane)

def leg_to_body(points_leg, hip_mount_body, mountYaw):
    R = rotZ(mountYaw)
    return [hip_mount_body + (R @ p) for p in points_leg]

# ============================
# Compute both legs for a given roll
# ============================
def compute_leg_points_body(roll_deg):
    roll = np.deg2rad(roll_deg)
    Rx = rotX(roll)

    # "Feet planted" compensation in BODY coords:
    # inverse rotation for the targets
    footR_body = Rx.T @ footR0_body
    footL_body = Rx.T @ footL0_body

    # BODY -> hip-local coordinates
    pR_hip = footR_body - hipR_body
    pL_hip = footL_body - hipL_body

    # Hip-local BODY coords -> LEG coords
    pR_leg = rotZ(-mountYawR) @ pR_hip
    pL_leg = rotZ(-mountYawL) @ pL_hip

    # IK
    th1R, th2R, th3R = ik_3d(*pR_leg)
    th1L, th2L, th3L = ik_3d(*pL_leg)

    # FK (in LEG frame) -> BODY frame for plotting
    ptsR_body = leg_to_body(list(fk_points_from_angles(th1R, th2R, th3R)), hipR_body, mountYawR)
    ptsL_body = leg_to_body(list(fk_points_from_angles(th1L, th2L, th3L)), hipL_body, mountYawL)

    return ptsR_body, ptsL_body, footR_body, footL_body

# ============================
# Plot with slider
# ============================
lw = 4  # bold line width

# Simple body "bar" between hip mounts
body_line = np.vstack([hipL_body, hipR_body])

fig = plt.figure(figsize=(14, 4))
ax_top = fig.add_subplot(1, 3, 1)
ax_front = fig.add_subplot(1, 3, 2)
ax_3d = fig.add_subplot(1, 3, 3, projection="3d")

ax_top.set_title("Top view (X-Y)")
ax_top.set_xlabel("X (backwards from top)")
ax_top.set_ylabel("Y (toward right leg)")
ax_top.grid(True)
ax_top.axis("equal")

ax_front.set_title("Front view (Y-Z)")
ax_front.set_xlabel("Y")
ax_front.set_ylabel("Z (up)")
ax_front.grid(True)
ax_front.axis("equal")

ax_3d.set_title("3D view (virtual roll)")
ax_3d.set_xlabel("X")
ax_3d.set_ylabel("Y")
ax_3d.set_zlabel("Z")
ax_3d.grid(True)

# Static body lines
ax_top.plot(body_line[:,0], body_line[:,1], linewidth=lw)
ax_front.plot(body_line[:,1], body_line[:,2], linewidth=lw)
ax_3d.plot(body_line[:,0], body_line[:,1], body_line[:,2], linewidth=lw)

# Initial draw
init_roll = 0.0
ptsR, ptsL, footR, footL = compute_leg_points_body(init_roll)

(top_R_line,) = ax_top.plot([p[0] for p in ptsR], [p[1] for p in ptsR], linewidth=lw)
(top_L_line,) = ax_top.plot([p[0] for p in ptsL], [p[1] for p in ptsL], linewidth=lw)
top_scatter = ax_top.scatter([footR[0], footL[0]], [footR[1], footL[1]])

(front_R_line,) = ax_front.plot([p[1] for p in ptsR], [p[2] for p in ptsR], linewidth=lw)
(front_L_line,) = ax_front.plot([p[1] for p in ptsL], [p[2] for p in ptsL], linewidth=lw)
front_scatter = ax_front.scatter([footR[1], footL[1]], [footR[2], footL[2]])

(ax3_R_line,) = ax_3d.plot([p[0] for p in ptsR], [p[1] for p in ptsR], [p[2] for p in ptsR], linewidth=lw)
(ax3_L_line,) = ax_3d.plot([p[0] for p in ptsL], [p[1] for p in ptsL], [p[2] for p in ptsL], linewidth=lw)
ax3_scatter = ax_3d.scatter([footR[0], footL[0]], [footR[1], footL[1]], [footR[2], footL[2]])

# Stable axis limits (sample roll extremes)
all_pts = []
for deg in (-20, 0, 20):
    pr, pl, fr, fl = compute_leg_points_body(deg)
    all_pts += pr + pl + [hipR_body, hipL_body, fr, fl]
all_pts = np.array(all_pts)
mins = all_pts.min(axis=0)
maxs = all_pts.max(axis=0)

ax_top.set_xlim(mins[0]-20, maxs[0]+20)
ax_top.set_ylim(mins[1]-20, maxs[1]+20)

ax_front.set_xlim(mins[1]-20, maxs[1]+20)
ax_front.set_ylim(mins[2]-20, maxs[2]+20)

ax_3d.set_xlim(mins[0]-20, maxs[0]+20)
ax_3d.set_ylim(mins[1]-20, maxs[1]+20)
ax_3d.set_zlim(mins[2]-20, maxs[2]+20)
ranges = maxs - mins
ax_3d.set_box_aspect(ranges if np.all(ranges > 0) else (1, 1, 1))

# Slider
slider_ax = fig.add_axes([0.15, 0.02, 0.7, 0.05])
roll_slider = Slider(slider_ax, "Roll (deg)", -20.0, 20.0, valinit=init_roll, valstep=0.5)

def update(val):
    roll_deg = roll_slider.val
    ptsR, ptsL, footR, footL = compute_leg_points_body(roll_deg)

    # Top
    top_R_line.set_data([p[0] for p in ptsR], [p[1] for p in ptsR])
    top_L_line.set_data([p[0] for p in ptsL], [p[1] for p in ptsL])
    top_scatter.set_offsets(np.array([[footR[0], footR[1]], [footL[0], footL[1]]]))

    # Front
    front_R_line.set_data([p[1] for p in ptsR], [p[2] for p in ptsR])
    front_L_line.set_data([p[1] for p in ptsL], [p[2] for p in ptsL])
    front_scatter.set_offsets(np.array([[footR[1], footR[2]], [footL[1], footL[2]]]))

    # 3D
    ax3_R_line.set_data([p[0] for p in ptsR], [p[1] for p in ptsR])
    ax3_R_line.set_3d_properties([p[2] for p in ptsR])
    ax3_L_line.set_data([p[0] for p in ptsL], [p[1] for p in ptsL])
    ax3_L_line.set_3d_properties([p[2] for p in ptsL])
    ax3_scatter._offsets3d = ([footR[0], footL[0]],
                              [footR[1], footL[1]],
                              [footR[2], footL[2]])

    ax_3d.set_title(f"3D view (virtual roll = {roll_deg:.1f}°)")
    fig.canvas.draw_idle()

roll_slider.on_changed(update)

plt.tight_layout(rect=[0, 0.08, 1, 1])
plt.show()
