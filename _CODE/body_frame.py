# hexapod_corner_fan_frames_viz.py
# Shows how "fanned out" corner legs (e.g., 30 deg) rotate the hip frames about +Z.
# Body frame: +z up, +y to robot's right, +x to robot's rear (forward = -x)

import numpy as np
import matplotlib.pyplot as plt


def rotz(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)


def draw_frame(ax, origin: np.ndarray, R: np.ndarray, name: str, L: float = 0.08):
    # R columns are the frame's x,y,z axes expressed in world/body coords
    colors = ['r', 'g', 'b']  # x,y,z
    labels = ['x', 'y', 'z']
    for i in range(3):
        v = R[:, i] * L
        ax.quiver(origin[0], origin[1], origin[2],
                  v[0], v[1], v[2],
                  arrow_length_ratio=0.2, linewidth=2, color=colors[i])
        tip = origin + v
        ax.text(tip[0], tip[1], tip[2], f"{name}.{labels[i]}", fontsize=9)

    ax.scatter([origin[0]], [origin[1]], [origin[2]], s=18)
    ax.text(origin[0], origin[1], origin[2], f" {name}", fontsize=10, fontweight='bold')


def draw_nominal_outward(ax, origin: np.ndarray, v_out: np.ndarray, name: str, L: float = 0.12):
    v = v_out / (np.linalg.norm(v_out) + 1e-12) * L
    ax.quiver(origin[0], origin[1], origin[2],
              v[0], v[1], v[2],
              arrow_length_ratio=0.15, linewidth=2)
    tip = origin + v
    ax.text(tip[0], tip[1], tip[2], f"{name}.out", fontsize=9, fontweight='bold')


def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max([x_range, y_range, z_range])
    x_mid = np.mean(x_limits)
    y_mid = np.mean(y_limits)
    z_mid = np.mean(z_limits)
    half = max_range / 2.0
    ax.set_xlim3d([x_mid - half, x_mid + half])
    ax.set_ylim3d([y_mid - half, y_mid + half])
    ax.set_zlim3d([z_mid - half, z_mid + half])


def format_ax(ax, title: str, lim: float = 0.45):
    ax.set_title(title)
    ax.set_xlabel("x (rear +)")
    ax.set_ylabel("y (right +)")
    ax.set_zlabel("z (up +)")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-0.05, lim)
    set_axes_equal(ax)


def main():
    # ---- Edit to match your robot ----
    half_body_width = 0.20
    front_x = -0.18
    mid_x   =  0.00
    rear_x  =  0.18

    fan_deg = 30.0                 # your corner fan-out angle
    fan = np.deg2rad(fan_deg)

    # Body frame as you defined it
    R_B = np.eye(3)
    O_B = np.array([0.0, 0.0, 0.0])

    # Hip locations (RF/RM/RR and LF/LM/LR)
    hips = {
        "RF": np.array([front_x, +half_body_width, 0.0]),
        "RM": np.array([mid_x,   +half_body_width, 0.0]),
        "RR": np.array([rear_x,  +half_body_width, 0.0]),
        "LF": np.array([front_x, -half_body_width, 0.0]),
        "LM": np.array([mid_x,   -half_body_width, 0.0]),
        "LR": np.array([rear_x,  -half_body_width, 0.0]),
    }

    # ---- Per-hip yaw about +Z (mounting angle) ----
    # Choose angles so the CORNERS "fan out" from the body center by ±fan.
    # Meanings:
    #  - Right side: outward nominal is +y
    #  - Left side:  outward nominal is -y
    # The corner hips are rotated so "outward" points diagonally.
    #
    # These choices are a typical convention:
    #  RF outward points forward-right => yaw +fan
    #  RR outward points rear-right    => yaw -fan
    #  LF outward points forward-left  => yaw -fan
    #  LR outward points rear-left     => yaw +fan
    hip_yaw = {
        "RF": +fan,  "RM": 0.0,  "RR": -fan,
        "LF": -fan,  "LM": 0.0,  "LR": +fan,
    }

    # ---- Convention 1 (robust/common): hip frames = body frame rotated by mount yaw only ----
    # Here +y is NOT guaranteed to be "outward" on the left; it's just consistent everywhere.
    R_hip_C1 = {name: rotz(hip_yaw[name]) @ R_B for name in hips.keys()}

    # ---- Convention 2 (IK-reuse-friendly): make each hip's +y be "outward" for that side ----
    # Do a 180° about Z for all LEFT hips, then apply the mount yaw.
    # (Stays right-handed; note this flips x for left hips too, which is expected.)
    R_hip_C2 = {}
    for name in hips.keys():
        left_flip = rotz(np.pi) if name.startswith("L") else np.eye(3)
        R_hip_C2[name] = rotz(hip_yaw[name]) @ left_flip @ R_B

    # Nominal outward directions in BODY coordinates (to annotate)
    out_dir_body = {}
    for name in hips.keys():
        if name.startswith("R"):
            base = np.array([0.0, +1.0, 0.0])  # +y
        else:
            base = np.array([0.0, -1.0, 0.0])  # -y
        out_dir_body[name] = rotz(hip_yaw[name]) @ base  # fan it

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        f"Corner leg fan-out frames (fan = {fan_deg:.0f}°)\n"
        "Red=x, Green=y, Blue=z   |   Body: +x rear, +y right, +z up",
        fontsize=14
    )

    # --- TOP views (focus on corners) ---
    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    format_ax(ax1, "Convention 1 (Top): R_hip = Rz(mount_yaw) · R_body (common/robust)")
    draw_frame(ax1, O_B, R_B, "B", L=0.11)
    for leg in ["RF", "RR", "LF", "LR"]:  # corners only
        draw_frame(ax1, hips[leg], R_hip_C1[leg], leg, L=0.08)
        draw_nominal_outward(ax1, hips[leg], out_dir_body[leg], leg, L=0.12)
    ax1.view_init(elev=90, azim=-45)

    ax2 = fig.add_subplot(2, 2, 2, projection="3d")
    format_ax(ax2, "Convention 2 (Top): left hips pre-rotated 180° so hip +y is outward (IK-reuse-friendly)")
    draw_frame(ax2, O_B, R_B, "B", L=0.11)
    for leg in ["RF", "RR", "LF", "LR"]:
        draw_frame(ax2, hips[leg], R_hip_C2[leg], leg, L=0.08)
        draw_nominal_outward(ax2, hips[leg], out_dir_body[leg], leg, L=0.12)
    ax2.view_init(elev=90, azim=-45)

    # --- 3D views (same corners) ---
    ax3 = fig.add_subplot(2, 2, 3, projection="3d")
    format_ax(ax3, "Convention 1 (3D): corners fanned by mount_yaw, frames stay globally consistent")
    draw_frame(ax3, O_B, R_B, "B", L=0.11)
    for leg in ["RF", "RR", "LF", "LR"]:
        draw_frame(ax3, hips[leg], R_hip_C1[leg], leg, L=0.08)
        draw_nominal_outward(ax3, hips[leg], out_dir_body[leg], leg, L=0.12)
    ax3.view_init(elev=22, azim=-55)

    ax4 = fig.add_subplot(2, 2, 4, projection="3d")
    format_ax(ax4, "Convention 2 (3D): corners fanned, and hip +y is outward on BOTH sides")
    draw_frame(ax4, O_B, R_B, "B", L=0.11)
    for leg in ["RF", "RR", "LF", "LR"]:
        draw_frame(ax4, hips[leg], R_hip_C2[leg], leg, L=0.08)
        draw_nominal_outward(ax4, hips[leg], out_dir_body[leg], leg, L=0.12)
    ax4.view_init(elev=22, azim=-55)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()