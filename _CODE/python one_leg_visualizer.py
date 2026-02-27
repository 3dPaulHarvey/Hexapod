import numpy as np
import matplotlib.pyplot as plt

# ----------------------------
# Geometry (mm) from your IK setup
# ----------------------------
COXA_L = 52.0   # L
FEMUR_A = 76.0  # A
TIBIA_B = 99.0  # B

# Middle-right hip mount (body frame), as you specified:
HIP_IN_BODY = np.array([0.0, 50.0, 0.0])  # [xB, yB, zB] mm


# ----------------------------
# Rotation helpers
# ----------------------------
def rot_x(deg: float) -> np.ndarray:
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]])

def rot_y(deg: float) -> np.ndarray:
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])

def rot_z(deg: float) -> np.ndarray:
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[c,-s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

def r_body_from_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Body rotation matrix.
    Convention here: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    return rot_z(yaw_deg) @ rot_y(pitch_deg) @ rot_x(roll_deg)


# ----------------------------
# Frame conversion: body vector -> leg vector (middle-right leg)
# We define:
#   x_leg = +Y_body (outward)
#   y_leg = +X_body
#   z_leg = +Z_body
# ----------------------------
def body_vec_to_leg_vec(v_body: np.ndarray) -> np.ndarray:
    x_leg = v_body[1]
    y_leg = v_body[0]
    z_leg = v_body[2]
    return np.array([x_leg, y_leg, z_leg])


# ----------------------------
# Your IK (same structure as your Arduino math)
# Returns angles in degrees: theta1 (yaw), theta2 (femur), theta3 (tibia)
# IMPORTANT: this is "math IK"; servo zero offsets / direction flips are separate.
# ----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def acos_safe(x):
    return np.arccos(clamp(x, -1.0, 1.0))

def leg_ik_math(x_leg: float, y_leg: float, z_leg: float):
    """
    Matches your working approach:
      R = sqrt(x^2 + y^2)
      theta1 = atan2(y, x)
      D = R - L
      g = atan2(D, -z)
      C = sqrt(D^2 + (-z)^2)
      b = acos((A^2 + C^2 - B^2) / (2AC))
      theta2 = b + g
      h = acos((A^2 + B^2 - C^2) / (2AB))
      theta3 = pi - h
    """
    R = np.sqrt(x_leg**2 + y_leg**2)
    theta1 = np.arctan2(y_leg, x_leg)  # yaw about leg Z

    D = R - COXA_L
    down = -z_leg  # z up; negative is "down"
    C = np.sqrt(D**2 + down**2)

    # keep within reachable range to avoid NaNs
    Cmax = FEMUR_A + TIBIA_B
    Cmin = abs(FEMUR_A - TIBIA_B)
    Cc = clamp(C, Cmin, Cmax)

    g = np.arctan2(D, down)
    b = acos_safe((FEMUR_A**2 + Cc**2 - TIBIA_B**2) / (2 * FEMUR_A * Cc))
    theta2 = b + g

    h = acos_safe((FEMUR_A**2 + TIBIA_B**2 - Cc**2) / (2 * FEMUR_A * TIBIA_B))
    theta3 = np.pi - h

    return np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)


# ----------------------------
# Forward kinematics (for drawing segments)
# Using the IK angles to draw a simple 3-link chain in the LEG frame.
# ----------------------------
def leg_fk_points(theta1_deg, theta2_deg, theta3_deg):
    """
    Returns points in leg frame:
      p0 = hip origin
      p1 = end of coxa
      p2 = end of femur
      p3 = end of tibia (foot)
    """
    t1 = np.deg2rad(theta1_deg)
    t2 = np.deg2rad(theta2_deg)
    t3 = np.deg2rad(theta3_deg)

    # Coxa end in horizontal plane (x-y plane of leg frame)
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([COXA_L * np.cos(t1),
                   COXA_L * np.sin(t1),
                   0.0])

    # Femur rotates in a plane that includes z; we approximate by applying femur/tibia in the radial direction from yaw
    # Direction in XY from yaw:
    dir_xy = np.array([np.cos(t1), np.sin(t1), 0.0])

    # Femur endpoint:
    femur_horiz = FEMUR_A * np.cos(t2)
    femur_z     = -FEMUR_A * np.sin(t2)
    p2 = p1 + dir_xy * femur_horiz + np.array([0.0, 0.0, femur_z])

    # Tibia endpoint:
    tibia_angle = t2 + (np.pi - t3)  # internal angle composition for this IK style
    tibia_horiz = TIBIA_B * np.cos(tibia_angle)
    tibia_z     = -TIBIA_B * np.sin(tibia_angle)
    p3 = p2 + dir_xy * tibia_horiz + np.array([0.0, 0.0, tibia_z])

    return p0, p1, p2, p3


# ----------------------------
# Demo scenario
# ----------------------------
def main():
    # 1) Choose a "neutral foot" position in BODY frame (a nice standing pose)
    # Feel free to tweak these numbers.
    neutral_foot_body = np.array([0.0, 50.0 + 120.0, -110.0])  # forward/back=0, right=hip+120, down=110

    # 2) Body pose command (start with roll only)
    body_roll_deg  = 15.0
    body_pitch_deg = 0.0
    body_yaw_deg   = 0.0
    body_translation = np.array([0.0, 0.0, 0.0])

    # 3) Build body rotation
    R_body = r_body_from_rpy(body_roll_deg, body_pitch_deg, body_yaw_deg)

    # 4) "Foot planted" target: inverse-rotate the neutral foot by the body pose
    # This is the trick that makes feet appear planted while body rolls.
    foot_target_body = (R_body.T @ (neutral_foot_body - body_translation))

    # 5) Convert target into leg frame
    foot_from_hip_body = foot_target_body - HIP_IN_BODY
    foot_target_leg = body_vec_to_leg_vec(foot_from_hip_body)

    # 6) IK
    t1, t2, t3 = leg_ik_math(*foot_target_leg)

    # 7) FK points for drawing in leg frame
    p0, p1, p2, p3 = leg_fk_points(t1, t2, t3)

    # 8) Print the “inputs/outputs” like a data pipeline
    print("=== INPUTS ===")
    print(f"body_roll_deg: {body_roll_deg}")
    print(f"neutral_foot_body (mm): {neutral_foot_body}")
    print("")
    print("=== TRANSFORMS ===")
    print(f"foot_target_body (mm): {foot_target_body}")
    print(f"foot_target_leg  (mm): {foot_target_leg}")
    print("")
    print("=== IK OUTPUTS (deg) ===")
    print(f"theta1 (yaw):   {t1:.2f}")
    print(f"theta2 (femur): {t2:.2f}")
    print(f"theta3 (tibia): {t3:.2f}")
    print("")
    print("NOTE: servo calibration (center pulses, reversal, joint zero offsets) happens after this.")

    # ----------------------------
    # Plotting: top (XY), side (XZ), and 3D
    # We'll draw in LEG frame because that's what IK uses.
    # ----------------------------
    fig = plt.figure(figsize=(14, 4))

    # Top view: X_leg vs Y_leg
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.set_title("Top View (Leg Frame): X_leg vs Y_leg")
    ax1.set_xlabel("X_leg (outward)")
    ax1.set_ylabel("Y_leg (forward/back)")
    xs = [p0[0], p1[0], p2[0], p3[0]]
    ys = [p0[1], p1[1], p2[1], p3[1]]
    ax1.plot(xs, ys, marker="o")
    ax1.scatter([foot_target_leg[0]], [foot_target_leg[1]], marker="x", s=80)
    ax1.axis("equal")
    ax1.grid(True)

    # Side view: X_leg vs Z_leg
    ax2 = fig.add_subplot(1, 3, 2)
    ax2.set_title("Side View (Leg Frame): X_leg vs Z_leg")
    ax2.set_xlabel("X_leg (outward)")
    ax2.set_ylabel("Z_leg (up)")
    xs = [p0[0], p1[0], p2[0], p3[0]]
    zs = [p0[2], p1[2], p2[2], p3[2]]
    ax2.plot(xs, zs, marker="o")
    ax2.scatter([foot_target_leg[0]], [foot_target_leg[2]], marker="x", s=80)
    ax2.axhline(0, linewidth=1)
    ax2.grid(True)

    # 3D view
    ax3 = fig.add_subplot(1, 3, 3, projection="3d")
    ax3.set_title("3D View (Leg Frame)")
    ax3.set_xlabel("X_leg")
    ax3.set_ylabel("Y_leg")
    ax3.set_zlabel("Z_leg")

    ax3.plot([p0[0], p1[0], p2[0], p3[0]],
             [p0[1], p1[1], p2[1], p3[1]],
             [p0[2], p1[2], p2[2], p3[2]],
             marker="o")
    ax3.scatter([foot_target_leg[0]],
                [foot_target_leg[1]],
                [foot_target_leg[2]],
                marker="x", s=80)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
