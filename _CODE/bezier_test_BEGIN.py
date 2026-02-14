# Swing-phase Bézier curve playground (edit T, S, A and re-run)
import numpy as np
import matplotlib.pyplot as plt

def bezier_quadratic(P1, P2, P3, t):
    """Quadratic Bézier: returns Nx2 array of points."""
    t = np.asarray(t)
    return ((1 - t)[:, None] ** 2) * P1 + (2 * (1 - t) * t)[:, None] * P2 + (t[:, None] ** 2) * P3

def plot_swing(T=120.0, S=-80.0, A=40.0, N=150, show_control_polygon=True):
    """
    T: step length (same units as your robot, e.g. mm)
    S: ground height (vertical offset relative to leg/body frame)
    A: step height (foot lift above ground)
    """
    # Points in (Y, Z) plane (matching your notes)
    P1 = np.array([-T/2, S])          # lift-off
    P2 = np.array([0.0,  S + 2*A])    # control (sets arc height)
    P3 = np.array([ T/2, S])          # touch-down

    t = np.linspace(0.0, 1.0, N)
    curve = bezier_quadratic(P1, P2, P3, t)

    plt.figure()
    plt.plot(curve[:, 0], curve[:, 1], label="Swing Bézier")
    plt.scatter([P1[0], P2[0], P3[0]], [P1[1], P2[1], P3[1]], label="P1,P2,P3")

    if show_control_polygon:
        plt.plot([P1[0], P2[0], P3[0]], [P1[1], P2[1], P3[1]], linestyle="--", label="Control polygon")

    # Ground line at Z = S
    plt.axhline(S, linestyle=":", label="Ground (Z=S)")

    plt.xlabel("Y (forward/back over step length T)")
    plt.ylabel("Z (vertical)")
    plt.title(f"Swing Phase (Quadratic Bézier)  T={T}, S={S}, A={A}")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

# --- Start here: tweak these three numbers ---
plot_swing(T=120.0, S=-80.0, A=40.0)
