import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def bezier_quadratic(P1, P2, P3, t):
    return (1 - t)**2 * P1 + 2*(1 - t)*t * P2 + t**2 * P3

# ---- tweak these ----
T = 120.0   # step length
S = -80.0   # ground height
A = 40.0    # step height
N = 200     # curve resolution
# ---------------------

# Control points in (Y, Z)
P1 = np.array([-T/2, S])
P2 = np.array([0.0,  S + 2*A])
P3 = np.array([ T/2, S])

# Precompute curve for display
ts = np.linspace(0, 1, N)
curve = np.array([bezier_quadratic(P1, P2, P3, t) for t in ts])

# Initial dot position
t0 = 0.0
p0 = bezier_quadratic(P1, P2, P3, t0)

# Figure + plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)  # room for slider

ax.plot(curve[:, 0], curve[:, 1], label="Swing Bézier")
ax.scatter([P1[0], P2[0], P3[0]], [P1[1], P2[1], P3[1]], label="P1,P2,P3")
ax.plot([P1[0], P2[0], P3[0]], [P1[1], P2[1], P3[1]], linestyle="--", label="Control polygon")
ax.axhline(S, linestyle=":", label="Ground (Z=S)")

dot, = ax.plot([p0[0]], [p0[1]], marker="o")  # moving point

ax.set_xlabel("Y (forward/back)")
ax.set_ylabel("Z (vertical)")
ax.set_title("Swing Phase: drag slider to move foot point")
ax.axis("equal")
ax.grid(True)
ax.legend()

# Slider
slider_ax = fig.add_axes([0.15, 0.06, 0.7, 0.04])
t_slider = Slider(slider_ax, "t", 0.0, 1.0, valinit=t0)

def update(val):
    t = t_slider.val
    p = bezier_quadratic(P1, P2, P3, t)
    dot.set_data([p[0]], [p[1]])
    fig.canvas.draw_idle()

t_slider.on_changed(update)

plt.show()
