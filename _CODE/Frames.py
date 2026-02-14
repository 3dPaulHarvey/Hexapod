import numpy as np
import matplotlib.pyplot as plt

# ---------- Helper ----------
def rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, -s, 0],
        [ s,  c, 0],
        [ 0,  0, 1]
    ])

# ---------- World + body pose ----------
t_WB = np.array([0.3, 0.2, 0.25])        # body position in world
R_WB = rotz(np.deg2rad(30))              # body yaw = 30 deg

# ---------- Hip + foot ----------
r_i_B = np.array([0.15, -0.10, 0.0])     # hip offset in body frame
p_i_W = np.array([0.45, 0.05, 0.0])      # planted foot in world

# ---------- Compute ----------
h_i_W = t_WB + R_WB @ r_i_B               # hip in world
v_i_W = p_i_W - h_i_W                     # hip -> foot vector

# ---------- Plot ----------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# World origin
ax.scatter(0, 0, 0, color='k')
ax.text(0, 0, 0, 'World Origin')

# Body origin
ax.scatter(*t_WB, color='blue')
ax.text(*t_WB, 'Body')

# Hip
ax.scatter(*h_i_W, color='red')
ax.text(*h_i_W, 'Hip')

# Foot
ax.scatter(*p_i_W, color='green')
ax.text(*p_i_W, 'Foot')

# Body -> hip (bold robot link)
ax.plot(
    [t_WB[0], h_i_W[0]],
    [t_WB[1], h_i_W[1]],
    [t_WB[2], h_i_W[2]],
    linewidth=4, color='red', label='Body → Hip'
)

# Hip -> foot (bold leg vector)
ax.plot(
    [h_i_W[0], p_i_W[0]],
    [h_i_W[1], p_i_W[1]],
    [h_i_W[2], p_i_W[2]],
    linewidth=4, color='green', label='Hip → Foot'
)

# World axes
ax.quiver(0,0,0, 0.2,0,0, color='r')
ax.quiver(0,0,0, 0,0.2,0, color='g')
ax.quiver(0,0,0, 0,0,0.2, color='b')
ax.text(0.2,0,0,'Xw')
ax.text(0,0.2,0,'Yw')
ax.text(0,0,0.2,'Zw')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Hexapod World / Body / Hip Geometry')
ax.legend()

ax.set_box_aspect([1,1,0.6])
plt.show()
