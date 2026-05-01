import numpy as np

# ---------- parameters ----------
N = 400
cx, cy, z0 = 0.0, 0.0, 0.

eight_width_total  = 0.40   # size of the eight shape [m]
eight_height_total = 0.15   # size of the eight shape [m]

band_width = 0.06           # line/band thickness [m]

# ---------- centerline ----------
t = np.linspace(0, 2*np.pi, N, endpoint=False)

W = eight_width_total / 2
H = eight_height_total / 2

x = cx + W * np.sin(t)
y = cy + H * np.sin(2*t)
z = z0 * np.ones_like(t)

center = np.column_stack((x, y, z))

# ---------- tangent and normal ----------
dx = np.gradient(x)
dy = np.gradient(y)

length = np.sqrt(dx**2 + dy**2)
tx = dx / length
ty = dy / length

# normal in xy plane
nx = -ty
ny = tx

half_band = band_width / 2

left = center.copy()
right = center.copy()

left[:, 0]  += half_band * nx
left[:, 1]  += half_band * ny

right[:, 0] -= half_band * nx
right[:, 1] -= half_band * ny

# ---------- write OBJ mesh ----------
with open("eight_band.obj", "w") as f:
    f.write("# Figure-eight band mesh\n")

    # vertices: left side then right side
    for p in left:
        f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
    for p in right:
        f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    # faces
    for i in range(N):
        j = (i + 1) % N

        l1 = i + 1
        l2 = j + 1
        r1 = N + i + 1
        r2 = N + j + 1

        f.write(f"f {l1} {l2} {r2} {r1}\n")