import numpy as np

# ---------- parameters ----------
points = 5
samples_per_edge = 40

cx, cy = 0.0, 0.0
z0 = 0.             # slightly behind/below marker

outer_radius = 0.15    # star outer size [m]
inner_radius = 0.065    # star inner size [m]

band_width = 0.05      # corridor thickness [m]
rotation_deg = 90      # rotate star orientation

# ---------- create star vertices ----------
rot = np.deg2rad(rotation_deg)

verts = []
for i in range(points * 2):
    angle = rot + i * np.pi / points
    r = outer_radius if i % 2 == 0 else inner_radius
    x = cx + r * np.cos(angle)
    y = cy + r * np.sin(angle)
    verts.append([x, y])

verts = np.array(verts)

# ---------- sample along star edges ----------
center_xy = []

for i in range(len(verts)):
    p1 = verts[i]
    p2 = verts[(i + 1) % len(verts)]

    for s in range(samples_per_edge):
        alpha = s / samples_per_edge
        p = (1 - alpha) * p1 + alpha * p2
        center_xy.append(p)

center_xy = np.array(center_xy)
N = len(center_xy)

x = center_xy[:, 0]
y = center_xy[:, 1]
z = z0 * np.ones(N)

center = np.column_stack((x, y, z))

# ---------- tangent and normal ----------
dx = np.gradient(x)
dy = np.gradient(y)

length = np.sqrt(dx**2 + dy**2)
length[length < 1e-9] = 1e-9

tx = dx / length
ty = dy / length

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
with open("star_band.obj", "w") as f:
    f.write("# Star band mesh\n")

    for p in left:
        f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    for p in right:
        f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    for i in range(N):
        j = (i + 1) % N

        l1 = i + 1
        l2 = j + 1
        r1 = N + i + 1
        r2 = N + j + 1

        f.write(f"f {l1} {l2} {r2} {r1}\n")