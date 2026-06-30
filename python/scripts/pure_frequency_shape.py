"""Generate pure-frequency curve band meshes from Huh and Sejnowski Fig. 4.

The paper defines the pure-frequency curves by their log-curvature profile:

    h(theta) = amplitude * sin(frequency * theta)
    curvature(theta) = exp(h(theta))

Here theta is the tangent-angle coordinate. The 2D curve is reconstructed by
integrating:

    ds = dtheta / curvature(theta)
    dx = cos(theta) * ds
    dy = sin(theta) * ds

The generated centerline is then scaled to the requested size and converted
into a flat OBJ band, similar to the other shape scripts in this folder.

Inputs:
    Change the values in the User Settings section.

Outputs:
    OBJ band mesh saved to write_obj_file.
"""

from fractions import Fraction
from pathlib import Path
import math


# -------------------- User Settings --------------------
frequency = "6"  # TODO examples from Fig. 4: 0, 2/5, 4/3, 2, 3, 6
shape_size_total = 0.25  # TODO max(width, height) of centerline [m]
# log_curvature_amplitude = 0.75  # TODO larger value makes stronger lobes
log_curvature_amplitude = 0.75  # TODO larger value makes stronger lobes

N = 1200  # TODO centerline samples; increase for smoother OBJ
cx, cy = 0.0, 0.0  # TODO center position [m]
z0 = 0.0  # TODO OBJ z height [m]
band_width = 0.025  # TODO corridor thickness [m]
rotation_deg = 0.0  # TODO rotate final shape [deg]

script_dir = Path(__file__).resolve().parent
write_obj_file = script_dir / "data/pure_frequency_band.obj"  # TODO


def parse_frequency(value):
    """Return frequency as a Fraction and as a float."""

    frequency_fraction = Fraction(str(value)).limit_denominator(200)
    frequency_float = float(frequency_fraction)
    return frequency_fraction, frequency_float


def get_theta_period(frequency_fraction):
    """Return the theta period needed for a closed pure-frequency curve."""

    if frequency_fraction == 0:
        return 2.0 * math.pi

    denominator = frequency_fraction.denominator
    return 2.0 * math.pi * denominator


def generate_centerline():
    """Generate the pure-frequency centerline from log curvature."""

    frequency_fraction, frequency_float = parse_frequency(frequency)
    theta_end = get_theta_period(frequency_fraction)

    dtheta = theta_end / N

    x_values = []
    y_values = []

    x_current = 0.0
    y_current = 0.0

    for i in range(N):
        theta = i * dtheta
        h = log_curvature_amplitude * math.sin(frequency_float * theta)
        curvature = math.exp(h)

        ds = dtheta / curvature
        x_current += math.cos(theta) * ds
        y_current += math.sin(theta) * ds

        x_values.append(x_current)
        y_values.append(y_current)

    x_mean = sum(x_values) / len(x_values)
    y_mean = sum(y_values) / len(y_values)

    x_values = [x - x_mean for x in x_values]
    y_values = [y - y_mean for y in y_values]

    width = max(x_values) - min(x_values)
    height = max(y_values) - min(y_values)
    current_size = max(width, height)

    scale = shape_size_total / current_size

    x_values = [x * scale for x in x_values]
    y_values = [y * scale for y in y_values]

    rot = math.radians(rotation_deg)

    centerline = []
    for x, y in zip(x_values, y_values):
        x_rot = math.cos(rot) * x - math.sin(rot) * y
        y_rot = math.sin(rot) * x + math.cos(rot) * y
        centerline.append([cx + x_rot, cy + y_rot, z0])

    return centerline, frequency_fraction


def make_band(center):
    """Create left and right band boundaries around a centerline."""

    half_band = band_width / 2.0

    left = []
    right = []

    for i, point in enumerate(center):
        previous_point = center[(i - 1) % N]
        next_point = center[(i + 1) % N]

        dx = next_point[0] - previous_point[0]
        dy = next_point[1] - previous_point[1]

        length = math.sqrt(dx**2 + dy**2)
        if length < 1e-9:
            length = 1e-9

        tx = dx / length
        ty = dy / length

        nx = -ty
        ny = tx

        left.append([
            point[0] + half_band * nx,
            point[1] + half_band * ny,
            point[2],
        ])

        right.append([
            point[0] - half_band * nx,
            point[1] - half_band * ny,
            point[2],
        ])

    return left, right


def write_obj_band(center, left, right, frequency_fraction):
    """Write the band mesh to an OBJ file."""

    write_obj_file.parent.mkdir(parents=True, exist_ok=True)

    with open(write_obj_file, "w", encoding="utf-8") as obj_file:
        obj_file.write("# Pure-frequency curve band mesh\n")
        obj_file.write("# From Huh and Sejnowski Fig. 4 definition\n")
        obj_file.write(f"# frequency = {frequency_fraction}\n")
        obj_file.write(
            f"# log_curvature_amplitude = {log_curvature_amplitude}\n"
        )
        obj_file.write(f"# shape_size_total = {shape_size_total}\n")
        obj_file.write(f"# band_width = {band_width}\n")

        for point in left:
            obj_file.write(
                f"v {point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n"
            )

        for point in right:
            obj_file.write(
                f"v {point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n"
            )

        for i in range(N):
            j = (i + 1) % N

            l1 = i + 1
            l2 = j + 1
            r1 = N + i + 1
            r2 = N + j + 1

            obj_file.write(f"f {l1} {l2} {r2} {r1}\n")

    center_path_length = 0.0
    for i, point in enumerate(center):
        next_point = center[(i + 1) % N]
        dx = next_point[0] - point[0]
        dy = next_point[1] - point[1]
        center_path_length += math.sqrt(dx**2 + dy**2)

    print(f"Frequency: {frequency_fraction}")
    print(f"Center path length: {center_path_length:.6f} m")
    print(f"OBJ written to: {write_obj_file}")


centerline, frequency_fraction = generate_centerline()
left_boundary, right_boundary = make_band(centerline)
write_obj_band(centerline, left_boundary, right_boundary, frequency_fraction)
