"""
Plot pure-frequency reference shapes and logged robot XY trajectories.

This script generates the pure-frequency curves from Huh and Sejnowski
Fig. 4 using the same formula as pure_frequency_shape.py:

    h(theta) = amplitude * sin(frequency * theta)
    curvature(theta) = exp(h(theta))

The generated reference centerline and band boundaries are plotted with the
robot end-effector XY trajectory from raw CSV files.

Inputs:
    Raw robot CSV files with X and Y columns [m].

Outputs:
    Figure:
        1) pure-frequency centerline [m]
        2) left and right band boundaries [m]
        3) robot end-effector XY trajectory [m]
"""

from fractions import Fraction
from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
import numpy as np


# -------------------- User Settings --------------------
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parents[1]

read_raw_data_dir = PROJECT_DIR / "matlab/VAC/raw_data"  # TODO

controller_suffix = "chen_var_adm_schunk"  # TODO chen_var_adm_schunk
# controller_suffix = "chen_kf_var_adm_schunk"  # TODO chen_kf_var_adm

shape_size_total = 0.25  # TODO max(width, height) of centerline [m]
log_curvature_amplitude = 0.75  # TODO larger value makes stronger lobes
num_reference_points = 1200  # TODO increase for smoother reference
band_width = 0.025  # TODO corridor thickness [m]

reference_center_x = 0.0  # TODO reference shape center x [m]
reference_center_y = 0.0  # TODO reference shape center y [m]
global_reference_rotation_deg = 0.0  # TODO added to each case rotation [deg]
global_phase_deg = 0.0  # TODO added to each case phase [deg]

robot_offset_x = 0.3725  # TODO robot x offset [m]
robot_offset_y = 0.0  # TODO robot y offset [m]
robot_rotation_deg = -90.0  # TODO robot planar rotation [deg]

subplot_rows = 3  # TODO subplot rows
subplot_cols = 4  # TODO subplot columns

shape_cases = [
    # frequency, file tag, title, shape_rotation_deg, phase_deg
    ("0", "nu_0", "nu = 0", 0.0, 0.0),  # TODO edit/add/remove cases
    ("2/5", "nu_2_fifth", "nu = 2/5", 0.0, 0.0),
    ("3/5", "nu_3_fifth", "nu = 3/5", -45.0, 0.0),
    ("2/3", "nu_2_third", "nu = 2/3", 0.0, 0.0),
    ("4/5", "nu_4_fifth", "nu = 4/5", 0.0, 0.0),
    ("4/3", "nu_4_third", "nu = 4/3", 0.0, 0.0),
    ("3/2", "nu_3_half", "nu = 3/2", 0.0, 0.0),
    ("2", "nu_2", "nu = 2", 0.0, 0.0),
    ("5/2", "nu_5_half", "nu = 5/2", 0.0, 0.0),
    ("3", "nu_3", "nu = 3", 0.0, 0.0),
    ("4", "nu_4", "nu = 4", 0.0, 0.0),
    ("6", "nu_6", "nu = 6", 0.0, 0.0),
]


def parse_frequency(value):
    """
    Convert frequency text to exact fraction and float.

    Inputs:
        value: string such as "2/5", "2", or "3/2"

    Outputs:
        frequency_fraction: exact Fraction value
        frequency_float: floating-point value
    """
    frequency_fraction = Fraction(str(value)).limit_denominator(200)
    frequency_float = float(frequency_fraction)
    return frequency_fraction, frequency_float


def get_theta_period(frequency_fraction):
    """
    Return the theta period for a closed pure-frequency curve.

    For frequency m/n, the curve closes after 2*pi*n in the tangent-angle
    coordinate.
    """
    if frequency_fraction == 0:
        return 2.0 * math.pi

    return 2.0 * math.pi * frequency_fraction.denominator


def generate_pure_frequency_xy(frequency_value, shape_rotation_deg, phase_deg):
    """
    Generate pure-frequency reference centerline.

    Inputs:
        frequency_value: frequency string or number
        shape_rotation_deg: planar rotation after generating shape [deg]
        phase_deg: phase in h(theta) before generating shape [deg]

    Outputs:
        x_ref: reference x coordinates [m]
        y_ref: reference y coordinates [m]
    """
    frequency_fraction, frequency_float = parse_frequency(frequency_value)
    theta_end = get_theta_period(frequency_fraction)
    dtheta = theta_end / num_reference_points
    phase_rad = np.deg2rad(global_phase_deg + phase_deg)

    x_values = []
    y_values = []
    x_current = 0.0
    y_current = 0.0

    for sample_idx in range(num_reference_points):
        theta = sample_idx * dtheta
        h = log_curvature_amplitude * math.sin(
            frequency_float * theta + phase_rad
        )
        curvature = math.exp(h)

        ds = dtheta / curvature
        x_current += math.cos(theta) * ds
        y_current += math.sin(theta) * ds

        x_values.append(x_current)
        y_values.append(y_current)

    x_ref = np.asarray(x_values)
    y_ref = np.asarray(y_values)

    x_ref = x_ref - np.mean(x_ref)
    y_ref = y_ref - np.mean(y_ref)

    width = np.max(x_ref) - np.min(x_ref)
    height = np.max(y_ref) - np.min(y_ref)
    current_size = max(width, height)

    x_ref = x_ref * shape_size_total / current_size
    y_ref = y_ref * shape_size_total / current_size

    rot = np.deg2rad(global_reference_rotation_deg + shape_rotation_deg)
    x_rot = np.cos(rot) * x_ref - np.sin(rot) * y_ref
    y_rot = np.sin(rot) * x_ref + np.cos(rot) * y_ref

    x_ref = reference_center_x + x_rot
    y_ref = reference_center_y + y_rot

    return x_ref, y_ref


def generate_boundaries(x_ref, y_ref):
    """
    Generate left and right boundaries around a reference centerline.

    Inputs:
        x_ref: reference x coordinates [m]
        y_ref: reference y coordinates [m]

    Outputs:
        x_left, y_left: left boundary coordinates [m]
        x_right, y_right: right boundary coordinates [m]
    """
    dx = np.gradient(x_ref)
    dy = np.gradient(y_ref)

    length = np.sqrt(dx**2 + dy**2)
    length[length < 1e-9] = 1e-9

    tx = dx / length
    ty = dy / length

    nx = -ty
    ny = tx

    half_band = band_width / 2.0

    x_left = x_ref + half_band * nx
    y_left = y_ref + half_band * ny
    x_right = x_ref - half_band * nx
    y_right = y_ref - half_band * ny

    return x_left, y_left, x_right, y_right


def read_robot_xy(read_csv_file):
    """
    Read robot XY trajectory from one CSV log.

    Inputs:
        read_csv_file: path to raw robot CSV

    Outputs:
        x_robot: logged robot x coordinates [m]
        y_robot: logged robot y coordinates [m]
    """
    x_robot = []
    y_robot = []

    with open(read_csv_file, "r", newline="") as csv_file:
        csv_reader = csv.DictReader(csv_file)

        for row in csv_reader:
            x_robot.append(float(row["X"]))
            y_robot.append(float(row["Y"]))

    return x_robot, y_robot


def transform_robot_xy(x_robot, y_robot):
    """
    Remove robot offset and rotate the XY trajectory.

    Outputs:
        x_robot_tf: transformed robot x coordinates [m]
        y_robot_tf: transformed robot y coordinates [m]
    """
    x_robot = np.asarray(x_robot) - robot_offset_x
    y_robot = np.asarray(y_robot) - robot_offset_y

    theta = np.deg2rad(robot_rotation_deg)

    x_robot_tf = np.cos(theta) * x_robot - np.sin(theta) * y_robot
    y_robot_tf = np.sin(theta) * x_robot + np.cos(theta) * y_robot

    return x_robot_tf, y_robot_tf


def get_robot_csv_file(file_tag):
    """
    Build the raw CSV path from the file tag and controller suffix.
    """
    filename = file_tag + "_" + controller_suffix + ".csv"
    return read_raw_data_dir / filename


def plot_one_case(frequency_value, file_tag, plot_title,
                  shape_rotation_deg, phase_deg):
    """
    Plot one pure-frequency reference and matching robot trajectory.
    """
    x_ref, y_ref = generate_pure_frequency_xy(
        frequency_value, shape_rotation_deg, phase_deg
    )
    x_left, y_left, x_right, y_right = generate_boundaries(x_ref, y_ref)

    read_csv_file = get_robot_csv_file(file_tag)
    x_robot, y_robot = read_robot_xy(read_csv_file)
    x_robot, y_robot = transform_robot_xy(x_robot, y_robot)

    plt.plot(x_ref, y_ref, color="tab:blue", linewidth=1.6,
             label="Reference")
    plt.plot(x_left, y_left, color="tab:cyan", linewidth=0.9,
             linestyle="--", label="Left boundary")
    plt.plot(x_right, y_right, color="tab:green", linewidth=0.9,
             linestyle="--", label="Right boundary")
    plt.plot(x_robot, y_robot, color="tab:red", linewidth=1.2,
             label="Robot trajectory")

    plt.title(plot_title)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis("equal")
    plt.grid(True)


def main():
    """
    Plot robot trajectory overlaps for multiple pure-frequency shapes.
    """
    plt.figure("pure_frequency_robot_trajectory_overlaps")

    for case_idx, shape_case in enumerate(shape_cases):
        frequency_value = shape_case[0]
        file_tag = shape_case[1]
        plot_title = shape_case[2]
        shape_rotation_deg = shape_case[3]
        phase_deg = shape_case[4]

        plt.subplot(subplot_rows, subplot_cols, case_idx + 1)
        plot_one_case(
            frequency_value, file_tag, plot_title,
            shape_rotation_deg, phase_deg
        )

        if case_idx == 0:
            plt.legend(fontsize=7)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
