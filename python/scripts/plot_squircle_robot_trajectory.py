"""
Plot the squircle reference and the logged robot XY trajectory.

Inputs:
    read_csv_file: CSV log file with X and Y columns [m].

Outputs:
    Figure:
        1) squircle centerline [m]
        2) squircle left and right boundaries [m]
        3) robot end-effector XY trajectory [m]
"""

from pathlib import Path
import csv

import matplotlib.pyplot as plt
import numpy as np


# -------------------- User Settings --------------------
SCRIPT_DIR = Path(__file__).resolve().parent
read_csv_file = "/home/hamid-tuf/projects/powerball/matlab/VAC/raw_data/squircle_chen_var_adm_schunk.csv"  # TODO

num_reference_points = 800  # TODO number of reference points
center_x = 0.0  # TODO squircle center x [m]
center_y = 0.0  # TODO squircle center y [m]
half_size_x = 0.16  # TODO half width of squircle [m]
half_size_y = 0.16  # TODO half height of squircle [m]
corner_roundness_power = 4.0  # TODO 2=circle, larger=more square
band_width = 0.045  # TODO corridor thickness [m]
squircle_rotation_deg = 0.0  # TODO squircle rotation [deg]

robot_offset_x = 0.3725  # TODO robot x offset [m]
robot_offset_y = 0.0  # TODO robot y offset [m]
robot_rotation_deg = -90.0  # TODO robot planar rotation [deg]


def generate_squircle_xy():
    """
    Generate the squircle centerline.

    Outputs:
        x_ref: reference x coordinates [m]
        y_ref: reference y coordinates [m]
    """
    theta = np.linspace(0.0, 2.0 * np.pi, num_reference_points,
                        endpoint=False)
    rot = np.deg2rad(squircle_rotation_deg)

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    x_local = half_size_x * np.sign(cos_t) * np.abs(cos_t) ** (
        2.0 / corner_roundness_power)
    y_local = half_size_y * np.sign(sin_t) * np.abs(sin_t) ** (
        2.0 / corner_roundness_power)

    x_ref = center_x + np.cos(rot) * x_local - np.sin(rot) * y_local
    y_ref = center_y + np.sin(rot) * x_local + np.cos(rot) * y_local

    return x_ref, y_ref


def generate_squircle_boundaries(x_ref, y_ref):
    """
    Generate the left and right boundaries of the squircle band.

    Inputs:
        x_ref: reference x coordinates [m]
        y_ref: reference y coordinates [m]

    Outputs:
        x_left: left boundary x coordinates [m]
        y_left: left boundary y coordinates [m]
        x_right: right boundary x coordinates [m]
        y_right: right boundary y coordinates [m]
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


def read_robot_xy():
    """
    Read robot end-effector XY points from the CSV log.

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
    Remove the robot offset and rotate the XY trajectory.

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


def main():
    """
    Plot the squircle reference, boundaries, and robot XY trajectory.
    """
    x_ref, y_ref = generate_squircle_xy()
    x_left, y_left, x_right, y_right = \
        generate_squircle_boundaries(x_ref, y_ref)
    x_robot, y_robot = read_robot_xy()
    x_robot, y_robot = transform_robot_xy(x_robot, y_robot)

    plt.figure("squircle_and_robot_trajectory")
    plt.plot(x_ref, y_ref, color="tab:blue", linewidth=2.0,
             label="Squircle reference")
    plt.plot(x_left, y_left, color="tab:cyan", linewidth=1.2,
             linestyle="--", label="Squircle left boundary")
    plt.plot(x_right, y_right, color="tab:green", linewidth=1.2,
             linestyle="--", label="Squircle right boundary")
    plt.plot(x_robot, y_robot, color="tab:red", linewidth=1.5,
             label="Robot end-effector trajectory")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Squircle reference and robot trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
