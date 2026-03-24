import csv
import tkinter as tk

CSV_FILE = "data.csv"

# -------------------------------------------------
# read csv
# -------------------------------------------------
time_data = []
q_data = [[] for _ in range(6)]
qdot_data = [[] for _ in range(6)]

with open(CSV_FILE, "r") as f:
    reader = csv.DictReader(f)

    t0 = None
    for row in reader:
        t_us = int(row["Time_us"])
        if t0 is None:
            t0 = t_us

        t_sec = (t_us - t0) / 1e6
        time_data.append(t_sec)

        for i in range(6):
            q_data[i].append(float(row[f"Q{i+1}"]))
            qdot_data[i].append(float(row[f"Qdot{i+1}"]))

# -------------------------------------------------
# plotting helpers
# -------------------------------------------------
colors = ["red", "blue", "green", "orange", "purple", "brown"]

def draw_axes(canvas, region, title, xlabel, ylabel):
    x0, y0, x1, y1 = region

    canvas.create_rectangle(x0, y0, x1, y1, outline="black")
    canvas.create_line(x0 + 50, y1 - 40, x1 - 20, y1 - 40, width=2)  # x-axis
    canvas.create_line(x0 + 50, y1 - 40, x0 + 50, y0 + 20, width=2)  # y-axis

    canvas.create_text((x0 + x1) / 2, y0 + 10, text=title)
    canvas.create_text((x0 + x1) / 2, y1 - 10, text=xlabel)
    canvas.create_text(x0 + 15, (y0 + y1) / 2, text=ylabel, angle=90)

def draw_multiline_plot(canvas, region, x, y_list, labels, colors):
    x0, y0, x1, y1 = region

    px0 = x0 + 50
    px1 = x1 - 20
    py0 = y1 - 40
    py1 = y0 + 20

    x_min = min(x)
    x_max = max(x)

    y_all = []
    for y in y_list:
        y_all.extend(y)

    y_min = min(y_all)
    y_max = max(y_all)

    if x_max == x_min:
        x_max = x_min + 1.0
    if y_max == y_min:
        y_max = y_min + 1.0

    def map_x(val):
        return px0 + (val - x_min) / (x_max - x_min) * (px1 - px0)

    def map_y(val):
        return py0 - (val - y_min) / (y_max - y_min) * (py0 - py1)

    # simple ticks
    for k in range(5):
        xx = px0 + k * (px1 - px0) / 4
        xv = x_min + k * (x_max - x_min) / 4
        canvas.create_line(xx, py0, xx, py0 + 5)
        canvas.create_text(xx, py0 + 15, text=f"{xv:.2f}")

    for k in range(5):
        yy = py0 - k * (py0 - py1) / 4
        yv = y_min + k * (y_max - y_min) / 4
        canvas.create_line(px0 - 5, yy, px0, yy)
        canvas.create_text(px0 - 25, yy, text=f"{yv:.2f}")

    # plot lines
    for j, y in enumerate(y_list):
        points = []
        for xi, yi in zip(x, y):
            points.extend([map_x(xi), map_y(yi)])
        canvas.create_line(points, fill=colors[j], width=2)

    # legend
    lx = x1 - 90
    ly = y0 + 30
    for j, label in enumerate(labels):
        yy = ly + 20 * j
        canvas.create_line(lx, yy, lx + 20, yy, fill=colors[j], width=2)
        canvas.create_text(lx + 25, yy, text=label, anchor="w")

colors = ["red", "blue", "green", "orange", "purple", "brown"]

def draw_plot(canvas, x, y_list, labels):
    canvas.delete("all")

    w = 900
    h = 500
    pad_left = 60
    pad_right = 20
    pad_top = 30
    pad_bottom = 50

    x_min, x_max = min(x), max(x)

    y_all = []
    for y in y_list:
        y_all.extend(y)
    y_min, y_max = min(y_all), max(y_all)

    if x_max == x_min:
        x_max = x_min + 1
    if y_max == y_min:
        y_max = y_min + 1

    def map_x(xv):
        return pad_left + (xv - x_min) / (x_max - x_min) * (w - pad_left - pad_right)

    def map_y(yv):
        return h - pad_bottom - (yv - y_min) / (y_max - y_min) * (h - pad_top - pad_bottom)

    # axes
    canvas.create_line(pad_left, h - pad_bottom, w - pad_right, h - pad_bottom, width=2)
    canvas.create_line(pad_left, h - pad_bottom, pad_left, pad_top, width=2)

    # labels
    canvas.create_text(w / 2, 15, text="Joint Angles")
    canvas.create_text(w / 2, h - 15, text="Time [s]")
    canvas.create_text(20, h / 2, text="Q [rad]", angle=90)

    # plot lines
    for j, y in enumerate(y_list):
        points = []
        for xi, yi in zip(x, y):
            points.extend([map_x(xi), map_y(yi)])
        canvas.create_line(points, fill=colors[j], width=2)

    # legend
    for j, label in enumerate(labels):
        yy = 30 + 20 * j
        canvas.create_line(w - 100, yy, w - 80, yy, fill=colors[j], width=2)
        canvas.create_text(w - 75, yy, text=label, anchor="w")

# -------------------------------------------------
# gui
# -------------------------------------------------
root = tk.Tk()
root.title("Robot Joint Data")

canvas = tk.Canvas(root, width=1000, height=700, bg="white")
canvas.pack()

region1 = (50, 40, 950, 320)   # top plot
region2 = (50, 360, 950, 640)  # bottom plot

draw_axes(canvas, region1, "Joint Angles", "Time [s]", "Q [rad]")
draw_multiline_plot(
    canvas,
    region1,
    time_data,
    q_data,
    [f"Q{i+1}" for i in range(6)],
    colors
)

draw_axes(canvas, region2, "Joint Velocities", "Time [s]", "Qdot [rad/s]")
draw_multiline_plot(
    canvas,
    region2,
    time_data,
    qdot_data,
    [f"Qdot{i+1}" for i in range(6)],
    colors
)

root.mainloop()