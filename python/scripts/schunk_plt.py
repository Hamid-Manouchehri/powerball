import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("../../data/admittance_control/test_1_020326/maze_damp_80.000000_01.csv")

# Fix timestamp: replace last ':' with '.'
df["TimeStamp "] = df["TimeStamp "].str.replace(
    r"(\d{2}:\d{2}:\d{2}):", r"\1.", regex=True
)

# Convert to datetime
df["TimeStamp "] = pd.to_datetime(df["TimeStamp "], format="%H:%M:%S.%f")

# Convert to relative time (seconds)
t0 = df["TimeStamp "].iloc[0]
df["t"] = (df["TimeStamp "] - t0).dt.total_seconds()

plt.figure()
plt.plot(df["t"], df[" Q1"], label="Q1")
# plt.plot(df["t"], df["Q2"], label="Q2")
# plt.plot(df["t"], df["Q3"], label="Q3")
plt.xlabel("Time [s]")
plt.ylabel("Joint position [rad]")
plt.legend()
plt.grid(True)
plt.show()
