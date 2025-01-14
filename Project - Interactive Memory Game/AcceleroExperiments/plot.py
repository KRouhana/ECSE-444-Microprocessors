# pyright: basic

import csv
from pathlib import Path
from statistics import mean
from typing import List

import matplotlib.pyplot as plt  # type: ignore

SAMPLING_PERIOD = 1 / 32
SMOOTH_N = 5
CALIBRATION_SAMPLES = 64
ACCELERATION_THRESHOLD = 200
DEACTIVATION_THRESHOLD = 50
DEACTIVATION_LEN = 10


def main():
    file = Path(__file__).parent.joinpath("final-up-down-down-up-32hz.csv")
    with open(file, "r", encoding="ascii", newline="") as f:
        reader = csv.reader(f)
        lines = [x for x in reader if x]

    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams["font.size"] = 28

    ax = plt.subplot()
    z_acceleration = [float(line[2]) for line in lines][50:220]
    baseline_z_acceleration = mean(z_acceleration[:CALIBRATION_SAMPLES])
    z_acceleration = [z - baseline_z_acceleration for z in z_acceleration]
    time = [i * SAMPLING_PERIOD for i in range(SMOOTH_N - 1, len(z_acceleration))]
    LINEWIDTH = 5
    accelero_line, *_ = ax.plot(
        time,
        smooth(z_acceleration, SMOOTH_N),
        "k-",
        linewidth=LINEWIDTH,
        label="Accelero reading minus baseline",
    )
    activation_line, *_ = ax.plot(
        time,
        [ACCELERATION_THRESHOLD for _ in time],
        "r:",
        linewidth=LINEWIDTH,
        label="Activation threshold",
    )
    ax.plot(
        time,
        [-ACCELERATION_THRESHOLD for _ in time],
        "r:",
        linewidth=LINEWIDTH,
        label="Activation threshold",
    )
    deactivation_line, *_ = ax.plot(
        time,
        [DEACTIVATION_THRESHOLD for _ in time],
        "c--",
        linewidth=LINEWIDTH,
        label="Deactivation threshold",
    )
    ax.plot(
        time,
        [-DEACTIVATION_THRESHOLD for _ in time],
        "c--",
        linewidth=LINEWIDTH,
        label="Deactivation threshold",
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylim(top=500)
    ax.legend(handles=[accelero_line, activation_line, deactivation_line])

    # fig, ax = plt.subplots(nrows=3, ncols=3)

    # x_acceleration = [float(line[0]) for line in lines]
    # baseline_x_acceleration = mean(x_acceleration[:CALIBRATION_SAMPLES])
    # x_acceleration = [x - baseline_x_acceleration for x in x_acceleration]
    # x_velocity = integrate(x_acceleration, SAMPLING_PERIOD)
    # x_position = integrate(x_velocity, SAMPLING_PERIOD)
    # ax[0, 0].plot(time, smooth(x_acceleration, SMOOTH_N))
    # ax[0, 0].plot(time, [ACCELERATION_THRESHOLD for _ in time])
    # ax[0, 0].plot(time, [-ACCELERATION_THRESHOLD for _ in time])
    # ax[0, 1].plot(time, smooth(x_velocity, SMOOTH_N))
    # ax[0, 2].plot(time, smooth(x_position, SMOOTH_N))

    # y_acceleration = [float(line[1]) for line in lines]
    # baseline_y_acceleration = mean(y_acceleration[:CALIBRATION_SAMPLES])
    # y_acceleration = [y - baseline_y_acceleration for y in y_acceleration]
    # y_velocity = integrate(y_acceleration, SAMPLING_PERIOD)
    # y_position = integrate(y_velocity, SAMPLING_PERIOD)
    # ax[1, 0].plot(time, smooth(y_acceleration, SMOOTH_N))
    # ax[1, 0].plot(time, [ACCELERATION_THRESHOLD for _ in time])
    # ax[1, 0].plot(time, [-ACCELERATION_THRESHOLD for _ in time])
    # ax[1, 1].plot(time, smooth(y_velocity, SMOOTH_N))
    # ax[1, 2].plot(time, smooth(y_position, SMOOTH_N))

    # z_acceleration = [float(line[2]) for line in lines]
    # baseline_z_acceleration = mean(z_acceleration[:CALIBRATION_SAMPLES])
    # z_acceleration = [z - baseline_z_acceleration for z in z_acceleration]
    # z_velocity = integrate(z_acceleration, SAMPLING_PERIOD)
    # z_position = integrate(z_velocity, SAMPLING_PERIOD)
    # ax[2, 0].plot(time, smooth(z_acceleration, SMOOTH_N))
    # ax[2, 0].plot(time, [ACCELERATION_THRESHOLD for _ in time])
    # ax[2, 0].plot(time, [-ACCELERATION_THRESHOLD for _ in time])
    # ax[2, 1].plot(time, smooth(z_velocity, SMOOTH_N))
    # ax[2, 2].plot(time, smooth(z_position, SMOOTH_N))

    # ax[0, 0].set_title("Acceleration")
    # ax[0, 1].set_title("Velocity")
    # ax[0, 2].set_title("Position")
    # ax[0, 0].set_ylabel("X")
    # ax[1, 0].set_ylabel("Y")
    # ax[2, 0].set_ylabel("Z")
    # ax[2, 0].set_xlabel("Time (s)")
    # ax[2, 1].set_xlabel("Time (s)")
    # ax[2, 2].set_xlabel("Time (s)")

    plt.show()


def integrate(f: List[float], delta_t: float) -> List[float]:
    integral = [0.0 for _ in f]
    for i in range(1, len(f)):
        integral[i] = integral[i - 1] + f[i] * delta_t
    return integral


def smooth(f: List[float], n: int) -> List[float]:
    return [sum(f[i + 1 - n : i + 1]) / n for i in range(n - 1, len(f))]


if __name__ == "__main__":
    main()
