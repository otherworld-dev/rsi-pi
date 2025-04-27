# Re-execute since code state was reset
static_plotter_path = "/mnt/data/static_plotter.py"

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class StaticPlotter:

    @staticmethod
    def _load_csv(csv_path):
        data = {
            "time": [],
            "x": [], "y": [], "z": [],
            "vx": [], "vy": [], "vz": [],
            "ax": [], "ay": [], "az": [],
            "joints": {f"A{i}": [] for i in range(1, 7)},
            "force": {f"A{i}": [] for i in range(1, 7)}
        }

        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data["time"].append(row.get("Timestamp", ""))
                data["x"].append(float(row.get("Receive.RIst.X", 0)))
                data["y"].append(float(row.get("Receive.RIst.Y", 0)))
                data["z"].append(float(row.get("Receive.RIst.Z", 0)))
                for i in range(1, 7):
                    data["joints"][f"A{i}"].append(float(row.get(f"Receive.AIPos.A{i}", 0)))
                    data["force"][f"A{i}"].append(float(row.get(f"Receive.MaCur.A{i}", 0)))
        return data

    @staticmethod
    def plot_3d_trajectory(csv_path):
        data = StaticPlotter._load_csv(csv_path)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(data["x"], data["y"], data["z"], label="TCP Path")
        ax.set_xlabel("X [mm]")
        ax.set_ylabel("Y [mm]")
        ax.set_zlabel("Z [mm]")
        ax.set_title("3D TCP Trajectory")
        ax.legend()
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_2d_projection(csv_path, plane="xy"):
        data = StaticPlotter._load_csv(csv_path)
        x, y = {
            "xy": (data["x"], data["y"]),
            "xz": (data["x"], data["z"]),
            "yz": (data["y"], data["z"]),
        }.get(plane, (data["x"], data["y"]))
        plt.plot(x, y)
        plt.title(f"2D Trajectory Projection ({plane.upper()})")
        plt.xlabel(f"{plane[0].upper()} [mm]")
        plt.ylabel(f"{plane[1].upper()} [mm]")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_position_vs_time(csv_path):
        data = StaticPlotter._load_csv(csv_path)
        plt.plot(data["time"], data["x"], label="X")
        plt.plot(data["time"], data["y"], label="Y")
        plt.plot(data["time"], data["z"], label="Z")
        plt.title("TCP Position vs Time")
        plt.xlabel("Time")
        plt.ylabel("Position [mm]")
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_joint_angles(csv_path):
        data = StaticPlotter._load_csv(csv_path)
        for joint, values in data["joints"].items():
            plt.plot(data["time"], values, label=joint)
        plt.title("Joint Angles vs Time")
        plt.xlabel("Time")
        plt.ylabel("Angle [deg]")
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_motor_currents(csv_path):
        data = StaticPlotter._load_csv(csv_path)
        for joint, values in data["force"].items():
            plt.plot(data["time"], values, label=joint)
        plt.title("Motor Current (Torque Proxy) vs Time")
        plt.xlabel("Time")
        plt.ylabel("Current [Nm]")
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_velocity_vs_time(csv_path):
        data = StaticPlotter._load_csv(csv_path)
        vx = [0] + [(data["x"][i] - data["x"][i - 1]) for i in range(1, len(data["x"]))]
        vy = [0] + [(data["y"][i] - data["y"][i - 1]) for i in range(1, len(data["y"]))]
        vz = [0] + [(data["z"][i] - data["z"][i - 1]) for i in range(1, len(data["z"]))]
        plt.plot(data["time"], vx, label="dX/dt")
        plt.plot(data["time"], vy, label="dY/dt")
        plt.plot(data["time"], vz, label="dZ/dt")
        plt.title("Velocity vs Time")
        plt.xlabel("Time")
        plt.ylabel("Velocity [mm/s]")
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_acceleration_vs_time(csv_path):
        data = StaticPlotter._load_csv(csv_path)
        vx = [0] + [(data["x"][i] - data["x"][i - 1]) for i in range(1, len(data["x"]))]
        vy = [0] + [(data["y"][i] - data["y"][i - 1]) for i in range(1, len(data["y"]))]
        vz = [0] + [(data["z"][i] - data["z"][i - 1]) for i in range(1, len(data["z"]))]
        ax = [0] + [(vx[i] - vx[i - 1]) for i in range(1, len(vx))]
        ay = [0] + [(vy[i] - vy[i - 1]) for i in range(1, len(vy))]
        az = [0] + [(vz[i] - vz[i - 1]) for i in range(1, len(vz))]
        plt.plot(data["time"], ax, label="d²X/dt²")
        plt.plot(data["time"], ay, label="d²Y/dt²")
        plt.plot(data["time"], az, label="d²Z/dt²")
        plt.title("Acceleration vs Time")
        plt.xlabel("Time")
        plt.ylabel("Acceleration [mm/s²]")
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_deviation(csv_actual, csv_planned):
        actual = StaticPlotter._load_csv(csv_actual)
        planned = StaticPlotter._load_csv(csv_planned)
        deviation = {
            "x": [abs(a - b) for a, b in zip(actual["x"], planned["x"])],
            "y": [abs(a - b) for a, b in zip(actual["y"], planned["y"])],
            "z": [abs(a - b) for a, b in zip(actual["z"], planned["z"])]
        }
        plt.plot(actual["time"], deviation["x"], label="X Deviation")
        plt.plot(actual["time"], deviation["y"], label="Y Deviation")
        plt.plot(actual["time"], deviation["z"], label="Z Deviation")
        plt.title("Deviation (Actual - Planned) vs Time")
        plt.xlabel("Time")
        plt.ylabel("Deviation [mm]")
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()
