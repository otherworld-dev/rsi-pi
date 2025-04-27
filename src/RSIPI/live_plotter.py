import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from threading import Thread, Lock
import time

class LivePlotter:
    def __init__(self, client, mode="3d", interval=100):
        self.client = client
        self.mode = mode
        self.interval = interval
        self.running = False

        # Plot data buffers
        self.time_data = deque(maxlen=500)
        self.position_data = {k: deque(maxlen=500) for k in ["X", "Y", "Z"]}
        self.velocity_data = {k: deque(maxlen=500) for k in ["X", "Y", "Z"]}
        self.acceleration_data = {k: deque(maxlen=500) for k in ["X", "Y", "Z"]}
        self.joint_data = {f"A{i}": deque(maxlen=500) for i in range(1, 7)}
        self.force_data = {f"A{i}": deque(maxlen=500) for i in range(1, 7)}

        self.previous_positions = {"X": 0, "Y": 0, "Z": 0}
        self.previous_velocities = {"X": 0, "Y": 0, "Z": 0}
        self.previous_time = time.time()

        self.lock = Lock()
        self.collector_thread = None

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d" if self.mode == "3d" else None)

    def start(self):
        self.running = True
        self.collector_thread = Thread(target=self.collect_data_loop, daemon=True)
        self.collector_thread.start()
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=self.interval)
        try:
            plt.show()
        except RuntimeError:
            print("⚠️ Matplotlib GUI interrupted during shutdown.")
        self.running = False

    def stop(self, save_path: str = None):
        self.running = False
        if save_path:
            try:
                self.fig.savefig(save_path, bbox_inches="tight")
                print(f"📸 Plot saved to '{save_path}'")
            except Exception as e:
                print(f"❌ Failed to save plot: {e}")
        plt.close(self.fig)

    def collect_data_loop(self):
        while self.running:
            with self.lock:
                current_time = time.time()
                dt = current_time - self.previous_time
                self.previous_time = current_time
                self.time_data.append(current_time)

                position = self.client.receive_variables.get("RIst", {"X": 0, "Y": 0, "Z": 0})
                joints = self.client.receive_variables.get("AIPos", {f"A{i}": 0 for i in range(1, 7)})
                force = self.client.receive_variables.get("MaCur", {f"A{i}": 0 for i in range(1, 7)})

                for axis in ["X", "Y", "Z"]:
                    vel = (position[axis] - self.previous_positions[axis]) / dt if dt > 0 else 0
                    acc = (vel - self.previous_velocities[axis]) / dt if dt > 0 else 0
                    self.previous_positions[axis] = position[axis]
                    self.previous_velocities[axis] = vel
                    self.position_data[axis].append(position[axis])
                    self.velocity_data[axis].append(vel)
                    self.acceleration_data[axis].append(acc)

                for i in range(1, 7):
                    self.joint_data[f"A{i}"].append(joints.get(f"A{i}", 0))
                    self.force_data[f"A{i}"].append(force.get(f"A{i}", 0))

            time.sleep(self.interval / 1000.0)

    def update_plot(self, frame):
        if not self.running:
            return

        with self.lock:
            self.ax.clear()
            self.render_plot()

    def render_plot(self):
        if self.mode == "3d":
            self.ax.set_title("Live 3D TCP Trajectory")
            self.ax.plot(self.position_data["X"], self.position_data["Y"], self.position_data["Z"], label="TCP Path")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")
        elif self.mode == "2d_xy":
            self.ax.set_title("Live 2D Trajectory (X-Y)")
            self.ax.plot(self.position_data["X"], self.position_data["Y"], label="XY Path")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
        elif self.mode == "velocity":
            self.ax.set_title("Live TCP Velocity")
            self.ax.plot(self.time_data, self.velocity_data["X"], label="dX/dt")
            self.ax.plot(self.time_data, self.velocity_data["Y"], label="dY/dt")
            self.ax.plot(self.time_data, self.velocity_data["Z"], label="dZ/dt")
            self.ax.set_ylabel("Velocity [mm/s]")
        elif self.mode == "acceleration":
            self.ax.set_title("Live TCP Acceleration")
            self.ax.plot(self.time_data, self.acceleration_data["X"], label="d²X/dt²")
            self.ax.plot(self.time_data, self.acceleration_data["Y"], label="d²Y/dt²")
            self.ax.plot(self.time_data, self.acceleration_data["Z"], label="d²Z/dt²")
            self.ax.set_ylabel("Acceleration [mm/s²]")
        elif self.mode == "joints":
            self.ax.set_title("Live Joint Angles")
            for j, values in self.joint_data.items():
                self.ax.plot(self.time_data, values, label=j)
            self.ax.set_ylabel("Angle [deg]")
        elif self.mode == "force":
            self.ax.set_title("Live Motor Currents")
            for j, values in self.force_data.items():
                self.ax.plot(self.time_data, values, label=j)
            self.ax.set_ylabel("Current [Nm]")

        self.ax.set_xlabel("Time")
        self.ax.legend()
        self.ax.grid(True)
        self.fig.tight_layout()

    def change_mode(self, mode):
        self.mode = mode
        self.ax = self.fig.add_subplot(111, projection="3d" if mode == "3d" else None)
