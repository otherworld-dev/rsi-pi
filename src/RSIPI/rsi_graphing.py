import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from .rsi_client import RSIClient
import csv

class RSIGraphing:
    """
    Handles real-time plotting of RSI data with support for:
    - Position/velocity/acceleration/force monitoring
    - Live deviation alerts
    - Optional overlay of planned vs actual trajectories
    """

    def __init__(self, client, mode="position", overlay=False, plan_file=None):
        """
        Initialise live graphing interface.

        Args:
            client (RSIClient): Live RSI client instance providing data.
            mode (str): One of "position", "velocity", "acceleration", "force".
            overlay (bool): Whether to show planned vs. actual overlays.
            plan_file (str): Optional CSV file containing planned trajectory.
        """
        self.client = client
        self.mode = mode
        self.overlay = overlay
        self.alerts_enabled = True
        self.deviation_threshold = 5.0      # mm
        self.force_threshold = 10.0         # Nm
        self.fig, self.ax = plt.subplots(figsize=(10, 6))

        # Live data buffers
        self.time_data = deque(maxlen=100)
        self.position_data = {axis: deque(maxlen=100) for axis in ["X", "Y", "Z"]}
        self.velocity_data = {axis: deque(maxlen=100) for axis in ["X", "Y", "Z"]}
        self.acceleration_data = {axis: deque(maxlen=100) for axis in ["X", "Y", "Z"]}
        self.force_data = {axis: deque(maxlen=100) for axis in ["A1", "A2", "A3", "A4", "A5", "A6"]}

        self.previous_positions = {"X": 0, "Y": 0, "Z": 0}
        self.previous_velocities = {"X": 0, "Y": 0, "Z": 0}
        self.previous_time = time.time()

        # Overlay comparison
        self.planned_data = {axis: deque(maxlen=100) for axis in ["X", "Y", "Z"]}
        self.deviation_data = {axis: deque(maxlen=100) for axis in ["X", "Y", "Z"]}

        if plan_file:
            self.load_plan(plan_file)

        self.ani = animation.FuncAnimation(self.fig, self.update_graph, interval=100, cache_frame_data=False)
        plt.show()

    def update_graph(self, frame):
        """
        Called periodically by matplotlib to refresh live graph based on current mode.
        Also checks for force spikes and deviation alerts.
        """
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        position = self.client.receive_variables.get("RIst", {"X": 0, "Y": 0, "Z": 0})
        force = self.client.receive_variables.get("MaCur", {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0})

        # Compute motion derivatives
        for axis in ["X", "Y", "Z"]:
            velocity = (position[axis] - self.previous_positions[axis]) / dt if dt > 0 else 0
            acceleration = (velocity - self.previous_velocities[axis]) / dt if dt > 0 else 0
            self.previous_positions[axis] = position[axis]
            self.previous_velocities[axis] = velocity

            self.position_data[axis].append(position[axis])
            self.velocity_data[axis].append(velocity)
            self.acceleration_data[axis].append(acceleration)

        for axis in ["A1", "A2", "A3", "A4", "A5", "A6"]:
            self.force_data[axis].append(force[axis])

        self.time_data.append(time.strftime("%H:%M:%S"))

        # Compare to planned overlay
        if self.overlay and self.planned_data:
            for axis in ["X", "Y", "Z"]:
                planned_value = self.planned_data[axis][-1] if len(self.planned_data[axis]) > 0 else position[axis]
                self.planned_data[axis].append(planned_value)
                deviation = abs(position[axis] - planned_value)
                self.deviation_data[axis].append(deviation)

                if self.alerts_enabled and deviation > self.deviation_threshold:
                    print(f"⚠️ Deviation Alert! {axis} exceeds {self.deviation_threshold} mm (Deviation: {deviation:.2f} mm)")

        if self.alerts_enabled:
            for axis in ["A1", "A2", "A3", "A4", "A5", "A6"]:
                if self.force_data[axis][-1] > self.force_threshold:
                    print(f"⚠️ Force Spike Alert! {axis} exceeds {self.force_threshold} Nm (Force: {self.force_data[axis][-1]:.2f} Nm)")

        self.ax.clear()

        if self.mode == "position":
            self.ax.plot(self.time_data, self.position_data["X"], label="X Position")
            self.ax.plot(self.time_data, self.position_data["Y"], label="Y Position")
            self.ax.plot(self.time_data, self.position_data["Z"], label="Z Position")
            self.ax.set_title("Live Position Tracking with Alerts")
            self.ax.set_ylabel("Position (mm)")

            if self.overlay:
                self.ax.plot(self.time_data, self.planned_data["X"], label="Planned X", linestyle="dashed")
                self.ax.plot(self.time_data, self.planned_data["Y"], label="Planned Y", linestyle="dashed")
                self.ax.plot(self.time_data, self.planned_data["Z"], label="Planned Z", linestyle="dashed")

        self.ax.legend()
        self.ax.set_xlabel("Time")
        self.ax.tick_params(axis='x', rotation=45)

    def change_mode(self, mode):
        """Switch graphing mode at runtime (position, velocity, acceleration, force)."""
        if mode in ["position", "velocity", "acceleration", "force"]:
            self.mode = mode
            print(f"Graphing mode changed to: {mode}")
        else:
            print("Invalid mode. Available: position, velocity, acceleration, force")

    def set_alert_threshold(self, alert_type, threshold):
        """Update threshold values for alerts."""
        if alert_type == "deviation":
            self.deviation_threshold = threshold
        elif alert_type == "force":
            self.force_threshold = threshold
        print(f"{alert_type.capitalize()} alert threshold set to {threshold}")

    def enable_alerts(self, enable):
        """Enable or disable real-time alerts."""
        self.alerts_enabled = enable
        print(f"Alerts {'enabled' if enable else 'disabled'}.")

    def stop(self):
        """Gracefully stop plotting by closing the figure."""
        plt.close(self.fig)

    def load_plan(self, plan_file):
        """Load planned XYZ trajectory from CSV for overlay comparison."""
        with open(plan_file, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                for axis in ["X", "Y", "Z"]:
                    key = f"Send.RKorr.{axis}"
                    value = float(row.get(key, 0.0))
                    self.planned_data[axis].append(value)

    @staticmethod
    def plot_csv_file(csv_path):
        """Standalone method to plot XYZ position from a log file (no live client required)."""
        timestamps = []
        x_data, y_data, z_data = [], [], []

        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                timestamps.append(row["Timestamp"])
                x_data.append(float(row.get("Receive.RIst.X", 0.0)))
                y_data.append(float(row.get("Receive.RIst.Y", 0.0)))
                z_data.append(float(row.get("Receive.RIst.Z", 0.0)))

        plt.figure(figsize=(10, 6))
        plt.plot(timestamps, x_data, label="X")
        plt.plot(timestamps, y_data, label="Y")
        plt.plot(timestamps, z_data, label="Z")
        plt.title("Position from CSV Log")
        plt.xlabel("Time")
        plt.ylabel("Position (mm)")
        plt.xticks(rotation=45)
        plt.legend()
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RSI Graphing Utility")
    parser.add_argument("--mode", choices=["position", "velocity", "acceleration", "force"], default="position", help="Graphing mode")
    parser.add_argument("--overlay", action="store_true", help="Enable planned vs. actual overlay")
    parser.add_argument("--plan", type=str, help="CSV file with planned trajectory")
    parser.add_argument("--alerts", action="store_true", help="Enable real-time alerts")
    args = parser.parse_args()

    client = RSIClient("../../examples/RSI_EthernetConfig.xml")
    graphing = RSIGraphing(client, mode=args.mode, overlay=args.overlay, plan_file=args.plan)

    if not args.alerts:
        graphing.enable_alerts(False)
