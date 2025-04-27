import logging
import pandas as pd
import numpy as np
import json
import matplotlib.pyplot as plt
from .kuka_visualiser import KukaRSIVisualiser
from .krl_to_csv_parser import KRLParser
from .inject_rsi_to_krl import inject_rsi_to_krl
import threading
from .trajectory_planner import generate_trajectory, execute_trajectory
import datetime
from src.RSIPI.static_plotter import StaticPlotter  # Make sure this file exists as described
import os
from src.RSIPI.live_plotter import LivePlotter
from threading import Thread
import asyncio

class RSIAPI:
    """RSI API for programmatic control, including alerts, logging, graphing, and data retrieval."""

    def __init__(self, config_file="RSI_EthernetConfig.xml"):
        """Initialize RSIAPI with an RSI client instance."""
        self.thread = None
        self.config_file = config_file
        self.client = None
        self.graph_process = None
        self.graphing_instance = None
        self.graph_thread = None#
        self.trajectory_queue = []
        self.live_plotter = None
        self.live_plot_thread = None

        self._ensure_client()

    def _ensure_client(self):
        """Ensure RSIClient is initialised before use."""
        if self.client is None:
            from .rsi_client import RSIClient
            self.client = RSIClient(self.config_file)

    def start_rsi(self):

        self.thread = threading.Thread(target=self.client.start, daemon=True)
        self.thread.start()
        return "RSI started in background."

    def stop_rsi(self):
        """Stop the RSI client."""
        self.client.stop()
        return "RSI stopped."

    def generate_report(filename, format_type):
        """
        Generate a statistical report from a CSV log file.

        Args:
            filename (str): Path to the CSV file (or base name without .csv).
            format_type (str): 'csv', 'json', or 'pdf'
        """
        # Ensure filename ends with .csv
        if not filename.endswith(".csv"):
            filename += ".csv"

        if not os.path.exists(filename):
            raise FileNotFoundError(f"❌ File not found: {filename}")

        df = pd.read_csv(filename)

        # Only keep relevant columns (e.g. actual positions)
        position_cols = [col for col in df.columns if col.startswith("Receive.RIst.")]
        if not position_cols:
            raise ValueError("❌ No 'Receive.RIst' position columns found in CSV.")

        report_data = {
            "Max Position": df[position_cols].max().to_dict(),
            "Mean Position": df[position_cols].mean().to_dict(),
        }

        report_base = filename.replace(".csv", "")
        output_path = f"{report_base}_report.{format_type.lower()}"

        if format_type == "csv":
            pd.DataFrame(report_data).T.to_csv(output_path)
        elif format_type == "json":
            with open(output_path, "w") as f:
                json.dump(report_data, f, indent=4)
        elif format_type == "pdf":
            fig, ax = plt.subplots()
            pd.DataFrame(report_data).T.plot(kind='bar', ax=ax)
            ax.set_title("RSI Position Report")
            plt.tight_layout()
            plt.savefig(output_path)
        else:
            raise ValueError(f"Unsupported format: {format_type}")

        return f"Report saved as {output_path}"

    def update_variable(self, name, value):
        if "." in name:
            parent, child = name.split(".", 1)
            full_path = f"{parent}.{child}"
            if parent in self.client.send_variables:
                current = dict(self.client.send_variables[parent])
                # 🛡️ Validate using SafetyManager
                safe_value = self.client.safety_manager.validate(full_path, float(value))
                current[child] = safe_value
                self.client.send_variables[parent] = current
                return f"Updated {name} to {safe_value}"
            else:
                raise KeyError(f"Parent variable '{parent}' not found in send_variables")
        else:
            safe_value = self.client.safety_manager.validate(name, float(value))
            self.client.send_variables[name] = safe_value
            return f"Updated {name} to {safe_value}"

    def show_variables(self):
        """Print available variable names in send and receive variables."""
        def format_grouped(var_dict):
            output = []
            for var, val in var_dict.items():
                if isinstance(val, dict):
                    sub_vars = ', '.join(val.keys())
                    output.append(f"{var}: {sub_vars}")
                else:
                    output.append(var)
            return output

        send_vars = format_grouped(self.client.send_variables)
        receive_vars = format_grouped(self.client.receive_variables)

        print("Send Variables:")
        for item in send_vars:
            print(f"  - {item}")

        print("\nReceive Variables:")
        for item in receive_vars:
            print(f"  - {item}")

    def get_live_data(self):
        """Retrieve real-time RSI data for external processing."""
        return {
            "position": self.client.receive_variables.get("RIst", {"X": 0, "Y": 0, "Z": 0}),
            "velocity": self.client.receive_variables.get("Velocity", {"X": 0, "Y": 0, "Z": 0}),
            "acceleration": self.client.receive_variables.get("Acceleration", {"X": 0, "Y": 0, "Z": 0}),
            "force": self.client.receive_variables.get("MaCur", {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0}),
            "ipoc": self.client.receive_variables.get("IPOC", "N/A")
        }

    def get_live_data_as_numpy(self):
        data = self.get_live_data()
        flat = []
        for section in ["position", "velocity", "acceleration", "force"]:
            values = list(data[section].values())
            flat.append(values)

        max_len = max(len(row) for row in flat)
        for row in flat:
            row.extend([0] * (max_len - len(row)))  # Pad missing values

        return np.array(flat)

    def get_live_data_as_dataframe(self):
        """Retrieve live RSI data as a Pandas DataFrame."""
        data = self.get_live_data()
        return pd.DataFrame([data])

    def get_ipoc(self):
        """Retrieve the latest IPOC value."""
        return self.client.receive_variables.get("IPOC", "N/A")

    def reconnect(self):
        """Restart the network connection without stopping RSI."""
        self.client.reconnect()
        return "Network connection restarted."

    def toggle_digital_io(self, io_group, io_name, state):
        """
        Toggle a digital IO variable.

        Args:
            io_group (str): Parent variable group (e.g., 'Digout', 'DiO', 'DiL')
            io_name (str): IO name or number within the group (e.g., 'o1', '1')
            state (bool | int): Desired state (True/False or 1/0)

        Returns:
            str: Success or failure message.
        """
        var_name = f"{io_group}.{io_name}"
        state_value = int(bool(state))  # ensures it's either 1 or 0
        return self.update_variable(var_name, state_value)

    def move_external_axis(self, axis, value):
        """Move an external axis."""
        return self.update_variable(f"ELPos.{axis}", value)

    def correct_position(self, correction_type, axis, value):
        """Apply correction to RKorr or AKorr."""
        return self.update_variable(f"{correction_type}.{axis}", value)

    def adjust_speed(self, tech_param, value):
        """Adjust speed settings (e.g., Tech.T21)."""
        return self.update_variable(tech_param, value)

    def reset_variables(self):
        """Reset send variables to default values."""
        self.client.reset_send_variables()
        return "✅ Send variables reset to default values."

    def show_config_file(self):
        """Retrieve key information from config file."""
        return {
            "Network": self.client.config_parser.get_network_settings(),
            "Send variables": dict(self.client.send_variables),
            "Receive variables": dict(self.client.receive_variables)
        }

    def start_logging(self, filename=None):
        if not filename:
            timestamp = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
            filename = f"logs/{timestamp}.csv"

        self.client.start_logging(filename)
        return filename

    def stop_logging(self):
        """Stop logging RSI data."""
        self.client.stop_logging()
        return "CSV Logging stopped."

    def is_logging_active(self):
        """Return logging status."""
        return self.client.is_logging_active()

    @staticmethod
    def generate_plot(csv_path: str, plot_type: str = "3d", overlay_path: str = None):
        """
        Generate a static plot based on RSI CSV data.

        Args:
            csv_path (str): Path to the CSV log file.
            plot_type (str): Type of plot to generate. Options:
                - "3d", "2d_xy", "2d_xz", "2d_yz"
                - "position", "velocity", "acceleration"
                - "joints", "force", "deviation"
            overlay_path (str): Optional CSV file for planned trajectory (used in "deviation" plots).

        Returns:
            str: Status message indicating plot success or failure.
        """
        if not os.path.exists(csv_path):
            return f"CSV file not found: {csv_path}"

        try:
            plot_type = plot_type.lower()

            match plot_type:
                case "3d":
                    StaticPlotter.plot_3d_trajectory(csv_path)
                case "2d_xy":
                    StaticPlotter.plot_2d_projection(csv_path, plane="xy")
                case "2d_xz":
                    StaticPlotter.plot_2d_projection(csv_path, plane="xz")
                case "2d_yz":
                    StaticPlotter.plot_2d_projection(csv_path, plane="yz")
                case "position":
                    StaticPlotter.plot_position_vs_time(csv_path)
                case "velocity":
                    StaticPlotter.plot_velocity_vs_time(csv_path)
                case "acceleration":
                    StaticPlotter.plot_acceleration_vs_time(csv_path)
                case "joints":
                    StaticPlotter.plot_joint_angles(csv_path)
                case "force":
                    StaticPlotter.plot_motor_currents(csv_path)
                case "deviation":
                    if overlay_path is None or not os.path.exists(overlay_path):
                        return "Deviation plot requires a valid overlay CSV file."
                    StaticPlotter.plot_deviation(csv_path, overlay_path)
                case _:
                    return f"Invalid plot type '{plot_type}'. Use one of: 3d, 2d_xy, 2d_xz, 2d_yz, position, velocity, acceleration, joints, force, deviation."

            return f"✅ Plot '{plot_type}' generated successfully."
        except Exception as e:
            return f"Failed to generate plot '{plot_type}': {str(e)}"



    def start_live_plot(self, mode="3d", interval=100):
        if self.live_plotter and self.live_plotter.running:
            return "Live plotting already active."

        def runner():
            self.live_plotter = LivePlotter(self.client, mode=mode, interval=interval)
            self.live_plotter.start()

        self.live_plot_thread = Thread(target=runner, daemon=True)
        self.live_plot_thread.start()
        return f"Live plot started in '{mode}' mode at {interval}ms interval."

    def stop_live_plot(self):
        if self.live_plotter and self.live_plotter.running:
            self.live_plotter.stop()
            return "Live plotting stopped."
        return "No live plot is currently running."

    def change_live_plot_mode(self, mode):
        if self.live_plotter and self.live_plotter.running:
            self.live_plotter.change_mode(mode)
            return f"Live plot mode changed to '{mode}'."
        return "No live plot is active to change mode."
    


    # ✅ ALERT METHODS
    def enable_alerts(self, enable):
        """Enable or disable real-time alerts."""
        self.client.enable_alerts(enable)
        return f"Alerts {'enabled' if enable else 'disabled'}."

    def override_safety(self, enabled: bool):
        self.client.safety_manager.override_safety(enabled)

    def is_safety_overridden(self) -> bool:
        return self.client.safety_manager.is_safety_overridden()

    def set_alert_threshold(self, alert_type, value):
        """Set threshold for deviation or force alerts."""
        if alert_type in ["deviation", "force"]:
            self.client.set_alert_threshold(alert_type, value)
            return f"{alert_type.capitalize()} alert threshold set to {value}"
        return "Invalid alert type. Use 'deviation' or 'force'."

    @staticmethod
    def visualise_csv_log(csv_file, export=False):
        """
        Visualize CSV log file directly via RSIAPI.

        Args:
            csv_file (str): Path to CSV log file.
            export (bool): Whether to export the plots.
        """
        visualizer = KukaRSIVisualiser(csv_file)
        visualizer.plot_trajectory()
        visualizer.plot_joint_positions()
        visualizer.plot_force_trends()

        if export:
            visualizer.export_graphs()

    @staticmethod
    def parse_krl_to_csv(src_file, dat_file, output_file):
        """
        Parses KRL files (.src, .dat) and exports coordinates to CSV.

        Args:
            src_file (str): Path to KRL .src file.
            dat_file (str): Path to KRL .dat file.
            output_file (str): Path for output CSV file.
        """
        try:
            parser = KRLParser(src_file, dat_file)
            parser.parse_src()
            parser.parse_dat()
            parser.export_csv(output_file)
            return f"KRL data successfully exported to {output_file}"
        except Exception as e:
            return f"Error parsing KRL files: {e}"

    @staticmethod
    def inject_rsi(input_krl, output_krl=None, rsi_config="RSIGatewayv1.rsi"):
        """
        Inject RSI commands into a KRL (.src) program file.

        Args:
            input_krl (str): Path to the input KRL file.
            output_krl (str, optional): Path to the output file (defaults to overwriting input).
            rsi_config (str, optional): RSI configuration file name.
        """
        try:
            inject_rsi_to_krl(input_krl, output_krl, rsi_config)
            output_path = output_krl if output_krl else input_krl
            return f"RSI successfully injected into {output_path}"
        except Exception as e:
            return f"RSI injection failed: {e}"

    @staticmethod
    def generate_trajectory(start, end, steps=100, space="cartesian", mode="absolute", include_resets=False):
        """Generates a linear trajectory (Cartesian or Joint)."""
        return generate_trajectory(start, end, steps, space, mode, include_resets)

    import asyncio

    def execute_trajectory(self, trajectory, space="cartesian", rate=0.012):
        """
        Executes a trajectory intelligently:
        - If already inside an asyncio loop -> schedules task in background
        - If no loop -> creates one and runs blocking
        """

        async def runner():
            for idx, point in enumerate(trajectory):
                if space == "cartesian":
                    self.update_cartesian(**point)
                elif space == "joint":
                    self.update_joints(**point)
                else:
                    raise ValueError("space must be 'cartesian' or 'joint'")
                print(f"Step {idx + 1}/{len(trajectory)} sent")
                await asyncio.sleep(rate)

        try:
            loop = asyncio.get_running_loop()
            # If inside event loop, schedule runner as background task
            asyncio.create_task(runner())
        except RuntimeError:
            # If no event loop is running, create and run one
            asyncio.run(runner())

    def queue_trajectory(self, trajectory, space="cartesian", rate=0.012):
        """Adds a trajectory to the internal queue."""
        self.trajectory_queue.append({
            "trajectory": trajectory,
            "space": space,
            "rate": rate,
        })

    def clear_trajectory_queue(self):
        """Clears all queued trajectories."""
        self.trajectory_queue.clear()

    def get_trajectory_queue(self):
        """Returns current queued trajectories (metadata only)."""
        return [
            {"space": item["space"], "steps": len(item["trajectory"]), "rate": item["rate"]}
            for item in self.trajectory_queue
        ]

    def execute_queued_trajectories(self):
        """Executes all queued trajectories in order."""
        for item in self.trajectory_queue:
            self.execute_trajectory(item["trajectory"], item["space"], item["rate"])
        self.clear_trajectory_queue()

    def export_movement_data(self, filename="movement_log.csv"):
        """
        Exports recorded movement data (if available) to a CSV file.
        Assumes self.client.logger has stored entries.
        """
        if not hasattr(self.client, "logger") or self.client.logger is None:
            raise RuntimeError("No logger attached to RSI client.")

        data = self.client.get_movement_data()
        if not data:
            raise RuntimeError("No data available to export.")

        df = pd.DataFrame(data)
        df.to_csv(filename, index=False)
        return f"Movement data exported to {filename}"

    @staticmethod
    def compare_test_runs(file1, file2):
        """
        Compares two test run CSV files.
        Returns a summary of average and max deviation for each axis.
        """
        import pandas as pd

        df1 = pd.read_csv(file1)
        df2 = pd.read_csv(file2)

        shared_cols = [col for col in df1.columns if col in df2.columns and col.startswith("Receive.RIst")]
        diffs = {}

        for col in shared_cols:
            delta = abs(df1[col] - df2[col])
            diffs[col] = {
                "mean_diff": delta.mean(),
                "max_diff": delta.max(),
            }

        return diffs

    def update_cartesian(self, **kwargs):
        """
        Update Cartesian correction values (RKorr).
        """
        self._ensure_client()
        if "RKorr" not in self.client.send_variables:
            logging.warning("Warning: RKorr not configured in send_variables. Skipping Cartesian update.")
            return

        for axis, value in kwargs.items():
            self.update_variable(f"RKorr.{axis}", float(value))

    def update_joints(self, **kwargs):
        """
        Update joint correction values (AKorr).
        """
        self._ensure_client()
        if "AKorr" not in self.client.send_variables:
            logging.warning("⚠️ Warning: AKorr not configured in send_variables. Skipping Joint update.")
            return

        for axis, value in kwargs.items():
            self.update_variable(f"AKorr.{axis}", float(value))

    def watch_network(self, duration: float = None, rate: float = 0.2):
        """
        Continuously prints current receive variables (and IPOC).
        If duration is None, runs until interrupted.
        """
        import time
        import datetime

        logging.info("Watching network... Press Ctrl+C to stop.\n")
        start_time = time.time()

        try:
            while True:
                live_data = self.get_live_data()
                ipoc = live_data.get("IPOC", "N/A")
                rpos = live_data.get("RIst", {})
                print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] IPOC: {ipoc} | RIst: {rpos}")
                time.sleep(rate)

                if duration and (time.time() - start_time) >= duration:
                    break

        except KeyboardInterrupt:
            logging.info("\nStopped network watch.")

    def move_cartesian_trajectory(self, start_pose, end_pose, steps=50, rate=0.012):
        """
        Generate and execute a Cartesian (TCP) movement between two poses.
        Args:
            start_pose (dict): e.g. {"X":0, "Y":0, "Z":500}
            end_pose (dict): e.g. {"X":100, "Y":0, "Z":500}
            steps (int): Number of interpolation points.
            rate (float): Time between points in seconds.
        """
        trajectory = self.generate_trajectory(start_pose, end_pose, steps=steps, space="cartesian")
        self.execute_trajectory(trajectory, space="cartesian", rate=rate)

    def move_joint_trajectory(self, start_joints, end_joints, steps=50, rate=0.4):
        """
        Generate and execute a Joint-space movement between two poses.
        Args:
            start_joints (dict): e.g. {"A1":0, "A2":0, "A3":0, ...}
            end_joints (dict): e.g. {"A1":90, "A2":0, "A3":0, ...}
            steps (int): Number of interpolation points.
            rate (float): Time between points in seconds.
        """
        trajectory = self.generate_trajectory(start_joints, end_joints, steps=steps, space="joint")
        self.execute_trajectory(trajectory, space="joint", rate=rate)

    def queue_cartesian_trajectory(self, start_pose, end_pose, steps=50, rate=0.012):
        """
        Generate and queue a Cartesian movement (no execution).
        """
        if not isinstance(start_pose, dict) or not isinstance(end_pose, dict):
            raise ValueError("start_pose and end_pose must be dictionaries (e.g., {'X': 0, 'Y': 0, 'Z': 500})")
        if steps <= 0:
            raise ValueError("Steps must be greater than zero.")
        if rate <= 0:
            raise ValueError("Rate must be greater than zero.")

        trajectory = self.generate_trajectory(start_pose, end_pose, steps=steps, space="cartesian")
        self.queue_trajectory(trajectory, "cartesian", rate)

    def queue_joint_trajectory(self, start_joints, end_joints, steps=50, rate=0.4):
        """
        Generate and queue a Joint-space movement (no execution).
        """
        if not isinstance(start_joints, dict) or not isinstance(end_joints, dict):
            raise ValueError("start_joints and end_joints must be dictionaries (e.g., {'A1': 0, 'A2': 0})")
        if steps <= 0:
            raise ValueError("Steps must be greater than zero.")
        if rate <= 0:
            raise ValueError("Rate must be greater than zero.")

        trajectory = self.generate_trajectory(start_joints, end_joints, steps=steps, space="joint")
        self.queue_trajectory(trajectory, "joint", rate)

    # --- 🛡️ Safety Management ---

    def safety_stop(self):
        """Trigger emergency stop."""
        self._ensure_client()
        self.client.safety_manager.emergency_stop()

    def safety_reset(self):
        """Reset emergency stop."""
        self._ensure_client()
        self.client.safety_manager.reset_stop()

    def safety_status(self):
        """Return detailed safety status."""
        self._ensure_client()
        sm = self.client.safety_manager
        return {
            "emergency_stop": sm.is_stopped(),
            "safety_override": self.is_safety_overridden(),
            "limits": sm.get_limits(),
        }

    def safety_set_limit(self, variable, lower, upper):
        """Set new safety limit bounds for a specific variable."""
        self._ensure_client()
        self.client.safety_manager.set_limit(variable, float(lower), float(upper))
