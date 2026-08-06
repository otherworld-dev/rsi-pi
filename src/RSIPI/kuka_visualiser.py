import logging
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os


class KukaRSIVisualiser:
    """
    Visualises robot motion and diagnostics from RSI-generated CSV logs.

    Supports:
    - 3D trajectory plotting (actual vs planned)
    - Joint position plotting with safety band overlays
    - Force correction trend visualisation
    - Optional graph export to PNG
    """

    def __init__(self, csv_file, safety_limits=None):
        """
        Initialise the visualiser.

        Args:
            csv_file (str): Path to the RSI CSV log.
            safety_limits (dict): Optional dict of axis limits (e.g., {"AIPos.A1": [-170, 170]}).
        """
        self.csv_file = csv_file
        self.safety_limits = safety_limits or {}

        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"CSV file {csv_file} not found.")

        self.df = pd.read_csv(csv_file)

    def _safe_col(self, name):
        """
        Resolve a bare variable name (e.g. 'RIst.X') to the actual CSV
        column name written by the logger, trying in order:
        bare name, then 'Send.<name>' (robot state - send_variables is what
        the ROBOT sends to us), then 'Receive.<name>' (our corrections) as a
        last resort.
        """
        if name in self.df.columns:
            return name
        if f"Send.{name}" in self.df.columns:
            return f"Send.{name}"
        return f"Receive.{name}"

    def plot_trajectory(self, save_path=None):
        """
        Plots the 3D robot trajectory from actual and planned data.

        Args:
            save_path (str): Optional path to save the figure.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        safe_col = self._safe_col

        ax.plot(self.df[safe_col("RIst.X")],
                self.df[safe_col("RIst.Y")],
                self.df[safe_col("RIst.Z")],
                label="Actual Trajectory", linestyle='-')

        rsol_x, rsol_y, rsol_z = safe_col("RSol.X"), safe_col("RSol.Y"), safe_col("RSol.Z")
        if rsol_x in self.df.columns and rsol_y in self.df.columns and rsol_z in self.df.columns:
            ax.plot(self.df[rsol_x], self.df[rsol_y], self.df[rsol_z],
                    label="Planned Trajectory", linestyle='--')

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.set_title("Robot Trajectory")
        ax.legend()

        if save_path:
            plt.savefig(save_path)
        plt.show()

    def has_column(self, col):
        """
        Checks if the given column exists in the dataset.

        Args:
            col (str): Column name to check.
        """
        return col in self.df.columns

    def plot_joint_positions(self, save_path=None):
        """
        Plots joint angle positions over time, with optional safety zone overlays.

        Joint position is logged as either 'Send.ASPos.A<n>' or
        'Send.AIPos.A<n>' depending on config (robot state comes from
        send_variables - what the ROBOT sends to us).

        Args:
            save_path (str): Optional path to save the figure.
        """
        plt.figure()
        time_series = range(len(self.df))

        plotted_any = False
        for i in range(1, 7):
            base = f"A{i}"
            col = None
            for candidate_name in (f"ASPos.{base}", f"AIPos.{base}"):
                candidate = self._safe_col(candidate_name)
                if candidate in self.df.columns:
                    col = candidate
                    break

            if col is None:
                continue

            plotted_any = True
            plt.plot(time_series, self.df[col], label=col)

            if col in self.safety_limits:
                low, high = self.safety_limits[col]
                plt.axhspan(low, high, color='red', alpha=0.1, label=f"{col} Safe Zone")

        if not plotted_any:
            logging.warning(
                "plot_joint_positions: no joint position columns "
                "('Send.ASPos.A1..A6' or 'Send.AIPos.A1..A6') found in %s",
                self.csv_file,
            )
            plt.close()
            return

        plt.xlabel("Time Steps")
        plt.ylabel("Joint Position (Degrees)")
        plt.title("Joint Positions Over Time")
        plt.legend()

        if save_path:
            plt.savefig(save_path)
        plt.show()

    def plot_force_trends(self, save_path=None):
        """
        Plots correction trends (RKorr.*) over time, if present.

        Corrections are logged with the 'Receive.' prefix (receive_variables
        is what the robot receives from us).

        Args:
            save_path (str): Optional path to save the figure.
        """
        force_columns = [self._safe_col(name) for name in ("RKorr.X", "RKorr.Y", "RKorr.Z")]
        plt.figure()
        time_series = range(len(self.df))

        plotted_any = False
        for col in force_columns:
            if col in self.df.columns:
                plotted_any = True
                plt.plot(time_series, self.df[col], label=col)

                if col in self.safety_limits:
                    low, high = self.safety_limits[col]
                    plt.axhspan(low, high, color='red', alpha=0.1, label=f"{col} Safe Zone")

        if not plotted_any:
            logging.warning(
                "plot_force_trends: no correction columns "
                "('Receive.RKorr.X/Y/Z') found in %s",
                self.csv_file,
            )
            plt.close()
            return

        plt.xlabel("Time Steps")
        plt.ylabel("Force Correction (N)")
        plt.title("Force Trends Over Time")
        plt.legend()

        if save_path:
            plt.savefig(save_path)
        plt.show()

    def export_graphs(self, export_dir="exports"):
        """
        Saves all graphs (trajectory, joints, force) as PNG images.

        Args:
            export_dir (str): Output directory.
        """
        os.makedirs(export_dir, exist_ok=True)
        self.plot_trajectory(save_path=os.path.join(export_dir, "trajectory.png"))
        self.plot_joint_positions(save_path=os.path.join(export_dir, "joint_positions.png"))
        self.plot_force_trends(save_path=os.path.join(export_dir, "force_trends.png"))
        print(f"Graphs exported to {export_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualise RSI data logs.")
    parser.add_argument("csv_file", type=str, help="Path to the RSI CSV log file.")
    parser.add_argument("--export", action="store_true", help="Export graphs as PNG/PDF.")
    parser.add_argument("--limits", type=str, help="Optional .rsi.xml file to overlay safety bands")

    args = parser.parse_args()

    if args.limits:
        from .rsi_limit_parser import parse_rsi_limits
        limits = parse_rsi_limits(args.limits)
        visualiser = KukaRSIVisualiser(args.csv_file, safety_limits=limits)
    else:
        visualiser = KukaRSIVisualiser(args.csv_file)

    visualiser.plot_trajectory()
    visualiser.plot_joint_positions()
    visualiser.plot_force_trends()

    if args.export:
        visualiser.export_graphs()
