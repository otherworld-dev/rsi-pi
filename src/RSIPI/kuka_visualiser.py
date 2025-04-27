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

    def plot_trajectory(self, save_path=None):
        """
        Plots the 3D robot trajectory from actual and planned data.

        Args:
            save_path (str): Optional path to save the figure.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        def safe_col(name):
            return name if name in self.df.columns else f"Receive.{name}"

        ax.plot(self.df[safe_col("RIst.X")],
                self.df[safe_col("RIst.Y")],
                self.df[safe_col("RIst.Z")],
                label="Actual Trajectory", linestyle='-')

        if "RSol.X" in self.df.columns:
            ax.plot(self.df["RSol.X"], self.df["RSol.Y"], self.df["RSol.Z"],
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

        Args:
            save_path (str): Optional path to save the figure.
        """
        plt.figure()
        time_series = range(len(self.df))

        for col in ["AIPos.A1", "AIPos.A2", "AIPos.A3", "AIPos.A4", "AIPos.A5", "AIPos.A6"]:
            if col in self.df.columns:
                plt.plot(time_series, self.df[col], label=col)

                if col in self.safety_limits:
                    low, high = self.safety_limits[col]
                    plt.axhspan(low, high, color='red', alpha=0.1, label=f"{col} Safe Zone")

        plt.xlabel("Time Steps")
        plt.ylabel("Joint Position (Degrees)")
        plt.title("Joint Positions Over Time")
        plt.legend()

        if save_path:
            plt.savefig(save_path)
        plt.show()

    def plot_force_trends(self, save_path=None):
        """
        Plots force correction trends (PosCorr.*) over time, if present.

        Args:
            save_path (str): Optional path to save the figure.
        """
        force_columns = ["PosCorr.X", "PosCorr.Y", "PosCorr.Z"]
        plt.figure()
        time_series = range(len(self.df))

        for col in force_columns:
            if col in self.df.columns:
                plt.plot(time_series, self.df[col], label=col)

                if col in self.safety_limits:
                    low, high = self.safety_limits[col]
                    plt.axhspan(low, high, color='red', alpha=0.1, label=f"{col} Safe Zone")

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
        from src.RSIPI.rsi_limit_parser import parse_rsi_limits
        limits = parse_rsi_limits(args.limits)
        visualiser = KukaRSIVisualiser(args.csv_file, safety_limits=limits)
    else:
        visualiser = KukaRSIVisualiser(args.csv_file)

    visualiser.plot_trajectory()
    visualiser.plot_joint_positions()
    visualiser.plot_force_trends()

    if args.export:
        visualiser.export_graphs()
