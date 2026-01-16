"""Visualization API namespace for RSIPI."""

import logging
import os
from typing import Optional, TYPE_CHECKING
from threading import Thread

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class VizAPI:
    """
    Data visualization interface for KUKA RSI robot control.

    Provides static plot generation from CSV logs and live plotting
    capabilities for real-time monitoring of robot motion.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize VizAPI namespace.

        Args:
            client: RSIClient instance for live data access
        """
        self.client = client
        self.live_plotter = None
        self.live_plot_thread = None

    @staticmethod
    def plot_static(csv_path: str, plot_type: str = "3d", overlay_path: Optional[str] = None) -> str:
        """
        Generate static plots from CSV log data.

        Creates matplotlib visualizations of recorded robot trajectories,
        velocities, forces, and other metrics.

        Args:
            csv_path: Path to CSV log file
            plot_type: Type of plot to generate:
                - "3d": 3D trajectory visualization
                - "2d_xy", "2d_xz", "2d_yz": 2D projections
                - "position": Position vs time
                - "velocity": Velocity vs time
                - "acceleration": Acceleration vs time
                - "joints": Joint angles vs time
                - "force": Motor currents vs time
                - "deviation": Deviation from planned path (requires overlay_path)
            overlay_path: Optional CSV with planned trajectory for deviation plots

        Returns:
            Status message

        Raises:
            FileNotFoundError: If CSV file doesn't exist
            ValueError: If plot_type is invalid

        Example:
            >>> api.viz.plot_static('logs/test.csv', '3d')
            'Plot 3d generated successfully'

            >>> api.viz.plot_static('logs/actual.csv', 'deviation', 'logs/planned.csv')
            'Plot deviation generated successfully'

        Note:
            Plots are displayed interactively. Close the plot window to continue.
        """
        if not os.path.exists(csv_path):
            return f"CSV file not found: {csv_path}"

        try:
            from .static_plotter import StaticPlotter

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
                        return "Deviation plot requires a valid overlay CSV file"
                    StaticPlotter.plot_deviation(csv_path, overlay_path)
                case _:
                    valid_types = "3d, 2d_xy, 2d_xz, 2d_yz, position, velocity, acceleration, joints, force, deviation"
                    return f"Invalid plot type '{plot_type}'. Use one of: {valid_types}"

            logging.info(f"Generated {plot_type} plot from {csv_path}")
            return f"Plot '{plot_type}' generated successfully"
        except Exception as e:
            logging.error(f"Plot generation failed: {e}")
            return f"Failed to generate plot '{plot_type}': {str(e)}"

    def start_live_plot(self, mode: str = "3d", interval: int = 100) -> str:
        """
        Start live plotting of robot data.

        Opens an interactive matplotlib window that updates in real-time
        as robot data is received. Useful for monitoring during operation.

        Args:
            mode: Plot mode:
                - "3d": Live 3D trajectory
                - "position": Position vs time
                - "velocity": Velocity vs time
                - "force": Motor currents vs time
            interval: Update interval in milliseconds (default: 100ms = 10Hz)

        Returns:
            Status message

        Example:
            >>> api.viz.start_live_plot('3d', interval=100)
            'Live plot started in 3d mode at 100ms interval'

            >>> # Watch robot move in real-time
            >>> # Close plot window or call stop_live_plot() to end

        Note:
            Live plotting runs in a background thread. The plot window must
            remain open for updates to continue.
        """
        if self.live_plotter and hasattr(self.live_plotter, 'running') and self.live_plotter.running:
            return "Live plotting already active"

        def runner():
            from .live_plotter import LivePlotter
            self.live_plotter = LivePlotter(self.client, mode=mode, interval=interval)
            self.live_plotter.start()

        self.live_plot_thread = Thread(target=runner, daemon=True)
        self.live_plot_thread.start()
        logging.info(f"Live plot started: {mode} mode at {interval}ms")
        return f"Live plot started in '{mode}' mode at {interval}ms interval"

    def stop_live_plot(self) -> str:
        """
        Stop live plotting.

        Closes the live plot window and stops the update thread.

        Returns:
            Status message

        Example:
            >>> api.viz.stop_live_plot()
            'Live plotting stopped'
        """
        if self.live_plotter and hasattr(self.live_plotter, 'running') and self.live_plotter.running:
            self.live_plotter.stop()
            logging.info("Live plotting stopped")
            return "Live plotting stopped"
        return "No live plot is currently running"

    def change_live_plot_mode(self, mode: str) -> str:
        """
        Change the mode of an active live plot.

        Switches between different visualization modes without restarting
        the plot window.

        Args:
            mode: New plot mode (e.g., '3d', 'position', 'velocity', 'force')

        Returns:
            Status message

        Example:
            >>> api.viz.start_live_plot('3d')
            >>> # ... watch for a while ...
            >>> api.viz.change_live_plot_mode('position')
            'Live plot mode changed to position'
        """
        if self.live_plotter and hasattr(self.live_plotter, 'running') and self.live_plotter.running:
            if hasattr(self.live_plotter, 'change_mode'):
                self.live_plotter.change_mode(mode)
                logging.info(f"Live plot mode changed to: {mode}")
                return f"Live plot mode changed to '{mode}'"
            else:
                return "Live plotter does not support mode changing"
        return "No live plot is active to change mode"

    @staticmethod
    def visualize_csv_log(csv_file: str, export: bool = False) -> None:
        """
        Comprehensive visualization of CSV log file.

        Generates multiple plots (trajectory, joints, force) using the
        KukaRSIVisualiser. Optionally exports plots to files.

        Args:
            csv_file: Path to CSV log file
            export: If True, save plots to disk instead of displaying

        Example:
            >>> # Display interactive plots
            >>> api.viz.visualize_csv_log('logs/test.csv')

            >>> # Export plots to files
            >>> api.viz.visualize_csv_log('logs/test.csv', export=True)

        Note:
            This generates three separate plots:
            - 3D trajectory with start/end markers
            - Joint angle evolution
            - Motor current trends
        """
        from .kuka_visualiser import KukaRSIVisualiser

        visualizer = KukaRSIVisualiser(csv_file)
        visualizer.plot_trajectory()
        visualizer.plot_joint_positions()
        visualizer.plot_force_trends()

        if export:
            visualizer.export_graphs()
            logging.info(f"Exported visualizations for {csv_file}")
        else:
            logging.info(f"Displayed visualizations for {csv_file}")

    @staticmethod
    def compare_runs(file1: str, file2: str) -> str:
        """
        Generate comparison plots for two test runs.

        Visualizes deviations between two recorded trajectories for
        repeatability analysis.

        Args:
            file1: Path to first CSV log file
            file2: Path to second CSV log file

        Returns:
            Status message

        Example:
            >>> api.viz.compare_runs('run1.csv', 'run2.csv')
            'Comparison plots generated successfully'

        Note:
            Uses plot_static() with deviation mode internally.
        """
        if not os.path.exists(file1):
            return f"First file not found: {file1}"
        if not os.path.exists(file2):
            return f"Second file not found: {file2}"

        try:
            from .static_plotter import StaticPlotter
            StaticPlotter.plot_deviation(file1, file2)
            logging.info(f"Generated comparison plot: {file1} vs {file2}")
            return "Comparison plots generated successfully"
        except Exception as e:
            logging.error(f"Comparison plot failed: {e}")
            return f"Failed to generate comparison: {str(e)}"
