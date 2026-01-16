"""Motion control API namespace for RSIPI."""

import logging
import asyncio
from typing import Dict, List, Any, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class MotionAPI:
    """
    Motion control interface for KUKA RSI robot control.

    Provides Cartesian and joint-space motion commands, trajectory generation,
    execution, and queueing capabilities. All motion commands go through the
    SafetyManager validation layer.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize MotionAPI namespace.

        Args:
            client: RSIClient instance for variable access and safety management
        """
        self.client = client
        self.trajectory_queue: List[Dict[str, Any]] = []

    def update_cartesian(self, **kwargs: float) -> None:
        """
        Update Cartesian correction values (RKorr).

        Applies corrections to TCP position in world coordinates. Values are
        added to the programmed path positions in the KRL program.

        Args:
            **kwargs: Axis corrections in mm/degrees:
                - X, Y, Z: Position corrections (mm)
                - A, B, C: Orientation corrections (degrees)

        Raises:
            RSISafetyViolation: If corrections exceed configured limits

        Example:
            >>> # Move TCP 10mm in X direction
            >>> api.motion.update_cartesian(X=10.0)

            >>> # Move in XYZ
            >>> api.motion.update_cartesian(X=5.0, Y=-3.0, Z=12.5)

            >>> # Full 6-axis correction
            >>> api.motion.update_cartesian(X=10, Y=5, Z=0, A=0, B=0, C=2.5)

        Note:
            RKorr must be configured in the RSI config file and enabled in KRL
            using RSI_MOVECORR() for corrections to take effect.
        """
        if "RKorr" not in self.client.send_variables:
            logging.warning("RKorr not configured in send_variables. Skipping Cartesian update.")
            return

        # Import here to avoid circular dependency
        from .tools_api import ToolsAPI
        tools = ToolsAPI(self.client)

        for axis, value in kwargs.items():
            tools.update_variable(f"RKorr.{axis}", float(value))
            logging.debug(f"RKorr.{axis} set to {value}")

    def update_joints(self, **kwargs: float) -> None:
        """
        Update joint correction values (AKorr).

        Applies corrections to individual joint angles. Values are added to
        the programmed joint positions in the KRL program.

        Args:
            **kwargs: Joint corrections in degrees:
                - A1, A2, A3, A4, A5, A6: Joint angle corrections

        Raises:
            RSISafetyViolation: If corrections exceed configured limits

        Example:
            >>> # Adjust joint 1 by 5 degrees
            >>> api.motion.update_joints(A1=5.0)

            >>> # Multi-axis correction
            >>> api.motion.update_joints(A1=10.0, A2=-5.0, A3=2.5)

        Note:
            AKorr must be configured in the RSI config file and enabled in KRL
            using RSI_MOVECORR() for corrections to take effect.
        """
        if "AKorr" not in self.client.send_variables:
            logging.warning("AKorr not configured in send_variables. Skipping Joint update.")
            return

        from .tools_api import ToolsAPI
        tools = ToolsAPI(self.client)

        for axis, value in kwargs.items():
            tools.update_variable(f"AKorr.{axis}", float(value))
            logging.debug(f"AKorr.{axis} set to {value}")

    def correct_position(self, correction_type: str, axis: str, value: float) -> str:
        """
        Apply a single correction to RKorr or AKorr.

        Lower-level method for explicit correction type and axis specification.
        Most users should use update_cartesian() or update_joints() instead.

        Args:
            correction_type: 'RKorr' or 'AKorr'
            axis: Axis name (e.g., 'X', 'Y', 'Z' for RKorr; 'A1'-'A6' for AKorr)
            value: Correction value

        Returns:
            Status message

        Raises:
            RSISafetyViolation: If correction exceeds configured limits

        Example:
            >>> api.motion.correct_position('RKorr', 'X', 10.0)
            'Updated RKorr.X to 10.0'
            >>> api.motion.correct_position('AKorr', 'A1', 5.0)
            'Updated AKorr.A1 to 5.0'
        """
        from .tools_api import ToolsAPI
        tools = ToolsAPI(self.client)
        return tools.update_variable(f"{correction_type}.{axis}", value)

    def move_external_axis(self, axis: str, value: float) -> str:
        """
        Move an external axis.

        Controls additional axes beyond the standard 6 robot axes, such as
        positioners, linear tracks, or tool changers.

        Args:
            axis: External axis name (e.g., 'E1', 'E2', 'E3')
            value: Position value (units depend on axis configuration)

        Returns:
            Status message

        Raises:
            RSISafetyViolation: If value exceeds configured limits

        Example:
            >>> # Move linear track (E1) to 500mm
            >>> api.motion.move_external_axis('E1', 500.0)
            'Updated ELPos.E1 to 500.0'
        """
        from .tools_api import ToolsAPI
        tools = ToolsAPI(self.client)
        return tools.update_variable(f"ELPos.{axis}", value)

    def adjust_speed(self, tech_param: str, value: float) -> str:
        """
        Adjust motion parameters via Tech variables.

        Tech variables allow runtime adjustment of motion parameters like
        velocity scaling, acceleration limits, or custom user parameters.

        Args:
            tech_param: Tech variable path (e.g., 'Tech.T21', 'Tech.C15')
            value: Parameter value

        Returns:
            Status message

        Example:
            >>> # Adjust velocity override via Tech.T21
            >>> api.motion.adjust_speed('Tech.T21', 0.5)  # 50% velocity
            'Updated Tech.T21 to 0.5'

        Note:
            Tech variable meanings depend on your KRL program implementation.
            Coordinate with your KRL developer on parameter assignments.
        """
        from .tools_api import ToolsAPI
        tools = ToolsAPI(self.client)
        return tools.update_variable(tech_param, value)

    @staticmethod
    def generate_trajectory(
        start: Dict[str, float],
        end: Dict[str, float],
        steps: int = 100,
        space: str = "cartesian",
        mode: str = "absolute",
        include_resets: bool = False
    ) -> List[Dict[str, float]]:
        """
        Generate linear interpolated trajectory between two poses.

        Creates a list of waypoints linearly interpolated between start and
        end positions. Supports both Cartesian and joint space.

        Args:
            start: Starting pose (e.g., {"X":0, "Y":0, "Z":500})
            end: Ending pose (e.g., {"X":100, "Y":0, "Z":500})
            steps: Number of interpolation points (default: 100)
            space: 'cartesian' or 'joint'
            mode: 'absolute' or 'relative' (reserved for future use)
            include_resets: Whether to reset to zero at end (default: False)

        Returns:
            List of waypoint dictionaries

        Example:
            >>> # Cartesian trajectory
            >>> traj = api.motion.generate_trajectory(
            ...     {"X":0, "Y":0, "Z":500},
            ...     {"X":100, "Y":0, "Z":500},
            ...     steps=50
            ... )
            >>> len(traj)
            50

            >>> # Joint trajectory
            >>> traj = api.motion.generate_trajectory(
            ...     {"A1":0, "A2":0, "A3":0},
            ...     {"A1":30, "A2":-15, "A3":45},
            ...     steps=100,
            ...     space="joint"
            ... )

        Note:
            This uses simple linear interpolation. For velocity-profiled
            trajectories, see Phase 4 enhancements (trapezoidal/S-curve).
        """
        from .trajectory_planner import generate_trajectory as gen_traj
        return gen_traj(start, end, steps, space, mode, include_resets)

    def execute_trajectory(
        self,
        trajectory: List[Dict[str, float]],
        space: str = "cartesian",
        rate: float = 0.012
    ) -> None:
        """
        Execute a trajectory asynchronously.

        Sends waypoints sequentially to the robot at the specified rate.
        Uses asyncio for non-blocking execution.

        Args:
            trajectory: List of waypoint dictionaries
            space: 'cartesian' or 'joint'
            rate: Time between waypoints in seconds (default: 0.012 = ~80Hz)

        Raises:
            RSITrajectoryError: If space is invalid

        Example:
            >>> # Generate and execute Cartesian trajectory
            >>> traj = api.motion.generate_trajectory(
            ...     {"X":0, "Y":0, "Z":500},
            ...     {"X":100, "Y":0, "Z":500},
            ...     steps=50
            ... )
            >>> api.motion.execute_trajectory(traj, space="cartesian", rate=0.02)

        Note:
            This method uses asyncio. If no event loop is running, one will
            be created automatically. The trajectory executes in the background.
        """
        from .exceptions import RSITrajectoryError

        async def runner():
            for idx, point in enumerate(trajectory):
                if space == "cartesian":
                    self.update_cartesian(**point)
                elif space == "joint":
                    self.update_joints(**point)
                else:
                    raise RSITrajectoryError("space must be 'cartesian' or 'joint'")
                logging.debug(f"Trajectory step {idx + 1}/{len(trajectory)}")
                await asyncio.sleep(rate)

        try:
            loop = asyncio.get_running_loop()
            asyncio.create_task(runner())
        except RuntimeError:
            # No event loop running, create one
            asyncio.run(runner())

    def move_cartesian_trajectory(
        self,
        start_pose: Dict[str, float],
        end_pose: Dict[str, float],
        steps: int = 50,
        rate: float = 0.012
    ) -> None:
        """
        Generate and execute Cartesian trajectory in one call.

        Convenience method that combines generate_trajectory() and
        execute_trajectory() for Cartesian motion.

        Args:
            start_pose: Starting Cartesian pose
            end_pose: Ending Cartesian pose
            steps: Number of waypoints (default: 50)
            rate: Time between waypoints in seconds (default: 0.012)

        Example:
            >>> api.motion.move_cartesian_trajectory(
            ...     {"X":0, "Y":0, "Z":500},
            ...     {"X":100, "Y":0, "Z":500},
            ...     steps=50,
            ...     rate=0.02
            ... )
        """
        trajectory = self.generate_trajectory(start_pose, end_pose, steps=steps, space="cartesian")
        self.execute_trajectory(trajectory, space="cartesian", rate=rate)

    def move_joint_trajectory(
        self,
        start_joints: Dict[str, float],
        end_joints: Dict[str, float],
        steps: int = 50,
        rate: float = 0.4
    ) -> None:
        """
        Generate and execute joint-space trajectory in one call.

        Convenience method for joint-space motion with sensible defaults
        (slower rate typical for joint motion).

        Args:
            start_joints: Starting joint configuration
            end_joints: Ending joint configuration
            steps: Number of waypoints (default: 50)
            rate: Time between waypoints in seconds (default: 0.4 for smooth joint motion)

        Example:
            >>> api.motion.move_joint_trajectory(
            ...     {"A1":0, "A2":0, "A3":0, "A4":0, "A5":0, "A6":0},
            ...     {"A1":30, "A2":-15, "A3":45, "A4":0, "A5":30, "A6":0},
            ...     steps=100
            ... )
        """
        trajectory = self.generate_trajectory(start_joints, end_joints, steps=steps, space="joint")
        self.execute_trajectory(trajectory, space="joint", rate=rate)

    def queue_trajectory(
        self,
        trajectory: List[Dict[str, float]],
        space: str = "cartesian",
        rate: float = 0.012
    ) -> None:
        """
        Add trajectory to execution queue without immediate execution.

        Allows building up a sequence of trajectories that can be executed
        together via execute_queued_trajectories().

        Args:
            trajectory: List of waypoint dictionaries
            space: 'cartesian' or 'joint'
            rate: Time between waypoints in seconds

        Example:
            >>> # Queue multiple trajectories
            >>> traj1 = api.motion.generate_trajectory(p0, p1, 50)
            >>> traj2 = api.motion.generate_trajectory(p1, p2, 50)
            >>> api.motion.queue_trajectory(traj1)
            >>> api.motion.queue_trajectory(traj2)
            >>> api.motion.execute_queued_trajectories()
        """
        self.trajectory_queue.append({
            "trajectory": trajectory,
            "space": space,
            "rate": rate,
        })
        logging.debug(f"Queued trajectory: {len(trajectory)} points, {space} space")

    def queue_cartesian_trajectory(
        self,
        start_pose: Dict[str, float],
        end_pose: Dict[str, float],
        steps: int = 50,
        rate: float = 0.012
    ) -> None:
        """
        Generate and queue Cartesian trajectory.

        Args:
            start_pose: Starting Cartesian pose
            end_pose: Ending Cartesian pose
            steps: Number of waypoints
            rate: Time between waypoints in seconds

        Raises:
            ValueError: If poses are invalid or parameters out of range

        Example:
            >>> api.motion.queue_cartesian_trajectory(
            ...     {"X":0, "Y":0, "Z":500},
            ...     {"X":100, "Y":0, "Z":500}
            ... )
        """
        if not isinstance(start_pose, dict) or not isinstance(end_pose, dict):
            raise ValueError("start_pose and end_pose must be dictionaries")
        if steps <= 0:
            raise ValueError("Steps must be greater than zero")
        if rate <= 0:
            raise ValueError("Rate must be greater than zero")

        trajectory = self.generate_trajectory(start_pose, end_pose, steps=steps, space="cartesian")
        self.queue_trajectory(trajectory, "cartesian", rate)

    def queue_joint_trajectory(
        self,
        start_joints: Dict[str, float],
        end_joints: Dict[str, float],
        steps: int = 50,
        rate: float = 0.4
    ) -> None:
        """
        Generate and queue joint-space trajectory.

        Args:
            start_joints: Starting joint configuration
            end_joints: Ending joint configuration
            steps: Number of waypoints
            rate: Time between waypoints in seconds

        Raises:
            ValueError: If joints are invalid or parameters out of range

        Example:
            >>> api.motion.queue_joint_trajectory(
            ...     {"A1":0, "A2":0, "A3":0},
            ...     {"A1":30, "A2":-15, "A3":45}
            ... )
        """
        if not isinstance(start_joints, dict) or not isinstance(end_joints, dict):
            raise ValueError("start_joints and end_joints must be dictionaries")
        if steps <= 0:
            raise ValueError("Steps must be greater than zero")
        if rate <= 0:
            raise ValueError("Rate must be greater than zero")

        trajectory = self.generate_trajectory(start_joints, end_joints, steps=steps, space="joint")
        self.queue_trajectory(trajectory, "joint", rate)

    def execute_queued_trajectories(self) -> None:
        """
        Execute all queued trajectories in sequence.

        Processes the trajectory queue in FIFO order, executing each with its
        configured space and rate. Clears the queue after execution.

        Example:
            >>> api.motion.queue_cartesian_trajectory(p0, p1, 50)
            >>> api.motion.queue_cartesian_trajectory(p1, p2, 50)
            >>> api.motion.execute_queued_trajectories()
            >>> # Both trajectories executed sequentially
        """
        logging.info(f"Executing {len(self.trajectory_queue)} queued trajectories")
        for idx, item in enumerate(self.trajectory_queue):
            logging.debug(f"Executing queued trajectory {idx + 1}/{len(self.trajectory_queue)}")
            self.execute_trajectory(item["trajectory"], item["space"], item["rate"])
        self.clear_queue()

    def clear_queue(self) -> None:
        """
        Clear all queued trajectories without execution.

        Example:
            >>> api.motion.queue_cartesian_trajectory(p0, p1, 50)
            >>> api.motion.clear_queue()  # Discard without executing
        """
        count = len(self.trajectory_queue)
        self.trajectory_queue.clear()
        logging.debug(f"Cleared {count} queued trajectories")

    def get_queue(self) -> List[Dict[str, Any]]:
        """
        Get metadata about queued trajectories.

        Returns summary information (space, step count, rate) without the
        full trajectory data.

        Returns:
            List of trajectory metadata dictionaries

        Example:
            >>> api.motion.queue_cartesian_trajectory(p0, p1, 50)
            >>> api.motion.queue_cartesian_trajectory(p1, p2, 100)
            >>> queue = api.motion.get_queue()
            >>> for item in queue:
            ...     print(f"{item['space']}: {item['steps']} steps at {item['rate']}s")
            cartesian: 50 steps at 0.012s
            cartesian: 100 steps at 0.012s
        """
        return [
            {"space": item["space"], "steps": len(item["trajectory"]), "rate": item["rate"]}
            for item in self.trajectory_queue
        ]

    # TODO (Phase 4): Implement advanced motion features
    # def generate_velocity_profile(self, trajectory, profile='trapezoidal'):
    #     """Apply velocity profiling to trajectory waypoints."""
    #     pass
    #
    # def generate_arc(self, center, radius, start_angle, end_angle, steps):
    #     """Generate circular arc trajectory."""
    #     pass
    #
    # def generate_circle(self, center, radius, steps):
    #     """Generate full circle trajectory."""
    #     pass
    #
    # def blend_trajectories(self, traj1, traj2, blend_radius):
    #     """Smooth transition between two trajectories."""
    #     pass
