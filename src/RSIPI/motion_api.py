"""Motion control API namespace for RSIPI."""

import logging
import threading
import math
import numpy as np
from typing import Dict, List, Any, Optional, Tuple, TYPE_CHECKING

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

        from .tools_api import ToolsAPI
        self._tools = ToolsAPI(client)

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
        if "RKorr" not in self.client.receive_variables:
            logging.warning("RKorr not configured in receive_variables. Skipping Cartesian update.")
            return

        for axis, value in kwargs.items():
            self._tools.update_variable(f"RKorr.{axis}", float(value))
            logging.debug("RKorr.%s set to %s", axis, value)

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
        if "AKorr" not in self.client.receive_variables:
            logging.warning("AKorr not configured in receive_variables. Skipping Joint update.")
            return

        for axis, value in kwargs.items():
            self._tools.update_variable(f"AKorr.{axis}", float(value))
            logging.debug("AKorr.%s set to %s", axis, value)

    def get_current_pose(self) -> Dict[str, float]:
        """
        Get current TCP position from robot.

        Returns:
            Dict with X, Y, Z (mm) and A, B, C (degrees)
        """
        rist = self.client.send_variables.get("RIst", {})
        return dict(rist) if isinstance(rist, dict) else {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0}

    def get_current_joints(self) -> Dict[str, float]:
        """
        Get current joint positions from robot.

        Returns:
            Dict with A1-A6 in degrees
        """
        aspos = self.client.send_variables.get("ASPos", {})
        return dict(aspos) if isinstance(aspos, dict) else {"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0}

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
        return self._tools.update_variable(f"{correction_type}.{axis}", value)

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
        return self._tools.update_variable(f"ELPos.{axis}", value)

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
        return self._tools.update_variable(tech_param, value)

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
        Execute a trajectory, blocking until complete.

        Sends waypoints sequentially to the robot at the specified rate.
        Can be cancelled via cancel_trajectory().

        Args:
            trajectory: List of waypoint dictionaries
            space: 'cartesian' or 'joint'
            rate: Time between waypoints in seconds (default: 0.012 = ~80Hz)

        Raises:
            RSITrajectoryError: If space is invalid
        """
        import time
        from .exceptions import RSITrajectoryError

        self._trajectory_cancel = threading.Event()

        def runner():
            for idx, point in enumerate(trajectory):
                if self._trajectory_cancel.is_set():
                    logging.info("Trajectory cancelled at step %d/%d", idx, len(trajectory))
                    return
                if space == "cartesian":
                    self.update_cartesian(**point)
                elif space == "joint":
                    self.update_joints(**point)
                else:
                    raise RSITrajectoryError("space must be 'cartesian' or 'joint'")
                logging.debug("Trajectory step %d/%d", idx + 1, len(trajectory))
                time.sleep(rate)

        self._trajectory_thread = threading.Thread(target=runner, daemon=True)
        self._trajectory_thread.start()
        self._trajectory_thread.join()  # Block until complete

        # Zero corrections after trajectory to stop motion in relative mode
        if space == "cartesian":
            self.update_cartesian(**{k: 0.0 for k in trajectory[0]})
        elif space == "joint":
            self.update_joints(**{k: 0.0 for k in trajectory[0]})

    def cancel_trajectory(self) -> None:
        """Cancel a running trajectory execution."""
        if hasattr(self, '_trajectory_cancel'):
            self._trajectory_cancel.set()

    def move_cartesian_trajectory(
        self,
        end_pose: Dict[str, float],
        start_pose: Optional[Dict[str, float]] = None,
        steps: int = 50,
        rate: float = 0.012
    ) -> None:
        """
        Generate and execute Cartesian trajectory in one call.

        Args:
            end_pose: Target Cartesian pose
            start_pose: Starting pose (default: current robot position)
            steps: Number of waypoints (default: 50)
            rate: Time between waypoints in seconds (default: 0.012)

        Example:
            >>> # Move to target from current position
            >>> api.motion.move_cartesian_trajectory({"X":100, "Y":0, "Z":500})
            >>> # Explicit start
            >>> api.motion.move_cartesian_trajectory(
            ...     {"X":100, "Y":0, "Z":500},
            ...     start_pose={"X":0, "Y":0, "Z":500}
            ... )
        """
        if start_pose is None:
            start_pose = self.get_current_pose()
        trajectory = self.generate_trajectory(start_pose, end_pose, steps=steps, space="cartesian")
        self.execute_trajectory(trajectory, space="cartesian", rate=rate)

    def move_joint_trajectory(
        self,
        end_joints: Dict[str, float],
        start_joints: Optional[Dict[str, float]] = None,
        steps: int = 50,
        rate: float = 0.4
    ) -> None:
        """
        Generate and execute joint-space trajectory in one call.

        Args:
            end_joints: Target joint configuration
            start_joints: Starting joints (default: current robot joints)
            steps: Number of waypoints (default: 50)
            rate: Time between waypoints in seconds (default: 0.4)

        Example:
            >>> # Move to target from current joints
            >>> api.motion.move_joint_trajectory({"A1":30, "A2":-15, "A3":45})
            >>> # Explicit start
            >>> api.motion.move_joint_trajectory(
            ...     {"A1":30, "A2":-15, "A3":45},
            ...     start_joints={"A1":0, "A2":0, "A3":0}
            ... )
        """
        if start_joints is None:
            start_joints = self.get_current_joints()
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
        logging.debug("Queued trajectory: %d points, %s space", len(trajectory), space)

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
        logging.info("Executing %d queued trajectories", len(self.trajectory_queue))
        for idx, item in enumerate(self.trajectory_queue):
            logging.debug("Executing queued trajectory %d/%d", idx + 1, len(self.trajectory_queue))
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
        logging.debug("Cleared %d queued trajectories", count)

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

    @staticmethod
    def generate_velocity_profile(
        trajectory: List[Dict[str, float]],
        max_velocity: float = 1.0,
        max_acceleration: float = 2.0,
        profile: str = 'trapezoidal'
    ) -> List[Tuple[Dict[str, float], float]]:
        """
        Apply velocity profiling to trajectory waypoints.

        Generates time-optimal velocity profiles that respect velocity and
        acceleration limits. Returns trajectory with timing information.

        Args:
            trajectory: List of waypoint dictionaries
            max_velocity: Maximum velocity (units/s)
            max_acceleration: Maximum acceleration (units/s²)
            profile: Velocity profile type - 'trapezoidal' or 's-curve'

        Returns:
            List of tuples (waypoint, velocity) for each point

        Example:
            >>> traj = api.motion.generate_trajectory(p0, p1, 100)
            >>> profiled = api.motion.generate_velocity_profile(
            ...     traj,
            ...     max_velocity=200.0,  # mm/s
            ...     max_acceleration=500.0,  # mm/s²
            ...     profile='trapezoidal'
            ... )
            >>> for waypoint, velocity in profiled:
            ...     print(f"Point: {waypoint}, Velocity: {velocity:.2f} mm/s")

        Note:
            Trapezoidal profiles have sharp velocity transitions (bang-bang control).
            S-curve profiles have smooth velocity transitions (jerk-limited).
            S-curve is recommended for sensitive applications requiring smooth motion.
        """
        if not trajectory:
            return []

        n = len(trajectory)
        if n < 2:
            return [(trajectory[0], 0.0)]

        # Calculate distances between consecutive points
        distances = []
        for i in range(n - 1):
            dist = _calculate_distance(trajectory[i], trajectory[i + 1])
            distances.append(dist)

        total_distance = sum(distances)

        if profile.lower() == 'trapezoidal':
            velocities = _trapezoidal_profile(distances, total_distance, max_velocity, max_acceleration)
        elif profile.lower() == 's-curve' or profile.lower() == 'scurve':
            velocities = _s_curve_profile(distances, total_distance, max_velocity, max_acceleration)
        else:
            raise ValueError(f"Unknown profile type: {profile}. Use 'trapezoidal' or 's-curve'")

        # Combine trajectory with velocities
        result = [(trajectory[i], velocities[i]) for i in range(n)]
        return result

    @staticmethod
    def generate_arc(
        center: Dict[str, float],
        radius: float,
        start_angle: float,
        end_angle: float,
        steps: int = 100,
        plane: str = 'XY'
    ) -> List[Dict[str, float]]:
        """
        Generate circular arc trajectory.

        Creates waypoints along a circular arc in the specified plane.

        Args:
            center: Arc center point (e.g., {"X": 100, "Y": 0, "Z": 500})
            radius: Arc radius in mm
            start_angle: Starting angle in degrees
            end_angle: Ending angle in degrees
            steps: Number of waypoints along arc
            plane: Plane for arc - 'XY', 'XZ', or 'YZ' (default: 'XY')

        Returns:
            List of Cartesian waypoints along the arc

        Example:
            >>> # 90-degree arc in XY plane
            >>> arc = api.motion.generate_arc(
            ...     center={"X": 100, "Y": 0, "Z": 500},
            ...     radius=50.0,
            ...     start_angle=0,
            ...     end_angle=90,
            ...     steps=50
            ... )
            >>> api.motion.execute_trajectory(arc, space='cartesian')

        Note:
            Angles are measured counterclockwise from the positive X/Y/Z axis
            depending on the plane. Arc direction follows right-hand rule.
        """
        if steps < 2:
            raise ValueError("Steps must be at least 2")
        if radius <= 0:
            raise ValueError("Radius must be positive")

        # Convert angles to radians
        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        angle_step = (end_rad - start_rad) / (steps - 1)

        trajectory = []
        plane = plane.upper()

        for i in range(steps):
            angle = start_rad + i * angle_step
            x_offset = radius * math.cos(angle)
            y_offset = radius * math.sin(angle)

            # Map to specified plane
            if plane == 'XY':
                point = {
                    "X": center.get("X", 0) + x_offset,
                    "Y": center.get("Y", 0) + y_offset,
                    "Z": center.get("Z", 0)
                }
            elif plane == 'XZ':
                point = {
                    "X": center.get("X", 0) + x_offset,
                    "Y": center.get("Y", 0),
                    "Z": center.get("Z", 0) + y_offset
                }
            elif plane == 'YZ':
                point = {
                    "X": center.get("X", 0),
                    "Y": center.get("Y", 0) + x_offset,
                    "Z": center.get("Z", 0) + y_offset
                }
            else:
                raise ValueError(f"Unknown plane: {plane}. Use 'XY', 'XZ', or 'YZ'")

            # Preserve orientation if present in center
            for key in ["A", "B", "C"]:
                if key in center:
                    point[key] = center[key]

            trajectory.append(point)

        return trajectory

    @staticmethod
    def generate_circle(
        center: Dict[str, float],
        radius: float,
        steps: int = 100,
        plane: str = 'XY'
    ) -> List[Dict[str, float]]:
        """
        Generate complete circle trajectory.

        Creates waypoints for a full 360-degree circular path.

        Args:
            center: Circle center point
            radius: Circle radius in mm
            steps: Number of waypoints around circle
            plane: Plane for circle - 'XY', 'XZ', or 'YZ'

        Returns:
            List of Cartesian waypoints around the circle

        Example:
            >>> # Full circle in XY plane
            >>> circle = api.motion.generate_circle(
            ...     center={"X": 100, "Y": 0, "Z": 500},
            ...     radius=50.0,
            ...     steps=100
            ... )
            >>> api.motion.execute_trajectory(circle, space='cartesian')
        """
        return MotionAPI.generate_arc(center, radius, 0, 360, steps, plane)

    @staticmethod
    def generate_spiral(
        center: Dict[str, float],
        start_radius: float,
        end_radius: float,
        pitch: float,
        revolutions: float = 1.0,
        steps: int = 100,
        plane: str = 'XY',
        axis: str = 'Z'
    ) -> List[Dict[str, float]]:
        """
        Generate spiral trajectory.

        Creates waypoints for a spiral path with changing radius and height.

        Args:
            center: Spiral center/start point
            start_radius: Starting radius in mm
            end_radius: Ending radius in mm
            pitch: Height change per revolution in mm
            revolutions: Number of complete rotations
            steps: Number of waypoints
            plane: Planar motion plane - 'XY', 'XZ', or 'YZ'
            axis: Axis for pitch motion - 'X', 'Y', or 'Z'

        Returns:
            List of Cartesian waypoints along spiral

        Example:
            >>> # Expanding spiral (drilling out)
            >>> spiral = api.motion.generate_spiral(
            ...     center={"X": 100, "Y": 0, "Z": 500},
            ...     start_radius=10.0,
            ...     end_radius=50.0,
            ...     pitch=5.0,  # 5mm per revolution
            ...     revolutions=5,
            ...     steps=200
            ... )

            >>> # Contracting spiral (retracting)
            >>> spiral = api.motion.generate_spiral(
            ...     center={"X": 100, "Y": 0, "Z": 500},
            ...     start_radius=50.0,
            ...     end_radius=10.0,
            ...     pitch=-5.0,  # Descending
            ...     revolutions=5
            ... )
        """
        if steps < 2:
            raise ValueError("Steps must be at least 2")

        total_angle = revolutions * 2 * math.pi
        angle_step = total_angle / (steps - 1)
        radius_step = (end_radius - start_radius) / (steps - 1)
        pitch_step = pitch * revolutions / (steps - 1)

        trajectory = []
        plane = plane.upper()
        axis = axis.upper()

        for i in range(steps):
            angle = i * angle_step
            radius = start_radius + i * radius_step
            pitch_offset = i * pitch_step

            x_offset = radius * math.cos(angle)
            y_offset = radius * math.sin(angle)

            # Map to specified plane
            point = {
                "X": center.get("X", 0),
                "Y": center.get("Y", 0),
                "Z": center.get("Z", 0)
            }

            if plane == 'XY':
                point["X"] += x_offset
                point["Y"] += y_offset
            elif plane == 'XZ':
                point["X"] += x_offset
                point["Z"] += y_offset
            elif plane == 'YZ':
                point["Y"] += x_offset
                point["Z"] += y_offset

            # Add pitch along specified axis
            point[axis] += pitch_offset

            # Preserve orientation
            for key in ["A", "B", "C"]:
                if key in center:
                    point[key] = center[key]

            trajectory.append(point)

        return trajectory

    @staticmethod
    def blend_trajectories(
        traj1: List[Dict[str, float]],
        traj2: List[Dict[str, float]],
        blend_radius: float,
        blend_steps: int = 20
    ) -> List[Dict[str, float]]:
        """
        Create smooth transition between two trajectories.

        Generates a blended zone between trajectory endpoints using cubic
        interpolation for smooth velocity transitions.

        Args:
            traj1: First trajectory
            traj2: Second trajectory
            blend_radius: Blend zone radius in mm (distance from junction point)
            blend_steps: Number of waypoints in blend zone

        Returns:
            Combined trajectory with smooth blend

        Example:
            >>> # Create two straight-line trajectories
            >>> traj1 = api.motion.generate_trajectory(p0, p1, 50)
            >>> traj2 = api.motion.generate_trajectory(p1, p2, 50)
            >>>
            >>> # Blend with 10mm radius
            >>> blended = api.motion.blend_trajectories(
            ...     traj1, traj2,
            ...     blend_radius=10.0,
            ...     blend_steps=20
            ... )
            >>> api.motion.execute_trajectory(blended)

        Note:
            Blend radius should be smaller than the length of either trajectory.
            Larger radii create smoother blends but deviate more from original path.
        """
        if not traj1 or not traj2:
            raise ValueError("Both trajectories must be non-empty")
        if blend_radius <= 0:
            raise ValueError("Blend radius must be positive")

        # Find blend start/end points based on radius
        blend_start_idx = _find_blend_point(traj1, blend_radius, from_end=True)
        blend_end_idx = _find_blend_point(traj2, blend_radius, from_end=False)

        # Extract segments
        before_blend = traj1[:blend_start_idx]
        after_blend = traj2[blend_end_idx:]

        # Generate blend zone using cubic interpolation
        p1 = traj1[blend_start_idx]
        p2 = traj2[blend_end_idx]
        blend_zone = _cubic_blend(p1, p2, blend_steps)

        # Combine all segments
        return before_blend + blend_zone + after_blend

    @staticmethod
    def transform_coordinates(
        pose: Dict[str, float],
        from_frame: str = 'BASE',
        to_frame: str = 'WORLD',
        frame_offset: Optional[Dict[str, float]] = None
    ) -> Dict[str, float]:
        """
        Transform pose between coordinate frames.

        Converts Cartesian coordinates between different reference frames
        (BASE, TOOL, WORLD, ROBROOT).

        Args:
            pose: Cartesian pose to transform
            from_frame: Source coordinate frame
            to_frame: Target coordinate frame
            frame_offset: Optional frame transformation (X, Y, Z, A, B, C)

        Returns:
            Transformed pose in target frame

        Example:
            >>> # Transform from BASE to WORLD frame
            >>> world_pose = api.motion.transform_coordinates(
            ...     pose={"X": 100, "Y": 0, "Z": 500},
            ...     from_frame='BASE',
            ...     to_frame='WORLD',
            ...     frame_offset={"X": 500, "Y": 200, "Z": 0}
            ... )
            >>> print(world_pose)
            {'X': 600, 'Y': 200, 'Z': 500}

        Note:
            For full 6-DOF transformations with rotations, use rotation
            matrices or quaternions. This implementation handles simple
            translational offsets and is suitable for most RSI applications.
        """
        if frame_offset is None:
            # Identity transformation if no offset specified
            return pose.copy()

        transformed = pose.copy()

        # Apply translational offset
        for axis in ['X', 'Y', 'Z']:
            if axis in pose and axis in frame_offset:
                transformed[axis] = pose[axis] + frame_offset[axis]

        # Apply rotational offset (simple addition for small angles)
        for axis in ['A', 'B', 'C']:
            if axis in pose and axis in frame_offset:
                transformed[axis] = pose[axis] + frame_offset[axis]

        return transformed


# Helper functions for velocity profiling

def _calculate_distance(p1: Dict[str, float], p2: Dict[str, float]) -> float:
    """Calculate Euclidean distance between two waypoints."""
    squared_diff = 0.0
    keys = set(p1.keys()) & set(p2.keys())
    for key in keys:
        if key in ['X', 'Y', 'Z', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
            squared_diff += (p2[key] - p1[key]) ** 2
    return math.sqrt(squared_diff)


def _trapezoidal_profile(
    distances: List[float],
    total_distance: float,
    max_velocity: float,
    max_acceleration: float
) -> List[float]:
    """
    Generate trapezoidal velocity profile.

    Accelerates at max_acceleration until reaching max_velocity,
    maintains constant velocity, then decelerates symmetrically.
    """
    n = len(distances) + 1
    velocities = [0.0] * n

    # Calculate acceleration/deceleration distances
    accel_distance = (max_velocity ** 2) / (2 * max_acceleration)

    if 2 * accel_distance >= total_distance:
        # Triangular profile (no constant velocity phase)
        peak_velocity = math.sqrt(max_acceleration * total_distance)
        distance_traveled = 0.0

        for i in range(n):
            if distance_traveled < total_distance / 2:
                # Acceleration phase
                velocities[i] = min(peak_velocity, math.sqrt(max(0, 2 * max_acceleration * distance_traveled)))
            else:
                # Deceleration phase
                remaining = total_distance - distance_traveled
                velocities[i] = min(peak_velocity, math.sqrt(max(0, 2 * max_acceleration * remaining)))

            if i < len(distances):
                distance_traveled += distances[i]
    else:
        # Full trapezoidal profile
        distance_traveled = 0.0

        for i in range(n):
            if distance_traveled < accel_distance:
                # Acceleration phase
                velocities[i] = math.sqrt(max(0, 2 * max_acceleration * distance_traveled))
            elif distance_traveled < (total_distance - accel_distance):
                # Constant velocity phase
                velocities[i] = max_velocity
            else:
                # Deceleration phase
                remaining = total_distance - distance_traveled
                velocities[i] = math.sqrt(max(0, 2 * max_acceleration * remaining))

            if i < len(distances):
                distance_traveled += distances[i]

    return velocities


def _s_curve_profile(
    distances: List[float],
    total_distance: float,
    max_velocity: float,
    max_acceleration: float
) -> List[float]:
    """
    Generate S-curve velocity profile (jerk-limited).

    Smooth acceleration/deceleration with limited jerk for
    reduced vibration and smoother motion.
    """
    n = len(distances) + 1
    velocities = [0.0] * n

    # Simplified S-curve: use sine function for smooth transitions
    distance_traveled = 0.0

    for i in range(n):
        # Normalized position [0, 1]
        s = distance_traveled / total_distance if total_distance > 0 else 0

        # S-curve using sine function
        # Smooth acceleration at start, smooth deceleration at end
        if s < 0.5:
            # First half: smooth acceleration
            v_normalized = 0.5 * (1 - math.cos(2 * math.pi * s))
        else:
            # Second half: smooth deceleration
            v_normalized = 0.5 * (1 - math.cos(2 * math.pi * (1 - s)))

        velocities[i] = v_normalized * max_velocity

        if i < len(distances):
            distance_traveled += distances[i]

    return velocities


def _find_blend_point(
    trajectory: List[Dict[str, float]],
    blend_radius: float,
    from_end: bool = False
) -> int:
    """Find trajectory index at specified distance from start or end."""
    if from_end:
        trajectory = list(reversed(trajectory))

    distance = 0.0
    for i in range(len(trajectory) - 1):
        distance += _calculate_distance(trajectory[i], trajectory[i + 1])
        if distance >= blend_radius:
            return (len(trajectory) - 1 - i) if from_end else i

    # Blend radius exceeds trajectory length
    return (len(trajectory) // 2) if from_end else (len(trajectory) // 2)


def _cubic_blend(
    p1: Dict[str, float],
    p2: Dict[str, float],
    steps: int
) -> List[Dict[str, float]]:
    """Generate cubic interpolation between two points."""
    blend = []
    keys = set(p1.keys()) | set(p2.keys())

    for i in range(steps):
        t = i / max(steps - 1, 1)
        # Cubic Hermite spline with zero velocity at endpoints
        h1 = 2 * t**3 - 3 * t**2 + 1
        h2 = -2 * t**3 + 3 * t**2

        point = {}
        for key in keys:
            v1 = p1.get(key, 0)
            v2 = p2.get(key, 0)
            point[key] = h1 * v1 + h2 * v2

        blend.append(point)

    return blend
