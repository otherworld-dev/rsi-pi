import logging
import time
from .safety_manager import SafetyManager


def generate_trajectory(start, end, steps=100, space="cartesian", mode="absolute", include_resets=False):
    """
    Generates a trajectory from start to end across N steps.

    - Absolute mode (default): waypoints are full interpolated poses.
    - Relative mode: waypoints are per-step deltas (end-start)/steps,
      optionally followed by zero-resets after each step.

    Safety is always checked against the CUMULATIVE interpolated pose, not
    the per-step delta — a delta like Z=-1.0 is not a workspace position and
    must not be rejected by absolute workspace bounds.
    """
    if mode not in ["relative", "absolute"]:
        raise ValueError("mode must be 'relative' or 'absolute'")
    if space not in ["cartesian", "joint"]:
        raise ValueError("space must be 'cartesian' or 'joint'")
    if steps <= 0:
        raise ValueError("steps must be greater than zero")

    if mode == "absolute":
        include_resets = False  # Resets only make sense for relative deltas

    axes = list(start.keys())
    trajectory = []

    safety_fn = SafetyManager.check_cartesian_limits if space == "cartesian" else SafetyManager.check_joint_limits

    for i in range(1, steps + 1):
        point = {}
        cumulative = {}
        for axis in axes:
            delta = end[axis] - start[axis]
            value = start[axis] + (delta * i / steps)
            cumulative[axis] = value
            point[axis] = delta / steps if mode == "relative" else value

        if not safety_fn(cumulative):
            raise ValueError(f"⚠️ Safety check failed at step {i}: {cumulative}")

        trajectory.append(point)

        if mode == "relative" and include_resets:
            # Insert a zero-correction step to prevent drift
            trajectory.append({axis: 0.0 for axis in axes})

    return trajectory


def execute_trajectory(api, trajectory, space="cartesian", rate=0.004):
    """
    Deprecated: use MotionAPI.execute_trajectory instead.

    When the passed object is a MotionAPI (has execute_trajectory), this
    delegates to it, which paces waypoints against the robot's IPOC clock.
    Note: delta trajectories (generate_trajectory mode='relative') should be
    executed via api.execute_trajectory(traj, points='delta') directly.

    The legacy sleep-based fallback only remains for foreign duck-typed
    objects that expose update_cartesian/update_joints but no executor.
    """
    if hasattr(api, "execute_trajectory"):
        logging.warning(
            "trajectory_planner.execute_trajectory is deprecated; "
            "use api.motion.execute_trajectory instead"
        )
        api.execute_trajectory(trajectory, space=space, rate=rate)
        return

    for point in trajectory:
        if space == "cartesian":
            api.update_cartesian(**point)
        elif space == "joint":
            api.update_joints(**point)
        else:
            raise ValueError("space must be 'cartesian' or 'joint'")
        time.sleep(rate)
