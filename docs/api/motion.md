# Motion

Cartesian and joint corrections, trajectories, queues, geometric primitives, velocity profiles, blending and frame transforms.

## Basics

```python
# Cartesian corrections (RKorr) -- mm for XYZ, degrees for ABC
api.motion.update_cartesian(X=10.0, Y=-5.0, Z=0.0)
api.motion.update_cartesian(A=2.5, B=0.0, C=0.0)

# Joint corrections (AKorr) -- degrees
api.motion.update_joints(A1=5.0, A2=-3.0)

# Read current state
pose = api.motion.get_current_pose()      # {X, Y, Z, A, B, C}
joints = api.motion.get_current_joints()   # {A1, A2, A3, A4, A5, A6}

# External axes (value is a per-cycle delta under rsi_mode="relative" -- see Core Lifecycle)
api.motion.move_external_axis("E1", 2.5)

# Tech parameters (runtime motion adjustment)
api.motion.adjust_speed("Tech.T21", 0.5)
```

## Trajectories

```python
# Generate linear trajectory
traj = api.motion.generate_trajectory(
    start={"X": 0, "Y": 0, "Z": 500},
    end={"X": 100, "Y": 0, "Z": 500},
    steps=50,
    space="cartesian"
)

# Execute (blocking). cycles_per_step paces one waypoint per N robot cycles
# (3 * 4ms default cycle_time = one waypoint every 12ms here); in relative
# mode each waypoint's delta is spread evenly over those N cycles, so a slow
# move is smooth rather than a burst-and-pause. `rate=` (seconds/waypoint)
# still works but is deprecated in favor of cycles_per_step.
api.motion.execute_trajectory(traj, space="cartesian", cycles_per_step=3)

# Or generate + execute in one call (end_pose first, start_pose defaults to current position)
api.motion.move_cartesian_trajectory(
    end_pose={"X": 100, "Y": 0, "Z": 500},
    start_pose={"X": 0, "Y": 0, "Z": 500},
    steps=50, cycles_per_step=5
)
api.motion.move_joint_trajectory(
    end_joints={"A1": 30, "A2": -15, "A3": 45, "A4": 0, "A5": 30, "A6": 0},
    start_joints={"A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0, "A6": 0},
    steps=100, cycles_per_step=100
)

# Cancel a running trajectory from another thread
api.motion.cancel_trajectory()
```

## Trajectory Queue

```python
# Each leg is paced in robot cycles per waypoint, like execute_trajectory
# (50 steps x 10 cycles x 4 ms = 2 s). rate= still works but is deprecated.
api.motion.queue_cartesian_trajectory(p0, p1, steps=50, cycles_per_step=10)
api.motion.queue_cartesian_trajectory(p1, p2, steps=50, cycles_per_step=10)
api.motion.queue_joint_trajectory(j0, j1, steps=30, cycles_per_step=25)

print(api.motion.get_queue())       # [{space, steps, cycles_per_step, rate}, ...]
api.motion.execute_queued_trajectories()  # Run all in sequence, then clear
api.motion.cancel_trajectory()      # From another thread: stops the current leg
api.motion.clear_queue()            # Discard without executing
```

## Geometric Primitives

```python
# Circular arc
arc = api.motion.generate_arc(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=50.0,
    start_angle=0, end_angle=90,
    steps=50, plane="XY"
)

# Full circle
circle = api.motion.generate_circle(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=50.0, steps=100, plane="XY"
)

# Spiral
spiral = api.motion.generate_spiral(
    center={"X": 100, "Y": 0, "Z": 500},
    start_radius=10.0, end_radius=50.0,
    pitch=5.0, revolutions=5,
    steps=200, plane="XY", axis="Z"
)

api.motion.execute_trajectory(arc, space="cartesian")
```

## Velocity Profiles

```python
traj = api.motion.generate_trajectory(p0, p1, steps=100)

# Trapezoidal (bang-bang acceleration)
profiled = api.motion.generate_velocity_profile(
    traj, max_velocity=200.0, max_acceleration=500.0,
    profile="trapezoidal"
)

# S-curve (jerk-limited, smoother)
profiled = api.motion.generate_velocity_profile(
    traj, max_velocity=200.0, max_acceleration=500.0,
    profile="s-curve"
)

# Each element is (waypoint_dict, velocity_float)
for waypoint, velocity in profiled:
    print(f"Velocity: {velocity:.2f} mm/s")
```

## Path Blending

```python
traj1 = api.motion.generate_trajectory(p0, p1, 50)
traj2 = api.motion.generate_trajectory(p1, p2, 50)

blended = api.motion.blend_trajectories(
    traj1, traj2,
    blend_radius=10.0,   # mm from junction
    blend_steps=20
)
api.motion.execute_trajectory(blended)
```

## Coordinate Transforms

```python
world_pose = api.motion.transform_coordinates(
    pose={"X": 100, "Y": 0, "Z": 500},
    from_frame="BASE", to_frame="WORLD",
    frame_offset={"X": 500, "Y": 200, "Z": 0}
)
```

## Reference

::: RSIPI.motion_api.MotionAPI
