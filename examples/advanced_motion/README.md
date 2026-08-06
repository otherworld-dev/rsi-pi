# Advanced Motion Control Examples

This directory contains Python examples demonstrating Phase 4 advanced motion control features including velocity profiling, geometric primitives, path blending, and coordinate transformations.

## Prerequisites

- RSIPI library installed (`pip install -e .` from rsi-pi directory)
- KUKA robot controller with RSI 3.3 configured
- RSI_EthernetConfig.xml configured for Cartesian corrections (RKorr)
- Basic understanding of robot motion programming

## Examples

### 01_velocity_profiles.py
**Optimize trajectory timing with velocity profiling**

Demonstrates trapezoidal and S-curve velocity profiles for time-optimal, smooth motion.

**Run**:
```bash
python 01_velocity_profiles.py --config path/to/RSI_EthernetConfig.xml
```

**Key Features**:
- Trapezoidal profile: Fast point-to-point motion with constant acceleration
- S-curve profile: Jerk-limited smooth motion for reduced mechanical stress
- Velocity comparison at different trajectory points
- Production-ready motion timing

**API Methods**:
- `api.motion.generate_velocity_profile(trajectory, max_velocity, max_acceleration, profile)`

**Use Cases**:
- High-speed pick and place (trapezoidal)
- Delicate assembly operations (s-curve)
- Painting/coating with constant velocity
- Time-optimized production cycles

---

### 02_geometric_primitives.py
**Generate complex motion patterns**

Demonstrates arc, circle, and spiral trajectory generation for drilling, milling, and inspection applications.

**Run**:
```bash
python 02_geometric_primitives.py --config path/to/RSI_EthernetConfig.xml
```

**Key Features**:
- Circular arcs (partial circles)
- Full 360° circles
- Expanding spirals (drilling pattern)
- Contracting spirals (retraction pattern)
- Multiple plane support (XY, XZ, YZ)

**API Methods**:
- `api.motion.generate_arc(center, radius, start_angle, end_angle, steps, plane)`
- `api.motion.generate_circle(center, radius, steps, plane)`
- `api.motion.generate_spiral(center, start_radius, end_radius, pitch, revolutions, steps, plane, axis)`

**Use Cases**:
- Drilling/milling: Expanding spirals for hole boring
- Assembly: Circular insertion paths with clearance
- Inspection: Scanning circular features
- Welding: Curved seam following

---

### 03_path_blending.py
**Smooth trajectory transitions**

Demonstrates cubic interpolation blending to eliminate stop-and-go motion at trajectory boundaries.

**Run**:
```bash
python 03_path_blending.py --config path/to/RSI_EthernetConfig.xml
```

**Key Features**:
- Sharp corners vs blended corners comparison
- Multiple blend zones in continuous paths
- Configurable blend radius
- Orientation blending for smooth rotation transitions

**API Methods**:
- `api.motion.blend_trajectories(traj1, traj2, blend_radius, blend_steps)`

**Use Cases**:
- Welding/gluing: Continuous bead without stop marks
- Pick and place: Reduced cycle time by eliminating stops
- Machining: Smooth tool paths without witness marks
- Painting: Even coating thickness at corners

---

### 04_coordinate_transforms.py
**Transform between coordinate frames**

Demonstrates position and trajectory transformations between BASE, TOOL, WORLD, and WORK coordinate systems.

**Run**:
```bash
python 04_coordinate_transforms.py --config path/to/RSI_EthernetConfig.xml
```

**Key Features**:
- BASE to WORLD transformations
- TOOL frame offsets (TCP calibration)
- Work object (pallet) transformations
- Sensor-guided motion with frame corrections
- Transforming entire trajectories

**API Methods**:
- `api.motion.transform_coordinates(pose, from_frame, to_frame, frame_offset)`

**Use Cases**:
- Multiple work objects: Define once, execute anywhere
- Tool changes: Adapt taught positions for different tools
- Vision integration: Apply sensor corrections to taught paths
- Multi-robot cells: Coordinate motion in shared workspace

---

### 05_combined_motion.py
**Complete production application**

Demonstrates combining all Phase 4 features in a realistic automated drilling and inspection scenario.

**Run**:
```bash
python 05_combined_motion.py --config path/to/RSI_EthernetConfig.xml
```

**Application Flow**:
1. Navigate to inspection position (blended path, S-curve, 300mm/s)
2. Execute spiral inspection pattern (S-curve, 50mm/s)
3. Navigate to drilling position (blended path, trapezoidal, 250mm/s)
4. Execute expanding spiral drilling (S-curve, 30mm/s, descending)
5. Return to home (blended path, trapezoidal, 350mm/s)

**Features Demonstrated**:
- ✅ Coordinate transformations (work object & tool offsets)
- ✅ Path blending (smooth navigation)
- ✅ Velocity profiling (optimized speeds per operation)
- ✅ Geometric primitives (spiral patterns)

**Production Benefits**:
- Optimized cycle time with velocity profiling
- Smooth motion reduces mechanical stress
- Coordinate transforms enable flexible part placement
- Geometric primitives simplify complex patterns

---

## Configuration Requirements

### RSI XML Configuration

Your `RSI_EthernetConfig.xml` must support Cartesian corrections:

**Cartesian Corrections (RKorr):**
```xml
<RECEIVE>
  <XML>
    <ELEMENT Tag=\"RKorr\" Type=\"DOUBLE\" Indizes=\"[1..6]\"/>
  </XML>
</RECEIVE>
```

### Network Settings

Ensure your RSI network settings match:
```xml
<IP_NUMBER>192.168.1.100</IP_NUMBER>  <!-- Your PC IP -->
<PORT>49152</PORT>
<SENTYPE>ImFree</SENTYPE>
```

## Running Examples

### Step 1: Verify RSI Configuration

```bash
# Check that your RSI_EthernetConfig.xml has required elements
grep -A 5 "RKorr" RSI_EthernetConfig.xml
```

### Step 2: Run Example

```bash
cd examples/advanced_motion
python 01_velocity_profiles.py --config ../../RSI_EthernetConfig.xml
```

### Step 3: Monitor Output

Examples log comprehensive information:
- Trajectory generation details
- Velocity profile characteristics
- Motion execution progress
- Application use cases

**Example Output**:
```
2026-01-17 14:32:01 - INFO - Starting RSI communication...
2026-01-17 14:32:01 - INFO - ✅ RSI started successfully
2026-01-17 14:32:01 - INFO - Generating trajectory with 100 waypoints...
2026-01-17 14:32:01 - INFO - Applying trapezoidal velocity profile
2026-01-17 14:32:01 - INFO - Max velocity: 200.0 mm/s
2026-01-17 14:32:01 - INFO - Executing profiled trajectory...
2026-01-17 14:32:05 - INFO - ✅ Motion complete
```

## API Reference

### Velocity Profiling

```python
# Generate trajectory with timing information
profiled_trajectory = api.motion.generate_velocity_profile(
    trajectory=waypoints,
    max_velocity=200.0,      # mm/s
    max_acceleration=500.0,  # mm/s²
    profile='trapezoidal'    # or 's-curve'
)

# Execute with precise timing
api.motion.execute_profiled_trajectory(profiled_trajectory, space="cartesian")
```

**Profiles**:
- `'trapezoidal'`: Bang-bang acceleration, constant velocity cruise, fast point-to-point
- `'s-curve'`: Jerk-limited smooth acceleration, reduced vibration and stress

### Geometric Primitives

```python
# Circular arc
arc = api.motion.generate_arc(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=50.0,
    start_angle=0,    # degrees
    end_angle=90,     # degrees
    steps=50,
    plane='XY'        # or 'XZ', 'YZ'
)

# Full circle
circle = api.motion.generate_circle(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=30.0,
    steps=100,
    plane='XY'
)

# Spiral (expanding or contracting)
spiral = api.motion.generate_spiral(
    center={"X": 100, "Y": 0, "Z": 500},
    start_radius=5.0,
    end_radius=40.0,
    pitch=10.0,        # mm per revolution (positive=descending)
    revolutions=3.0,
    steps=150,
    plane='XY',
    axis='Z'
)
```

### Path Blending

```python
# Generate two trajectories
traj1 = api.motion.generate_trajectory(p0, p1, steps=50)
traj2 = api.motion.generate_trajectory(p1, p2, steps=50)

# Blend for smooth transition
blended = api.motion.blend_trajectories(
    traj1=traj1,
    traj2=traj2,
    blend_radius=20.0,  # Start blending 20mm before corner
    blend_steps=20      # Number of interpolation points
)

# Execute smooth continuous motion
api.motion.execute_trajectory(blended, space='cartesian', rate=0.02)
```

### Coordinate Transformations

```python
# Define frame offset
work_offset = {
    "X": 500.0,
    "Y": -200.0,
    "Z": 50.0,
    "A": 0.0,
    "B": 0.0,
    "C": 15.0
}

# Transform single pose
pose_work = {"X": 100, "Y": 50, "Z": 30}
pose_base = api.motion.transform_coordinates(
    pose=pose_work,
    from_frame='WORK',
    to_frame='BASE',
    frame_offset=work_offset
)

# Transform entire trajectory
trajectory_base = []
for waypoint in trajectory_work:
    transformed = api.motion.transform_coordinates(
        waypoint, 'WORK', 'BASE', work_offset
    )
    trajectory_base.append(transformed)
```

**Supported Frames** (documentation labels only - see note below):
- `'BASE'`: Robot base coordinate system
- `'WORLD'`: Global world coordinates
- `'TOOL'`: Tool center point (TCP) coordinates
- `'WORK'`: Work object (pallet/fixture) coordinates
- `'ROBROOT'`: Robot root system

> **Note**: `transform_coordinates()` never reads `from_frame`/`to_frame` -
> they are documentation labels only, not validated, and no frame-specific
> math is applied for any pair. `frame_offset` is added directly onto the
> pose (`pose[axis] + frame_offset[axis]` for X/Y/Z and A/B/C); it never
> rotates X/Y by A/B/C. If a frame is genuinely rotated relative to its
> parent, rotate X/Y yourself before calling this function.

## Customizing Examples

### Adjust Velocity Limits

```python
# Faster motion (use with caution)
profiled = api.motion.generate_velocity_profile(
    trajectory,
    max_velocity=500.0,      # Increase speed
    max_acceleration=1200.0, # Increase acceleration
    profile='trapezoidal'
)

# Slower, more precise motion
profiled = api.motion.generate_velocity_profile(
    trajectory,
    max_velocity=50.0,       # Reduce speed
    max_acceleration=150.0,  # Gentle acceleration
    profile='s-curve'
)
```

### Modify Geometric Patterns

```python
# Larger spiral for bigger holes
spiral = api.motion.generate_spiral(
    center={"X": 100, "Y": 0, "Z": 500},
    start_radius=10.0,    # Larger start
    end_radius=80.0,      # Larger end
    pitch=15.0,           # Deeper per revolution
    revolutions=5.0,      # More turns
    steps=250,            # More waypoints for smoothness
    plane='XY',
    axis='Z'
)

# Vertical circle (XZ plane)
circle_vertical = api.motion.generate_circle(
    center={"X": 100, "Y": 0, "Z": 500},
    radius=40.0,
    steps=100,
    plane='XZ'  # Vertical circle
)
```

### Adjust Blend Radius

```python
# Tighter blend (closer to sharp corner)
blended_tight = api.motion.blend_trajectories(
    traj1, traj2,
    blend_radius=5.0,   # Small radius
    blend_steps=10
)

# Wide blend (very smooth, cuts corner more)
blended_wide = api.motion.blend_trajectories(
    traj1, traj2,
    blend_radius=50.0,  # Large radius
    blend_steps=30
)
```

## Troubleshooting

### "RSI Communication Error"

**Problem**: Cannot establish RSI connection

**Solutions**:
1. Verify robot controller is powered on and in correct mode
2. Check network connectivity (ping robot IP)
3. Verify RSI XML configuration file path
4. Ensure RSI is enabled on robot controller
5. Check firewall allows UDP port 49152

### Motion Limits Exceeded

**Problem**: Trajectory exceeds robot workspace or velocity limits

**Solutions**:
1. Reduce `max_velocity` parameter in velocity profiling
2. Reduce `max_acceleration` parameter
3. Check trajectory waypoints are within robot workspace
4. Verify coordinate transformations are correct
5. Add safety margin to trajectory boundaries

### Jerky Motion Despite S-Curve Profile

**Problem**: Motion not smooth even with S-curve velocity profile

**Solutions**:
1. Increase number of waypoints (`steps` parameter)
2. Reduce `max_acceleration` for gentler motion
3. Check RSI communication rate (should be ~250Hz)
4. Verify no network latency issues
5. Ensure trajectory waypoints are well-distributed

### Blend Zone Too Large

**Problem**: Blending cuts corners too much or exceeds trajectory bounds

**Solutions**:
1. Reduce `blend_radius` parameter
2. Increase trajectory length before attempting blend
3. Check that blend_radius < half of shortest trajectory segment
4. Use smaller `blend_steps` for tighter control
5. Consider using multiple smaller blend zones

### Coordinate Transform Incorrect

**Problem**: Transformed positions don't match expected values

**Solutions**:
1. Verify `frame_offset` values are correct
2. Check from_frame and to_frame parameters are correct
3. Ensure frame offset includes both position and orientation
4. Test with simple known transforms first
5. Visualize transformed trajectory before execution

### Velocity Profile Not Applied

**Problem**: Robot doesn't follow calculated velocity profile

**Solutions**:
1. Verify you're passing the profiled trajectory to `execute_profiled_trajectory()`, not iterating and sending waypoints yourself
2. Check that `space` matches the trajectory type (`"cartesian"` or `"joint"`)
3. Ensure system has sufficient timing resolution
4. Monitor actual execution timing with logs
5. Consider system overhead in timing calculations

## Advanced Usage

### Combining Multiple Features

```python
# 1. Generate geometric primitive
spiral = api.motion.generate_spiral(...)

# 2. Transform to correct frame
spiral_base = [
    api.motion.transform_coordinates(wp, 'WORK', 'BASE', offset)
    for wp in spiral
]

# 3. Apply velocity profile
spiral_profiled = api.motion.generate_velocity_profile(
    spiral_base,
    max_velocity=100.0,
    max_acceleration=300.0,
    profile='s-curve'
)

# 4. Execute with precise timing
api.motion.execute_profiled_trajectory(spiral_profiled, space="cartesian")
```

### Dynamic Trajectory Modification

```python
# Start with base trajectory (world/absolute poses)
trajectory = api.motion.generate_circle(...)

# Apply a sensor correction as a small per-cycle DELTA, not the absolute
# corrected pose. update_cartesian() writes RKorr directly, and in relative
# mode (the default) that value is re-applied every 4ms cycle it is held -
# passing an absolute/world pose would command a jump on the order of the
# waypoint's world coordinates every cycle it stays latched.
sensor_offset = get_sensor_reading()  # small delta, e.g. {"X": 0.3, "Y": -0.1}
api.motion.update_cartesian(**sensor_offset)
# ... after one robot cycle, zero it so the nudge is not re-applied forever
api.motion.update_cartesian(**{axis: 0.0 for axis in sensor_offset})
```

**Do not** do this - `transform_coordinates()` only adds, so feeding its
result straight into `update_cartesian()` sends the *absolute* corrected
pose as a correction, not the small delta you intended:

```python
# WRONG - corrected is waypoint + sensor_offset (an absolute/world pose,
# e.g. hundreds of mm), applied every cycle it is held.
corrected = api.motion.transform_coordinates(
    waypoint, 'BASE', 'BASE', frame_offset=sensor_offset
)
api.motion.update_cartesian(**corrected)
```

### Multi-Layer Patterns

```python
# Generate pattern at multiple Z heights
layers = []
for z in range(0, 50, 10):  # Every 10mm
    circle = api.motion.generate_circle(
        center={"X": 100, "Y": 0, "Z": z},
        radius=30.0,
        steps=50,
        plane='XY'
    )
    layers.extend(circle)

# Blend between layers
for i in range(len(layers) - 1):
    segment = [layers[i], layers[i+1]]
    # Execute segment...
```

## Performance Optimization

### Trajectory Generation

- Use appropriate `steps` parameter: More steps = smoother but slower generation
- Generate complex patterns once, reuse multiple times
- Consider pre-generating common patterns at startup

### Velocity Profiling

- Trapezoidal profile is faster to compute than S-curve
- Use S-curve only where smoothness is critical
- Cache profiled trajectories for repeated operations

### Coordinate Transformations

- Transform entire trajectory once, not waypoint-by-waypoint in real-time
- Pre-calculate frame offsets before motion execution
- Use simple translational offsets when possible (faster than full 6-DOF)

### Path Blending

- Larger `blend_steps` = smoother but slower generation
- Balance blend_radius vs trajectory accuracy requirements
- Consider blending offline for repeated paths

## Next Steps

1. **Run basic examples** (01, 02) to understand individual features
2. **Experiment with parameters** to see their effects
3. **Combine features** using example 05 as a template
4. **Adapt to your application** specific requirements
5. **Integrate with sensors** for adaptive motion control

## References

- [RSIPI API Documentation](../../README.md)
- [Basic Motion Examples](../basic_motion/)
- [Coordination Examples](../coordination/)
- [KUKA RSI 3.3 Manual](https://www.kuka.com)

---

**Last Updated**: January 17, 2026
**RSIPI Internal API Version**: 2.0.0 (`RSIPI.__version__` in `src/RSIPI/__init__.py`; the
installable package version in `pyproject.toml`/`setup.py` is tracked separately)
**Phase**: 4 (Advanced Motion Control)
