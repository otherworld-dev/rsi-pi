# Visualization

Static plots from logs, comparisons between runs, and live plotting.

## Usage

```python
# Static plots from CSV logs
api.viz.plot_static("logs/test.csv", "3d")
api.viz.plot_static("logs/test.csv", "position")    # Position vs time
api.viz.plot_static("logs/test.csv", "velocity")
api.viz.plot_static("logs/test.csv", "joints")
api.viz.plot_static("logs/test.csv", "force")
api.viz.plot_static("logs/test.csv", "2d_xy")       # 2D projections

# Deviation from planned path
api.viz.plot_static("logs/actual.csv", "deviation", overlay_path="logs/planned.csv")

# Comprehensive multi-plot visualization
api.viz.visualize_csv_log("logs/test.csv")
api.viz.visualize_csv_log("logs/test.csv", export=True)  # Save to disk

# Compare two runs
api.viz.compare_runs("run1.csv", "run2.csv")

# Live plotting (runs in background thread)
api.viz.start_live_plot("3d", interval=100)     # 100ms update
api.viz.change_live_plot_mode("position")
api.viz.stop_live_plot()
```

## Reference

::: RSIPI.viz_api.VizAPI
