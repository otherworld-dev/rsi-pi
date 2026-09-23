# Tools

Low-level variable access, config and variable listings, reports.

## Usage

```python
# Low-level variable access
api.tools.update_variable("RKorr.X", 10.0)
api.tools.show_variables()     # Print all available variables
api.tools.show_config()        # Network settings + variable structure
api.tools.reset_variables()    # Zero out corrections

# Reports and comparison
api.tools.generate_report("logs/test.csv", "pdf")
diffs = api.tools.compare_runs("run1.csv", "run2.csv")
```

## Reference

::: RSIPI.tools_api.ToolsAPI
