"""Utility tools API namespace for RSIPI."""

import logging
import os
import json
from typing import Dict, Any, Union, TYPE_CHECKING
import pandas as pd
import matplotlib.pyplot as plt

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class ToolsAPI:
    """
    Utility tools interface for KUKA RSI robot control.

    Provides debugging, inspection, data comparison, and reporting utilities
    for analyzing RSI performance and robot behavior.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize ToolsAPI namespace.

        Args:
            client: RSIClient instance for variable access
        """
        self.client = client

    def update_variable(self, name: str, value: Union[float, int]) -> str:
        """
        Low-level variable update with safety validation.

        Direct access to send_variables for advanced users. Most users should
        use higher-level methods like api.motion.update_cartesian() instead.

        Args:
            name: Variable name (e.g., 'IPOC', 'RKorr.X', 'Tech.C11')
            value: New value to set

        Returns:
            Status message

        Raises:
            RSIVariableError: If variable not found
            RSISafetyViolation: If value violates safety limits

        Example:
            >>> api.tools.update_variable('RKorr.X', 10.0)
            'Updated RKorr.X to 10.0'
            >>> api.tools.update_variable('Tech.C11', 42)
            'Updated Tech.C11 to 42.0'

        Note:
            This bypasses higher-level abstractions and directly modifies
            send_variables. Use with caution and prefer namespace-specific
            methods when available.
        """
        from .exceptions import RSIVariableError

        if "." in name:
            parent, child = name.split(".", 1)
            full_path = f"{parent}.{child}"

            if parent in self.client.send_variables:
                current = dict(self.client.send_variables[parent])
                # Validate using SafetyManager
                safe_value = self.client.safety_manager.validate(full_path, float(value))
                current[child] = safe_value
                self.client.send_variables[parent] = current
                logging.debug(f"Updated {name} to {safe_value}")
                return f"Updated {name} to {safe_value}"
            else:
                raise RSIVariableError(f"Parent variable '{parent}' not found in send_variables")
        else:
            # Top-level variable
            safe_value = self.client.safety_manager.validate(name, float(value))
            self.client.send_variables[name] = safe_value
            logging.debug(f"Updated {name} to {safe_value}")
            return f"Updated {name} to {safe_value}"

    def show_variables(self) -> None:
        """
        Print available send/receive variables to console.

        Displays a formatted list of all configured RSI variables with their
        nested structure. Useful for debugging and discovering available data.

        Example:
            >>> api.tools.show_variables()
            Send Variables:
              - IPOC
              - RKorr: X, Y, Z, A, B, C
              - AKorr: A1, A2, A3, A4, A5, A6
              - Tech: C11, C12, C13, ... T11, T12, ...

            Receive Variables:
              - IPOC
              - RIst: X, Y, Z, A, B, C
              - RSol: X, Y, Z, A, B, C
              - ASPos: A1, A2, A3, A4, A5, A6
              - MaCur: A1, A2, A3, A4, A5, A6
        """
        def format_grouped(var_dict):
            output = []
            for var, val in var_dict.items():
                if isinstance(val, dict):
                    sub_vars = ', '.join(val.keys())
                    output.append(f"{var}: {sub_vars}")
                else:
                    output.append(var)
            return output

        send_vars = format_grouped(self.client.send_variables)
        receive_vars = format_grouped(self.client.receive_variables)

        print("\nSend Variables:")
        for item in send_vars:
            print(f"  - {item}")

        print("\nReceive Variables:")
        for item in receive_vars:
            print(f"  - {item}")
        print()

    def show_config(self) -> Dict[str, Any]:
        """
        Retrieve configuration information.

        Returns network settings and current variable structure from the
        active RSI configuration.

        Returns:
            Dictionary containing:
                - Network: IP, port, sentype, onlysend settings
                - Send variables: Current send variable structure
                - Receive variables: Current receive variable structure

        Example:
            >>> config = api.tools.show_config()
            >>> print(config['Network'])
            {'ip': '192.168.1.100', 'port': 49152, 'sentype': 'ImFree', 'onlysend': False}
            >>> print(config['Send variables'].keys())
            dict_keys(['IPOC', 'RKorr', 'AKorr', 'Tech'])
        """
        return {
            "Network": self.client.config_parser.get_network_settings(),
            "Send variables": dict(self.client.send_variables),
            "Receive variables": dict(self.client.receive_variables)
        }

    def reset_variables(self) -> str:
        """
        Reset send variables to default values.

        Calls the client's reset_send_variables() method if available,
        otherwise returns a not-implemented message.

        Returns:
            Status message

        Example:
            >>> api.tools.reset_variables()
            'Send variables reset to default values'

        Note:
            This typically resets correction values (RKorr, AKorr) to zero
            and restores default Tech variable values. IPOC is not affected.
        """
        if hasattr(self.client, 'reset_send_variables'):
            self.client.reset_send_variables()
            logging.info("Send variables reset to defaults")
            return "Send variables reset to default values"
        else:
            return "reset_send_variables() not implemented on client"

    @staticmethod
    def generate_report(filename: str, format_type: str = "csv") -> str:
        """
        Generate statistical report from CSV log file.

        Analyzes recorded RSI data and produces summary statistics for
        position, velocity, and other metrics.

        Args:
            filename: Path to CSV log file (with or without .csv extension)
            format_type: Output format - 'csv', 'json', or 'pdf'

        Returns:
            Path to generated report file

        Raises:
            FileNotFoundError: If CSV file doesn't exist
            ValueError: If format_type is unsupported or CSV has no position data

        Example:
            >>> api.tools.generate_report('logs/test_run.csv', 'pdf')
            'Report saved as logs/test_run_report.pdf'

        Note:
            PDF reports include bar charts of max/mean position values.
            CSV and JSON formats provide tabular statistical data.
        """
        # Ensure filename ends with .csv
        if not filename.endswith(".csv"):
            filename += ".csv"

        if not os.path.exists(filename):
            raise FileNotFoundError(f"File not found: {filename}")

        df = pd.read_csv(filename)

        # Extract position columns
        position_cols = [col for col in df.columns if col.startswith("Receive.RIst.")]
        if not position_cols:
            raise ValueError("No 'Receive.RIst' position columns found in CSV")

        report_data = {
            "Max Position": df[position_cols].max().to_dict(),
            "Mean Position": df[position_cols].mean().to_dict(),
        }

        report_base = filename.replace(".csv", "")
        output_path = f"{report_base}_report.{format_type.lower()}"

        if format_type == "csv":
            pd.DataFrame(report_data).T.to_csv(output_path)
        elif format_type == "json":
            with open(output_path, "w") as f:
                json.dump(report_data, f, indent=4)
        elif format_type == "pdf":
            fig, ax = plt.subplots()
            pd.DataFrame(report_data).T.plot(kind='bar', ax=ax)
            ax.set_title("RSI Position Report")
            plt.tight_layout()
            plt.savefig(output_path)
            plt.close(fig)
        else:
            raise ValueError(f"Unsupported format: {format_type}. Use 'csv', 'json', or 'pdf'.")

        logging.info(f"Report generated: {output_path}")
        return f"Report saved as {output_path}"

    @staticmethod
    def compare_runs(file1: str, file2: str) -> Dict[str, Dict[str, float]]:
        """
        Compare two test run CSV files.

        Calculates mean and max deviation between corresponding position
        columns in two log files. Useful for repeatability analysis.

        Args:
            file1: Path to first CSV log file
            file2: Path to second CSV log file

        Returns:
            Dictionary mapping column names to deviation statistics:
                - mean_diff: Average absolute difference
                - max_diff: Maximum absolute difference

        Raises:
            FileNotFoundError: If either file doesn't exist

        Example:
            >>> diffs = api.tools.compare_runs('run1.csv', 'run2.csv')
            >>> for col, stats in diffs.items():
            ...     print(f"{col}: mean={stats['mean_diff']:.3f}, max={stats['max_diff']:.3f}")
            Receive.RIst.X: mean=0.234, max=1.456
            Receive.RIst.Y: mean=0.178, max=0.892
            Receive.RIst.Z: mean=0.312, max=1.023

        Note:
            Only compares columns present in both files. Typically used for
            comparing repeatability of the same motion program.
        """
        df1 = pd.read_csv(file1)
        df2 = pd.read_csv(file2)

        shared_cols = [col for col in df1.columns if col in df2.columns and col.startswith("Receive.RIst")]
        diffs = {}

        for col in shared_cols:
            delta = abs(df1[col] - df2[col])
            diffs[col] = {
                "mean_diff": float(delta.mean()),
                "max_diff": float(delta.max()),
            }

        logging.info(f"Compared {len(shared_cols)} position columns between runs")
        return diffs
