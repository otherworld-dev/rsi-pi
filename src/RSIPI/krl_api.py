"""KRL program manipulation API namespace for RSIPI."""

import logging
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class KRLAPI:
    """
    KUKA Robot Language (KRL) program manipulation interface.

    Provides utilities for parsing KRL programs, extracting coordinate data,
    and injecting RSI control commands into existing KRL workflows.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize KRLAPI namespace.

        Args:
            client: RSIClient instance (currently unused, reserved for future features)
        """
        self.client = client

    @staticmethod
    def parse_to_csv(src_file: str, dat_file: str, output_file: str) -> str:
        """
        Parse KRL source and data files, extract coordinates to CSV.

        Reads .src (program logic) and .dat (position data) files, extracts
        point definitions and movement commands, and exports to CSV format.

        Args:
            src_file: Path to KRL .src program file
            dat_file: Path to KRL .dat data file
            output_file: Path for output CSV file

        Returns:
            Status message indicating success or failure

        Raises:
            FileNotFoundError: If source or data files don't exist
            Exception: If parsing fails

        Example:
            >>> api.krl.parse_to_csv('robot_prog.src', 'robot_prog.dat', 'output.csv')
            'KRL data successfully exported to output.csv'

        Note:
            The parser extracts position data (E6POS, E6AXIS, FRAME) from the
            .dat file and matches them with movement commands (PTP, LIN, CIRC)
            from the .src file.
        """
        try:
            from .krl_to_csv_parser import KRLParser

            parser = KRLParser(src_file, dat_file)
            parser.parse_src()
            parser.parse_dat()
            parser.export_csv(output_file)
            logging.info(f"KRL data exported to {output_file}")
            return f"KRL data successfully exported to {output_file}"
        except Exception as e:
            logging.error(f"KRL parsing failed: {e}")
            return f"Error parsing KRL files: {e}"

    @staticmethod
    def inject_rsi(input_krl: str, output_krl: Optional[str] = None, rsi_config: str = "RSIGatewayv1.rsi") -> str:
        """
        Inject RSI control commands into a KRL program.

        Automatically modifies a KRL .src file to include RSI initialization,
        sensor communication, and cleanup code. This allows adding real-time
        external control to existing robot programs.

        Args:
            input_krl: Path to input KRL .src file
            output_krl: Optional output path (defaults to overwriting input)
            rsi_config: RSI configuration file name (default: 'RSIGatewayv1.rsi')

        Returns:
            Status message indicating success or failure

        Raises:
            FileNotFoundError: If input KRL file doesn't exist
            Exception: If injection fails

        Example:
            >>> # Modify in place
            >>> api.krl.inject_rsi('robot_prog.src')
            'RSI successfully injected into robot_prog.src'

            >>> # Create new file
            >>> api.krl.inject_rsi('robot_prog.src', 'robot_prog_rsi.src')
            'RSI successfully injected into robot_prog_rsi.src'

        Note:
            The injection adds:
            - RSI_CREATE() at program start
            - RSI_ON() before motion commands
            - RSI_MOVECORR() during movement
            - RSI_OFF() after motion
            This allows Python to send corrections during program execution.
        """
        try:
            from .inject_rsi_to_krl import inject_rsi_to_krl

            inject_rsi_to_krl(input_krl, output_krl, rsi_config)
            output_path = output_krl if output_krl else input_krl
            logging.info(f"RSI injected into {output_path}")
            return f"RSI successfully injected into {output_path}"
        except Exception as e:
            logging.error(f"RSI injection failed: {e}")
            return f"RSI injection failed: {e}"

    # TODO (Phase 3): Implement KRL coordination helpers
    # def wait_for_signal(self, channel: int, timeout: float = 5.0) -> bool:
    #     """Wait for KRL to set a specific I/O signal."""
    #     pass
    #
    # def signal_complete(self, channel: int) -> None:
    #     """Signal to KRL that Python-side operation is complete."""
    #     pass
    #
    # def write_param(self, slot: str, value: float) -> str:
    #     """Write parameter to Tech.C variable for KRL to read."""
    #     pass
    #
    # def read_param(self, slot: str) -> float:
    #     """Read parameter from Tech.T variable written by KRL."""
    #     pass
