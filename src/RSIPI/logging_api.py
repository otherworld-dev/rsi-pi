"""CSV logging API namespace for RSIPI."""

import logging
import datetime
import os
from typing import Optional, TYPE_CHECKING
import pandas as pd

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class LoggingAPI:
    """
    CSV data logging interface for KUKA RSI robot control.

    Manages high-frequency data logging to CSV files with British date format
    timestamps. Logging runs in a separate process to avoid timing interference
    with the real-time control loop.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize LoggingAPI namespace.

        Args:
            client: RSIClient instance for logging control
        """
        self.client = client

    def start(self, filename: Optional[str] = None) -> str:
        """
        Start CSV logging to file.

        Creates a background logging process that writes send/receive variables
        to CSV with British date format timestamps (DD/MM/YYYY HH:MM:SS.mmm).

        Args:
            filename: Optional output file path. Auto-generated if not provided
                     with format: logs/DD-MM-YYYY_HH-MM-SS.csv

        Returns:
            Path to the log file being written

        Example:
            >>> # Auto-generated filename
            >>> path = api.logging.start()
            >>> print(path)
            logs/16-01-2026_14-32-45.csv

            >>> # Custom filename
            >>> path = api.logging.start('my_experiment.csv')
            >>> print(path)
            my_experiment.csv

        Note:
            Logging runs in a separate process and uses a queue-based buffering
            system to prevent blocking the real-time control loop. If the queue
            fills, old entries are dropped rather than blocking.
        """
        if not filename:
            timestamp = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
            filename = f"logs/{timestamp}.csv"

        # Ensure logs directory exists
        log_dir = os.path.dirname(filename)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)
            logging.info(f"Created logging directory: {log_dir}")

        self.client.start_logging(filename)
        logging.info(f"CSV logging started: {filename}")
        return filename

    def stop(self) -> str:
        """
        Stop CSV logging.

        Signals the logging process to flush remaining data and close the file.
        The logging process will terminate gracefully.

        Returns:
            Status message

        Example:
            >>> api.logging.stop()
            'CSV logging stopped'

        Note:
            There may be a brief delay (up to 2 seconds) while the logging
            process completes writing buffered data and shuts down.
        """
        self.client.stop_logging()
        logging.info("CSV logging stopped")
        return "CSV logging stopped"

    def is_active(self) -> bool:
        """
        Check if CSV logging is currently running.

        Returns:
            True if logging process is active and writing data

        Example:
            >>> api.logging.start('test.csv')
            >>> api.logging.is_active()
            True
            >>> api.logging.stop()
            >>> api.logging.is_active()
            False
        """
        return self.client.is_logging_active()

    def export(self, filename: str = "movement_log.csv") -> str:
        """
        Export recorded movement data to CSV (if logger is attached).

        This is separate from the real-time CSV logging and is intended for
        exporting pre-recorded data from an attached logger object.

        Args:
            filename: Output CSV file path

        Returns:
            Status message with export path

        Raises:
            RuntimeError: If no logger is attached or no data available

        Example:
            >>> api.logging.export('my_data.csv')
            'Movement data exported to my_data.csv'

        Note:
            This method is currently reserved for future use with an attached
            data logger. Real-time logging uses start()/stop() instead.
        """
        if not hasattr(self.client, "logger") or self.client.logger is None:
            raise RuntimeError("No logger attached to RSI client")

        data = self.client.get_movement_data()
        if not data:
            raise RuntimeError("No data available to export")

        df = pd.DataFrame(data)
        df.to_csv(filename, index=False)
        logging.info(f"Movement data exported to {filename}")
        return f"Movement data exported to {filename}"
