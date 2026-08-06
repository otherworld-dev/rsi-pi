"""CSV logging API namespace for RSIPI."""

import logging
import datetime
import os
from typing import Optional, TYPE_CHECKING

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

    def export(self, filename: str = "movement_log.csv", source: Optional[str] = None) -> str:
        """
        Export the most recent recorded CSV log to a new file.

        Copies the log recorded via start()/stop() (or an explicit source
        file) to the given destination — a snapshot you can hand off while
        later runs overwrite the working log.

        Args:
            filename: Output CSV file path
            source: Source log to export (default: the last file passed to
                start() in this session)

        Returns:
            Status message with export path

        Raises:
            RuntimeError: If no log has been recorded this session and no
                source was given
            FileNotFoundError: If the source log file does not exist

        Example:
            >>> api.logging.start('logs/run1.csv')
            >>> # ... motion ...
            >>> api.logging.stop()
            >>> api.logging.export('run1_snapshot.csv')
            'Movement data exported to run1_snapshot.csv'
        """
        import shutil

        if source is None:
            source = getattr(self.client, "last_log_file", None)
        if source is None:
            raise RuntimeError(
                "No log has been recorded this session - start()/stop() a log "
                "first, or pass source= explicitly"
            )
        if not os.path.exists(source):
            raise FileNotFoundError(f"Log file not found: {source}")

        out_dir = os.path.dirname(filename)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        shutil.copyfile(source, filename)
        logging.info(f"Movement data exported to {filename}")
        return f"Movement data exported to {filename}"
