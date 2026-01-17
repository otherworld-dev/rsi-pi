"""
Auto-reconnection manager for RSI network reliability.

Monitors network health and automatically reconnects when communication
is lost, with configurable retry logic and backoff strategies.
"""

import logging
import time
import threading
from typing import Optional, Callable, TYPE_CHECKING
from enum import Enum, auto

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class ReconnectStrategy(Enum):
    """Reconnection strategy options."""
    IMMEDIATE = auto()      # Reconnect immediately
    LINEAR_BACKOFF = auto() # Increase delay linearly
    EXPONENTIAL_BACKOFF = auto()  # Double delay each retry


class AutoReconnectManager:
    """
    Automatic reconnection manager for RSI communication.

    Monitors network health via watchdog timer and automatically
    attempts reconnection when communication is lost.
    """

    def __init__(
        self,
        client: 'RSIClient',
        enabled: bool = True,
        check_interval: float = 2.0,
        max_retries: int = 5,
        retry_delay: float = 5.0,
        strategy: ReconnectStrategy = ReconnectStrategy.LINEAR_BACKOFF,
        on_reconnect: Optional[Callable] = None,
        on_failure: Optional[Callable] = None,
    ):
        """
        Initialize auto-reconnect manager.

        Args:
            client: RSIClient instance to monitor
            enabled: Whether auto-reconnect is enabled
            check_interval: How often to check health (seconds)
            max_retries: Maximum reconnection attempts (0 = unlimited)
            retry_delay: Base delay between retries (seconds)
            strategy: Reconnection strategy (IMMEDIATE, LINEAR_BACKOFF, EXPONENTIAL_BACKOFF)
            on_reconnect: Optional callback called after successful reconnect
            on_failure: Optional callback called when max retries exceeded
        """
        self.client = client
        self.enabled = enabled
        self.check_interval = check_interval
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.strategy = strategy
        self.on_reconnect = on_reconnect
        self.on_failure = on_failure

        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False

        # Statistics
        self.total_reconnects = 0
        self.failed_reconnects = 0
        self.last_reconnect_time: Optional[float] = None

    def start(self) -> None:
        """Start the auto-reconnect monitor thread."""
        if self._running:
            logging.warning("Auto-reconnect manager already running")
            return

        if not self.enabled:
            logging.info("Auto-reconnect is disabled")
            return

        self._stop_event.clear()
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        logging.info("Auto-reconnect manager started")

    def stop(self) -> None:
        """Stop the auto-reconnect monitor thread."""
        if not self._running:
            return

        self._running = False
        self._stop_event.set()

        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=5)

        logging.info("Auto-reconnect manager stopped")

    def _monitor_loop(self) -> None:
        """Main monitoring loop (runs in background thread)."""
        while not self._stop_event.is_set():
            try:
                # Check if watchdog has timed out
                if hasattr(self.client, 'metrics_dict'):
                    metrics = dict(self.client.metrics_dict)
                    watchdog_timeout = metrics.get('watchdog_timeout', False)

                    if watchdog_timeout and self.client.is_running():
                        logging.error("Watchdog timeout detected - initiating auto-reconnect")
                        self._attempt_reconnection()

            except Exception as e:
                logging.error(f"Error in auto-reconnect monitor: {e}")

            # Sleep with interruptible wait
            self._stop_event.wait(self.check_interval)

    def _attempt_reconnection(self) -> bool:
        """
        Attempt to reconnect with configured retry logic.

        Returns:
            True if reconnection successful, False otherwise
        """
        retry_count = 0
        current_delay = self.retry_delay

        while True:
            # Check if we've exceeded max retries
            if self.max_retries > 0 and retry_count >= self.max_retries:
                logging.error(f"Max reconnection retries ({self.max_retries}) exceeded")
                self.failed_reconnects += 1
                if self.on_failure:
                    try:
                        self.on_failure()
                    except Exception as e:
                        logging.error(f"Error in on_failure callback: {e}")
                return False

            retry_count += 1
            logging.info(f"Reconnection attempt {retry_count}/{self.max_retries if self.max_retries > 0 else '∞'}")

            try:
                # Attempt reconnect
                self.client.reconnect()

                # Wait a moment for connection to stabilize
                time.sleep(2)

                # Verify connection is working
                if self._verify_connection():
                    logging.info(f"✅ Reconnection successful after {retry_count} attempt(s)")
                    self.total_reconnects += 1
                    self.last_reconnect_time = time.time()

                    if self.on_reconnect:
                        try:
                            self.on_reconnect()
                        except Exception as e:
                            logging.error(f"Error in on_reconnect callback: {e}")

                    return True
                else:
                    logging.warning("Reconnection completed but connection verification failed")

            except Exception as e:
                logging.error(f"Reconnection attempt {retry_count} failed: {e}")

            # Calculate delay for next retry based on strategy
            if self.strategy == ReconnectStrategy.IMMEDIATE:
                delay = 0
            elif self.strategy == ReconnectStrategy.LINEAR_BACKOFF:
                delay = self.retry_delay * retry_count
            elif self.strategy == ReconnectStrategy.EXPONENTIAL_BACKOFF:
                delay = self.retry_delay * (2 ** (retry_count - 1))
            else:
                delay = self.retry_delay

            if delay > 0:
                logging.info(f"Waiting {delay:.1f}s before next reconnection attempt...")
                self._stop_event.wait(delay)

                # Check if we were stopped during the wait
                if self._stop_event.is_set():
                    return False

    def _verify_connection(self) -> bool:
        """
        Verify that the connection is actually working.

        Returns:
            True if connection is healthy, False otherwise
        """
        # Wait a moment for metrics to update
        time.sleep(1)

        if not hasattr(self.client, 'metrics_dict'):
            return False

        metrics = dict(self.client.metrics_dict)

        # Check that we're receiving packets
        total_cycles = metrics.get('total_cycles', 0)
        if total_cycles == 0:
            return False

        # Check that watchdog is not timing out
        watchdog_timeout = metrics.get('watchdog_timeout', True)
        if watchdog_timeout:
            return False

        # Connection appears healthy
        return True

    def get_stats(self) -> dict:
        """
        Get auto-reconnect statistics.

        Returns:
            Dictionary with reconnection statistics
        """
        return {
            'enabled': self.enabled,
            'running': self._running,
            'total_reconnects': self.total_reconnects,
            'failed_reconnects': self.failed_reconnects,
            'last_reconnect_time': self.last_reconnect_time,
            'strategy': self.strategy.name,
            'max_retries': self.max_retries,
            'retry_delay': self.retry_delay,
        }
