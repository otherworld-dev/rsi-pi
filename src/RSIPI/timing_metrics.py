"""
Timing instrumentation for RSI network communication.

Tracks latency, jitter, cycle time, and network quality metrics for
diagnostic analysis and performance monitoring.
"""

import time
import logging
from collections import deque
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import statistics


@dataclass
class TimingSnapshot:
    """Single timing measurement snapshot."""
    timestamp: float
    cycle_time: float
    ipoc: int
    packet_received: bool = True


@dataclass
class TimingMetrics:
    """
    Real-time timing metrics for RSI communication.

    Tracks latency, jitter, cycle time, IPOC gaps, and packet loss
    with configurable history window for statistical analysis.
    """

    # Configuration
    history_size: int = 1000  # Number of samples to retain
    expected_cycle_time: float = 0.004  # 4ms nominal cycle (250Hz)

    # Internal state
    cycle_times: deque = field(default_factory=lambda: deque(maxlen=1000))
    timestamps: deque = field(default_factory=lambda: deque(maxlen=1000))
    ipoc_values: deque = field(default_factory=lambda: deque(maxlen=1000))

    # Statistics
    total_cycles: int = 0
    total_packets_lost: int = 0
    total_ipoc_gaps: int = 0

    # Timing state
    last_timestamp: Optional[float] = None
    last_ipoc: Optional[int] = None
    start_time: float = field(default_factory=time.time)

    # Watchdog
    watchdog_timeout: float = 1.0  # 1 second
    # None until the first packet arrives. Defaulting this to the construction
    # time made the watchdog "expire" one second after start-up, logging
    # "communication lost" while simply waiting for the robot to be started -
    # a loss that cannot have happened, since nothing was ever received.
    last_packet_time: Optional[float] = None

    def __post_init__(self):
        """Adjust deque maxlen to match history_size."""
        self.cycle_times = deque(maxlen=self.history_size)
        self.timestamps = deque(maxlen=self.history_size)
        self.ipoc_values = deque(maxlen=self.history_size)
        # The robot advances IPOC by the cycle time in ms each cycle
        # (4 at 4ms IPO_FAST, 12 at 12ms IPO).
        self.ipoc_increment = max(1, round(self.expected_cycle_time * 1000))

    def record_cycle(self, ipoc: int) -> None:
        """
        Record a successful communication cycle.

        Args:
            ipoc: Current IPOC value from robot controller
        """
        current_time = time.time()

        # Calculate cycle time
        if self.last_timestamp is not None:
            cycle_time = current_time - self.last_timestamp
            self.cycle_times.append(cycle_time)

        # Check for IPOC gaps (missed packets)
        if self.last_ipoc is not None:
            expected_ipoc = self.last_ipoc + self.ipoc_increment
            ipoc_gap = ipoc - expected_ipoc

            if ipoc_gap != 0:
                self.total_ipoc_gaps += 1
                packets_lost = ipoc_gap // self.ipoc_increment
                self.total_packets_lost += packets_lost
                logging.warning(f"IPOC gap detected: {ipoc_gap} (lost ~{packets_lost} packets)")

        # Record values
        self.timestamps.append(current_time)
        self.ipoc_values.append(ipoc)
        self.last_timestamp = current_time
        self.last_ipoc = ipoc
        self.last_packet_time = current_time
        self.total_cycles += 1

    def check_watchdog(self) -> bool:
        """
        Check if communication has timed out.

        Returns:
            True if watchdog timeout exceeded, False otherwise
        """
        if self.last_packet_time is None:
            return False

        elapsed = time.time() - self.last_packet_time
        return elapsed > self.watchdog_timeout

    def get_current_stats(self) -> Dict[str, float]:
        """
        Get current timing statistics.

        Returns:
            Dictionary with timing metrics:
                - mean_cycle_time: Average cycle time in seconds
                - std_cycle_time: Standard deviation of cycle time
                - min_cycle_time: Minimum cycle time
                - max_cycle_time: Maximum cycle time
                - jitter: Cycle time standard deviation (alias)
                - packet_loss_rate: Percentage of packets lost
                - ipoc_gap_rate: IPOC gaps per 1000 cycles
                - total_cycles: Total cycles recorded
                - uptime: Total time since start in seconds
        """
        if not self.cycle_times:
            return {
                "mean_cycle_time": 0.0,
                "std_cycle_time": 0.0,
                "min_cycle_time": 0.0,
                "max_cycle_time": 0.0,
                "jitter": 0.0,
                "packet_loss_rate": 0.0,
                "ipoc_gap_rate": 0.0,
                "total_cycles": 0,
                "uptime": time.time() - self.start_time,
            }

        cycle_times_list = list(self.cycle_times)
        mean_ct = statistics.mean(cycle_times_list)
        std_ct = statistics.stdev(cycle_times_list) if len(cycle_times_list) > 1 else 0.0

        packet_loss_rate = (self.total_packets_lost / max(self.total_cycles, 1)) * 100
        ipoc_gap_rate = (self.total_ipoc_gaps / max(self.total_cycles, 1)) * 1000

        return {
            "mean_cycle_time": mean_ct,
            "std_cycle_time": std_ct,
            "min_cycle_time": min(cycle_times_list),
            "max_cycle_time": max(cycle_times_list),
            "jitter": std_ct,  # Jitter is typically measured as std deviation
            "packet_loss_rate": packet_loss_rate,
            "ipoc_gap_rate": ipoc_gap_rate,
            "total_cycles": self.total_cycles,
            "uptime": time.time() - self.start_time,
        }

    def get_detailed_stats(self) -> Dict[str, any]:
        """
        Get detailed statistics including percentiles.

        Returns:
            Dictionary with detailed metrics including percentiles
        """
        stats = self.get_current_stats()

        if self.cycle_times:
            cycle_times_sorted = sorted(self.cycle_times)
            n = len(cycle_times_sorted)

            stats.update({
                "p50_cycle_time": cycle_times_sorted[n // 2],
                "p95_cycle_time": cycle_times_sorted[int(n * 0.95)],
                "p99_cycle_time": cycle_times_sorted[int(n * 0.99)],
                "samples": n,
            })

        return stats

    def get_health_status(self) -> Dict[str, any]:
        """
        Get overall health status with warnings.

        Returns:
            Dictionary with health indicators and warnings
        """
        stats = self.get_current_stats()
        watchdog_timeout = self.check_watchdog()

        warnings = []
        is_healthy = True

        # Check for watchdog timeout
        if watchdog_timeout:
            warnings.append("Communication timeout - no packets received")
            is_healthy = False

        # Check for high jitter
        if stats["jitter"] > 0.002:  # 2ms jitter threshold
            warnings.append(f"High jitter detected: {stats['jitter']*1000:.2f}ms")
            is_healthy = False

        # Check for packet loss
        if stats["packet_loss_rate"] > 1.0:  # 1% packet loss threshold
            warnings.append(f"Packet loss detected: {stats['packet_loss_rate']:.2f}%")
            is_healthy = False

        # Check for high cycle time
        if stats["mean_cycle_time"] > (self.expected_cycle_time * 1.5):
            warnings.append(f"Cycle time exceeds expected: {stats['mean_cycle_time']*1000:.2f}ms")
            is_healthy = False

        return {
            "is_healthy": is_healthy,
            "warnings": warnings,
            "watchdog_timeout": watchdog_timeout,
            "stats": stats,
        }

    def reset(self) -> None:
        """Reset all metrics to initial state."""
        self.cycle_times.clear()
        self.timestamps.clear()
        self.ipoc_values.clear()
        self.total_cycles = 0
        self.total_packets_lost = 0
        self.total_ipoc_gaps = 0
        self.last_timestamp = None
        self.last_ipoc = None
        self.start_time = time.time()
        # Same reasoning as the field default: after a reset no packet has
        # been seen, so the watchdog must stay disarmed until one is.
        self.last_packet_time = None
        logging.info("Timing metrics reset")


class NetworkQualityMonitor:
    """
    High-level network quality monitoring.

    Provides easy access to network health and performance metrics
    with automatic threshold-based alerting.
    """

    def __init__(
        self,
        metrics: TimingMetrics,
        jitter_threshold: float = 0.002,
        packet_loss_threshold: float = 1.0,
        cycle_time_threshold: float = 0.006,
    ):
        """
        Initialize network quality monitor.

        Args:
            metrics: TimingMetrics instance to monitor
            jitter_threshold: Jitter threshold in seconds (default: 2ms)
            packet_loss_threshold: Packet loss rate threshold % (default: 1%)
            cycle_time_threshold: Cycle time threshold in seconds (default: 6ms)
        """
        self.metrics = metrics
        self.jitter_threshold = jitter_threshold
        self.packet_loss_threshold = packet_loss_threshold
        self.cycle_time_threshold = cycle_time_threshold

    def is_healthy(self) -> bool:
        """
        Check if network is healthy.

        Returns:
            True if all metrics within acceptable thresholds
        """
        health = self.metrics.get_health_status()
        return health["is_healthy"]

    def get_warnings(self) -> List[str]:
        """
        Get current network warnings.

        Returns:
            List of warning messages
        """
        health = self.metrics.get_health_status()
        return health["warnings"]

    def get_quality_score(self) -> float:
        """
        Calculate overall network quality score (0-100).

        Returns:
            Quality score where 100 is perfect, 0 is unusable
        """
        stats = self.metrics.get_current_stats()

        if stats["total_cycles"] == 0:
            return 0.0

        # Score components (each 0-100)
        jitter_score = max(0, 100 - (stats["jitter"] / self.jitter_threshold) * 100)
        loss_score = max(0, 100 - (stats["packet_loss_rate"] / self.packet_loss_threshold) * 100)
        cycle_score = max(0, 100 - ((stats["mean_cycle_time"] - self.metrics.expected_cycle_time) /
                                     (self.cycle_time_threshold - self.metrics.expected_cycle_time)) * 100)

        # Weighted average
        quality = (jitter_score * 0.4 + loss_score * 0.4 + cycle_score * 0.2)

        return min(100.0, max(0.0, quality))
