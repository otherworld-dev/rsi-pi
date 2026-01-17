"""Diagnostics API namespace for RSIPI (Phase 2)."""

import logging
from typing import Dict, Any, List, TYPE_CHECKING

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class DiagnosticsAPI:
    """
    Network and performance diagnostics interface for KUKA RSI robot control.

    Provides real-time access to:
    - Timing metrics (latency, jitter, cycle time)
    - Network quality monitoring (packet loss, IPOC gaps)
    - Watchdog timer status
    - Communication health checks
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize DiagnosticsAPI namespace.

        Args:
            client: RSIClient instance with metrics_dict
        """
        self.client = client
        logging.debug("DiagnosticsAPI initialized")

    def get_stats(self) -> Dict[str, Any]:
        """
        Get comprehensive network and performance statistics.

        Returns:
            Dictionary with diagnostic information:
                - mean_cycle_time: Average cycle time in seconds
                - std_cycle_time: Standard deviation (jitter)
                - min_cycle_time: Minimum cycle time
                - max_cycle_time: Maximum cycle time
                - jitter: Cycle time variance (alias for std)
                - packet_loss_rate: Packet loss percentage
                - ipoc_gap_rate: IPOC gaps per 1000 cycles
                - total_cycles: Total cycles recorded
                - uptime: Time since start in seconds
                - is_healthy: Overall health boolean
                - warnings: List of warning messages
                - watchdog_timeout: Whether watchdog timed out

        Example:
            >>> stats = api.diagnostics.get_stats()
            >>> print(f"Jitter: {stats['jitter']*1000:.2f}ms")
            >>> print(f"Packet loss: {stats['packet_loss_rate']:.2f}%")
        """
        if not hasattr(self.client, 'metrics_dict'):
            return {"error": "Metrics not available"}

        # Return a copy of the metrics dict
        return dict(self.client.metrics_dict)

    def get_timing(self) -> Dict[str, float]:
        """
        Get timing-specific metrics.

        Returns:
            Dictionary with timing statistics:
                - mean_cycle_time: Average in seconds
                - std_cycle_time: Standard deviation
                - min_cycle_time: Minimum
                - max_cycle_time: Maximum
                - jitter: Variance (alias)

        Example:
            >>> timing = api.diagnostics.get_timing()
            >>> print(f"Avg cycle: {timing['mean_cycle_time']*1000:.2f}ms")
            >>> print(f"Jitter: {timing['jitter']*1000:.2f}ms")
        """
        stats = self.get_stats()

        if 'error' in stats:
            return stats

        return {
            'mean_cycle_time': stats.get('mean_cycle_time', 0.0),
            'std_cycle_time': stats.get('std_cycle_time', 0.0),
            'min_cycle_time': stats.get('min_cycle_time', 0.0),
            'max_cycle_time': stats.get('max_cycle_time', 0.0),
            'jitter': stats.get('jitter', 0.0),
        }

    def get_network_quality(self) -> Dict[str, float]:
        """
        Get network quality metrics.

        Returns:
            Dictionary with network metrics:
                - packet_loss_rate: Percentage of lost packets
                - ipoc_gap_rate: IPOC gaps per 1000 cycles
                - total_cycles: Total communication cycles

        Example:
            >>> quality = api.diagnostics.get_network_quality()
            >>> if quality['packet_loss_rate'] > 1.0:
            ...     print("Warning: High packet loss!")
        """
        stats = self.get_stats()

        if 'error' in stats:
            return stats

        return {
            'packet_loss_rate': stats.get('packet_loss_rate', 0.0),
            'ipoc_gap_rate': stats.get('ipoc_gap_rate', 0.0),
            'total_cycles': stats.get('total_cycles', 0),
        }

    def is_healthy(self) -> bool:
        """
        Check overall system health.

        Evaluates:
        - Jitter within acceptable limits (< 2ms)
        - Packet loss < 1%
        - No watchdog timeout
        - Client in RUNNING state

        Returns:
            True if all health checks pass

        Example:
            >>> if not api.diagnostics.is_healthy():
            ...     warnings = api.diagnostics.get_warnings()
            ...     for w in warnings:
            ...         print(f"Warning: {w}")
        """
        if not hasattr(self.client, 'metrics_dict'):
            return False

        if not self.client.is_running():
            return False

        stats = dict(self.client.metrics_dict)
        return stats.get('is_healthy', False)

    def get_warnings(self) -> List[str]:
        """
        Get current network health warnings.

        Returns:
            List of warning messages (empty if healthy)

        Example:
            >>> warnings = api.diagnostics.get_warnings()
            >>> for warning in warnings:
            ...     print(f"⚠️  {warning}")
        """
        if not hasattr(self.client, 'metrics_dict'):
            return ["Metrics not available"]

        stats = dict(self.client.metrics_dict)
        return stats.get('warnings', [])

    def check_watchdog(self) -> bool:
        """
        Check if watchdog timer has triggered.

        The watchdog detects communication loss when no packets
        are received for >1 second.

        Returns:
            True if watchdog timeout detected

        Example:
            >>> if api.diagnostics.check_watchdog():
            ...     print("Communication lost!")
            ...     api.reconnect()
        """
        if not hasattr(self.client, 'metrics_dict'):
            return False

        stats = dict(self.client.metrics_dict)
        return stats.get('watchdog_timeout', False)

    def get_uptime(self) -> float:
        """
        Get network uptime in seconds.

        Returns:
            Seconds since network process started

        Example:
            >>> uptime = api.diagnostics.get_uptime()
            >>> hours = uptime / 3600
            >>> print(f"Uptime: {hours:.1f} hours")
        """
        stats = self.get_stats()
        return stats.get('uptime', 0.0)

    def reset_metrics(self) -> None:
        """
        Reset all diagnostic metrics.

        Note:
            This is not yet implemented. Metrics are automatically
            reset on reconnect().
        """
        logging.warning("reset_metrics() not yet implemented - use reconnect() to reset")

    def format_stats(self) -> str:
        """
        Format statistics as human-readable string.

        Returns:
            Formatted string with key metrics

        Example:
            >>> print(api.diagnostics.format_stats())
            Network Diagnostics:
              Cycle Time: 4.01ms (±0.12ms jitter)
              Packet Loss: 0.05%
              IPOC Gaps: 0.2 per 1000 cycles
              Uptime: 120.5s
              Health: ✅ Healthy
        """
        stats = self.get_stats()

        if 'error' in stats:
            return f"Diagnostics Error: {stats['error']}"

        mean_ct = stats.get('mean_cycle_time', 0) * 1000  # Convert to ms
        jitter = stats.get('jitter', 0) * 1000
        packet_loss = stats.get('packet_loss_rate', 0)
        ipoc_gaps = stats.get('ipoc_gap_rate', 0)
        uptime = stats.get('uptime', 0)
        is_healthy = stats.get('is_healthy', False)
        warnings = stats.get('warnings', [])

        health_icon = "✅" if is_healthy else "⚠️"
        health_text = "Healthy" if is_healthy else "Issues Detected"

        output = f"""Network Diagnostics:
  Cycle Time: {mean_ct:.2f}ms (±{jitter:.2f}ms jitter)
  Packet Loss: {packet_loss:.2f}%
  IPOC Gaps: {ipoc_gaps:.1f} per 1000 cycles
  Total Cycles: {stats.get('total_cycles', 0)}
  Uptime: {uptime:.1f}s
  Health: {health_icon} {health_text}"""

        if warnings:
            output += "\n  Warnings:"
            for warning in warnings:
                output += f"\n    - {warning}"

        return output
