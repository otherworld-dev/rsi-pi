"""
24-Hour RSI Stability Test

Long-duration stability test for RSIPI network communication.
Monitors connection health, tracks metrics, and generates detailed
performance reports.

Usage:
    python stability_test.py [--duration HOURS] [--config CONFIG_FILE] [--output OUTPUT_FILE]

Example:
    # Run for 24 hours
    python stability_test.py --duration 24

    # Run for 1 hour with custom config
    python stability_test.py --duration 1 --config custom_config.xml

    # Quick 5-minute test
    python stability_test.py --duration 0.083  # 5 minutes
"""

import sys
import os
import time
import argparse
import logging
import json
import datetime
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from RSIPI import RSIAPI


class StabilityTest:
    """Long-duration stability test for RSI communication."""

    def __init__(
        self,
        config_file: str,
        duration_hours: float,
        output_file: str,
        check_interval: float = 60.0
    ):
        """
        Initialize stability test.

        Args:
            config_file: Path to RSI config file
            duration_hours: Test duration in hours
            output_file: Path for results JSON file
            check_interval: How often to sample metrics (seconds)
        """
        self.config_file = config_file
        self.duration_hours = duration_hours
        self.output_file = output_file
        self.check_interval = check_interval

        self.start_time = None
        self.end_time = None
        self.samples = []
        self.api = None

    def setup(self) -> None:
        """Set up logging and RSI connection."""
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(f'stability_test_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.log'),
                logging.StreamHandler()
            ]
        )

        logging.info(f"=== RSI Stability Test ===")
        logging.info(f"Config: {self.config_file}")
        logging.info(f"Duration: {self.duration_hours} hours")
        logging.info(f"Check interval: {self.check_interval}s")
        logging.info(f"Output: {self.output_file}")
        logging.info("=" * 50)

        # Initialize API with auto-reconnect enabled
        self.api = RSIAPI(
            self.config_file,
            enable_auto_reconnect=True,
            auto_reconnect_retries=0,  # Unlimited retries
            auto_reconnect_delay=10.0
        )

        logging.info("Starting RSI communication...")
        self.api.start()

        # Wait for connection to stabilize
        time.sleep(3)

        if not self.api.is_running():
            raise RuntimeError("Failed to start RSI communication")

        logging.info("✅ RSI communication started successfully")

    def run(self) -> None:
        """Run the stability test."""
        self.start_time = time.time()
        end_time = self.start_time + (self.duration_hours * 3600)

        sample_count = 0
        error_count = 0

        logging.info(f"Test started at {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info(f"Will run until {datetime.datetime.fromtimestamp(end_time).strftime('%Y-%m-%d %H:%M:%S')}")

        try:
            while time.time() < end_time:
                try:
                    # Collect metrics sample
                    sample = self._collect_sample()
                    self.samples.append(sample)
                    sample_count += 1

                    # Log progress
                    elapsed_hours = (time.time() - self.start_time) / 3600
                    remaining_hours = self.duration_hours - elapsed_hours
                    progress = (elapsed_hours / self.duration_hours) * 100

                    self._log_progress(sample, elapsed_hours, remaining_hours, progress, sample_count, error_count)

                except Exception as e:
                    error_count += 1
                    logging.error(f"Error collecting sample: {e}")

                # Sleep until next check
                time.sleep(self.check_interval)

        except KeyboardInterrupt:
            logging.warning("\n⚠️  Test interrupted by user")

        finally:
            self.end_time = time.time()
            self._cleanup()

    def _collect_sample(self) -> dict:
        """Collect a single metrics sample."""
        stats = self.api.diagnostics.get_stats()

        sample = {
            'timestamp': time.time(),
            'mean_cycle_time': stats.get('mean_cycle_time', 0),
            'jitter': stats.get('jitter', 0),
            'packet_loss_rate': stats.get('packet_loss_rate', 0),
            'ipoc_gap_rate': stats.get('ipoc_gap_rate', 0),
            'total_cycles': stats.get('total_cycles', 0),
            'is_healthy': stats.get('is_healthy', False),
            'warnings': stats.get('warnings', []),
            'uptime': stats.get('uptime', 0),
        }

        return sample

    def _log_progress(
        self,
        sample: dict,
        elapsed_hours: float,
        remaining_hours: float,
        progress: float,
        sample_count: int,
        error_count: int
    ) -> None:
        """Log current progress."""
        health_icon = "✅" if sample['is_healthy'] else "⚠️"

        logging.info(
            f"{health_icon} Progress: {progress:.1f}% | "
            f"Elapsed: {elapsed_hours:.2f}h | "
            f"Remaining: {remaining_hours:.2f}h | "
            f"Samples: {sample_count} | "
            f"Jitter: {sample['jitter']*1000:.2f}ms | "
            f"Loss: {sample['packet_loss_rate']:.2f}%"
        )

        if sample['warnings']:
            for warning in sample['warnings']:
                logging.warning(f"  ⚠️  {warning}")

    def _cleanup(self) -> None:
        """Clean up and generate report."""
        logging.info("\n=== Test Complete ===")

        # Stop RSI
        logging.info("Stopping RSI communication...")
        self.api.stop()

        # Generate report
        logging.info("Generating report...")
        report = self._generate_report()

        # Save results
        with open(self.output_file, 'w') as f:
            json.dump(report, f, indent=2)

        logging.info(f"✅ Report saved to: {self.output_file}")

        # Print summary
        self._print_summary(report)

    def _generate_report(self) -> dict:
        """Generate comprehensive test report."""
        if not self.samples:
            return {'error': 'No samples collected'}

        # Calculate statistics
        jitter_values = [s['jitter'] for s in self.samples]
        packet_loss_values = [s['packet_loss_rate'] for s in self.samples]
        cycle_time_values = [s['mean_cycle_time'] for s in self.samples]

        healthy_samples = sum(1 for s in self.samples if s['is_healthy'])
        unhealthy_samples = len(self.samples) - healthy_samples

        report = {
            'test_info': {
                'config_file': self.config_file,
                'duration_hours': self.duration_hours,
                'start_time': datetime.datetime.fromtimestamp(self.start_time).isoformat(),
                'end_time': datetime.datetime.fromtimestamp(self.end_time).isoformat(),
                'actual_duration_hours': (self.end_time - self.start_time) / 3600,
                'total_samples': len(self.samples),
            },
            'health_summary': {
                'healthy_samples': healthy_samples,
                'unhealthy_samples': unhealthy_samples,
                'health_percentage': (healthy_samples / len(self.samples)) * 100,
            },
            'timing_stats': {
                'mean_cycle_time_ms': statistics.mean(cycle_time_values) * 1000 if cycle_time_values else 0,
                'min_cycle_time_ms': min(cycle_time_values) * 1000 if cycle_time_values else 0,
                'max_cycle_time_ms': max(cycle_time_values) * 1000 if cycle_time_values else 0,
                'mean_jitter_ms': statistics.mean(jitter_values) * 1000 if jitter_values else 0,
                'max_jitter_ms': max(jitter_values) * 1000 if jitter_values else 0,
            },
            'network_stats': {
                'mean_packet_loss_percent': statistics.mean(packet_loss_values) if packet_loss_values else 0,
                'max_packet_loss_percent': max(packet_loss_values) if packet_loss_values else 0,
            },
            'final_metrics': self.samples[-1] if self.samples else {},
        }

        return report

    def _print_summary(self, report: dict) -> None:
        """Print human-readable summary."""
        print("\n" + "=" * 60)
        print("STABILITY TEST SUMMARY")
        print("=" * 60)

        info = report['test_info']
        health = report['health_summary']
        timing = report['timing_stats']
        network = report['network_stats']

        print(f"\nTest Duration: {info['actual_duration_hours']:.2f} hours")
        print(f"Total Samples: {info['total_samples']}")

        print(f"\nHealth: {health['health_percentage']:.1f}% healthy")
        print(f"  Healthy samples: {health['healthy_samples']}")
        print(f"  Unhealthy samples: {health['unhealthy_samples']}")

        print(f"\nTiming Performance:")
        print(f"  Mean cycle time: {timing['mean_cycle_time_ms']:.2f}ms")
        print(f"  Cycle time range: {timing['min_cycle_time_ms']:.2f} - {timing['max_cycle_time_ms']:.2f}ms")
        print(f"  Mean jitter: {timing['mean_jitter_ms']:.2f}ms")
        print(f"  Max jitter: {timing['max_jitter_ms']:.2f}ms")

        print(f"\nNetwork Quality:")
        print(f"  Mean packet loss: {network['mean_packet_loss_percent']:.3f}%")
        print(f"  Max packet loss: {network['max_packet_loss_percent']:.3f}%")

        health_icon = "✅ PASS" if health['health_percentage'] >= 95 else "⚠️  NEEDS IMPROVEMENT"
        print(f"\nOverall Result: {health_icon}")
        print("=" * 60)


import statistics


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='RSI 24-Hour Stability Test')
    parser.add_argument(
        '--duration',
        type=float,
        default=24.0,
        help='Test duration in hours (default: 24)'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='RSI_EthernetConfig.xml',
        help='Path to RSI config file'
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Output JSON file (default: stability_test_TIMESTAMP.json)'
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=60.0,
        help='Check interval in seconds (default: 60)'
    )

    args = parser.parse_args()

    # Generate default output filename if not specified
    if args.output is None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f'stability_test_{timestamp}.json'

    # Run test
    test = StabilityTest(
        config_file=args.config,
        duration_hours=args.duration,
        output_file=args.output,
        check_interval=args.interval
    )

    test.setup()
    test.run()


if __name__ == '__main__':
    main()
