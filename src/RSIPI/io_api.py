"""Digital I/O API namespace for RSIPI."""

import logging
import time
from typing import Union, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .rsi_client import RSIClient


class IOAPI:
    """
    Digital I/O control interface for KUKA RSI robot control.

    Manages digital input/output signals for coordinating with external systems,
    controlling pneumatic tools, and synchronizing with sensors.
    """

    def __init__(self, client: 'RSIClient') -> None:
        """
        Initialize IOAPI namespace.

        Args:
            client: RSIClient instance for variable access
        """
        self.client = client
        from .tools_api import ToolsAPI
        self._tools = ToolsAPI(client)

    def toggle(self, group: str, name: str, state: Union[bool, int]) -> str:
        """
        Set a digital I/O variable to the specified state.

        Args:
            group: Parent I/O variable group (e.g., 'Digout', 'DiO', 'DiL')
            name: I/O channel name or number within the group (e.g., 'o1', '1')
            state: Desired state (True/False or 1/0)

        Returns:
            Status message indicating success or failure

        Raises:
            RSIVariableError: If the specified I/O group or channel doesn't exist
            RSISafetyViolation: If safety checks prevent the operation

        Example:
            >>> api.io.toggle('Digout', 'o1', True)   # Turn on output 1
            'Updated Digout.o1 to 1'
            >>> api.io.toggle('DiL', '5', False)      # Turn off input latch 5
            'Updated DiL.5 to 0'

        Note:
            This method goes through the full safety validation chain. I/O
            variables can have safety limits configured just like motion axes.
        """
        var_name = f"{group}.{name}"
        state_value = int(bool(state))  # Ensure binary 0 or 1

        result = self._tools.update_variable(var_name, state_value)
        logging.debug("I/O %s set to %d", var_name, state_value)
        return result

    def _find_output_group(self, channel: int) -> Optional[str]:
        """Find a per-bit writable output group declaring o<channel>."""
        channel_name = f"o{channel}"
        candidates = []
        for key, val in dict(self.client.receive_variables).items():
            if isinstance(val, dict) and channel_name in val:
                candidates.append(key)
        # Deterministic preference if several groups declare the channel
        for preferred in ("Digout", "DiO"):
            if preferred in candidates:
                return preferred
        return candidates[0] if candidates else None

    def set_output(self, channel: int, value: bool, group: Optional[str] = None) -> str:
        """
        Set digital output by channel number.

        Auto-detects the write path from the loaded config (RSI supports both
        notations):
        - a per-bit group declared in RECEIVE (e.g. MyOut.o1) is written
          directly;
        - otherwise the DiO LONG word (shipped configs) gets bit channel-1
          set/cleared via read-modify-write.

        Args:
            channel: Output channel number (1-based; bit channel-1 of DiO)
            value: Desired state (True = ON, False = OFF)
            group: Explicit per-bit group name (skips auto-detection)

        Returns:
            Status message indicating success

        Raises:
            RSIVariableError: If no writable output path exists in the config
            RSISafetyViolation: If safety checks prevent the operation

        Example:
            >>> api.io.set_output(1, True)    # Turn ON output 1
            >>> api.io.set_output(3, False)   # Turn OFF output 3

        Note:
            Digout.o* in the shipped configs is the SEND section - the
            robot's read-back of its own outputs - and is never writable.
            Outputs are commanded through the RECEIVE side (DiO word wired
            to MAP2DIGOUT in the RSI context).
        """
        from .exceptions import RSIVariableError

        if group is not None:
            return self.toggle(group, f"o{channel}", value)

        detected = self._find_output_group(channel)
        if detected is not None:
            return self.toggle(detected, f"o{channel}", value)

        # Word notation: DiO LONG bitmask (both shipped configs)
        receive = self.client.receive_variables
        if "DiO" in receive and not isinstance(receive.get("DiO"), dict):
            current = int(receive.get("DiO") or 0)
            bit = 1 << (channel - 1)
            new_word = (current | bit) if value else (current & ~bit)
            result = self._tools.update_variable("DiO", new_word)
            logging.debug("DiO bit %d set to %d (word: %d)", channel - 1, int(bool(value)), new_word)
            return result

        raise RSIVariableError(
            f"No writable digital-output path for channel {channel}: the config "
            "declares neither a per-bit output group nor a DiO word in RECEIVE"
        )

    def get_input(self, channel: int, group: Optional[str] = None) -> bool:
        """
        Read digital input by channel number.

        Auto-detects the read path from the loaded config:
        - a per-bit group in SEND (e.g. Digin.i1, if declared) is read
          directly;
        - otherwise bit channel-1 of the DiL LONG word (shipped configs).

        Args:
            channel: Input channel number (1-based; bit channel-1 of DiL)
            group: Explicit per-bit group name (skips auto-detection)

        Returns:
            True if input is HIGH/ON, False if LOW/OFF

        Raises:
            RSIVariableError: If no input path exists in the config

        Example:
            >>> if api.io.get_input(1):
            ...     print("Sensor triggered!")
            Sensor triggered!

        Note:
            Inputs come from the robot, so this reads send_variables (what
            the robot SENDS us), updated every RSI cycle (~4ms).
        """
        from .exceptions import RSIVariableError

        channel_name = f"i{channel}"
        send = self.client.send_variables

        if group is not None:
            group_dict = send.get(group)
            if isinstance(group_dict, dict) and channel_name in group_dict:
                return bool(group_dict[channel_name])
            raise RSIVariableError(
                f"Input channel '{channel_name}' not found in group '{group}'"
            )

        # Per-bit notation (e.g. Digin.i1-4 in the full config)
        for key, val in dict(send).items():
            if isinstance(val, dict) and channel_name in val:
                return bool(val[channel_name])

        # Word notation: DiL LONG bitmask (minimal config)
        if "DiL" in send and not isinstance(send.get("DiL"), dict):
            try:
                word = int(float(send.get("DiL") or 0))
            except (TypeError, ValueError):
                word = 0
            return bool(word & (1 << (channel - 1)))

        raise RSIVariableError(
            f"No digital-input path for channel {channel}: the config declares "
            "neither a per-bit input group nor a DiL word in SEND"
        )

    def pulse(self, channel: int, duration: float = 0.1, group: Optional[str] = None) -> str:
        """
        Generate a timed pulse on the specified output channel.

        Turns the output ON, waits for the specified duration, then turns it OFF.
        Useful for triggering pneumatic actuators, solenoids, or signaling events.

        Args:
            channel: Output channel number (1-based)
            duration: Pulse duration in seconds (default: 0.1 = 100ms)
            group: Explicit per-bit group name (default: auto-detect, same
                rules as set_output)

        Returns:
            Status message indicating completion

        Raises:
            RSIVariableError: If the output channel doesn't exist
            RSISafetyViolation: If safety checks prevent the operation

        Example:
            >>> # 100ms pulse on output 2
            >>> api.io.pulse(2)
            'Pulse completed on Digout.o2 (duration: 0.1s)'

            >>> # 500ms pulse on output 5
            >>> api.io.pulse(5, duration=0.5)
            'Pulse completed on Digout.o5 (duration: 0.5s)'

            >>> # Trigger pneumatic gripper on custom channel
            >>> api.io.pulse(3, duration=0.2, group='DiO')
            'Pulse completed on DiO.o3 (duration: 0.2s)'

        Warning:
            This method blocks for the duration of the pulse. For non-blocking
            pulses, consider using threading or async I/O patterns.

        Note:
            Pulse timing accuracy depends on system load and RSI cycle time.
            For critical timing requirements, consider hardware-timed outputs
            or KRL-based pulse generation.
        """
        channel_name = f"o{channel}"
        var_name = f"{group or 'auto'}.{channel_name}"

        # Turn ON
        self.set_output(channel, True, group=group)
        logging.debug("Pulse started on %s", var_name)

        # Wait for duration
        time.sleep(duration)

        # Turn OFF
        self.set_output(channel, False, group=group)
        logging.info("Pulse completed on %s (duration: %ss)", var_name, duration)

        return f"Pulse completed on {var_name} (duration: {duration}s)"
