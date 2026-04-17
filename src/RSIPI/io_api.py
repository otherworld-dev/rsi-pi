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

        # Import here to avoid circular dependency
        from .tools_api import ToolsAPI

        tools = ToolsAPI(self.client)
        result = tools.update_variable(var_name, state_value)
        logging.debug(f"I/O {var_name} set to {state_value}")
        return result

    def set_output(self, channel: int, value: bool, group: str = 'Digout') -> str:
        """
        Set digital output by channel number.

        High-level wrapper for setting digital outputs. More convenient than
        toggle() when working with standard digital output channels.

        Args:
            channel: Output channel number (1-based, e.g., 1 for o1)
            value: Desired state (True = ON, False = OFF)
            group: I/O group name (default: 'Digout')

        Returns:
            Status message indicating success

        Raises:
            RSIVariableError: If the output channel doesn't exist
            RSISafetyViolation: If safety checks prevent the operation

        Example:
            >>> api.io.set_output(1, True)    # Turn ON output 1
            'Updated Digout.o1 to 1'
            >>> api.io.set_output(3, False)   # Turn OFF output 3
            'Updated Digout.o3 to 0'
            >>> api.io.set_output(5, True, group='DiO')  # Custom group
            'Updated DiO.5 to 1'

        Note:
            The default group 'Digout' corresponds to standard KUKA digital
            outputs configured in RSI. Channel numbering starts at 1 to match
            KUKA controller conventions.
        """
        channel_name = f"o{channel}"
        return self.toggle(group, channel_name, value)

    def get_input(self, channel: int, group: str = 'Digin') -> bool:
        """
        Read digital input by channel number.

        High-level wrapper for reading digital input states from the robot
        controller. Returns current state as boolean.

        Args:
            channel: Input channel number (1-based, e.g., 1 for i1)
            group: I/O group name (default: 'Digin')

        Returns:
            True if input is HIGH/ON, False if LOW/OFF

        Raises:
            RSIVariableError: If the input channel doesn't exist in receive_variables

        Example:
            >>> # Check if input 1 is active
            >>> if api.io.get_input(1):
            ...     print("Sensor triggered!")
            Sensor triggered!

            >>> # Read from custom group
            >>> state = api.io.get_input(5, group='DiI')
            >>> print(f"Input 5 state: {state}")
            Input 5 state: True

        Note:
            This reads from receive_variables, which contains the robot
            controller's current I/O state. Values are updated every RSI
            cycle (~4ms).
        """
        from .exceptions import RSIVariableError

        channel_name = f"i{channel}"
        var_name = f"{group}.{channel_name}"

        # Digital inputs come from the robot (send_variables = what robot sends us)
        if group in self.client.send_variables:
            group_dict = self.client.send_variables.get(group, {})
            if isinstance(group_dict, dict) and channel_name in group_dict:
                value = group_dict[channel_name]
                return bool(value)
            else:
                raise RSIVariableError(f"Input channel '{channel_name}' not found in group '{group}'")
        else:
            raise RSIVariableError(f"Input group '{group}' not found in send_variables")

    def pulse(self, channel: int, duration: float = 0.1, group: str = 'Digout') -> str:
        """
        Generate a timed pulse on the specified output channel.

        Turns the output ON, waits for the specified duration, then turns it OFF.
        Useful for triggering pneumatic actuators, solenoids, or signaling events.

        Args:
            channel: Output channel number (1-based)
            duration: Pulse duration in seconds (default: 0.1 = 100ms)
            group: I/O group name (default: 'Digout')

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
        var_name = f"{group}.{channel_name}"

        # Turn ON
        self.set_output(channel, True, group=group)
        logging.debug(f"Pulse started on {var_name}")

        # Wait for duration
        time.sleep(duration)

        # Turn OFF
        self.set_output(channel, False, group=group)
        logging.info(f"Pulse completed on {var_name} (duration: {duration}s)")

        return f"Pulse completed on {var_name} (duration: {duration}s)"
