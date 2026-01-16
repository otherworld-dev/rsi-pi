"""Digital I/O API namespace for RSIPI."""

import logging
from typing import Union, TYPE_CHECKING

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

    # TODO (Phase 3): Implement high-level I/O helpers
    # def set_output(self, channel: int, value: bool) -> str:
    #     """Set digital output by channel number."""
    #     pass
    #
    # def get_input(self, channel: int) -> bool:
    #     """Read digital input by channel number."""
    #     pass
    #
    # def pulse(self, channel: int, duration: float = 0.1) -> None:
    #     """Generate a pulse on the specified output channel."""
    #     pass
