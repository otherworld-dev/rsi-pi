"""KRL program manipulation API namespace for RSIPI."""

import logging
import time
from typing import Optional, Union, TYPE_CHECKING

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

    def wait_for_signal(
        self,
        channel: int,
        timeout: float = 5.0,
        check_interval: float = 0.01,
        group: str = 'Digin'
    ) -> bool:
        """
        Wait for KRL to set a specific I/O signal.

        Blocks until the specified digital input becomes HIGH, or timeout occurs.
        Commonly used for synchronization where Python waits for KRL to signal
        completion of a robot operation before proceeding.

        Args:
            channel: Input channel number to monitor (1-based)
            timeout: Maximum wait time in seconds (default: 5.0)
            check_interval: Polling interval in seconds (default: 0.01 = 10ms)
            group: I/O group name (default: 'Digin')

        Returns:
            True if signal received, False if timeout occurred

        Example:
            >>> # Wait for KRL to signal ready on input 3
            >>> if api.krl.wait_for_signal(3, timeout=10.0):
            ...     print("KRL signaled ready!")
            ...     # Proceed with Python-side processing
            ... else:
            ...     print("Timeout waiting for KRL signal")

            >>> # Handshake pattern: Python waits → KRL signals → Python continues
            >>> api.motion.update_cartesian(X=100)  # Send correction
            >>> api.krl.wait_for_signal(1)  # Wait for KRL to acknowledge
            >>> # KRL has processed the correction, safe to continue

        Note:
            This is a blocking operation. The check_interval determines polling
            frequency - lower values provide faster response but higher CPU usage.
            For typical RSI applications, 10-50ms intervals are appropriate.

        Warning:
            Ensure the KRL program actually sets the signal, otherwise this will
            block until timeout. Consider using try/except for timeout handling.
        """
        from .io_api import IOAPI

        io_api = IOAPI(self.client)
        start_time = time.time()

        logging.debug(f"Waiting for signal on {group}.i{channel} (timeout: {timeout}s)")

        while (time.time() - start_time) < timeout:
            if io_api.get_input(channel, group=group):
                elapsed = time.time() - start_time
                logging.info(f"Signal received on {group}.i{channel} after {elapsed:.3f}s")
                return True
            time.sleep(check_interval)

        logging.warning(f"Timeout waiting for signal on {group}.i{channel} after {timeout}s")
        return False

    def signal_complete(self, channel: int, group: str = 'Digout') -> str:
        """
        Signal to KRL that Python-side operation is complete.

        Sets the specified digital output HIGH to notify the KRL program that
        Python has finished processing and KRL can proceed.

        Args:
            channel: Output channel number to signal on (1-based)
            group: I/O group name (default: 'Digout')

        Returns:
            Status message indicating success

        Raises:
            RSIVariableError: If the output channel doesn't exist
            RSISafetyViolation: If safety checks prevent the operation

        Example:
            >>> # Signal KRL that data processing is complete
            >>> api.krl.signal_complete(2)
            'Signaled complete on Digout.o2'

            >>> # Typical coordination pattern:
            >>> # 1. KRL sends data via Tech variables
            >>> # 2. Python processes data
            >>> result = api.krl.read_param('T11')  # Read from KRL
            >>> processed = result * 2.0  # Process
            >>> api.krl.write_param('C11', processed)  # Write result
            >>> api.krl.signal_complete(1)  # Tell KRL we're done

        Note:
            This sets the output and leaves it HIGH. If you need to reset the
            signal after KRL acknowledges, use api.io.pulse() instead or manually
            call api.io.set_output(channel, False) after KRL reads the signal.

        See Also:
            wait_for_signal() - Complementary method for waiting on inputs
            api.io.pulse() - For temporary signal pulses
        """
        from .io_api import IOAPI

        io_api = IOAPI(self.client)
        io_api.set_output(channel, True, group=group)
        logging.info(f"Signaled complete on {group}.o{channel}")
        return f"Signaled complete on {group}.o{channel}"

    def write_param(self, slot: Union[int, str], value: float) -> str:
        """
        Write parameter to Tech.C variable for KRL to read.

        Tech.C variables (C11-C199) are "Control" parameters written by Python
        and read by KRL programs. Used for passing numerical data from Python
        to the robot controller.

        Args:
            slot: Tech.C slot number (11-199) or string like 'C11', 'c15'
            value: Numerical value to write

        Returns:
            Status message indicating success

        Raises:
            ValueError: If slot number is invalid (must be 11-199)
            RSIVariableError: If Tech variable group doesn't exist
            RSISafetyViolation: If safety checks prevent the operation

        Example:
            >>> # Send target position to KRL
            >>> api.krl.write_param(11, 650.5)  # Tech.C11 = 650.5
            'Updated Tech.C11 to 650.5'

            >>> # Send multiple parameters
            >>> api.krl.write_param('C12', 120.0)  # X coordinate
            >>> api.krl.write_param('C13', -50.0)  # Y coordinate
            >>> api.krl.write_param('C14', 800.0)  # Z coordinate

            >>> # KRL side reads with: target_x = $TECH.C[12]

        Note:
            KUKA RSI Tech variables support slots 11-199. Slots 1-10 are reserved.
            The KRL program must read from $TECH.C[n] to access these values.

        KRL Example:
            ```krl
            DEF my_program()
              DECL REAL target_x, target_y, target_z
              ; Python writes to C12, C13, C14
              target_x = $TECH.C[12]
              target_y = $TECH.C[13]
              target_z = $TECH.C[14]
              ; Use coordinates...
            END
            ```

        See Also:
            read_param() - Read Tech.T variables written by KRL
        """
        # Normalize slot to integer
        if isinstance(slot, str):
            slot_str = slot.upper().strip()
            if slot_str.startswith('C'):
                slot_num = int(slot_str[1:])
            else:
                slot_num = int(slot_str)
        else:
            slot_num = int(slot)

        # Validate slot range (KUKA reserves 1-10, usable range is 11-199)
        if not (11 <= slot_num <= 199):
            raise ValueError(f"Tech slot must be between 11-199, got {slot_num}")

        from .tools_api import ToolsAPI

        tools = ToolsAPI(self.client)
        var_name = f"Tech.C{slot_num}"
        result = tools.update_variable(var_name, value)
        logging.debug(f"Wrote {value} to {var_name}")
        return result

    def read_param(self, slot: Union[int, str]) -> float:
        """
        Read parameter from Tech.T variable written by KRL.

        Tech.T variables (T11-T199) are "Transfer" parameters written by KRL
        programs and read by Python. Used for passing numerical data from the
        robot controller to Python.

        Args:
            slot: Tech.T slot number (11-199) or string like 'T11', 't15'

        Returns:
            Numerical value from the Tech.T slot

        Raises:
            ValueError: If slot number is invalid (must be 11-199)
            RSIVariableError: If Tech variable group doesn't exist or slot not found

        Example:
            >>> # Read sensor value from KRL
            >>> force = api.krl.read_param(11)  # Read Tech.T11
            >>> print(f"Force reading: {force}")
            Force reading: 125.5

            >>> # Read multiple parameters
            >>> actual_x = api.krl.read_param('T12')
            >>> actual_y = api.krl.read_param('T13')
            >>> actual_z = api.krl.read_param('T14')

            >>> # KRL side writes with: $TECH.T[12] = actual_pos.X

        Note:
            Tech.T variables are updated every RSI cycle (~4ms) from the robot
            controller. Values reflect the KRL program's last write operation.

        KRL Example:
            ```krl
            DEF my_program()
              DECL E6POS actual_pos
              actual_pos = $POS_ACT
              ; Write to Tech.T for Python to read
              $TECH.T[12] = actual_pos.X
              $TECH.T[13] = actual_pos.Y
              $TECH.T[14] = actual_pos.Z
            END
            ```

        Warning:
            Ensure the KRL program has written to the Tech.T slot before reading,
            otherwise you'll receive the default value (typically 0.0).

        See Also:
            write_param() - Write Tech.C variables for KRL to read
        """
        from .exceptions import RSIVariableError

        # Normalize slot to integer
        if isinstance(slot, str):
            slot_str = slot.upper().strip()
            if slot_str.startswith('T'):
                slot_num = int(slot_str[1:])
            else:
                slot_num = int(slot_str)
        else:
            slot_num = int(slot)

        # Validate slot range
        if not (11 <= slot_num <= 199):
            raise ValueError(f"Tech slot must be between 11-199, got {slot_num}")

        # Read from receive_variables
        if 'Tech' in self.client.receive_variables:
            tech_dict = self.client.receive_variables.get('Tech', {})
            var_name = f"T{slot_num}"
            if isinstance(tech_dict, dict) and var_name in tech_dict:
                value = tech_dict[var_name]
                logging.debug(f"Read {value} from Tech.{var_name}")
                return float(value)
            else:
                raise RSIVariableError(f"Tech.{var_name} not found in receive_variables")
        else:
            raise RSIVariableError("Tech variable group not found in receive_variables")
