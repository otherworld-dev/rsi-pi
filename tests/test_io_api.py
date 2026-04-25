"""Tests for IOAPI."""
import pytest
import sys
import os
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.io_api import IOAPI
from RSIPI.exceptions import RSIVariableError


def _make_mock_client(send_vars=None, receive_vars=None):
    """
    Build a mock RSIClient with plain dict send/receive variables
    and a safety_manager that passes everything through.
    """
    client = MagicMock()
    client.send_variables = send_vars if send_vars is not None else {}
    client.receive_variables = receive_vars if receive_vars is not None else {}

    # Safety manager: validate returns the value unchanged
    client.safety_manager.validate.side_effect = lambda path, val: val

    return client


@pytest.fixture
def io_with_digout():
    """IOAPI with Digout in receive_variables and Digin in send_variables."""
    send = {
        "Digin": {"i1": 1, "i2": 0, "i3": 1},
        "IPOC": 12345,
    }
    recv = {
        "Digout": {"o1": 0, "o2": 0, "o3": 0},
        "DiO": 0,
    }
    client = _make_mock_client(send_vars=send, receive_vars=recv)
    return IOAPI(client)


# ---------------------------------------------------------------------------
# toggle
# ---------------------------------------------------------------------------
class TestToggle:
    """Tests for IOAPI.toggle()."""

    def test_toggle_sets_value_via_update_variable(self, io_with_digout):
        """toggle should call _tools.update_variable with group.name and int state."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "Updated Digout.o1 to 1"

        result = io.toggle("Digout", "o1", True)

        io._tools.update_variable.assert_called_once_with("Digout.o1", 1)
        assert result == "Updated Digout.o1 to 1"

    def test_toggle_false_sends_zero(self, io_with_digout):
        """toggle with False should send 0."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "Updated Digout.o2 to 0"

        io.toggle("Digout", "o2", False)
        io._tools.update_variable.assert_called_once_with("Digout.o2", 0)

    def test_toggle_int_one(self, io_with_digout):
        """toggle with int 1 should send 1."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "Updated Digout.o3 to 1"

        io.toggle("Digout", "o3", 1)
        io._tools.update_variable.assert_called_once_with("Digout.o3", 1)

    def test_toggle_truthy_value_normalizes_to_one(self, io_with_digout):
        """Any truthy value should normalize to 1."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "ok"

        io.toggle("Digout", "o1", 42)
        io._tools.update_variable.assert_called_once_with("Digout.o1", 1)


# ---------------------------------------------------------------------------
# set_output
# ---------------------------------------------------------------------------
class TestSetOutput:
    """Tests for IOAPI.set_output()."""

    def test_formats_channel_name(self, io_with_digout):
        """set_output(3, True) should call toggle with 'o3'."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "Updated Digout.o3 to 1"

        io.set_output(3, True)
        io._tools.update_variable.assert_called_once_with("Digout.o3", 1)

    def test_custom_group(self, io_with_digout):
        """set_output with custom group should use that group."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "Updated DiO.o5 to 1"

        io.set_output(5, True, group="DiO")
        io._tools.update_variable.assert_called_once_with("DiO.o5", 1)

    def test_set_output_off(self, io_with_digout):
        """set_output with False should send 0."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "Updated Digout.o1 to 0"

        io.set_output(1, False)
        io._tools.update_variable.assert_called_once_with("Digout.o1", 0)


# ---------------------------------------------------------------------------
# get_input
# ---------------------------------------------------------------------------
class TestGetInput:
    """Tests for IOAPI.get_input()."""

    def test_reads_true_from_send_variables(self, io_with_digout):
        """get_input should read from send_variables (what robot sends us)."""
        result = io_with_digout.get_input(1, group="Digin")
        assert result is True  # i1 = 1

    def test_reads_false_from_send_variables(self, io_with_digout):
        """get_input for channel with value 0 should return False."""
        result = io_with_digout.get_input(2, group="Digin")
        assert result is False  # i2 = 0

    def test_missing_group_raises(self, io_with_digout):
        """get_input with non-existent group should raise RSIVariableError."""
        with pytest.raises(RSIVariableError, match="not found in send_variables"):
            io_with_digout.get_input(1, group="NonExistent")

    def test_missing_channel_raises(self, io_with_digout):
        """get_input with non-existent channel should raise RSIVariableError."""
        with pytest.raises(RSIVariableError, match="not found in group"):
            io_with_digout.get_input(99, group="Digin")

    def test_default_group_is_digin(self):
        """Default group should be 'Digin'."""
        send = {"Digin": {"i1": 1}}
        client = _make_mock_client(send_vars=send)
        io = IOAPI(client)
        assert io.get_input(1) is True


# ---------------------------------------------------------------------------
# pulse (basic structure test, not timing)
# ---------------------------------------------------------------------------
class TestPulse:
    """Tests for IOAPI.pulse()."""

    def test_pulse_calls_on_then_off(self, io_with_digout):
        """pulse should turn output on then off."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "ok"

        result = io.pulse(1, duration=0.0)  # zero duration for fast test

        calls = io._tools.update_variable.call_args_list
        # First call: ON (value=1), second call: OFF (value=0)
        assert calls[0].args == ("Digout.o1", 1)
        assert calls[1].args == ("Digout.o1", 0)
        assert "Pulse completed" in result

    def test_pulse_returns_message_with_duration(self, io_with_digout):
        """Return string should include the duration."""
        io = io_with_digout
        io._tools = MagicMock()
        io._tools.update_variable.return_value = "ok"

        result = io.pulse(2, duration=0.0)
        assert "0.0s" in result
        assert "Digout.o2" in result
