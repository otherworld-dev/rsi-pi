"""
RSIPI - Robot Sensor Interface Python Integration

A lightweight Python library for real-time control of KUKA industrial robots
via the RSI 3.3 protocol. Provides high-level namespaced API for motion control,
I/O, logging, visualization, and KRL program manipulation.

Example:
    >>> from RSIPI import RSIAPI, context
    >>> api = RSIAPI(context("joints"))   # a context that ships with RSIPI
    >>> api.start()
    >>> api.motion.update_cartesian(X=10, Y=5, Z=0)
    >>> api.stop()

``context(name)`` returns the path to one of the packaged RSI contexts;
pass your own config path instead once you have one. The SAME context must
be loaded on the controller - see ``context_files()`` for the four files to
copy there.
"""

__version__ = "2.0.0"
__author__ = "RSIPI Development Team"

# Main API
from .rsi_api import RSIAPI

# Namespace APIs (for type hints and advanced use)
from .motion_api import MotionAPI
from .io_api import IOAPI
from .krl_api import KRLAPI
from .safety_api import SafetyAPI
from .monitoring_api import MonitoringAPI
from .logging_api import LoggingAPI
from .diagnostics_api import DiagnosticsAPI
from .viz_api import VizAPI
from .tools_api import ToolsAPI

# Core client (for advanced use)
from .rsi_client import RSIClient, ClientState

# Shipped RSI contexts
from .context import (
    context,
    context_files,
    available_contexts,
    describe_contexts,
    DEFAULT_CONTEXT,
)

# Exceptions
from .exceptions import (
    RSIError,
    RSINetworkError,
    RSIConnectionError,
    RSITimeoutError,
    RSIPacketError,
    RSISafetyError,
    RSISafetyViolation,
    RSIEmergencyStop,
    RSILimitExceeded,
    RSIConfigError,
    RSIConfigParseError,
    RSIMissingConfigError,
    RSIStateError,
    RSIInvalidTransition,
    RSIClientNotReady,
    RSIDataError,
    RSILoggingError,
    RSIVariableError,
    RSIMotionError,
    RSITrajectoryError,
    RSIKinematicsError,
)

__all__ = [
    # Main API (primary entry point)
    "RSIAPI",

    # Shipped RSI contexts
    "context",
    "context_files",
    "available_contexts",
    "describe_contexts",
    "DEFAULT_CONTEXT",

    # Namespace APIs
    "MotionAPI",
    "IOAPI",
    "KRLAPI",
    "SafetyAPI",
    "MonitoringAPI",
    "LoggingAPI",
    "DiagnosticsAPI",
    "VizAPI",
    "ToolsAPI",

    # Core
    "RSIClient",
    "ClientState",

    # Exceptions
    "RSIError",
    "RSINetworkError",
    "RSIConnectionError",
    "RSITimeoutError",
    "RSIPacketError",
    "RSISafetyError",
    "RSISafetyViolation",
    "RSIEmergencyStop",
    "RSILimitExceeded",
    "RSIConfigError",
    "RSIConfigParseError",
    "RSIMissingConfigError",
    "RSIStateError",
    "RSIInvalidTransition",
    "RSIClientNotReady",
    "RSIDataError",
    "RSILoggingError",
    "RSIVariableError",
    "RSIMotionError",
    "RSITrajectoryError",
    "RSIKinematicsError",

    # Version
    "__version__",
]
