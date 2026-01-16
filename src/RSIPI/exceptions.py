"""
Custom exception hierarchy for RSIPI library.

Provides specific exception types for different failure modes to enable
targeted error handling in applications using RSIPI.
"""


class RSIError(Exception):
    """Base exception for all RSIPI-related errors."""
    pass


# ============================================================================
# Network & Communication Errors
# ============================================================================

class RSINetworkError(RSIError):
    """Base class for network-related errors."""
    pass


class RSIConnectionError(RSINetworkError):
    """Failed to establish or maintain connection with robot controller."""
    pass


class RSITimeoutError(RSINetworkError):
    """Network operation timed out."""
    pass


class RSIPacketError(RSINetworkError):
    """Invalid or corrupted packet received."""
    pass


# ============================================================================
# Safety & Validation Errors
# ============================================================================

class RSISafetyError(RSIError):
    """Base class for safety-related errors."""
    pass


class RSISafetyViolation(RSISafetyError):
    """Motion command violates safety limits."""
    pass


class RSIEmergencyStop(RSISafetyError):
    """Emergency stop is active, blocking all motion."""
    pass


class RSILimitExceeded(RSISafetyError):
    """Value exceeds configured safety limits."""
    pass


# ============================================================================
# Configuration Errors
# ============================================================================

class RSIConfigError(RSIError):
    """Base class for configuration-related errors."""
    pass


class RSIConfigParseError(RSIConfigError):
    """Failed to parse XML configuration file."""
    pass


class RSIConfigValidationError(RSIConfigError):
    """Configuration file contains invalid values."""
    pass


class RSIMissingConfigError(RSIConfigError):
    """Required configuration parameter is missing."""
    pass


# ============================================================================
# State Machine Errors
# ============================================================================

class RSIStateError(RSIError):
    """Base class for state machine errors."""
    pass


class RSIInvalidTransition(RSIStateError):
    """Attempted invalid state transition."""
    pass


class RSIClientNotReady(RSIStateError):
    """Client is not in appropriate state for requested operation."""
    pass


# ============================================================================
# Data & Logging Errors
# ============================================================================

class RSIDataError(RSIError):
    """Base class for data-related errors."""
    pass


class RSILoggingError(RSIDataError):
    """Error during CSV logging operations."""
    pass


class RSIVariableError(RSIDataError):
    """Invalid variable name or value."""
    pass


# ============================================================================
# Trajectory & Motion Errors
# ============================================================================

class RSIMotionError(RSIError):
    """Base class for motion-related errors."""
    pass


class RSITrajectoryError(RSIMotionError):
    """Invalid trajectory definition or execution."""
    pass


class RSIKinematicsError(RSIMotionError):
    """Kinematic calculation failure."""
    pass
