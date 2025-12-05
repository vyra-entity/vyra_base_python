"""
State type definitions for the 3-layer state machine.

This module defines the valid states for each layer according to
industrial automation standards (ISO 13849, IEC 61508).
"""

from enum import Enum
from typing import Set


class LifecycleState(Enum):
    """
    Lifecycle states define the existence and high-level life of a module.
    
    Similar to ROS2 Lifecycle nodes and IEC 61131-3 PLC states.
    """
    UNINITIALIZED = "Uninitialized"  # Module exists but not configured
    INITIALIZING = "Initializing"     # Startup/configuration in progress
    ACTIVE = "Active"                 # Fully operational
    RECOVERING = "Recovering"         # Error recovery/reset in progress
    SHUTTING_DOWN = "ShuttingDown"    # Controlled shutdown
    DEACTIVATED = "Deactivated"       # Module is stopped/offline


class OperationalState(Enum):
    """
    Operational states define the runtime activity of a module.
    
    Represents what the module is currently doing during normal operation.
    """
    IDLE = "Idle"                     # Inactive, waiting for work (Resting)
    READY = "Ready"                   # Prepared to accept tasks (Attentive)
    RUNNING = "Running"               # Actively executing a task (Active)
    PROCESSING = "Processing"         # Background processing (Reflecting)
    DELEGATING = "Delegating"         # Waiting for other modules
    PAUSED = "Paused"                 # Task suspended, can resume
    BLOCKED = "Blocked"               # Waiting for resources/locks
    COMPLETED = "Completed"           # Task finished successfully


class HealthState(Enum):
    """
    Health states describe the integrity and error conditions of a module.
    
    Follows safety integrity levels (SIL) from IEC 61508.
    """
    OK = "OK"                         # All systems nominal
    WARNING = "Warning"               # Minor issues detected (Alert)
    OVERLOADED = "Overloaded"        # Resource constraints active
    FAULTED = "Faulted"              # Error state, operation impaired
    CRITICAL = "Critical"             # Severe error, immediate action required


# Valid state transitions for each layer (from -> to)
LIFECYCLE_TRANSITIONS = {
    (LifecycleState.UNINITIALIZED, LifecycleState.INITIALIZING),
    (LifecycleState.INITIALIZING, LifecycleState.ACTIVE),
    (LifecycleState.INITIALIZING, LifecycleState.RECOVERING),
    (LifecycleState.ACTIVE, LifecycleState.SHUTTING_DOWN),
    (LifecycleState.ACTIVE, LifecycleState.RECOVERING),
    (LifecycleState.RECOVERING, LifecycleState.ACTIVE),
    (LifecycleState.RECOVERING, LifecycleState.DEACTIVATED),
    (LifecycleState.SHUTTING_DOWN, LifecycleState.DEACTIVATED),
}

OPERATIONAL_TRANSITIONS = {
    (OperationalState.IDLE, OperationalState.READY),
    (OperationalState.READY, OperationalState.RUNNING),
    (OperationalState.RUNNING, OperationalState.PAUSED),
    (OperationalState.RUNNING, OperationalState.BLOCKED),
    (OperationalState.RUNNING, OperationalState.DELEGATING),
    (OperationalState.RUNNING, OperationalState.PROCESSING),
    (OperationalState.RUNNING, OperationalState.COMPLETED),
    (OperationalState.PROCESSING, OperationalState.RUNNING),
    (OperationalState.DELEGATING, OperationalState.RUNNING),
    (OperationalState.DELEGATING, OperationalState.COMPLETED),
    (OperationalState.PAUSED, OperationalState.RUNNING),
    (OperationalState.BLOCKED, OperationalState.RUNNING),
    (OperationalState.COMPLETED, OperationalState.READY),
    (OperationalState.COMPLETED, OperationalState.IDLE),
}

HEALTH_TRANSITIONS = {
    (HealthState.OK, HealthState.WARNING),
    (HealthState.WARNING, HealthState.OK),
    (HealthState.WARNING, HealthState.OVERLOADED),
    (HealthState.WARNING, HealthState.FAULTED),
    (HealthState.OVERLOADED, HealthState.WARNING),
    (HealthState.OVERLOADED, HealthState.FAULTED),
    (HealthState.FAULTED, HealthState.OK),
    (HealthState.FAULTED, HealthState.CRITICAL),
    (HealthState.CRITICAL, HealthState.FAULTED),
}

# Operational states allowed per lifecycle state
LIFECYCLE_OPERATIONAL_RULES = {
    LifecycleState.UNINITIALIZED: set(),  # No operational activity
    LifecycleState.INITIALIZING: set(),   # Operational FSM locked
    LifecycleState.ACTIVE: {              # Full operational capability
        OperationalState.IDLE,
        OperationalState.READY,
        OperationalState.RUNNING,
        OperationalState.PROCESSING,
        OperationalState.DELEGATING,
        OperationalState.PAUSED,
        OperationalState.BLOCKED,
        OperationalState.COMPLETED,
    },
    LifecycleState.RECOVERING: {          # Limited operational states
        OperationalState.IDLE,
        OperationalState.PAUSED,
        OperationalState.BLOCKED,
    },
    LifecycleState.SHUTTING_DOWN: {       # Freezing operational state
        OperationalState.PAUSED,
        OperationalState.IDLE,
    },
    LifecycleState.DEACTIVATED: {         # Only idle when deactivated
        OperationalState.IDLE,
    },
}


def is_valid_lifecycle_transition(from_state: LifecycleState, to_state: LifecycleState) -> bool:
    """Check if lifecycle transition is valid."""
    return (from_state, to_state) in LIFECYCLE_TRANSITIONS


def is_valid_operational_transition(from_state: OperationalState, to_state: OperationalState) -> bool:
    """Check if operational transition is valid."""
    return (from_state, to_state) in OPERATIONAL_TRANSITIONS


def is_valid_health_transition(from_state: HealthState, to_state: HealthState) -> bool:
    """Check if health transition is valid."""
    return (from_state, to_state) in HEALTH_TRANSITIONS


def is_operational_allowed_in_lifecycle(lifecycle: LifecycleState, operational: OperationalState) -> bool:
    """Check if operational state is allowed in current lifecycle state."""
    allowed_states = LIFECYCLE_OPERATIONAL_RULES.get(lifecycle, set())
    return operational in allowed_states
