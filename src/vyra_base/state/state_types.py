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
    INITIALIZING = "Initializing"     # Startup/configuration in progress
    ACTIVE = "Active"                 # Fully operational
    RECOVERING = "Recovering"         # Error recovery/restart in progress
    SUSPENDED = "Suspended"           # Temporarily paused
    SHUTTING_DOWN = "ShuttingDown"    # Controlled shutdown
    OFFLINE = "Offline"       # Module is stopped/offline


class OperationalState(Enum):
    """
    Operational states define the runtime activity of a module.
    
    Represents what the module is currently doing during normal operation.
    """
    IDLE = "Idle"                     # Inactive, waiting for work (Resting)
    READY = "Ready"                   # Prepared to accept tasks (Attentive)
    RUNNING = "Running"               # Actively executing a task (Active)
    BACKGROUND_RUNNING = "BackgroundRunning"  # Running background tasks
    PAUSED = "Paused"                 # Task suspended, can resume
    STOPPED = "Stopped"               # Process stopped, need to reset and reinitialize


class HealthState(Enum):
    """
    Health states describe the integrity and error conditions of a module.
    
    Follows safety integrity levels (SIL) from IEC 61508.
    """
    HEALTHY = "Healthy"               # All systems nominal
    WARNING = "Warning"               # Minor issues detected (Alert)
    CRITICAL = "Critical"             # Severe error, immediate action required


# Valid state transitions for each layer (from -> to)
LIFECYCLE_TRANSITIONS = {
    (LifecycleState.OFFLINE, LifecycleState.INITIALIZING),
    (LifecycleState.INITIALIZING, LifecycleState.ACTIVE),
    (LifecycleState.INITIALIZING, LifecycleState.RECOVERING),
    (LifecycleState.ACTIVE, LifecycleState.SHUTTING_DOWN),
    (LifecycleState.ACTIVE, LifecycleState.RECOVERING),
    (LifecycleState.RECOVERING, LifecycleState.ACTIVE),
    (LifecycleState.RECOVERING, LifecycleState.SHUTTING_DOWN),
    (LifecycleState.SHUTTING_DOWN, LifecycleState.OFFLINE)
}

OPERATIONAL_TRANSITIONS = {
    (OperationalState.IDLE, OperationalState.READY),
    (OperationalState.READY, OperationalState.RUNNING),
    (OperationalState.RUNNING, OperationalState.PAUSED),
    (OperationalState.RUNNING, OperationalState.STOPPED),
    (OperationalState.RUNNING, OperationalState.BACKGROUND_RUNNING),
    (OperationalState.BACKGROUND_RUNNING, OperationalState.RUNNING),
    (OperationalState.BACKGROUND_RUNNING, OperationalState.PAUSED),
    (OperationalState.BACKGROUND_RUNNING, OperationalState.STOPPED),
    (OperationalState.PAUSED, OperationalState.READY),
    (OperationalState.STOPPED, OperationalState.IDLE)
}

HEALTH_TRANSITIONS = {
    (HealthState.HEALTHY, HealthState.WARNING),
    (HealthState.HEALTHY, HealthState.CRITICAL),
    (HealthState.WARNING, HealthState.HEALTHY),
    (HealthState.WARNING, HealthState.CRITICAL),
    (HealthState.CRITICAL, HealthState.HEALTHY),
    (HealthState.CRITICAL, HealthState.WARNING),
}

# Operational states allowed per lifecycle state
LIFECYCLE_OPERATIONAL_RULES = {
    LifecycleState.INITIALIZING: set(),   # Operational FSM locked
    LifecycleState.ACTIVE: {              # Full operational capability
        OperationalState.IDLE,
        OperationalState.READY,
        OperationalState.RUNNING,
        OperationalState.BACKGROUND_RUNNING,
        OperationalState.PAUSED,
        OperationalState.STOPPED
    },
    LifecycleState.RECOVERING: {          # Limited operational states
        OperationalState.IDLE,
        OperationalState.PAUSED,
        OperationalState.STOPPED,
    },
    LifecycleState.SHUTTING_DOWN: {       # Freezing operational state
        OperationalState.IDLE
    },
    LifecycleState.OFFLINE: {         # Only idle when offline
        OperationalState.IDLE,
        OperationalState.STOPPED
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
