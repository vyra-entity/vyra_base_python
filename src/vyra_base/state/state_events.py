"""
Event definitions for the 3-layer state machine.

Events trigger state transitions according to the industrial automation
event-driven architecture pattern.
"""

from enum import Enum
from typing import Optional, Any, Dict
from dataclasses import dataclass, field
from datetime import datetime


class EventType(Enum):
    """Event types for state transitions."""
    
    # Lifecycle events
    START = "start"
    INIT_SUCCESS = "init_success"
    INIT_FAILURE = "init_failure"
    SHUTDOWN = "shutdown"
    FINISHED = "finished"
    FAULT_DETECTED = "fault_detected"
    RECOVERY_SUCCESS = "recovery_success"
    RECOVERY_FAILED = "recovery_failed"
    
    # Operational events
    SET_READY = "set_ready"
    TASK_START = "task_start"
    SET_BACKGROUND = "set_background"
    TASK_PAUSE = "task_pause"
    TASK_RESUME = "task_resume"
    TASK_COMPLETE = "task_complete"
    TASK_STOP = "task_stop"
    TASK_RESET = "task_reset"
    
    # Health events
    WARN = "warn"
    CLEAR_WARNING = "clear_warning"
    FAULT = "fault"
    RECOVER = "recover"
    RESET = "reset"
    
    # Special interrupt events
    INTERRUPT = "interrupt"
    EMERGENCY_STOP = "emergency_stop"
    PRIORITY_OVERRIDE = "priority_override"


@dataclass(frozen=True)
class StateEvent:
    """
    Immutable event for state machine transitions.
    
    Contains all information about a state transition event including
    timestamp, origin, and optional payload for tracing and debugging.
    """
    event_type: EventType
    timestamp: datetime = field(default_factory=datetime.now)
    origin_layer: Optional[str] = None
    payload: Optional[Dict[str, Any]] = None
    event_id: Optional[str] = None
    
    def __post_init__(self):
        """Generate event ID and initialize payload if not provided."""
        if self.event_id is None:
            object.__setattr__(self, 'event_id', f"{self.event_type.value}_{self.timestamp.strftime('%Y%m%d%H%M%S%f')}")
        if self.payload is None:
            object.__setattr__(self, 'payload', {})
    
    def __str__(self) -> str:
        """String representation for logging."""
        return f"Event({self.event_type.value}, id={self.event_id}, origin={self.origin_layer})"
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert event to dictionary for serialization."""
        return {
            "event_type": self.event_type.value,
            "timestamp": self.timestamp.isoformat(),
            "origin_layer": self.origin_layer,
            "payload": self.payload,
            "event_id": self.event_id,
        }


# Event to layer mapping (which events affect which layers)
EVENT_LAYER_MAP = {
    # Lifecycle events
    EventType.START: "lifecycle",
    EventType.INIT_SUCCESS: "lifecycle",
    EventType.INIT_FAILURE: "lifecycle",
    EventType.SHUTDOWN: "lifecycle",
    EventType.FINISHED: "lifecycle",
    EventType.FAULT_DETECTED: "lifecycle",  # Can also affect health
    EventType.RECOVERY_SUCCESS: "lifecycle",
    EventType.RECOVERY_FAILED: "lifecycle",
    
    # Operational events
    EventType.SET_READY: "operational",
    EventType.TASK_START: "operational",
    EventType.SET_BACKGROUND: "operational",
    EventType.TASK_PAUSE: "operational",
    EventType.TASK_RESUME: "operational",
    EventType.TASK_STOP: "operational",
    EventType.TASK_RESET: "operational",
    
    # Health events
    EventType.WARN: "health",
    EventType.CLEAR_WARNING: "health",
    EventType.FAULT: "health",
    EventType.RECOVER: "health",
    EventType.RESET: "health",
    
    # Interrupt events (cross-layer)
    EventType.INTERRUPT: "interrupt",
    EventType.EMERGENCY_STOP: "interrupt",
    EventType.PRIORITY_OVERRIDE: "interrupt",
}


def get_event_target_layer(event_type: EventType) -> str:
    """Get the primary layer affected by an event."""
    return EVENT_LAYER_MAP.get(event_type, "unknown")


def is_interrupt_event(event_type: EventType) -> bool:
    """Check if event is a system interrupt."""
    return get_event_target_layer(event_type) == "interrupt"
