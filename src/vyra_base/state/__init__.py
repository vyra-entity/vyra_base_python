"""
3-Layer State Machine System for VYRA Modules

This module implements a hierarchical state machine with three layers:
- Lifecycle Layer: Module existence and initialization states
- Operational Layer: Runtime activity states
- Health Layer: Diagnostic and error states

Each layer has specific rules about how it can influence other layers:
- Lifecycle controls Operational and Health
- Health can override Operational and escalate Lifecycle
- Operational cannot directly change Lifecycle or Health (only reports events)

Conforms to industrial standards:
- IEC 61508 (Functional Safety)
- IEC 61131-3 (PLC Programming)
- ISO 13849 (Safety of Machinery)

Author: VYRA Development Team
License: See LICENSE file
"""

from .state_machine import (
    StateMachine,
    StateMachineConfig,
    StateMachineError,
    InvalidTransitionError,
    LayerViolationError,
    StateTransition,
)
from .state_types import (
    LifecycleState,
    OperationalState,
    HealthState,
    is_valid_lifecycle_transition,
    is_valid_operational_transition,
    is_valid_health_transition,
    is_operational_allowed_in_lifecycle,
)
from .state_events import (
    StateEvent,
    EventType,
    get_event_target_layer,
    is_interrupt_event,
)
from .lifecycle_layer import LifecycleLayer
from .operational_layer import OperationalLayer
from .health_layer import HealthLayer
from .unified import UnifiedStateMachine
from .operational_metaclass import MetaOperationalState, OperationalStateError
from .operational_state_machine import OperationalStateMachine
from .operation_decorator import operation, OperationConfig


__all__ = [
    # State Machine Core
    "StateMachine",
    "StateMachineConfig",
    "StateMachineError",
    "InvalidTransitionError",
    "LayerViolationError",
    "StateTransition",
    # State Types
    "LifecycleState",
    "OperationalState",
    "HealthState",
    "is_valid_lifecycle_transition",
    "is_valid_operational_transition",
    "is_valid_health_transition",
    "is_operational_allowed_in_lifecycle",
    # State Events
    "StateEvent",
    "EventType",
    "get_event_target_layer",
    "is_interrupt_event",
    # Layers
    "LifecycleLayer",
    "OperationalLayer",
    "HealthLayer",
    # Unified State Machine
    "UnifiedStateMachine",
    # Operational State Management
    "MetaOperationalState",
    "OperationalStateError",
    "OperationalStateMachine",
    # Operation Decorator
    "operation",
    "OperationConfig",
]
