"""
Core 3-layer state machine engine for industrial automation.

This implementation follows industrial standards:
- IEC 61508 (Functional Safety)
- IEC 61131-3 (PLC Programming)
- ISO 13849 (Safety of Machinery)

Features:
- Thread-safe operation with RLock
- Event-driven architecture
- Comprehensive logging and tracing
- Rule-based state transitions
- Layer interaction validation
"""

import threading
import logging
from typing import Dict, List, Callable, Optional, Any, Tuple
from dataclasses import dataclass, field
from datetime import datetime
from collections import deque

from .state_types import (
    LifecycleState,
    OperationalState,
    HealthState,
    is_valid_lifecycle_transition,
    is_valid_operational_transition,
    is_valid_health_transition,
    is_operational_allowed_in_lifecycle,
)
from .state_events import StateEvent, EventType, get_event_target_layer, is_interrupt_event
from vyra_base.helper.logger import Logger


class StateMachineError(Exception):
    """Base exception for state machine errors."""
    pass


class InvalidTransitionError(StateMachineError):
    """Raised when an invalid state transition is attempted."""
    pass


class LayerViolationError(StateMachineError):
    """Raised when layer interaction rules are violated."""
    pass


@dataclass
class StateTransition:
    """Record of a state transition for auditing and debugging."""
    from_state: str
    to_state: str
    layer: str
    event: StateEvent
    timestamp: datetime = field(default_factory=datetime.now)
    success: bool = True
    error_message: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert transition to dictionary."""
        return {
            "from_state": self.from_state,
            "to_state": self.to_state,
            "layer": self.layer,
            "event": self.event.to_dict(),
            "timestamp": self.timestamp.isoformat(),
            "success": self.success,
            "error_message": self.error_message,
        }


@dataclass
class StateMachineConfig:
    """Configuration for state machine behavior."""
    
    # Initial states
    initial_lifecycle: LifecycleState = LifecycleState.OFFLINE
    initial_operational: OperationalState = OperationalState.IDLE
    initial_health: HealthState = HealthState.HEALTHY
    
    # Operational state when entering Recovering lifecycle
    def operational_on_recovery(
            self, current_operational: OperationalState = OperationalState.IDLE) -> OperationalState:
        if current_operational in [OperationalState.READY, OperationalState.RUNNING]:
            return OperationalState.STOPPED
        else:
            return current_operational
    
    # Operational state when entering ShuttingDown
    def operational_on_shutdown(
            self, current_operational: OperationalState = OperationalState.IDLE) -> OperationalState:
        if current_operational in [OperationalState.READY, OperationalState.RUNNING]:
            return OperationalState.STOPPED
        else:
            return current_operational
    
    # Enable detailed logging
    enable_transition_log: bool = True
    
    # Maximum history size (0 = unlimited)
    max_history_size: int = 1000
    
    # Enable strict mode (raises exceptions on invalid transitions)
    strict_mode: bool = True


class StateMachine:
    """
    Professional 3-layer state machine for industrial automation.
    
    Architecture:
    - **Lifecycle Layer**: Controls module existence (startup/lifetime/shutdown)
    - **Operational Layer**: Manages runtime activity (tasks/processing)
    - **Health Layer**: Monitors system integrity (health/warnings/errors)
    
    Layer Interaction Rules:
    1. Lifecycle → Operational: Controls allowed states
    2. Health → Operational: Regulates behavior
    3. Health → Lifecycle: Escalates critical issues
    4. Operational cannot directly affect Lifecycle or Health
    
    Thread Safety:
    All public methods are thread-safe using reentrant locks.
    
    Example:
        >>> config = StateMachineConfig()
        >>> fsm = StateMachine(config)
        >>> fsm.send_event(StateEvent(EventType.START))
        >>> fsm.get_current_state()
        {'lifecycle': 'Initializing', 'operational': 'Idle', 'health': 'OK'}
    """
    
    def __init__(self, config: Optional[StateMachineConfig] = None):
        """
        Initialize the state machine.
        
        Args:
            config: Configuration object. Uses defaults if None.
        """
        self.config = config or StateMachineConfig()
        
        # Current states
        self._lifecycle = self.config.initial_lifecycle
        self._operational = self.config.initial_operational
        self._health = self.config.initial_health
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Event history
        self._history: deque = deque(maxlen=self.config.max_history_size if self.config.max_history_size > 0 else None)
        
        # Callbacks: layer -> [(callback, priority)]
        self._callbacks: Dict[str, List[Tuple[Callable, int]]] = {
            "lifecycle": [],
            "operational": [],
            "health": [],
            "any": [],
        }
        
        Logger.info(f"StateMachine initialized: lifecycle={self._lifecycle.value}, "
                   f"operational={self._operational.value}, health={self._health.value}")
    
    # -------------------------------------------------------------------------
    # Public API - State Query
    # -------------------------------------------------------------------------
    
    def get_current_state(self) -> Dict[str, str]:
        """
        Get current state of all layers.
        
        Returns:
            Dictionary with keys: 'lifecycle', 'operational', 'health'
        """
        with self._lock:
            return {
                "lifecycle": self._lifecycle.value,
                "operational": self._operational.value,
                "health": self._health.value,
            }
    
    def get_lifecycle_state(self) -> LifecycleState:
        """Get current lifecycle state."""
        with self._lock:
            return self._lifecycle
    
    def get_operational_state(self) -> OperationalState:
        """Get current operational state."""
        with self._lock:
            return self._operational
    
    def get_health_state(self) -> HealthState:
        """Get current health state."""
        with self._lock:
            return self._health
    
    def is_active(self) -> bool:
        """Check if module is in Active lifecycle state."""
        return self.get_lifecycle_state() == LifecycleState.ACTIVE
    
    def is_operational(self) -> bool:
        """Check if module can accept operational tasks."""
        lifecycle = self.get_lifecycle_state()
        return lifecycle == LifecycleState.ACTIVE and self._health in (HealthState.HEALTHY, HealthState.WARNING)
    
    def is_healthy(self) -> bool:
        """Check if module health is OK."""
        return self.get_health_state() == HealthState.HEALTHY
    
    # -------------------------------------------------------------------------
    # Public API - Event Sending
    # -------------------------------------------------------------------------
    
    def send_event(self, event: StateEvent) -> Dict[str, str]:
        """
        Send event to state machine and trigger transitions.
        
        Args:
            event: StateEvent object
            
        Returns:
            New state after processing event
            
        Raises:
            InvalidTransitionError: In strict mode, if transition is invalid
            LayerViolationError: If layer rules are violated
        """
        with self._lock:
            Logger.debug(f"Processing event: {event}")
            
            try:
                # Handle interrupt events first (highest priority)
                if is_interrupt_event(event.event_type):
                    self._handle_interrupt(event)
                    return self.get_current_state()
                
                # Determine target layer
                target_layer = get_event_target_layer(event.event_type)
                
                # Route to appropriate handler
                if target_layer == "lifecycle":
                    self._handle_lifecycle_event(event)
                elif target_layer == "health":
                    self._handle_health_event(event)
                elif target_layer == "operational":
                    self._handle_operational_event(event)
                else:
                    Logger.warning(f"Unknown event target layer: {target_layer}")
                
                return self.get_current_state()
                
            except Exception as e:
                Logger.error(f"Error processing event {event}: {e}")
                if self.config.strict_mode:
                    raise
                return self.get_current_state()
    
    # -------------------------------------------------------------------------
    # Public API - Callbacks
    # -------------------------------------------------------------------------
    
    def subscribe(self, layer: str, callback: Callable[[str, str, str], None], priority: int = 0):
        """
        Subscribe to state changes on a specific layer.
        
        Args:
            layer: Layer name ('lifecycle', 'operational', 'health', 'any')
            callback: Function(layer, old_state, new_state)
            priority: Higher priority callbacks are called first
            
        Example:
            >>> def on_state_change(layer, old, new):
            ...     print(f"{layer}: {old} -> {new}")
            >>> fsm.subscribe("lifecycle", on_state_change)
        """
        with self._lock:
            if layer not in self._callbacks:
                raise ValueError(f"Invalid layer: {layer}")
            
            self._callbacks[layer].append((callback, priority))
            # Sort by priority (descending)
            self._callbacks[layer].sort(key=lambda x: x[1], reverse=True)
            
            Logger.debug(f"Subscribed callback to {layer} layer with priority {priority}")
    
    def unsubscribe(self, layer: str, callback: Callable):
        """Remove a callback subscription."""
        with self._lock:
            if layer in self._callbacks:
                self._callbacks[layer] = [(cb, pri) for cb, pri in self._callbacks[layer] if cb != callback]
    
    # -------------------------------------------------------------------------
    # Public API - History and Diagnostics
    # -------------------------------------------------------------------------
    
    def get_history(self, limit: Optional[int] = None) -> List[StateTransition]:
        """
        Get state transition history.
        
        Args:
            limit: Maximum number of entries to return (newest first)
            
        Returns:
            List of StateTransition objects
        """
        with self._lock:
            history_list = list(self._history)
            if limit:
                return history_list[-limit:]
            return history_list
    
    def clear_history(self):
        """Clear transition history."""
        with self._lock:
            self._history.clear()
            Logger.info("State machine history cleared")
    
    def get_diagnostic_info(self) -> Dict[str, Any]:
        """
        Get comprehensive diagnostic information.
        
        Returns:
            Dictionary with current states, history stats, and configuration
        """
        with self._lock:
            return {
                "current_state": self.get_current_state(),
                "is_active": self.is_active(),
                "is_operational": self.is_operational(),
                "is_healthy": self.is_healthy(),
                "history_size": len(self._history),
                "config": {
                    "strict_mode": self.config.strict_mode,
                    "transition_log": self.config.enable_transition_log,
                    "max_history": self.config.max_history_size,
                },
                "callbacks": {
                    layer: len(cbs) for layer, cbs in self._callbacks.items()
                },
            }
    
    # -------------------------------------------------------------------------
    # Internal - Event Handlers
    # -------------------------------------------------------------------------
    
    def _handle_lifecycle_event(self, event: StateEvent):
        """Handle events targeting lifecycle layer."""
        current = self._lifecycle
        new_state = self._get_lifecycle_target(current, event.event_type)
        
        if new_state is None:
            Logger.debug(f"No lifecycle transition for event {event.event_type} in state {current}")
            return
        
        # Validate transition
        if not is_valid_lifecycle_transition(current, new_state):
            msg = f"Invalid lifecycle transition: {current.value} -> {new_state.value}"
            if self.config.strict_mode:
                raise InvalidTransitionError(msg)
            Logger.warning(msg)
            return
        
        # Execute transition
        self._lifecycle = new_state
        self._record_transition("lifecycle", current.value, new_state.value, event, True)
        self._notify_callbacks("lifecycle", current.value, new_state.value)
        
        Logger.info(f"Lifecycle: {current.value} -> {new_state.value}")
        
        # Apply lifecycle → operational rules
        self._apply_lifecycle_to_operational_rules(new_state, event)
    
    def _handle_operational_event(self, event: StateEvent):
        """Handle events targeting operational layer."""
        current = self._operational
        new_state = self._get_operational_target(current, event.event_type)
        
        if new_state is None:
            Logger.debug(f"No operational transition for event {event.event_type} in state {current}")
            return
        
        # Validate transition
        if not is_valid_operational_transition(current, new_state):
            msg = f"Invalid operational transition: {current.value} -> {new_state.value}"
            if self.config.strict_mode:
                raise InvalidTransitionError(msg)
            Logger.warning(msg)
            return
        
        # Check if lifecycle allows this operational state
        if not is_operational_allowed_in_lifecycle(self._lifecycle, new_state):
            msg = f"Operational state {new_state.value} not allowed in lifecycle {self._lifecycle.value}"
            if self.config.strict_mode:
                raise LayerViolationError(msg)
            Logger.warning(msg)
            # Force to safe state
            new_state = OperationalState.IDLE
        
        # Execute transition
        self._operational = new_state
        self._record_transition("operational", current.value, new_state.value, event, True)
        self._notify_callbacks("operational", current.value, new_state.value)
        
        Logger.info(f"Operational: {current.value} -> {new_state.value}")
    
    def _handle_health_event(self, event: StateEvent):
        """Handle events targeting health layer."""
        current = self._health
        new_state = self._get_health_target(current, event.event_type)
        
        if new_state is None:
            Logger.debug(f"No health transition for event {event.event_type} in state {current}")
            return
        
        # Validate transition
        if not is_valid_health_transition(current, new_state):
            msg = f"Invalid health transition: {current.value} -> {new_state.value}"
            if self.config.strict_mode:
                raise InvalidTransitionError(msg)
            Logger.warning(msg)
            return
        
        # Execute transition
        self._health = new_state
        self._record_transition("health", current.value, new_state.value, event, True)
        self._notify_callbacks("health", current.value, new_state.value)
        
        Logger.info(f"Health: {current.value} -> {new_state.value}")
        
        # Apply health escalation rules
        self._apply_health_escalation_rules(current, new_state, event)
    
    def _handle_interrupt(self, event: StateEvent):
        """Handle system interrupt events."""
        Logger.warning(f"Processing interrupt: {event.event_type}")
        
        if event.event_type == EventType.EMERGENCY_STOP:
            # Emergency stop: immediately deactivate
            old_lifecycle = self._lifecycle
            old_operational = self._operational
            old_health = self._health
            
            if self._lifecycle != LifecycleState.ACTIVE:
                Logger.info(
                    f"Lifecycle state is not ACTIVE during emergency "
                    f"stop: {self._lifecycle.value}"
                )

            if self._operational in [
                    OperationalState.RUNNING, 
                    OperationalState.PAUSED, 
                    OperationalState.READY]:
                self._operational = OperationalState.STOPPED
            
            if self._health == HealthState.HEALTHY:
                self._health = HealthState.WARNING
            
            self._record_transition("lifecycle", old_lifecycle.value, self._lifecycle.value, event, True)
            self._record_transition("operational", old_operational.value, self._operational.value, event, True)
            self._record_transition("health", old_health.value, self._health.value, event, True)
            
            self._notify_callbacks("lifecycle", old_lifecycle.value, self._lifecycle.value)
            self._notify_callbacks("operational", old_operational.value, self._operational.value)
            self._notify_callbacks("health", old_health.value, self._health.value)
            
        elif event.event_type == EventType.INTERRUPT:
            # Regular interrupt: pause operational
            if self._operational == OperationalState.RUNNING:
                old = self._operational
                self._operational = OperationalState.PAUSED
                self._record_transition("operational", old.value, self._operational.value, event, True)
                self._notify_callbacks("operational", old.value, self._operational.value)
    
        elif event.event_type == EventType.PRIORITY_OVERRIDE:
            # Priority override: no state change, just log
            Logger.info("Priority override event received - no state change applied")


    # -------------------------------------------------------------------------
    # Internal - State Resolution
    # -------------------------------------------------------------------------
    
    def _get_lifecycle_target(self, current: LifecycleState, event_type: EventType) -> Optional[LifecycleState]:
        """Determine target lifecycle state for given event."""
        transitions = {
            (LifecycleState.OFFLINE, EventType.START): LifecycleState.INITIALIZING,
            (LifecycleState.INITIALIZING, EventType.INIT_SUCCESS): LifecycleState.ACTIVE,
            (LifecycleState.INITIALIZING, EventType.INIT_FAILURE): LifecycleState.RECOVERING,
            (LifecycleState.ACTIVE, EventType.SHUTDOWN): LifecycleState.SHUTTING_DOWN,
            (LifecycleState.ACTIVE, EventType.FAULT_DETECTED): LifecycleState.RECOVERING,
            (LifecycleState.ACTIVE, EventType.SET_SUSPENDED): LifecycleState.SUSPENDED,
            (LifecycleState.SUSPENDED, EventType.RESUME_SUSPENDED): LifecycleState.ACTIVE,
            (LifecycleState.RECOVERING, EventType.RECOVERY_SUCCESS): LifecycleState.ACTIVE,
            (LifecycleState.RECOVERING, EventType.RECOVERY_FAILED): LifecycleState.SHUTTING_DOWN,
            (LifecycleState.SHUTTING_DOWN, EventType.FINISHED): LifecycleState.OFFLINE,
        }
        return transitions.get((current, event_type))
    
    def _get_operational_target(self, current: OperationalState, event_type: EventType) -> Optional[OperationalState]:
        """Determine target operational state for given event."""
        transitions = {
            (OperationalState.IDLE, EventType.SET_READY): OperationalState.READY,
            (OperationalState.IDLE, EventType.TASK_ERROR): OperationalState.ERROR,
            (OperationalState.READY, EventType.TASK_START): OperationalState.RUNNING,
            (OperationalState.READY, EventType.TASK_STOP): OperationalState.STOPPED,
            (OperationalState.READY, EventType.TASK_ERROR): OperationalState.ERROR,
            (OperationalState.READY, EventType.TASK_PAUSE): OperationalState.PAUSED,
            (OperationalState.RUNNING, EventType.TASK_START): OperationalState.RUNNING,
            (OperationalState.RUNNING, EventType.TASK_COMPLETE): OperationalState.READY,
            (OperationalState.RUNNING, EventType.TASK_PAUSE): OperationalState.PAUSED,
            (OperationalState.RUNNING, EventType.TASK_STOP): OperationalState.STOPPED,
            (OperationalState.RUNNING, EventType.TASK_ERROR): OperationalState.ERROR,
            (OperationalState.PAUSED, EventType.TASK_RESUME): OperationalState.READY,
            (OperationalState.PAUSED, EventType.TASK_STOP): OperationalState.STOPPED,
            (OperationalState.PAUSED, EventType.TASK_ERROR): OperationalState.ERROR,
            (OperationalState.STOPPED, EventType.TASK_RESET): OperationalState.IDLE,
            (OperationalState.ERROR, EventType.TASK_RESET): OperationalState.IDLE,
        }
        return transitions.get((current, event_type))
    
    def _get_health_target(self, current: HealthState, event_type: EventType) -> Optional[HealthState]:
        """Determine target health state for given event."""
        transitions = {
            (HealthState.HEALTHY, EventType.WARN): HealthState.WARNING,
            (HealthState.HEALTHY, EventType.FAULT): HealthState.CRITICAL,
            (HealthState.WARNING, EventType.CLEAR_WARNING): HealthState.HEALTHY,
            (HealthState.WARNING, EventType.FAULT): HealthState.CRITICAL,
            (HealthState.CRITICAL, EventType.FAULT): HealthState.CRITICAL,
            (HealthState.CRITICAL, EventType.RECOVER): HealthState.HEALTHY,
        }
        return transitions.get((current, event_type))
    
    # -------------------------------------------------------------------------
    # Internal - Layer Interaction Rules
    # -------------------------------------------------------------------------
    
    def _apply_lifecycle_to_operational_rules(self, new_lifecycle: LifecycleState, event: StateEvent):
        """Apply lifecycle → operational control rules."""
        old: OperationalState = self._operational

        if new_lifecycle == LifecycleState.RECOVERING:
            # Force operational to recovery state    
            self._operational = self.config.operational_on_recovery(self._operational)

        elif new_lifecycle == LifecycleState.SHUTTING_DOWN:
            # Freeze operational
            self._operational = self.config.operational_on_shutdown(self._operational)

        elif new_lifecycle == LifecycleState.OFFLINE:
            # Disable operational
            self._operational = OperationalState.IDLE

        if old != self._operational:
            self._record_transition("operational", old.value, self._operational.value, event, True)
            self._notify_callbacks("operational", old.value, self._operational.value)
    
    def _apply_health_escalation_rules(self, old_health: HealthState, new_health: HealthState, event: StateEvent):
        """Apply health → lifecycle/operational escalation rules."""
        if new_health == HealthState.CRITICAL:
            # Critical health: force shutdown
            if self._lifecycle not in (LifecycleState.SHUTTING_DOWN, LifecycleState.OFFLINE):
                old = self._lifecycle
                self._lifecycle = LifecycleState.RECOVERING
                self._record_transition("lifecycle", old.value, self._lifecycle.value, event, True)
                self._notify_callbacks("lifecycle", old.value, self._lifecycle.value)
                self._apply_lifecycle_to_operational_rules(self._lifecycle, event)
    
    # -------------------------------------------------------------------------
    # Internal - Utilities
    # -------------------------------------------------------------------------
    
    def _record_transition(self, layer: str, from_state: str, to_state: str, event: StateEvent, success: bool):
        """Record transition in history."""
        if self.config.enable_transition_log:
            transition = StateTransition(
                from_state=from_state,
                to_state=to_state,
                layer=layer,
                event=event,
                success=success,
            )
            self._history.append(transition)
    
    def _notify_callbacks(self, layer: str, old_state: str, new_state: str):
        """Notify all subscribed callbacks."""
        # Notify layer-specific callbacks
        for callback, _ in self._callbacks.get(layer, []):
            try:
                callback(layer, old_state, new_state)
            except Exception as e:
                Logger.error(f"Callback error on {layer}: {e}")
        
        # Notify global callbacks
        for callback, _ in self._callbacks.get("any", []):
            try:
                callback(layer, old_state, new_state)
            except Exception as e:
                Logger.error(f"Global callback error: {e}")
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        state = self.get_current_state()
        return f"StateMachine(lifecycle={state['lifecycle']}, operational={state['operational']}, health={state['health']})"
