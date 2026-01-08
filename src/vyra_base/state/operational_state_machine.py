"""
Base class for operational state machines using metaclass-based automation.

This module provides a base class that modules can inherit from to get
automatic operational state management through lifecycle methods.
"""

import logging
from typing import Optional, Dict, Any

from .operational_metaclass import MetaOperationalState
from .state_machine import StateMachine
from .state_types import OperationalState


logger = logging.getLogger(__name__)


class OperationalStateMachine(metaclass=MetaOperationalState):
    """
    Base class for modules that need automatic operational state management.
    
    This class integrates with the 3-layer StateMachine and provides a
    clean interface for defining lifecycle methods that automatically
    handle state transitions.
    
    Subclasses should implement one or more of the following methods:
    
    - on_initialize(): Setup and initialization logic
    - on_pause(): Pause current operation
    - on_resume(): Resume paused operation
    - on_stop(): Stop current operation
    - on_reset(): Reset to initial state
    
    For dynamic operations, use the @operation decorator instead of on_start().
    
    All methods should return True on success, False on failure.
    Exceptions are caught and treated as failures.
    
    Example:
        >>> class MyModule(OperationalStateMachine):
        ...     def __init__(self, state_machine):
        ...         super().__init__(state_machine)
        ...         self.data = None
        ...
        ...     def on_initialize(self):
        ...         # This runs when initialize() is called
        ...         # Pre-condition: IDLE state
        ...         # On success: IDLE -> READY (operation counter reset)\n        ...         # On failure: IDLE -> ERROR
        ...         self.data = []
        ...         print("Initialized!")
        ...         return True
        >>>
        >>> # Usage
        >>> fsm = StateMachine()
        >>> module = MyModule(fsm)
        >>> module.initialize()  # Automatic state management
        >>> # Now in READY state, ready for operations
    """
    
    def __init__(self, state_machine: StateMachine):
        """
        Initialize the operational state machine.
        
        Args:
            state_machine: The underlying 3-layer StateMachine instance
        """
        self._state_machine = state_machine
        self._operation_counter = 0  # Reference counter for active operations
        logger.info(f"{self.__class__.__name__} initialized with state machine")
    
    # -------------------------------------------------------------------------
    # State Query Methods
    # -------------------------------------------------------------------------
    
    def get_operational_state(self) -> OperationalState:
        """Get current operational state."""
        return self._state_machine.get_operational_state()
    
    def get_all_states(self) -> Dict[str, str]:
        """Get all current states (lifecycle, operational, health)."""
        return self._state_machine.get_current_state()
    
    def is_idle(self) -> bool:
        """Check if in IDLE state."""
        return self.get_operational_state() == OperationalState.IDLE
    
    def is_ready(self) -> bool:
        """Check if in READY state."""
        return self.get_operational_state() == OperationalState.READY
    
    def is_running(self) -> bool:
        """Check if in RUNNING state."""
        return self.get_operational_state() == OperationalState.RUNNING
    
    def is_paused(self) -> bool:
        """Check if in PAUSED state."""
        return self.get_operational_state() == OperationalState.PAUSED
    
    def is_stopped(self) -> bool:
        """Check if in STOPPED state."""
        return self.get_operational_state() == OperationalState.STOPPED
    
    def is_error(self) -> bool:
        """Check if in ERROR state."""
        return self.get_operational_state() == OperationalState.ERROR
    
    # -------------------------------------------------------------------------
    # Internal State Transition Method
    # -------------------------------------------------------------------------
    
    def _set_operational_state(self, target_state: OperationalState):
        """
        Internal method to set operational state.
        
        This is called by the metaclass wrapper to perform state transitions.
        
        Args:
            target_state: Target operational state
        """
        from .state_events import StateEvent, EventType
        
        # Map target states to appropriate events
        event_mapping = {
            OperationalState.READY: EventType.SET_READY,
            OperationalState.RUNNING: EventType.TASK_START,
            OperationalState.PAUSED: EventType.TASK_PAUSE,
            OperationalState.STOPPED: EventType.TASK_STOP,
            OperationalState.IDLE: EventType.TASK_RESET,
            OperationalState.ERROR: EventType.TASK_ERROR,
        }
        
        event_type = event_mapping.get(target_state)
        if event_type is None:
            logger.error(f"No event mapping for target state {target_state.value}")
            return
        
        event = StateEvent(event_type, payload={"source": "OperationalStateMachine"})
        self._state_machine.send_event(event)
    
    # -------------------------------------------------------------------------
    # Operation Reference Counting Methods
    # -------------------------------------------------------------------------
    
    def _increment_operation_counter(self):
        """
        Increment the operation reference counter.
        
        If counter goes from 0 to 1 and current state is READY,
        automatically transition to RUNNING.
        
        This is called by the @operation decorator when starting an operation.
        """
        current_state = self.get_operational_state()
        
        # Only increment if in valid state
        if current_state not in {OperationalState.READY, OperationalState.RUNNING}:
            logger.warning(
                f"Cannot increment operation counter in state {current_state.value}. "
                f"Counter remains at {self._operation_counter}"
            )
            return
        
        self._operation_counter += 1
        logger.debug(f"Operation counter incremented to {self._operation_counter}")
        
        # Transition to RUNNING if this is the first operation
        if self._operation_counter == 1 and current_state == OperationalState.READY:
            logger.info("First operation started, transitioning READY -> RUNNING")
            self._set_operational_state(OperationalState.RUNNING)
    
    def _decrement_operation_counter(self):
        """
        Decrement the operation reference counter.
        
        If counter reaches 0 and current state is RUNNING,
        automatically transition back to READY.
        
        This is called by the @operation decorator when completing an operation.
        """
        if self._operation_counter <= 0:
            logger.warning("Operation counter already at 0, cannot decrement")
            return
        
        self._operation_counter -= 1
        logger.debug(f"Operation counter decremented to {self._operation_counter}")
        
        # Transition to READY if all operations completed
        if self._operation_counter == 0 and self.get_operational_state() == OperationalState.RUNNING:
            logger.info("All operations completed, transitioning RUNNING -> READY")
            self._set_operational_state(OperationalState.READY)
    
    def get_operation_counter(self) -> int:
        """
        Get the current operation reference counter value.
        
        Returns:
            Number of currently active operations
        """
        return self._operation_counter
    
    def _reset_operation_counter(self):
        """
        Reset the operation reference counter to zero.
        
        This is called by on_initialize() and on_resume() to ensure
        a clean state after initialization or resuming from pause.
        """
        if self._operation_counter != 0:
            logger.warning(
                f"Resetting operation counter from {self._operation_counter} to 0. "
                f"This may indicate incomplete operations."
            )
        self._operation_counter = 0
        logger.debug("Operation counter reset to 0")

    
    # -------------------------------------------------------------------------
    # Public Lifecycle API
    # -------------------------------------------------------------------------
    
    def initialize(self, *args, **kwargs) -> bool:
        """
        Initialize the module.
        
        Calls on_initialize() if implemented.
        Automatic state management:
        - Pre-condition: IDLE
        - On success: IDLE -> READY (resets operation counter)
        - On failure: IDLE -> ERROR
        """
        if hasattr(self, 'on_initialize'):
            func = getattr(self, 'on_initialize', None)
            if func:
                return func(*args, **kwargs)
            else:
                return False
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_initialize()")
            return False
    
    def pause(self, *args, **kwargs) -> bool:
        """
        Pause the module.
        
        Calls on_pause() if implemented.
        Automatic state management:
        - Pre-condition: RUNNING
        - On success: RUNNING -> PAUSED
        - On failure: RUNNING -> ERROR
        """
        if hasattr(self, 'on_pause'):
            func = getattr(self, 'on_pause', None)
            if func:
                return func(*args, **kwargs)
            else:
                return False
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_pause()")
            return False
    
    def resume(self, *args, **kwargs) -> bool:
        """
        Resume the module.
        
        Calls on_resume() if implemented.
        Automatic state management:
        - Pre-condition: PAUSED
        - On success: PAUSED -> READY (resets operation counter)
        - On failure: PAUSED -> ERROR
        """
        if hasattr(self, 'on_resume'):
            func = getattr(self, 'on_resume', None)
            if func:
                return func(*args, **kwargs)
            else:
                return False
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_resume()")
            return False
    
    def stop(self, *args, **kwargs) -> bool:
        """
        Stop the module.
        
        Calls on_stop() if implemented.
        Automatic state management:
        - Pre-condition: RUNNING or PAUSED
        - On success: (current) -> STOPPED
        - On failure: (current) -> ERROR
        """
        if hasattr(self, 'on_stop'):
            func = getattr(self, 'on_stop', None)
            if func:
                return func(*args, **kwargs)
            else:
                return False
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_stop()")
            return False
    
    def reset(self, *args, **kwargs) -> bool:
        """
        Reset the module.
        
        Calls on_reset() if implemented.
        Automatic state management:
        - Pre-condition: STOPPED or ERROR
        - On success: (current) -> IDLE
        - On failure: No state change
        """
        if hasattr(self, 'on_reset'):
            func = getattr(self, 'on_reset', None)
            if func:
                return func(*args, **kwargs)
            else:
                return False
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_reset()")
            return False
    
    # -------------------------------------------------------------------------
    # Optional: Override these in subclasses
    # -------------------------------------------------------------------------
    
    # Note: Subclasses should define on_initialize(), on_pause(), etc.
    # These will be automatically wrapped by the metaclass.
    # For dynamic operations, use the @operation decorator.
    
    def __repr__(self) -> str:
        """String representation."""
        return f"{self.__class__.__name__}(state={self.get_operational_state().value})"
