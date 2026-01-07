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
    - on_start(): Start main processing
    - on_pause(): Pause current operation
    - on_resume(): Resume paused operation
    - on_stop(): Stop current operation
    - on_reset(): Reset to initial state
    
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
        ...         # Auto-transition: IDLE -> READY before execution
        ...         # On success: READY -> RUNNING
        ...         # On failure: READY -> STOPPED
        ...         self.data = []
        ...         print("Initialized!")
        ...         return True
        ...
        ...     def on_start(self):
        ...         # Pre-condition: READY state
        ...         # On success: READY -> RUNNING
        ...         print("Started!")
        ...         return True
        >>>
        >>> # Usage
        >>> fsm = StateMachine()
        >>> module = MyModule(fsm)
        >>> module.initialize()  # Automatic state management
        >>> # Now in RUNNING state
    """
    
    def __init__(self, state_machine: StateMachine):
        """
        Initialize the operational state machine.
        
        Args:
            state_machine: The underlying 3-layer StateMachine instance
        """
        self._state_machine = state_machine
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
        }
        
        event_type = event_mapping.get(target_state)
        if event_type is None:
            logger.error(f"No event mapping for target state {target_state.value}")
            return
        
        event = StateEvent(event_type, payload={"source": "OperationalStateMachine"})
        self._state_machine.send_event(event)
    
    # -------------------------------------------------------------------------
    # Public Lifecycle API
    # -------------------------------------------------------------------------
    
    def initialize(self, *args, **kwargs):
        """
        Initialize the module.
        
        Calls on_initialize() if implemented.
        Automatic state management:
        - Pre-condition: IDLE
        - Pre-transition: IDLE -> READY
        - On success: READY -> RUNNING
        - On failure: READY -> STOPPED
        """
        if hasattr(self, 'on_initialize'):
            return self.on_initialize(*args, **kwargs)
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_initialize()")
            return False
    
    def start(self, *args, **kwargs):
        """
        Start the module.
        
        Calls on_start() if implemented.
        Automatic state management:
        - Pre-condition: READY
        - On success: READY -> RUNNING
        - On failure: READY -> STOPPED
        """
        if hasattr(self, 'on_start'):
            return self.on_start(*args, **kwargs)
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_start()")
            return False
    
    def pause(self, *args, **kwargs):
        """
        Pause the module.
        
        Calls on_pause() if implemented.
        Automatic state management:
        - Pre-condition: RUNNING
        - On success: (current) -> PAUSED
        """
        if hasattr(self, 'on_pause'):
            return self.on_pause(*args, **kwargs)
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_pause()")
            return False
    
    def resume(self, *args, **kwargs):
        """
        Resume the module.
        
        Calls on_resume() if implemented.
        Automatic state management:
        - Pre-condition: PAUSED
        - On success: PAUSED -> READY
        - On failure: PAUSED -> STOPPED
        """
        if hasattr(self, 'on_resume'):
            return self.on_resume(*args, **kwargs)
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_resume()")
            return False
    
    def stop(self, *args, **kwargs):
        """
        Stop the module.
        
        Calls on_stop() if implemented.
        Automatic state management:
        - Pre-condition: RUNNING or PAUSED
        - On success: (current) -> STOPPED
        """
        if hasattr(self, 'on_stop'):
            return self.on_stop(*args, **kwargs)
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_stop()")
            return False
    
    def reset(self, *args, **kwargs):
        """
        Reset the module.
        
        Calls on_reset() if implemented.
        Automatic state management:
        - Pre-condition: STOPPED
        - On success: STOPPED -> IDLE
        """
        if hasattr(self, 'on_reset'):
            return self.on_reset(*args, **kwargs)
        else:
            logger.warning(f"{self.__class__.__name__} does not implement on_reset()")
            return False
    
    # -------------------------------------------------------------------------
    # Optional: Override these in subclasses
    # -------------------------------------------------------------------------
    
    # Note: Subclasses should define on_initialize(), on_start(), etc.
    # These will be automatically wrapped by the metaclass.
    
    def __repr__(self) -> str:
        """String representation."""
        return f"{self.__class__.__name__}(state={self.get_operational_state().value})"
