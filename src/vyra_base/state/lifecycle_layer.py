"""
Lifecycle Layer - High-level API for module lifecycle management.

This layer controls module existence states:
- Startup and initialization
- Activation and deactivation
- Recovery from failures
- Controlled shutdown

Thread-safe wrapper around StateMachine for lifecycle operations.
"""

import logging
from typing import Optional, Dict, Any

from .state_machine import StateMachine, StateMachineConfig
from .state_types import LifecycleState
from .state_events import StateEvent, EventType


logger = logging.getLogger(__name__)


class LifecycleLayer:
    """
    High-level API for lifecycle state management.
    
    Provides intuitive methods for module lifecycle control:
    - start() - Begin initialization
    - activate() - Enter active state
    - shutdown() - Controlled deactivation
    - recover() - Recover from faults
    
    Example:
        >>> lifecycle = LifecycleLayer()
        >>> lifecycle.start()
        >>> lifecycle.complete_initialization()
        >>> lifecycle.get_state()
        'Active'
    """
    
    def __init__(self, fsm: Optional[StateMachine] = None):
        """
        Initialize lifecycle layer.
        
        Args:
            fsm: Existing StateMachine instance. Creates new if None.
        """
        self.fsm = fsm if fsm is not None else StateMachine()
        logger.info("LifecycleLayer initialized")
    
    # -------------------------------------------------------------------------
    # State Query
    # -------------------------------------------------------------------------
    
    def get_state(self) -> LifecycleState:
        """Get current lifecycle state."""
        return self.fsm.get_lifecycle_state()
    
    def get_state_name(self) -> str:
        """Get current lifecycle state as string."""
        return self.get_state().value
    
    def is_uninitialized(self) -> bool:
        """Check if module is uninitialized."""
        return self.get_state() == LifecycleState.UNINITIALIZED
    
    def is_initializing(self) -> bool:
        """Check if module is initializing."""
        return self.get_state() == LifecycleState.INITIALIZING
    
    def is_active(self) -> bool:
        """Check if module is active."""
        return self.get_state() == LifecycleState.ACTIVE
    
    def is_recovering(self) -> bool:
        """Check if module is recovering."""
        return self.get_state() == LifecycleState.RECOVERING
    
    def is_shutting_down(self) -> bool:
        """Check if module is shutting down."""
        return self.get_state() == LifecycleState.SHUTTING_DOWN
    
    def is_deactivated(self) -> bool:
        """Check if module is deactivated."""
        return self.get_state() == LifecycleState.DEACTIVATED
    
    def can_accept_tasks(self) -> bool:
        """Check if module can accept operational tasks."""
        return self.is_active() and self.fsm.is_healthy()
    
    # -------------------------------------------------------------------------
    # Lifecycle Control
    # -------------------------------------------------------------------------
    
    def start(self, metadata: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Start module initialization.
        
        Transitions: Uninitialized → Initializing
        
        Args:
            metadata: Optional initialization parameters
            
        Returns:
            New lifecycle state
            
        Raises:
            InvalidTransitionError: If not in Uninitialized state
        """
        event = StateEvent(EventType.START, payload=metadata)
        self.fsm.send_event(event)
        logger.info("Module start initiated")
        return self.get_state()
    
    def complete_initialization(self, result: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Mark initialization as successful.
        
        Transitions: Initializing → Active
        
        Args:
            result: Optional initialization results
            
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.INIT_SUCCESS, payload=result)
        self.fsm.send_event(event)
        logger.info("Initialization completed successfully")
        return self.get_state()
    
    def fail_initialization(self, error: Optional[str] = None) -> LifecycleState:
        """
        Mark initialization as failed.
        
        Transitions: Initializing → Recovering
        
        Args:
            error: Error description
            
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.INIT_FAILURE, payload={"error": error})
        self.fsm.send_event(event)
        logger.error(f"Initialization failed: {error}")
        return self.get_state()
    
    def shutdown(self, reason: Optional[str] = None) -> LifecycleState:
        """
        Begin controlled shutdown.
        
        Transitions: Active → ShuttingDown
        
        Args:
            reason: Shutdown reason
            
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.SHUTDOWN, payload={"reason": reason})
        self.fsm.send_event(event)
        logger.info(f"Shutdown initiated: {reason}")
        return self.get_state()
    
    def complete_shutdown(self) -> LifecycleState:
        """
        Complete shutdown process.
        
        Transitions: ShuttingDown → Deactivated
        
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.FINISHED)
        self.fsm.send_event(event)
        logger.info("Shutdown completed")
        return self.get_state()
    
    def enter_recovery(self, fault_info: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Enter recovery mode due to fault.
        
        Transitions: Active → Recovering
        
        Args:
            fault_info: Fault description and context
            
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.FAULT_DETECTED, payload=fault_info)
        self.fsm.send_event(event)
        logger.warning(f"Entering recovery mode: {fault_info}")
        return self.get_state()
    
    def complete_recovery(self, recovery_info: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Complete recovery successfully.
        
        Transitions: Recovering → Active
        
        Args:
            recovery_info: Recovery details
            
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.RECOVERY_SUCCESS, payload=recovery_info)
        self.fsm.send_event(event)
        logger.info("Recovery completed successfully")
        return self.get_state()
    
    def fail_recovery(self, error: Optional[str] = None) -> LifecycleState:
        """
        Mark recovery as failed.
        
        Transitions: Recovering → Deactivated
        
        Args:
            error: Recovery failure reason
            
        Returns:
            New lifecycle state
        """
        event = StateEvent(EventType.RECOVERY_FAILED, payload={"error": error})
        self.fsm.send_event(event)
        logger.error(f"Recovery failed: {error}")
        return self.get_state()
    
    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    
    def on_state_change(self, callback, priority: int = 0):
        """
        Register callback for lifecycle state changes.
        
        Args:
            callback: Function(layer, old_state, new_state)
            priority: Callback priority (higher = earlier execution)
        """
        self.fsm.subscribe("lifecycle", callback, priority)
    
    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------
    
    def get_info(self) -> Dict[str, Any]:
        """
        Get lifecycle layer information.
        
        Returns:
            Dictionary with state and capability info
        """
        return {
            "state": self.get_state_name(),
            "is_active": self.is_active(),
            "can_accept_tasks": self.can_accept_tasks(),
            "is_healthy": self.fsm.is_healthy(),
        }
    
    def __repr__(self) -> str:
        """String representation."""
        return f"LifecycleLayer(state={self.get_state_name()})"
