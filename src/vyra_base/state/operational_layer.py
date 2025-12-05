"""
Operational Layer - High-level API for task and activity management.

This layer controls operational states:
- Task lifecycle (ready, running, paused, completed)
- Processing and delegation
- Blocking and unblocking
- Auto-reset mechanisms

Thread-safe wrapper around StateMachine for operational control.
"""

import logging
from typing import Optional, Dict, Any

from .state_machine import StateMachine
from .state_types import OperationalState
from .state_events import StateEvent, EventType


logger = logging.getLogger(__name__)


class OperationalLayer:
    """
    High-level API for operational state management.
    
    Provides intuitive methods for task control:
    - ready() - Signal readiness for tasks
    - start_task() - Begin task execution
    - pause() / resume() - Pause and resume tasks
    - complete() - Mark task completion
    
    Example:
        >>> operational = OperationalLayer(fsm)
        >>> operational.ready()
        >>> operational.start_task({'task_id': '123'})
        >>> operational.get_state()
        'Running'
    """
    
    def __init__(self, fsm: StateMachine):
        """
        Initialize operational layer.
        
        Args:
            fsm: StateMachine instance to control
        """
        self.fsm = fsm
        logger.info("OperationalLayer initialized")
    
    # -------------------------------------------------------------------------
    # State Query
    # -------------------------------------------------------------------------
    
    def get_state(self) -> OperationalState:
        """Get current operational state."""
        return self.fsm.get_operational_state()
    
    def get_state_name(self) -> str:
        """Get current operational state as string."""
        return self.get_state().value
    
    def is_idle(self) -> bool:
        """Check if operational is idle."""
        return self.get_state() == OperationalState.IDLE
    
    def is_ready(self) -> bool:
        """Check if operational is ready for tasks."""
        return self.get_state() == OperationalState.READY
    
    def is_running(self) -> bool:
        """Check if task is running."""
        return self.get_state() == OperationalState.RUNNING
    
    def is_processing(self) -> bool:
        """Check if background processing active."""
        return self.get_state() == OperationalState.PROCESSING
    
    def is_delegating(self) -> bool:
        """Check if delegating to other module."""
        return self.get_state() == OperationalState.DELEGATING
    
    def is_paused(self) -> bool:
        """Check if task is paused."""
        return self.get_state() == OperationalState.PAUSED
    
    def is_blocked(self) -> bool:
        """Check if operational is blocked."""
        return self.get_state() == OperationalState.BLOCKED
    
    def is_completed(self) -> bool:
        """Check if task is completed."""
        return self.get_state() == OperationalState.COMPLETED
    
    def can_start_task(self) -> bool:
        """Check if new task can be started."""
        return self.get_state() == OperationalState.READY and self.fsm.is_active()
    
    def is_busy(self) -> bool:
        """Check if actively working (running, processing, delegating)."""
        return self.get_state() in (
            OperationalState.RUNNING,
            OperationalState.PROCESSING,
            OperationalState.DELEGATING,
        )
    
    # -------------------------------------------------------------------------
    # Task Lifecycle
    # -------------------------------------------------------------------------
    
    def ready(self, metadata: Optional[Dict[str, Any]] = None) -> OperationalState:
        """
        Signal readiness for task execution.
        
        Transitions: Idle → Ready
        
        Args:
            metadata: Optional readiness metadata
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.READY, payload=metadata, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info("Operational ready for tasks")
        return self.get_state()
    
    def start_task(self, task_info: Optional[Dict[str, Any]] = None) -> OperationalState:
        """
        Start task execution.
        
        Transitions: Ready → Running
        
        Args:
            task_info: Task description and parameters
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.TASK_START, payload=task_info, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info(f"Task started: {task_info}")
        return self.get_state()
    
    def pause(self, reason: Optional[str] = None) -> OperationalState:
        """
        Pause current task.
        
        Transitions: Running → Paused
        
        Args:
            reason: Pause reason
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.TASK_PAUSE, payload={"reason": reason}, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info(f"Task paused: {reason}")
        return self.get_state()
    
    def resume(self) -> OperationalState:
        """
        Resume paused task.
        
        Transitions: Paused → Running
        
        Returns:
            New operational state
        """
        event = StateEvent(EventType.TASK_RESUME, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info("Task resumed")
        return self.get_state()
    
    def complete(self, result: Optional[Dict[str, Any]] = None) -> OperationalState:
        """
        Mark task as completed.
        
        Transitions: Running → Completed
        
        Args:
            result: Task results and metrics
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.TASK_COMPLETE, payload=result, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info(f"Task completed: {result}")
        return self.get_state()
    
    def reset(self) -> OperationalState:
        """
        Reset from Completed to Ready state.
        
        Transitions: Completed → Ready
        
        Returns:
            New operational state
        """
        event = StateEvent(EventType.AUTO_RESET, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info("Operational reset to ready")
        return self.get_state()
    
    # -------------------------------------------------------------------------
    # Special Operations
    # -------------------------------------------------------------------------
    
    def start_background_processing(self, process_info: Optional[Dict[str, Any]] = None) -> OperationalState:
        """
        Enter background processing mode.
        
        Transitions: Running → Processing
        
        Args:
            process_info: Processing task details
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.BACKGROUND_PROCESSING, payload=process_info, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info(f"Background processing started: {process_info}")
        return self.get_state()
    
    def complete_processing(self) -> OperationalState:
        """
        Complete background processing.
        
        Transitions: Processing → Running
        
        Returns:
            New operational state
        """
        event = StateEvent(EventType.PROCESSING_DONE, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info("Background processing completed")
        return self.get_state()
    
    def delegate_to_other(self, delegation_info: Optional[Dict[str, Any]] = None) -> OperationalState:
        """
        Delegate task to another module.
        
        Transitions: Running → Delegating
        
        Args:
            delegation_info: Target module and task details
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.DELEGATE_TO_OTHER, payload=delegation_info, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info(f"Task delegated: {delegation_info}")
        return self.get_state()
    
    def complete_delegation(self, result: Optional[Dict[str, Any]] = None) -> OperationalState:
        """
        Complete task delegation.
        
        Transitions: Delegating → Running
        
        Args:
            result: Delegation results
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.DELEGATE_DONE, payload=result, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info(f"Delegation completed: {result}")
        return self.get_state()
    
    def block(self, block_reason: Optional[str] = None) -> OperationalState:
        """
        Block operational execution.
        
        Transitions: Running → Blocked
        
        Args:
            block_reason: Reason for blocking
            
        Returns:
            New operational state
        """
        event = StateEvent(EventType.BLOCK_DETECTED, payload={"reason": block_reason}, origin_layer="operational")
        self.fsm.send_event(event)
        logger.warning(f"Operational blocked: {block_reason}")
        return self.get_state()
    
    def unblock(self) -> OperationalState:
        """
        Unblock operational execution.
        
        Transitions: Blocked → Running
        
        Returns:
            New operational state
        """
        event = StateEvent(EventType.UNBLOCK, origin_layer="operational")
        self.fsm.send_event(event)
        logger.info("Operational unblocked")
        return self.get_state()
    
    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    
    def on_state_change(self, callback, priority: int = 0):
        """
        Register callback for operational state changes.
        
        Args:
            callback: Function(layer, old_state, new_state)
            priority: Callback priority (higher = earlier execution)
        """
        self.fsm.subscribe("operational", callback, priority)
    
    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------
    
    def get_info(self) -> Dict[str, Any]:
        """
        Get operational layer information.
        
        Returns:
            Dictionary with state and capability info
        """
        return {
            "state": self.get_state_name(),
            "can_start_task": self.can_start_task(),
            "is_busy": self.is_busy(),
            "lifecycle_active": self.fsm.is_active(),
            "health_ok": self.fsm.is_healthy(),
        }
    
    def __repr__(self) -> str:
        """String representation."""
        return f"OperationalLayer(state={self.get_state_name()})"
