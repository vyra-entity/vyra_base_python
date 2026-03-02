"""
Integration module for easy unified state machine access.

Provides a simple API that combines all three layers into a single
unified interface for module state management.
"""

import logging
from typing import Optional, Dict, Any, Callable

from .state_machine import StateMachine, StateMachineConfig
from .lifecycle_layer import LifecycleLayer
from .operational_layer import OperationalLayer
from .health_layer import HealthLayer
from .state_types import LifecycleState, OperationalState, HealthState


logger = logging.getLogger(__name__)


class UnifiedStateMachine:
    """
    Unified interface for 3-layer state machine.
    
    Combines Lifecycle, Operational, and Health layers into a single
    easy-to-use API. Recommended for most use cases.
    
    Example:
        >>> usm = UnifiedStateMachine()
        >>> usm.start()  # Lifecycle
        >>> usm.complete_initialization()
        >>> usm.set_ready()  # Operational
        >>> usm.start_task({'task_id': '123'})
        >>> usm.report_warning({'cpu': '85%'})  # Health
        >>> usm.get_all_states()
        {'lifecycle': 'Active', 'operational': 'Running', 'health': 'Warning'}
    """
    
    def __init__(self, config: Optional[StateMachineConfig] = None):
        """
        Initialize unified state machine.
        
        Args:
            config: Configuration for state machine behavior
        """
        # Create core FSM
        self.fsm = StateMachine(config)
        
        # Create layer interfaces
        self.lifecycle = LifecycleLayer(self.fsm)
        self.operational = OperationalLayer(self.fsm)
        self.health = HealthLayer(self.fsm)
        
        logger.info("UnifiedStateMachine initialized")
    
    # -------------------------------------------------------------------------
    # Unified State Query
    # -------------------------------------------------------------------------
    
    def get_all_states(self) -> Dict[str, str]:
        """Get current state of all layers."""
        return self.fsm.get_current_state()
    
    def get_lifecycle_state(self) -> LifecycleState:
        """Get lifecycle state."""
        return self.lifecycle.get_state()
    
    def get_operational_state(self) -> OperationalState:
        """Get operational state."""
        return self.operational.get_state()
    
    def get_health_state(self) -> HealthState:
        """Get health state."""
        return self.health.get_state()
    
    def is_operational(self) -> bool:
        """Check if module can execute tasks."""
        return self.fsm.is_operational()
    
    def is_healthy(self) -> bool:
        """Check if module health is OK."""
        return self.fsm.is_healthy()

    # Lifecycle state shortcuts
    def is_initializing(self) -> bool:
        """Check if module is initializing."""
        return self.lifecycle.is_initializing()

    def is_active(self) -> bool:
        """Check if module lifecycle is Active."""
        return self.lifecycle.is_active()

    def is_suspended(self) -> bool:
        """Check if module is suspended."""
        return self.lifecycle.is_suspended()

    def is_recovering(self) -> bool:
        """Check if module is in recovery."""
        return self.lifecycle.is_recovering()

    def is_shutting_down(self) -> bool:
        """Check if module is shutting down."""
        return self.lifecycle.is_shutting_down()

    def is_offline(self) -> bool:
        """Check if module is offline."""
        return self.lifecycle.is_offline()
    
    # -------------------------------------------------------------------------
    # Lifecycle Methods (delegated)
    # -------------------------------------------------------------------------
    
    def start(self, metadata: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """Start module initialization."""
        return self.lifecycle.start(metadata)
    
    def complete_initialization(self, result: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """Complete initialization successfully."""
        return self.lifecycle.complete_initialization(result)
    
    def fail_initialization(self, error: Optional[str] = None) -> LifecycleState:
        """Mark initialization as failed."""
        return self.lifecycle.fail_initialization(error)

    def shutdown(self, reason: Optional[str] = None) -> LifecycleState:
        """Begin controlled shutdown."""
        return self.lifecycle.shutdown(reason)
    
    def complete_shutdown(self) -> LifecycleState:
        """Complete shutdown process."""
        return self.lifecycle.complete_shutdown()

    def suspend(self, reason: Optional[str] = None) -> LifecycleState:
        """
        Suspend the module temporarily (e.g. for maintenance or updates).

        Transitions: Active → Suspended

        Args:
            reason: Optional reason for suspension

        Returns:
            New lifecycle state
        """
        return self.lifecycle.enter_suspend(reason)

    def resume_from_suspend(self, info: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Resume module from Suspended state.

        Transitions: Suspended → Active

        Args:
            info: Optional context about the resume

        Returns:
            New lifecycle state
        """
        return self.lifecycle.resume_from_suspend(info)

    def enter_recovery(self, fault_info: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Enter recovery mode due to fault.

        Transitions: Active → Recovering

        Args:
            fault_info: Fault description and context

        Returns:
            New lifecycle state
        """
        return self.lifecycle.enter_recovery(fault_info)

    def complete_recovery(self, recovery_info: Optional[Dict[str, Any]] = None) -> LifecycleState:
        """
        Complete recovery successfully.

        Transitions: Recovering → Active

        Args:
            recovery_info: Recovery details

        Returns:
            New lifecycle state
        """
        return self.lifecycle.complete_recovery(recovery_info)

    def fail_recovery(self, error: Optional[str] = None) -> LifecycleState:
        """
        Mark recovery as failed.

        Transitions: Recovering → ShuttingDown

        Args:
            error: Recovery failure reason

        Returns:
            New lifecycle state
        """
        return self.lifecycle.fail_recovery(error)
    
    # -------------------------------------------------------------------------
    # Operational Methods (delegated)
    # -------------------------------------------------------------------------
    
    def set_ready(self, metadata: Optional[Dict[str, Any]] = None) -> OperationalState:
        """Signal readiness for tasks."""
        return self.operational.set_ready(metadata)
    
    def start_task(self, task_info: Optional[Dict[str, Any]] = None) -> OperationalState:
        """Start task execution."""
        return self.operational.start_task(task_info)
    
    def pause(self, reason: Optional[str] = None) -> OperationalState:
        """Pause current task."""
        return self.operational.pause(reason)
    
    def resume(self) -> OperationalState:
        """Resume paused task."""
        return self.operational.resume()
    
    def stop(self, result: Optional[Dict[str, Any]] = None) -> OperationalState:
        """Mark task as completed."""
        return self.operational.stop(result)
    
    def reset(self) -> OperationalState:
        """Reset operational state to Idle."""
        return self.operational.reset()
    
    # -------------------------------------------------------------------------
    # Health Methods (delegated)
    # -------------------------------------------------------------------------
    
    def report_warning(self, warning_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """Report non-critical warning."""
        return self.health.report_warning(warning_info)
    
    def report_fault(self, fault_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """Report critical fault."""
        return self.health.report_fault(fault_info)
    
    def recover(self, recovery_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """Attempt recovery from fault."""
        return self.health.recover(recovery_info)
    
    def emergency_stop(self, reason: str) -> Dict[str, str]:
        """Trigger emergency stop."""
        return self.health.emergency_stop(reason)
    
    # -------------------------------------------------------------------------
    # Callbacks and Monitoring
    # -------------------------------------------------------------------------
    
    def on_lifecycle_change(self, callback: Callable, priority: int = 0):
        """Subscribe to lifecycle state changes."""
        self.lifecycle.on_state_change(callback, priority)
    
    def on_operational_change(self, callback: Callable, priority: int = 0):
        """Subscribe to operational state changes."""
        self.operational.on_state_change(callback, priority)
    
    def on_health_change(self, callback: Callable, priority: int = 0):
        """Subscribe to health state changes."""
        self.health.on_state_change(callback, priority)
    
    def on_any_change(self, callback: Callable, priority: int = 0):
        """Subscribe to any state change across all layers."""
        self.fsm.subscribe("any", callback, priority)
    
    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------
    
    def get_diagnostic_info(self) -> Dict[str, Any]:
        """Get comprehensive diagnostic information."""
        return {
            "states": self.get_all_states(),
            "lifecycle_info": self.lifecycle.get_info(),
            "operational_info": self.operational.get_info(),
            "health_info": self.health.get_info(),
            "fsm_diagnostics": self.fsm.get_diagnostic_info(),
        }
    
    def get_history(self, limit: Optional[int] = None):
        """Get state transition history."""
        return self.fsm.get_history(limit)
    
    def clear_history(self):
        """Clear transition history."""
        self.fsm.clear_history()
    
    # -------------------------------------------------------------------------
    # Event Handling (for OperationalStateMachine compatibility)
    # -------------------------------------------------------------------------
    
    def send_event(self, event):
        """
        Send an event to the state machine.
        
        This method is required for compatibility with OperationalStateMachine
        which calls _state_machine.send_event(event).
        
        Args:
            event: StateEvent to send to the FSM
        """
        self.fsm.send_event(event)
    
    # -------------------------------------------------------------------------
    # Standard Startup Sequence
    # -------------------------------------------------------------------------
    
    def standard_startup(self) -> bool:
        """
        Execute standard module startup sequence.
        
        Sequence:
        1. Start initialization
        2. Complete initialization
        3. Set operational ready
        
        Returns:
            True if startup successful, False otherwise
        """
        try:
            self.start()
            logger.info("Startup: Initializing...")
            
            # Allow custom initialization logic here
            # In real implementation, this would initialize resources
            
            self.complete_initialization()
            logger.info("Startup: Initialization complete")
            
            self.set_ready()
            logger.info("Startup: Ready for tasks")
            
            return True
            
        except Exception as e:
            logger.error(f"Startup failed: {e}")
            self.fail_initialization(str(e))
            return False
    
    def standard_shutdown(self) -> bool:
        """
        Execute standard module shutdown sequence.
        
        Sequence:
        1. Begin shutdown
        2. Complete any pending tasks (module responsibility)
        3. Complete shutdown
        
        Returns:
            True if shutdown successful, False otherwise
        """
        try:
            self.shutdown("Standard shutdown")
            logger.info("Shutdown: Initiated")
            
            # Allow cleanup logic here
            # In real implementation, this would clean up resources
            
            self.complete_shutdown()
            logger.info("Shutdown: Complete")
            
            return True
            
        except Exception as e:
            logger.error(f"Shutdown failed: {e}")
            return False
    
    def __repr__(self) -> str:
        """String representation."""
        states = self.get_all_states()
        return f"UnifiedStateMachine(lifecycle={states['lifecycle']}, operational={states['operational']}, health={states['health']})"
