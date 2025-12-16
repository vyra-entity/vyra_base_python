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
        >>> usm.ready()  # Operational
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
