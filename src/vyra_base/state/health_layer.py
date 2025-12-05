"""
Health Layer - High-level API for system health monitoring and management.

This layer monitors system integrity:
- Health status tracking (OK, Warning, Overloaded, Faulted, Critical)
- Warning and fault reporting
- Load management
- Recovery coordination
- Escalation to lifecycle layer

Thread-safe wrapper around StateMachine for health management.
"""

import logging
from typing import Optional, Dict, Any

from .state_machine import StateMachine
from .state_types import HealthState
from .state_events import StateEvent, EventType


logger = logging.getLogger(__name__)


class HealthLayer:
    """
    High-level API for health state management.
    
    Provides intuitive methods for health monitoring:
    - report_warning() - Report non-critical issues
    - report_fault() - Report critical faults
    - report_overload() - Report resource overload
    - recover() - Attempt recovery
    - escalate() - Escalate to critical state
    
    Example:
        >>> health = HealthLayer(fsm)
        >>> health.report_warning({'cpu': '85%'})
        >>> health.is_degraded()
        True
        >>> health.clear_warning()
    """
    
    def __init__(self, fsm: StateMachine):
        """
        Initialize health layer.
        
        Args:
            fsm: StateMachine instance to control
        """
        self.fsm = fsm
        logger.info("HealthLayer initialized")
    
    # -------------------------------------------------------------------------
    # State Query
    # -------------------------------------------------------------------------
    
    def get_state(self) -> HealthState:
        """Get current health state."""
        return self.fsm.get_health_state()
    
    def get_state_name(self) -> str:
        """Get current health state as string."""
        return self.get_state().value
    
    def is_ok(self) -> bool:
        """Check if health is OK."""
        return self.get_state() == HealthState.OK
    
    def is_warning(self) -> bool:
        """Check if there are warnings."""
        return self.get_state() == HealthState.WARNING
    
    def is_overloaded(self) -> bool:
        """Check if system is overloaded."""
        return self.get_state() == HealthState.OVERLOADED
    
    def is_faulted(self) -> bool:
        """Check if system has faults."""
        return self.get_state() == HealthState.FAULTED
    
    def is_critical(self) -> bool:
        """Check if health is critical."""
        return self.get_state() == HealthState.CRITICAL
    
    def is_degraded(self) -> bool:
        """Check if health is degraded (warning or worse)."""
        return self.get_state() in (
            HealthState.WARNING,
            HealthState.OVERLOADED,
            HealthState.FAULTED,
            HealthState.CRITICAL,
        )
    
    def is_operational_safe(self) -> bool:
        """Check if safe for operational tasks."""
        return self.get_state() in (HealthState.OK, HealthState.WARNING)
    
    # -------------------------------------------------------------------------
    # Health Reporting
    # -------------------------------------------------------------------------
    
    def report_warning(self, warning_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """
        Report non-critical warning.
        
        Transitions: OK → Warning
        
        Args:
            warning_info: Warning details (metrics, thresholds, etc.)
            
        Returns:
            New health state
        """
        event = StateEvent(EventType.WARN, payload=warning_info, origin_layer="health")
        self.fsm.send_event(event)
        logger.warning(f"Warning reported: {warning_info}")
        return self.get_state()
    
    def clear_warning(self, clearance_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """
        Clear active warnings.
        
        Transitions: Warning → OK
        
        Args:
            clearance_info: Clearance details
            
        Returns:
            New health state
        """
        event = StateEvent(EventType.CLEAR_WARNING, payload=clearance_info, origin_layer="health")
        self.fsm.send_event(event)
        logger.info(f"Warning cleared: {clearance_info}")
        return self.get_state()
    
    def report_overload(self, load_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """
        Report resource overload.
        
        Transitions: Warning → Overloaded
        
        Args:
            load_info: Resource usage details (CPU, memory, etc.)
            
        Returns:
            New health state
        """
        event = StateEvent(EventType.OVERLOAD, payload=load_info, origin_layer="health")
        self.fsm.send_event(event)
        logger.warning(f"Overload reported: {load_info}")
        return self.get_state()
    
    def reduce_load(self, reduction_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """
        Report load reduction.
        
        Transitions: Overloaded → Warning
        
        Args:
            reduction_info: Load reduction details
            
        Returns:
            New health state
        """
        event = StateEvent(EventType.LOAD_REDUCED, payload=reduction_info, origin_layer="health")
        self.fsm.send_event(event)
        logger.info(f"Load reduced: {reduction_info}")
        return self.get_state()
    
    def report_fault(self, fault_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """
        Report critical fault.
        
        Transitions: Warning/Overloaded → Faulted
        Note: Triggers lifecycle escalation to Recovering state
        
        Args:
            fault_info: Fault details and diagnostics
            
        Returns:
            New health state
        """
        event = StateEvent(EventType.FAULT, payload=fault_info, origin_layer="health")
        self.fsm.send_event(event)
        logger.error(f"Fault reported: {fault_info}")
        return self.get_state()
    
    # -------------------------------------------------------------------------
    # Recovery and Escalation
    # -------------------------------------------------------------------------
    
    def recover(self, recovery_info: Optional[Dict[str, Any]] = None) -> HealthState:
        """
        Attempt recovery from faulted state.
        
        Transitions: Faulted → OK
        
        Args:
            recovery_info: Recovery actions taken
            
        Returns:
            New health state
        """
        event = StateEvent(EventType.RECOVER, payload=recovery_info, origin_layer="health")
        self.fsm.send_event(event)
        logger.info(f"Recovery successful: {recovery_info}")
        return self.get_state()
    
    def escalate(self, escalation_reason: Optional[str] = None) -> HealthState:
        """
        Escalate fault to critical state.
        
        Transitions: Faulted → Critical
        Note: Triggers lifecycle shutdown
        
        Args:
            escalation_reason: Reason for escalation
            
        Returns:
            New health state
        """
        event = StateEvent(
            EventType.ESCALATE,
            payload={"reason": escalation_reason},
            origin_layer="health"
        )
        self.fsm.send_event(event)
        logger.critical(f"Health escalated to critical: {escalation_reason}")
        return self.get_state()
    
    def reset_from_critical(self) -> HealthState:
        """
        Reset from critical to faulted state.
        
        Transitions: Critical → Faulted
        
        Returns:
            New health state
        """
        event = StateEvent(EventType.RESET, origin_layer="health")
        self.fsm.send_event(event)
        logger.info("Critical state reset to faulted")
        return self.get_state()
    
    # -------------------------------------------------------------------------
    # Convenience Methods
    # -------------------------------------------------------------------------
    
    def check_and_report(
        self,
        metrics: Dict[str, Any],
        warning_threshold: Optional[float] = None,
        overload_threshold: Optional[float] = None,
        fault_threshold: Optional[float] = None,
    ) -> HealthState:
        """
        Check metrics and automatically report appropriate health state.
        
        Args:
            metrics: System metrics to evaluate
            warning_threshold: Threshold for warning state
            overload_threshold: Threshold for overload state
            fault_threshold: Threshold for fault state
            
        Returns:
            New health state after evaluation
            
        Example:
            >>> health.check_and_report(
            ...     {'cpu_usage': 0.85},
            ...     warning_threshold=0.7,
            ...     overload_threshold=0.9
            ... )
        """
        # Extract numeric value for comparison (simplified example)
        value = next((v for v in metrics.values() if isinstance(v, (int, float))), None)
        
        if value is None:
            logger.debug("No numeric metrics for threshold check")
            return self.get_state()
        
        if fault_threshold and value >= fault_threshold:
            return self.report_fault(metrics)
        elif overload_threshold and value >= overload_threshold:
            return self.report_overload(metrics)
        elif warning_threshold and value >= warning_threshold:
            return self.report_warning(metrics)
        elif self.is_warning():
            # Metrics below warning threshold - clear if currently warning
            return self.clear_warning(metrics)
        
        return self.get_state()
    
    def emergency_stop(self, reason: str) -> Dict[str, str]:
        """
        Trigger emergency stop (affects all layers).
        
        This is an interrupt event that immediately:
        - Sets lifecycle to Deactivated
        - Sets operational to Idle
        - Sets health to Faulted
        
        Args:
            reason: Emergency stop reason
            
        Returns:
            New state of all layers
        """
        event = StateEvent(
            EventType.EMERGENCY_STOP,
            payload={"reason": reason},
            origin_layer="health"
        )
        return self.fsm.send_event(event)
    
    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    
    def on_state_change(self, callback, priority: int = 0):
        """
        Register callback for health state changes.
        
        Args:
            callback: Function(layer, old_state, new_state)
            priority: Callback priority (higher = earlier execution)
        """
        self.fsm.subscribe("health", callback, priority)
    
    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------
    
    def get_info(self) -> Dict[str, Any]:
        """
        Get health layer information.
        
        Returns:
            Dictionary with state and status info
        """
        return {
            "state": self.get_state_name(),
            "is_ok": self.is_ok(),
            "is_degraded": self.is_degraded(),
            "is_operational_safe": self.is_operational_safe(),
            "lifecycle_state": self.fsm.get_lifecycle_state().value,
        }
    
    def get_severity_level(self) -> int:
        """
        Get health severity as numeric level.
        
        Returns:
            0 = OK, 1 = Warning, 2 = Overloaded, 3 = Faulted, 4 = Critical
        """
        severity_map = {
            HealthState.OK: 0,
            HealthState.WARNING: 1,
            HealthState.OVERLOADED: 2,
            HealthState.FAULTED: 3,
            HealthState.CRITICAL: 4,
        }
        return severity_map.get(self.get_state(), 0)
    
    def __repr__(self) -> str:
        """String representation."""
        return f"HealthLayer(state={self.get_state_name()}, severity={self.get_severity_level()})"
