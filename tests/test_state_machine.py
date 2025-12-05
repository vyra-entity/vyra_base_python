"""
Test Suite for 3-Layer State Machine - Core Engine

Tests StateMachine class, state transitions, and layer interactions.
"""

import pytest
import threading
import time
from vyra_base.state.state_machine import (
    StateMachine,
    StateMachineConfig,
    StateMachineError,
    InvalidTransitionError,
    LayerViolationError,
    StateTransition,
)
from vyra_base.state.state_types import (
    LifecycleState,
    OperationalState,
    HealthState,
)
from vyra_base.state.state_events import StateEvent, EventType


class TestStateMachineInitialization:
    """Test StateMachine initialization and configuration."""
    
    def test_default_initialization(self):
        """Test state machine with default config."""
        fsm = StateMachine()
        
        assert fsm.get_lifecycle_state() == LifecycleState.UNINITIALIZED
        assert fsm.get_operational_state() == OperationalState.IDLE
        assert fsm.get_health_state() == HealthState.OK
    
    def test_custom_initial_states(self):
        """Test state machine with custom initial states."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.READY,
            initial_health=HealthState.WARNING,
        )
        fsm = StateMachine(config)
        
        assert fsm.get_lifecycle_state() == LifecycleState.ACTIVE
        assert fsm.get_operational_state() == OperationalState.READY
        assert fsm.get_health_state() == HealthState.WARNING
    
    def test_config_strict_mode(self):
        """Test strict mode configuration."""
        config_strict = StateMachineConfig(strict_mode=True)
        config_lenient = StateMachineConfig(strict_mode=False)
        
        fsm_strict = StateMachine(config_strict)
        fsm_lenient = StateMachine(config_lenient)
        
        # Try invalid transition
        invalid_event = StateEvent(EventType.SHUTDOWN)  # Can't shutdown from Uninitialized
        
        # Strict mode should raise error
        with pytest.raises(InvalidTransitionError):
            fsm_strict.send_event(invalid_event)
        
        # Lenient mode should not raise (just log warning)
        fsm_lenient.send_event(invalid_event)  # Should not raise


class TestLifecycleTransitions:
    """Test lifecycle layer transitions."""
    
    def test_standard_startup_sequence(self):
        """Test standard module startup."""
        fsm = StateMachine()
        
        # Start initialization
        fsm.send_event(StateEvent(EventType.START))
        assert fsm.get_lifecycle_state() == LifecycleState.INITIALIZING
        
        # Complete initialization
        fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
        assert fsm.get_lifecycle_state() == LifecycleState.ACTIVE
        assert fsm.is_active()
    
    def test_initialization_failure(self):
        """Test failed initialization recovery."""
        fsm = StateMachine()
        
        fsm.send_event(StateEvent(EventType.START))
        assert fsm.get_lifecycle_state() == LifecycleState.INITIALIZING
        
        # Fail initialization
        fsm.send_event(StateEvent(EventType.INIT_FAILURE))
        assert fsm.get_lifecycle_state() == LifecycleState.RECOVERING
    
    def test_shutdown_sequence(self):
        """Test controlled shutdown."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.ACTIVE)
        fsm = StateMachine(config)
        
        # Begin shutdown
        fsm.send_event(StateEvent(EventType.SHUTDOWN))
        assert fsm.get_lifecycle_state() == LifecycleState.SHUTTING_DOWN
        
        # Complete shutdown
        fsm.send_event(StateEvent(EventType.FINISHED))
        assert fsm.get_lifecycle_state() == LifecycleState.DEACTIVATED
    
    def test_recovery_success(self):
        """Test successful recovery from fault."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.RECOVERING)
        fsm = StateMachine(config)
        
        fsm.send_event(StateEvent(EventType.RECOVERY_SUCCESS))
        assert fsm.get_lifecycle_state() == LifecycleState.ACTIVE
    
    def test_recovery_failure(self):
        """Test failed recovery leads to deactivation."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.RECOVERING)
        fsm = StateMachine(config)
        
        fsm.send_event(StateEvent(EventType.RECOVERY_FAILED))
        assert fsm.get_lifecycle_state() == LifecycleState.DEACTIVATED


class TestOperationalTransitions:
    """Test operational layer transitions."""
    
    def test_task_execution_cycle(self):
        """Test complete task execution cycle."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.IDLE,
        )
        fsm = StateMachine(config)
        
        # Ready for tasks
        fsm.send_event(StateEvent(EventType.READY))
        assert fsm.get_operational_state() == OperationalState.READY
        
        # Start task
        fsm.send_event(StateEvent(EventType.TASK_START))
        assert fsm.get_operational_state() == OperationalState.RUNNING
        
        # Complete task
        fsm.send_event(StateEvent(EventType.TASK_COMPLETE))
        assert fsm.get_operational_state() == OperationalState.COMPLETED
        
        # Reset to ready
        fsm.send_event(StateEvent(EventType.AUTO_RESET))
        assert fsm.get_operational_state() == OperationalState.READY
    
    def test_pause_resume(self):
        """Test task pause and resume."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)
        
        # Pause
        fsm.send_event(StateEvent(EventType.TASK_PAUSE))
        assert fsm.get_operational_state() == OperationalState.PAUSED
        
        # Resume
        fsm.send_event(StateEvent(EventType.TASK_RESUME))
        assert fsm.get_operational_state() == OperationalState.RUNNING
    
    def test_background_processing(self):
        """Test background processing state."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)
        
        # Enter processing
        fsm.send_event(StateEvent(EventType.BACKGROUND_PROCESSING))
        assert fsm.get_operational_state() == OperationalState.PROCESSING
        
        # Complete processing
        fsm.send_event(StateEvent(EventType.PROCESSING_DONE))
        assert fsm.get_operational_state() == OperationalState.RUNNING
    
    def test_delegation(self):
        """Test task delegation."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)
        
        # Delegate
        fsm.send_event(StateEvent(EventType.DELEGATE_TO_OTHER))
        assert fsm.get_operational_state() == OperationalState.DELEGATING
        
        # Complete delegation
        fsm.send_event(StateEvent(EventType.DELEGATE_DONE))
        assert fsm.get_operational_state() == OperationalState.RUNNING
    
    def test_blocking(self):
        """Test operational blocking."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)
        
        # Block
        fsm.send_event(StateEvent(EventType.BLOCK_DETECTED))
        assert fsm.get_operational_state() == OperationalState.BLOCKED
        
        # Unblock
        fsm.send_event(StateEvent(EventType.UNBLOCK))
        assert fsm.get_operational_state() == OperationalState.RUNNING


class TestHealthTransitions:
    """Test health layer transitions."""
    
    def test_warning_sequence(self):
        """Test warning detection and clearance."""
        fsm = StateMachine()
        
        # Report warning
        fsm.send_event(StateEvent(EventType.WARN))
        assert fsm.get_health_state() == HealthState.WARNING
        assert not fsm.is_healthy()
        
        # Clear warning
        fsm.send_event(StateEvent(EventType.CLEAR_WARNING))
        assert fsm.get_health_state() == HealthState.OK
        assert fsm.is_healthy()
    
    def test_overload_sequence(self):
        """Test overload detection and recovery."""
        config = StateMachineConfig(initial_health=HealthState.WARNING)
        fsm = StateMachine(config)
        
        # Report overload
        fsm.send_event(StateEvent(EventType.OVERLOAD))
        assert fsm.get_health_state() == HealthState.OVERLOADED
        
        # Reduce load
        fsm.send_event(StateEvent(EventType.LOAD_REDUCED))
        assert fsm.get_health_state() == HealthState.WARNING
    
    def test_fault_escalation(self):
        """Test fault detection and escalation."""
        config = StateMachineConfig(initial_health=HealthState.WARNING)
        fsm = StateMachine(config)
        
        # Report fault
        fsm.send_event(StateEvent(EventType.FAULT))
        assert fsm.get_health_state() == HealthState.FAULTED
        
        # Escalate to critical
        fsm.send_event(StateEvent(EventType.ESCALATE))
        assert fsm.get_health_state() == HealthState.CRITICAL
    
    def test_fault_recovery(self):
        """Test recovery from faulted state."""
        config = StateMachineConfig(initial_health=HealthState.FAULTED)
        fsm = StateMachine(config)
        
        # Recover
        fsm.send_event(StateEvent(EventType.RECOVER))
        assert fsm.get_health_state() == HealthState.OK


class TestLayerInteractions:
    """Test interactions between layers."""
    
    def test_health_escalates_lifecycle(self):
        """Test health faults escalate to lifecycle recovery."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_health=HealthState.WARNING,
        )
        fsm = StateMachine(config)
        
        # Health fault should trigger lifecycle recovery
        fsm.send_event(StateEvent(EventType.FAULT))
        assert fsm.get_health_state() == HealthState.FAULTED
        assert fsm.get_lifecycle_state() == LifecycleState.RECOVERING
    
    def test_critical_health_forces_shutdown(self):
        """Test critical health forces lifecycle shutdown."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_health=HealthState.FAULTED,
        )
        fsm = StateMachine(config)
        
        # Escalate to critical
        fsm.send_event(StateEvent(EventType.ESCALATE))
        assert fsm.get_health_state() == HealthState.CRITICAL
        assert fsm.get_lifecycle_state() == LifecycleState.SHUTTING_DOWN
    
    def test_lifecycle_controls_operational(self):
        """Test lifecycle states control operational capabilities."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)
        
        # Shutdown lifecycle
        fsm.send_event(StateEvent(EventType.SHUTDOWN))
        assert fsm.get_lifecycle_state() == LifecycleState.SHUTTING_DOWN
        # Operational should be paused
        assert fsm.get_operational_state() == OperationalState.PAUSED
    
    def test_overload_pauses_operational(self):
        """Test overload health pauses operational."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
            initial_health=HealthState.WARNING,
        )
        fsm = StateMachine(config)
        
        # Report overload
        fsm.send_event(StateEvent(EventType.OVERLOAD))
        assert fsm.get_health_state() == HealthState.OVERLOADED
        assert fsm.get_operational_state() == OperationalState.PAUSED


class TestInterruptEvents:
    """Test interrupt event handling."""
    
    def test_emergency_stop(self):
        """Test emergency stop affects all layers."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
            initial_health=HealthState.OK,
        )
        fsm = StateMachine(config)
        
        # Emergency stop
        fsm.send_event(StateEvent(EventType.EMERGENCY_STOP))
        
        # All layers should be affected
        assert fsm.get_lifecycle_state() == LifecycleState.DEACTIVATED
        assert fsm.get_operational_state() == OperationalState.IDLE
        assert fsm.get_health_state() == HealthState.FAULTED
    
    def test_interrupt_pauses_running(self):
        """Test interrupt pauses running task."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)
        
        # Interrupt
        fsm.send_event(StateEvent(EventType.INTERRUPT))
        assert fsm.get_operational_state() == OperationalState.PAUSED


class TestCallbacks:
    """Test callback subscription and notification."""
    
    def test_lifecycle_callback(self):
        """Test lifecycle state change callback."""
        fsm = StateMachine()
        
        callback_data = []
        def callback(layer, old_state, new_state):
            callback_data.append((layer, old_state, new_state))
        
        fsm.subscribe("lifecycle", callback)
        
        # Trigger transition
        fsm.send_event(StateEvent(EventType.START))
        
        assert len(callback_data) == 1
        assert callback_data[0][0] == "lifecycle"
        assert callback_data[0][1] == "Uninitialized"
        assert callback_data[0][2] == "Initializing"
    
    def test_multiple_callbacks(self):
        """Test multiple callbacks on same layer."""
        fsm = StateMachine()
        
        calls1 = []
        calls2 = []
        
        fsm.subscribe("lifecycle", lambda l, o, n: calls1.append(n))
        fsm.subscribe("lifecycle", lambda l, o, n: calls2.append(n))
        
        fsm.send_event(StateEvent(EventType.START))
        
        assert len(calls1) == 1
        assert len(calls2) == 1
    
    def test_callback_priority(self):
        """Test callback execution order by priority."""
        fsm = StateMachine()
        
        call_order = []
        
        fsm.subscribe("lifecycle", lambda l, o, n: call_order.append("low"), priority=0)
        fsm.subscribe("lifecycle", lambda l, o, n: call_order.append("high"), priority=10)
        fsm.subscribe("lifecycle", lambda l, o, n: call_order.append("medium"), priority=5)
        
        fsm.send_event(StateEvent(EventType.START))
        
        # Should be called in priority order (high to low)
        assert call_order == ["high", "medium", "low"]
    
    def test_global_callback(self):
        """Test global callback receives all state changes."""
        fsm = StateMachine()
        
        callbacks = []
        fsm.subscribe("any", lambda l, o, n: callbacks.append(l))
        
        # Trigger different layer changes
        fsm.send_event(StateEvent(EventType.START))  # lifecycle
        fsm.send_event(StateEvent(EventType.WARN))  # health
        
        assert "lifecycle" in callbacks
        assert "health" in callbacks


class TestHistory:
    """Test state transition history."""
    
    def test_history_recording(self):
        """Test transitions are recorded in history."""
        fsm = StateMachine()
        
        fsm.send_event(StateEvent(EventType.START))
        history = fsm.get_history()
        
        assert len(history) > 0
        assert isinstance(history[0], StateTransition)
    
    def test_history_limit(self):
        """Test history size limit."""
        config = StateMachineConfig(max_history_size=5)
        fsm = StateMachine(config)
        
        # Generate more transitions than limit
        for _ in range(10):
            fsm.send_event(StateEvent(EventType.WARN))
            fsm.send_event(StateEvent(EventType.CLEAR_WARNING))
        
        history = fsm.get_history()
        assert len(history) <= 5
    
    def test_history_clear(self):
        """Test clearing history."""
        fsm = StateMachine()
        
        fsm.send_event(StateEvent(EventType.START))
        assert len(fsm.get_history()) > 0
        
        fsm.clear_history()
        assert len(fsm.get_history()) == 0


class TestThreadSafety:
    """Test thread-safe operation."""
    
    def test_concurrent_state_queries(self):
        """Test concurrent state queries are safe."""
        fsm = StateMachine()
        results = []
        
        def query_state():
            for _ in range(100):
                state = fsm.get_current_state()
                results.append(state)
        
        threads = [threading.Thread(target=query_state) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # All queries should succeed
        assert len(results) == 500
    
    def test_concurrent_transitions(self):
        """Test concurrent event sending is safe."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.ACTIVE)
        fsm = StateMachine(config)
        
        def send_events():
            for _ in range(10):
                fsm.send_event(StateEvent(EventType.WARN))
                time.sleep(0.001)
                fsm.send_event(StateEvent(EventType.CLEAR_WARNING))
        
        threads = [threading.Thread(target=send_events) for _ in range(3)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # Final state should be valid
        state = fsm.get_health_state()
        assert state in (HealthState.OK, HealthState.WARNING)


class TestDiagnostics:
    """Test diagnostic information."""
    
    def test_diagnostic_info(self):
        """Test get_diagnostic_info returns complete information."""
        fsm = StateMachine()
        
        info = fsm.get_diagnostic_info()
        
        assert "current_state" in info
        assert "is_active" in info
        assert "is_operational" in info
        assert "is_healthy" in info
        assert "config" in info
        assert "callbacks" in info
    
    def test_is_operational(self):
        """Test is_operational checks lifecycle and health."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_health=HealthState.OK,
        )
        fsm = StateMachine(config)
        
        assert fsm.is_operational()
        
        # Degrade health
        fsm.send_event(StateEvent(EventType.FAULT))
        assert not fsm.is_operational()
