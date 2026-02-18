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

        assert fsm.get_lifecycle_state() == LifecycleState.OFFLINE
        assert fsm.get_operational_state() == OperationalState.IDLE
        assert fsm.get_health_state() == HealthState.HEALTHY

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

        # Try invalid transition: TASK_START from IDLE (no handler returns None, not an error)
        # Use INIT_FAILURE from ACTIVE which is not a valid transition
        config_active_strict = StateMachineConfig(
            strict_mode=True, initial_lifecycle=LifecycleState.ACTIVE
        )
        fsm_active_strict = StateMachine(config_active_strict)

        # INIT_SUCCESS from ACTIVE has no transition → strict mode should raise
        with pytest.raises(InvalidTransitionError):
            fsm_active_strict.send_event(StateEvent(EventType.INIT_SUCCESS))

        # Lenient mode should not raise
        config_active_lenient = StateMachineConfig(
            strict_mode=False, initial_lifecycle=LifecycleState.ACTIVE
        )
        fsm_active_lenient = StateMachine(config_active_lenient)
        fsm_active_lenient.send_event(StateEvent(EventType.INIT_SUCCESS))  # Should not raise


class TestLifecycleTransitions:
    """Test lifecycle layer transitions."""

    def test_standard_startup_sequence(self):
        """Test standard module startup."""
        fsm = StateMachine()

        fsm.send_event(StateEvent(EventType.START))
        assert fsm.get_lifecycle_state() == LifecycleState.INITIALIZING

        fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
        assert fsm.get_lifecycle_state() == LifecycleState.ACTIVE
        assert fsm.is_active()

    def test_initialization_failure(self):
        """Test failed initialization recovery."""
        fsm = StateMachine()

        fsm.send_event(StateEvent(EventType.START))
        assert fsm.get_lifecycle_state() == LifecycleState.INITIALIZING

        fsm.send_event(StateEvent(EventType.INIT_FAILURE))
        assert fsm.get_lifecycle_state() == LifecycleState.RECOVERING

    def test_shutdown_sequence(self):
        """Test controlled shutdown."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.ACTIVE)
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.SHUTDOWN))
        assert fsm.get_lifecycle_state() == LifecycleState.SHUTTING_DOWN

        fsm.send_event(StateEvent(EventType.FINISHED))
        assert fsm.get_lifecycle_state() == LifecycleState.OFFLINE

    def test_recovery_success(self):
        """Test successful recovery from fault."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.RECOVERING)
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.RECOVERY_SUCCESS))
        assert fsm.get_lifecycle_state() == LifecycleState.ACTIVE

    def test_recovery_failure(self):
        """Test failed recovery leads to shutting down."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.RECOVERING)
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.RECOVERY_FAILED))
        assert fsm.get_lifecycle_state() == LifecycleState.SHUTTING_DOWN

    def test_suspend_sequence(self):
        """Test suspend sequence: ACTIVE -> SUSPENDED, resume -> ACTIVE, shutdown -> OFFLINE."""
        config = StateMachineConfig(initial_lifecycle=LifecycleState.ACTIVE)
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.SET_SUSPENDED))
        assert fsm.get_lifecycle_state() == LifecycleState.SUSPENDED

        # Resume from suspended goes back to ACTIVE
        fsm.send_event(StateEvent(EventType.RESUME_SUSPENDED))
        assert fsm.get_lifecycle_state() == LifecycleState.ACTIVE

        # Normal shutdown to offline
        fsm.send_event(StateEvent(EventType.SHUTDOWN))
        fsm.send_event(StateEvent(EventType.FINISHED))
        assert fsm.get_lifecycle_state() == LifecycleState.OFFLINE


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
        fsm.send_event(StateEvent(EventType.SET_READY))
        assert fsm.get_operational_state() == OperationalState.READY

        # Start task
        fsm.send_event(StateEvent(EventType.TASK_START))
        assert fsm.get_operational_state() == OperationalState.RUNNING

        # Complete task -> back to READY
        fsm.send_event(StateEvent(EventType.TASK_COMPLETE))
        assert fsm.get_operational_state() == OperationalState.READY

        # Stop
        fsm.send_event(StateEvent(EventType.TASK_STOP))
        assert fsm.get_operational_state() == OperationalState.STOPPED

        # Reset to IDLE
        fsm.send_event(StateEvent(EventType.TASK_RESET))
        assert fsm.get_operational_state() == OperationalState.IDLE

    def test_pause_resume(self):
        """Test task pause and resume."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.TASK_PAUSE))
        assert fsm.get_operational_state() == OperationalState.PAUSED

        fsm.send_event(StateEvent(EventType.TASK_RESUME))
        assert fsm.get_operational_state() == OperationalState.READY

    def test_task_error(self):
        """Test task error transition."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.TASK_ERROR))
        assert fsm.get_operational_state() == OperationalState.ERROR

        # Reset from ERROR
        fsm.send_event(StateEvent(EventType.TASK_RESET))
        assert fsm.get_operational_state() == OperationalState.IDLE


class TestHealthTransitions:
    """Test health layer transitions."""

    def test_warning_sequence(self):
        """Test warning detection and clearance."""
        fsm = StateMachine()

        fsm.send_event(StateEvent(EventType.WARN))
        assert fsm.get_health_state() == HealthState.WARNING
        assert not fsm.is_healthy()

        fsm.send_event(StateEvent(EventType.CLEAR_WARNING))
        assert fsm.get_health_state() == HealthState.HEALTHY
        assert fsm.is_healthy()

    def test_fault_to_critical(self):
        """Test fault moves health to CRITICAL."""
        fsm = StateMachine()

        fsm.send_event(StateEvent(EventType.FAULT))
        assert fsm.get_health_state() == HealthState.CRITICAL

    def test_fault_recovery(self):
        """Test recovery from CRITICAL state."""
        config = StateMachineConfig(initial_health=HealthState.CRITICAL)
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.RECOVER))
        assert fsm.get_health_state() == HealthState.HEALTHY

    def test_warn_then_fault(self):
        """Test warning followed by fault."""
        config = StateMachineConfig(initial_health=HealthState.WARNING)
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.FAULT))
        assert fsm.get_health_state() == HealthState.CRITICAL


class TestLayerInteractions:
    """Test interactions between layers."""

    def test_health_critical_escalates_lifecycle(self):
        """Test critical health escalates to lifecycle recovering."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_health=HealthState.HEALTHY,
        )
        fsm = StateMachine(config)

        # HEALTHY -> CRITICAL triggers lifecycle RECOVERING
        fsm.send_event(StateEvent(EventType.FAULT))
        assert fsm.get_health_state() == HealthState.CRITICAL
        assert fsm.get_lifecycle_state() == LifecycleState.RECOVERING

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
        # Operational should be forced to STOPPED (was RUNNING)
        assert fsm.get_operational_state() == OperationalState.STOPPED

    def test_lifecycle_offline_forces_idle(self):
        """Test going OFFLINE forces operational to IDLE."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.SHUTTING_DOWN,
            initial_operational=OperationalState.IDLE,
        )
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.FINISHED))
        assert fsm.get_lifecycle_state() == LifecycleState.OFFLINE
        assert fsm.get_operational_state() == OperationalState.IDLE


class TestInterruptEvents:
    """Test interrupt event handling."""

    def test_emergency_stop_pauses_running(self):
        """Test emergency stop affects running operational."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
            initial_health=HealthState.HEALTHY,
        )
        fsm = StateMachine(config)

        fsm.send_event(StateEvent(EventType.EMERGENCY_STOP))

        # Running should be stopped
        assert fsm.get_operational_state() == OperationalState.STOPPED

    def test_interrupt_pauses_running(self):
        """Test interrupt pauses running task."""
        config = StateMachineConfig(
            initial_lifecycle=LifecycleState.ACTIVE,
            initial_operational=OperationalState.RUNNING,
        )
        fsm = StateMachine(config)

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

        fsm.send_event(StateEvent(EventType.START))

        assert len(callback_data) == 1
        assert callback_data[0][0] == "lifecycle"
        assert callback_data[0][1] == "Offline"
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

        assert call_order == ["high", "medium", "low"]

    def test_global_callback(self):
        """Test global callback receives all state changes."""
        fsm = StateMachine()

        callbacks = []
        fsm.subscribe("any", lambda l, o, n: callbacks.append(l))

        fsm.send_event(StateEvent(EventType.START))   # lifecycle
        fsm.send_event(StateEvent(EventType.WARN))    # health

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

        state = fsm.get_health_state()
        assert state in (HealthState.HEALTHY, HealthState.WARNING)


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
            initial_health=HealthState.HEALTHY,
        )
        fsm = StateMachine(config)

        assert fsm.is_operational()

        # Degrade health to CRITICAL (triggers recovering)
        fsm.send_event(StateEvent(EventType.FAULT))
        assert not fsm.is_operational()
