"""
Test Suite for 3-Layer State Machine - State Types

Tests state enums, validation functions, and transition rules.
"""

import pytest
from vyra_base.state.state_types import (
    LifecycleState,
    OperationalState,
    HealthState,
    is_valid_lifecycle_transition,
    is_valid_operational_transition,
    is_valid_health_transition,
    is_operational_allowed_in_lifecycle,
    LIFECYCLE_TRANSITIONS,
    OPERATIONAL_TRANSITIONS,
    HEALTH_TRANSITIONS,
    LIFECYCLE_OPERATIONAL_RULES,
)


class TestLifecycleState:
    """Test LifecycleState enum."""

    def test_all_states_exist(self):
        """Verify all lifecycle states are defined."""
        expected_states = {
            "Offline",
            "Initializing",
            "Active",
            "Recovering",
            "ShuttingDown",
            "Suspended",
        }
        actual_states = {state.value for state in LifecycleState}
        assert actual_states == expected_states

    def test_state_values_are_strings(self):
        """Verify state values are properly formatted strings."""
        for state in LifecycleState:
            assert isinstance(state.value, str)
            assert state.value[0].isupper()  # CamelCase


class TestOperationalState:
    """Test OperationalState enum."""

    def test_all_states_exist(self):
        """Verify all operational states are defined."""
        expected_states = {
            "Idle",
            "Ready",
            "Running",
            "Paused",
            "Stopped",
            "Error",
        }
        actual_states = {state.value for state in OperationalState}
        assert actual_states == expected_states

    def test_state_values_are_strings(self):
        """Verify state values are properly formatted strings."""
        for state in OperationalState:
            assert isinstance(state.value, str)
            assert state.value[0].isupper()


class TestHealthState:
    """Test HealthState enum."""

    def test_all_states_exist(self):
        """Verify all health states are defined."""
        expected_states = {
            "Healthy",
            "Warning",
            "Critical",
        }
        actual_states = {state.value for state in HealthState}
        assert actual_states == expected_states

    def test_severity_ordering(self):
        """Verify health states can be compared by severity."""
        severity = {
            HealthState.HEALTHY: 0,
            HealthState.WARNING: 1,
            HealthState.CRITICAL: 2,
        }

        assert severity[HealthState.HEALTHY] < severity[HealthState.WARNING]
        assert severity[HealthState.WARNING] < severity[HealthState.CRITICAL]


class TestLifecycleTransitions:
    """Test lifecycle state transition validation."""

    def test_valid_startup_sequence(self):
        """Test valid startup transitions."""
        assert is_valid_lifecycle_transition(
            LifecycleState.OFFLINE,
            LifecycleState.INITIALIZING
        )
        assert is_valid_lifecycle_transition(
            LifecycleState.INITIALIZING,
            LifecycleState.ACTIVE
        )

    def test_valid_shutdown_sequence(self):
        """Test valid shutdown transitions."""
        assert is_valid_lifecycle_transition(
            LifecycleState.ACTIVE,
            LifecycleState.SHUTTING_DOWN
        )
        assert is_valid_lifecycle_transition(
            LifecycleState.SHUTTING_DOWN,
            LifecycleState.OFFLINE
        )

    def test_valid_recovery_sequence(self):
        """Test valid recovery transitions."""
        assert is_valid_lifecycle_transition(
            LifecycleState.ACTIVE,
            LifecycleState.RECOVERING
        )
        assert is_valid_lifecycle_transition(
            LifecycleState.RECOVERING,
            LifecycleState.ACTIVE
        )
        # Failed recovery goes to ShuttingDown
        assert is_valid_lifecycle_transition(
            LifecycleState.RECOVERING,
            LifecycleState.SHUTTING_DOWN
        )

    def test_valid_suspend_sequence(self):
        """Test valid suspend transitions."""
        assert is_valid_lifecycle_transition(
            LifecycleState.ACTIVE,
            LifecycleState.SUSPENDED
        )
        assert is_valid_lifecycle_transition(
            LifecycleState.SUSPENDED,
            LifecycleState.OFFLINE
        )

    def test_invalid_transitions(self):
        """Test that invalid transitions are rejected."""
        assert not is_valid_lifecycle_transition(
            LifecycleState.OFFLINE,
            LifecycleState.ACTIVE
        )
        assert not is_valid_lifecycle_transition(
            LifecycleState.ACTIVE,
            LifecycleState.OFFLINE
        )
        # SUSPENDED -> RECOVERING is invalid (must go through ACTIVE first)
        assert not is_valid_lifecycle_transition(
            LifecycleState.SUSPENDED,
            LifecycleState.RECOVERING
        )

    def test_transition_set_completeness(self):
        """Verify LIFECYCLE_TRANSITIONS covers all valid transitions."""
        assert len(LIFECYCLE_TRANSITIONS) >= 8


class TestOperationalTransitions:
    """Test operational state transition validation."""

    def test_valid_task_lifecycle(self):
        """Test valid task execution transitions."""
        assert is_valid_operational_transition(OperationalState.IDLE, OperationalState.READY)
        assert is_valid_operational_transition(OperationalState.READY, OperationalState.RUNNING)
        assert is_valid_operational_transition(OperationalState.RUNNING, OperationalState.STOPPED)
        assert is_valid_operational_transition(OperationalState.STOPPED, OperationalState.IDLE)

    def test_valid_pause_resume(self):
        """Test pause and resume transitions."""
        assert is_valid_operational_transition(OperationalState.RUNNING, OperationalState.PAUSED)
        # Paused -> Ready (resume)
        assert is_valid_operational_transition(OperationalState.PAUSED, OperationalState.READY)

    def test_valid_running_complete(self):
        """Test running to ready (task complete)."""
        assert is_valid_operational_transition(OperationalState.RUNNING, OperationalState.READY)

    def test_valid_error_transitions(self):
        """Test error state transitions."""
        assert is_valid_operational_transition(OperationalState.IDLE, OperationalState.ERROR)
        assert is_valid_operational_transition(OperationalState.ERROR, OperationalState.IDLE)

    def test_invalid_transitions(self):
        """Test that invalid operational transitions are rejected."""
        assert not is_valid_operational_transition(OperationalState.IDLE, OperationalState.RUNNING)
        assert not is_valid_operational_transition(OperationalState.STOPPED, OperationalState.RUNNING)


class TestHealthTransitions:
    """Test health state transition validation."""

    def test_valid_degradation_sequence(self):
        """Test valid health degradation."""
        assert is_valid_health_transition(HealthState.HEALTHY, HealthState.WARNING)
        assert is_valid_health_transition(HealthState.WARNING, HealthState.CRITICAL)
        assert is_valid_health_transition(HealthState.HEALTHY, HealthState.CRITICAL)

    def test_valid_recovery_sequence(self):
        """Test valid health recovery."""
        assert is_valid_health_transition(HealthState.WARNING, HealthState.HEALTHY)
        assert is_valid_health_transition(HealthState.CRITICAL, HealthState.WARNING)
        assert is_valid_health_transition(HealthState.CRITICAL, HealthState.HEALTHY)

    def test_invalid_transitions(self):
        """Test that self-transitions are not defined."""
        assert not is_valid_health_transition(HealthState.HEALTHY, HealthState.HEALTHY)


class TestLayerInteractionRules:
    """Test lifecycle-operational interaction rules."""

    def test_initializing_allows_no_operational(self):
        """Initializing lifecycle locks operational FSM (empty set)."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.INITIALIZING]
        assert allowed == set()

    def test_active_allows_all_operational(self):
        """Active lifecycle allows all operational states."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.ACTIVE]
        assert OperationalState.READY in allowed
        assert OperationalState.RUNNING in allowed
        assert OperationalState.PAUSED in allowed
        assert len(allowed) == 6

    def test_recovering_restricts_operational(self):
        """Recovering lifecycle restricts operational states."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.RECOVERING]
        assert OperationalState.PAUSED in allowed
        assert OperationalState.RUNNING not in allowed
        assert OperationalState.READY not in allowed

    def test_shutting_down_restricts_operational(self):
        """ShuttingDown lifecycle restricts operational states."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.SHUTTING_DOWN]
        assert OperationalState.RUNNING not in allowed
        assert OperationalState.IDLE in allowed

    def test_suspended_allows_only_idle(self):
        """Suspended lifecycle only allows Idle operational."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.SUSPENDED]
        assert allowed == {OperationalState.IDLE}

    def test_offline_allows_safe_states(self):
        """Offline lifecycle allows only IDLE, STOPPED, ERROR."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.OFFLINE]
        assert OperationalState.IDLE in allowed
        assert OperationalState.STOPPED in allowed
        assert OperationalState.ERROR in allowed
        assert OperationalState.RUNNING not in allowed
        assert OperationalState.READY not in allowed

    def test_validation_function_uses_rules(self):
        """Test is_operational_allowed_in_lifecycle uses defined rules."""
        assert is_operational_allowed_in_lifecycle(LifecycleState.ACTIVE, OperationalState.RUNNING)
        assert not is_operational_allowed_in_lifecycle(LifecycleState.SUSPENDED, OperationalState.RUNNING)
        assert not is_operational_allowed_in_lifecycle(LifecycleState.INITIALIZING, OperationalState.READY)


class TestTransitionDataStructures:
    """Test transition rule data structures."""

    def test_lifecycle_transitions_structure(self):
        """Verify LIFECYCLE_TRANSITIONS has correct structure."""
        assert isinstance(LIFECYCLE_TRANSITIONS, set)
        for transition in LIFECYCLE_TRANSITIONS:
            assert isinstance(transition, tuple)
            assert len(transition) == 2
            assert isinstance(transition[0], LifecycleState)
            assert isinstance(transition[1], LifecycleState)

    def test_operational_transitions_structure(self):
        """Verify OPERATIONAL_TRANSITIONS has correct structure."""
        assert isinstance(OPERATIONAL_TRANSITIONS, set)
        for transition in OPERATIONAL_TRANSITIONS:
            assert isinstance(transition, tuple)
            assert len(transition) == 2
            assert isinstance(transition[0], OperationalState)
            assert isinstance(transition[1], OperationalState)

    def test_health_transitions_structure(self):
        """Verify HEALTH_TRANSITIONS has correct structure."""
        assert isinstance(HEALTH_TRANSITIONS, set)
        for transition in HEALTH_TRANSITIONS:
            assert isinstance(transition, tuple)
            assert len(transition) == 2
            assert isinstance(transition[0], HealthState)
            assert isinstance(transition[1], HealthState)

    def test_lifecycle_operational_rules_structure(self):
        """Verify LIFECYCLE_OPERATIONAL_RULES has correct structure."""
        assert isinstance(LIFECYCLE_OPERATIONAL_RULES, dict)
        for lifecycle_state in LifecycleState:
            assert lifecycle_state in LIFECYCLE_OPERATIONAL_RULES
            allowed_states = LIFECYCLE_OPERATIONAL_RULES[lifecycle_state]
            assert isinstance(allowed_states, set)
            for op_state in allowed_states:
                assert isinstance(op_state, OperationalState)
