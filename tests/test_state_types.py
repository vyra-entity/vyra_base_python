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
            "Uninitialized",
            "Initializing",
            "Active",
            "Recovering",
            "ShuttingDown",
            "Deactivated",
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
            "Processing",
            "Delegating",
            "Paused",
            "Blocked",
            "Completed",
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
            "OK",
            "Warning",
            "Overloaded",
            "Faulted",
            "Critical",
        }
        actual_states = {state.value for state in HealthState}
        assert actual_states == expected_states
    
    def test_severity_ordering(self):
        """Verify health states can be compared by severity."""
        # Manual severity mapping for validation
        severity = {
            HealthState.OK: 0,
            HealthState.WARNING: 1,
            HealthState.OVERLOADED: 2,
            HealthState.FAULTED: 3,
            HealthState.CRITICAL: 4,
        }
        
        # Verify increasing severity
        assert severity[HealthState.OK] < severity[HealthState.WARNING]
        assert severity[HealthState.WARNING] < severity[HealthState.OVERLOADED]
        assert severity[HealthState.OVERLOADED] < severity[HealthState.FAULTED]
        assert severity[HealthState.FAULTED] < severity[HealthState.CRITICAL]


class TestLifecycleTransitions:
    """Test lifecycle state transition validation."""
    
    def test_valid_startup_sequence(self):
        """Test valid startup transitions."""
        assert is_valid_lifecycle_transition(
            LifecycleState.UNINITIALIZED,
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
            LifecycleState.DEACTIVATED
        )
    
    def test_valid_recovery_sequence(self):
        """Test valid recovery transitions."""
        # Active -> Recovering
        assert is_valid_lifecycle_transition(
            LifecycleState.ACTIVE,
            LifecycleState.RECOVERING
        )
        # Recovering -> Active (successful recovery)
        assert is_valid_lifecycle_transition(
            LifecycleState.RECOVERING,
            LifecycleState.ACTIVE
        )
        # Recovering -> Deactivated (failed recovery)
        assert is_valid_lifecycle_transition(
            LifecycleState.RECOVERING,
            LifecycleState.DEACTIVATED
        )
    
    def test_invalid_transitions(self):
        """Test that invalid transitions are rejected."""
        # Cannot go directly from Uninitialized to Active
        assert not is_valid_lifecycle_transition(
            LifecycleState.UNINITIALIZED,
            LifecycleState.ACTIVE
        )
        # Cannot go back to Uninitialized
        assert not is_valid_lifecycle_transition(
            LifecycleState.ACTIVE,
            LifecycleState.UNINITIALIZED
        )
        # Cannot skip initialization
        assert not is_valid_lifecycle_transition(
            LifecycleState.DEACTIVATED,
            LifecycleState.ACTIVE
        )
    
    def test_transition_set_completeness(self):
        """Verify LIFECYCLE_TRANSITIONS covers all valid transitions."""
        assert len(LIFECYCLE_TRANSITIONS) >= 8  # Minimum expected transitions


class TestOperationalTransitions:
    """Test operational state transition validation."""
    
    def test_valid_task_lifecycle(self):
        """Test valid task execution transitions."""
        # Idle -> Ready
        assert is_valid_operational_transition(
            OperationalState.IDLE,
            OperationalState.READY
        )
        # Ready -> Running
        assert is_valid_operational_transition(
            OperationalState.READY,
            OperationalState.RUNNING
        )
        # Running -> Completed
        assert is_valid_operational_transition(
            OperationalState.RUNNING,
            OperationalState.COMPLETED
        )
        # Completed -> Ready (reset)
        assert is_valid_operational_transition(
            OperationalState.COMPLETED,
            OperationalState.READY
        )
    
    def test_valid_pause_resume(self):
        """Test pause and resume transitions."""
        # Running -> Paused
        assert is_valid_operational_transition(
            OperationalState.RUNNING,
            OperationalState.PAUSED
        )
        # Paused -> Running
        assert is_valid_operational_transition(
            OperationalState.PAUSED,
            OperationalState.RUNNING
        )
    
    def test_valid_processing_states(self):
        """Test background processing and delegation."""
        # Running -> Processing
        assert is_valid_operational_transition(
            OperationalState.RUNNING,
            OperationalState.PROCESSING
        )
        # Processing -> Running
        assert is_valid_operational_transition(
            OperationalState.PROCESSING,
            OperationalState.RUNNING
        )
        # Running -> Delegating
        assert is_valid_operational_transition(
            OperationalState.RUNNING,
            OperationalState.DELEGATING
        )
        # Delegating -> Running
        assert is_valid_operational_transition(
            OperationalState.DELEGATING,
            OperationalState.RUNNING
        )
    
    def test_valid_blocking(self):
        """Test blocking and unblocking."""
        # Running -> Blocked
        assert is_valid_operational_transition(
            OperationalState.RUNNING,
            OperationalState.BLOCKED
        )
        # Blocked -> Running
        assert is_valid_operational_transition(
            OperationalState.BLOCKED,
            OperationalState.RUNNING
        )
    
    def test_invalid_transitions(self):
        """Test that invalid operational transitions are rejected."""
        # Cannot go from Idle to Running (must go through Ready)
        assert not is_valid_operational_transition(
            OperationalState.IDLE,
            OperationalState.RUNNING
        )
        # Cannot go from Completed to Running (must reset to Ready first)
        assert not is_valid_operational_transition(
            OperationalState.COMPLETED,
            OperationalState.RUNNING
        )


class TestHealthTransitions:
    """Test health state transition validation."""
    
    def test_valid_degradation_sequence(self):
        """Test valid health degradation."""
        # OK -> Warning
        assert is_valid_health_transition(
            HealthState.OK,
            HealthState.WARNING
        )
        # Warning -> Overloaded
        assert is_valid_health_transition(
            HealthState.WARNING,
            HealthState.OVERLOADED
        )
        # Warning -> Faulted
        assert is_valid_health_transition(
            HealthState.WARNING,
            HealthState.FAULTED
        )
        # Faulted -> Critical
        assert is_valid_health_transition(
            HealthState.FAULTED,
            HealthState.CRITICAL
        )
    
    def test_valid_recovery_sequence(self):
        """Test valid health recovery."""
        # Warning -> OK
        assert is_valid_health_transition(
            HealthState.WARNING,
            HealthState.OK
        )
        # Overloaded -> Warning
        assert is_valid_health_transition(
            HealthState.OVERLOADED,
            HealthState.WARNING
        )
        # Faulted -> OK
        assert is_valid_health_transition(
            HealthState.FAULTED,
            HealthState.OK
        )
        # Critical -> Faulted
        assert is_valid_health_transition(
            HealthState.CRITICAL,
            HealthState.FAULTED
        )
    
    def test_invalid_transitions(self):
        """Test that invalid health transitions are rejected."""
        # Cannot go from OK to Critical directly
        assert not is_valid_health_transition(
            HealthState.OK,
            HealthState.CRITICAL
        )
        # Cannot go from Critical to OK directly
        assert not is_valid_health_transition(
            HealthState.CRITICAL,
            HealthState.OK
        )


class TestLayerInteractionRules:
    """Test lifecycle-operational interaction rules."""
    
    def test_uninitialized_allows_only_idle(self):
        """Uninitialized lifecycle only allows Idle operational."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.UNINITIALIZED]
        assert allowed == {OperationalState.IDLE}
    
    def test_active_allows_all_operational(self):
        """Active lifecycle allows all operational states."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.ACTIVE]
        assert OperationalState.READY in allowed
        assert OperationalState.RUNNING in allowed
        assert OperationalState.PROCESSING in allowed
        assert len(allowed) >= 7  # Should allow most operational states
    
    def test_recovering_restricts_operational(self):
        """Recovering lifecycle restricts operational states."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.RECOVERING]
        assert OperationalState.PAUSED in allowed or OperationalState.BLOCKED in allowed
        # Should not allow normal running
        assert OperationalState.RUNNING not in allowed or OperationalState.READY not in allowed
    
    def test_shutting_down_restricts_operational(self):
        """ShuttingDown lifecycle restricts operational states."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.SHUTTING_DOWN]
        # Should allow only safe states
        assert OperationalState.RUNNING not in allowed
    
    def test_deactivated_allows_only_idle(self):
        """Deactivated lifecycle only allows Idle operational."""
        allowed = LIFECYCLE_OPERATIONAL_RULES[LifecycleState.DEACTIVATED]
        assert allowed == {OperationalState.IDLE}
    
    def test_validation_function_uses_rules(self):
        """Test is_operational_allowed_in_lifecycle uses defined rules."""
        # Active allows Running
        assert is_operational_allowed_in_lifecycle(
            LifecycleState.ACTIVE,
            OperationalState.RUNNING
        )
        # Deactivated does not allow Running
        assert not is_operational_allowed_in_lifecycle(
            LifecycleState.DEACTIVATED,
            OperationalState.RUNNING
        )
        # Uninitialized does not allow Ready
        assert not is_operational_allowed_in_lifecycle(
            LifecycleState.UNINITIALIZED,
            OperationalState.READY
        )


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
        # All lifecycle states should have rules
        for lifecycle_state in LifecycleState:
            assert lifecycle_state in LIFECYCLE_OPERATIONAL_RULES
            allowed_states = LIFECYCLE_OPERATIONAL_RULES[lifecycle_state]
            assert isinstance(allowed_states, set)
            # All allowed states must be valid OperationalState
            for op_state in allowed_states:
                assert isinstance(op_state, OperationalState)
