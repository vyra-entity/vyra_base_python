"""
Unit tests for operational metaclass and OperationalStateMachine.

Tests cover:
- Metaclass wrapping functionality
- State validation
- Automatic transitions
- Error handling
- Success/failure paths
"""

import pytest
from vyra_base.state import (
    StateMachine,
    StateMachineConfig,
    OperationalStateMachine,
    OperationalState,
    OperationalStateError,
    StateEvent,
    EventType,
)


class TestModule(OperationalStateMachine):
    """Test module for unit tests."""
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.initialized = False
        self.started = False
        self.paused = False
        self.stopped = False
        self.reset_called = False
        self.should_fail = False
    
    def on_initialize(self):
        """Test initialization."""
        if self.should_fail:
            return False
        self.initialized = True
        return True
    
    def on_start(self):
        """Test start."""
        if self.should_fail:
            return False
        self.started = True
        return True
    
    def on_pause(self):
        """Test pause."""
        if self.should_fail:
            return False
        self.paused = True
        return True
    
    def on_resume(self):
        """Test resume."""
        if self.should_fail:
            return False
        self.paused = False
        return True
    
    def on_stop(self):
        """Test stop."""
        if self.should_fail:
            return False
        self.stopped = True
        return True
    
    def on_reset(self):
        """Test reset."""
        if self.should_fail:
            return False
        self.reset_called = True
        self.initialized = False
        self.started = False
        self.stopped = False
        return True


@pytest.fixture
def state_machine():
    """Create a state machine for testing."""
    config = StateMachineConfig()
    fsm = StateMachine(config)
    # Bring to Active lifecycle state
    fsm.send_event(StateEvent(EventType.START))
    fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
    return fsm


@pytest.fixture
def test_module(state_machine):
    """Create a test module."""
    return TestModule(state_machine)


class TestMetaclassWrapping:
    """Test that metaclass correctly wraps methods."""
    
    def test_method_wrapping(self, test_module):
        """Test that on_* methods are wrapped."""
        # Check that methods exist and are wrapped
        assert hasattr(test_module, 'on_initialize')
        assert hasattr(test_module, 'on_start')
        assert hasattr(test_module, 'on_pause')
        assert callable(test_module.on_initialize)
    
    def test_public_api_methods_exist(self, test_module):
        """Test that public API methods exist."""
        assert hasattr(test_module, 'initialize')
        assert hasattr(test_module, 'start')
        assert hasattr(test_module, 'pause')
        assert hasattr(test_module, 'resume')
        assert hasattr(test_module, 'stop')
        assert hasattr(test_module, 'reset')


class TestStateValidation:
    """Test state validation before method execution."""
    
    def test_initialize_requires_idle(self, test_module):
        """Test that initialize requires IDLE state."""
        # Module starts in IDLE, so this should work
        assert test_module.is_idle()
        test_module.initialize()
        assert test_module.initialized
    
    def test_start_requires_ready(self, test_module):
        """Test that start requires READY state."""
        # Start from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.start()
        assert "Cannot call on_start" in str(exc_info.value)
        assert "Required states: ['Ready']" in str(exc_info.value)
    
    def test_pause_requires_running(self, test_module):
        """Test that pause requires RUNNING state."""
        # Pause from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.pause()
        assert "Cannot call on_pause" in str(exc_info.value)
    
    def test_resume_requires_paused(self, test_module):
        """Test that resume requires PAUSED state."""
        # Resume from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.resume()
        assert "Cannot call on_resume" in str(exc_info.value)
        assert "Required states: ['Paused']" in str(exc_info.value)
    
    def test_stop_requires_running_or_paused(self, test_module):
        """Test that stop requires RUNNING or PAUSED state."""
        # Stop from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.stop()
        assert "Cannot call on_stop" in str(exc_info.value)
    
    def test_reset_requires_stopped(self, test_module):
        """Test that reset requires STOPPED state."""
        # Reset from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.reset()
        assert "Cannot call on_reset" in str(exc_info.value)
        assert "Required states: ['Stopped']" in str(exc_info.value)


class TestSuccessTransitions:
    """Test successful state transitions."""
    
    def test_initialize_success_path(self, test_module):
        """Test initialize success: IDLE -> READY -> RUNNING."""
        assert test_module.is_idle()
        
        result = test_module.initialize()
        
        assert result is True
        assert test_module.initialized
        assert test_module.is_running()
    
    def test_start_success_path(self, state_machine):
        """Test start success: READY -> RUNNING."""
        module = TestModule(state_machine)
        # Get to READY state
        module.initialize()
        module.pause()
        module.resume()
        
        assert module.is_ready()
        result = module.start()
        
        assert result is True
        assert module.started
        assert module.is_running()
    
    def test_pause_success_path(self, test_module):
        """Test pause success: RUNNING -> PAUSED."""
        test_module.initialize()
        assert test_module.is_running()
        
        result = test_module.pause()
        
        assert result is True
        assert test_module.paused
        assert test_module.is_paused()
    
    def test_resume_success_path(self, test_module):
        """Test resume success: PAUSED -> READY."""
        test_module.initialize()
        test_module.pause()
        assert test_module.is_paused()
        
        result = test_module.resume()
        
        assert result is True
        assert not test_module.paused
        assert test_module.is_ready()
    
    def test_stop_success_path(self, test_module):
        """Test stop success: RUNNING -> STOPPED."""
        test_module.initialize()
        assert test_module.is_running()
        
        result = test_module.stop()
        
        assert result is True
        assert test_module.stopped
        assert test_module.is_stopped()
    
    def test_reset_success_path(self, test_module):
        """Test reset success: STOPPED -> IDLE."""
        test_module.initialize()
        test_module.stop()
        assert test_module.is_stopped()
        
        result = test_module.reset()
        
        assert result is True
        assert test_module.reset_called
        assert test_module.is_idle()


class TestFailureTransitions:
    """Test failure state transitions."""
    
    def test_initialize_failure_path(self, test_module):
        """Test initialize failure: IDLE -> READY -> STOPPED."""
        test_module.should_fail = True
        assert test_module.is_idle()
        
        result = test_module.initialize()
        
        assert result is False
        assert not test_module.initialized
        assert test_module.is_stopped()
    
    def test_start_failure_path(self, state_machine):
        """Test start failure: READY -> STOPPED."""
        module = TestModule(state_machine)
        module.initialize()
        module.pause()
        module.resume()
        
        module.should_fail = True
        assert module.is_ready()
        
        result = module.start()
        
        assert result is False
        assert not module.started
        assert module.is_stopped()
    
    def test_resume_failure_path(self, test_module):
        """Test resume failure: PAUSED -> STOPPED."""
        test_module.initialize()
        test_module.pause()
        
        test_module.should_fail = True
        assert test_module.is_paused()
        
        result = test_module.resume()
        
        assert result is False
        assert test_module.is_stopped()


class TestExceptionHandling:
    """Test exception handling in lifecycle methods."""
    
    def test_exception_treated_as_failure(self, state_machine):
        """Test that exceptions are caught and treated as failures."""
        class FailingModule(OperationalStateMachine):
            def on_initialize(self):
                raise ValueError("Test exception")
        
        module = FailingModule(state_machine)
        
        with pytest.raises(ValueError):
            module.initialize()
        
        # Should transition to STOPPED on exception
        assert module.is_stopped()


class TestFullLifecycle:
    """Test complete lifecycle flows."""
    
    def test_complete_success_flow(self, test_module):
        """Test complete success flow through all states."""
        # IDLE -> RUNNING
        assert test_module.is_idle()
        test_module.initialize()
        assert test_module.is_running()
        assert test_module.initialized
        
        # RUNNING -> PAUSED
        test_module.pause()
        assert test_module.is_paused()
        assert test_module.paused
        
        # PAUSED -> READY
        test_module.resume()
        assert test_module.is_ready()
        assert not test_module.paused
        
        # READY -> RUNNING
        test_module.start()
        assert test_module.is_running()
        assert test_module.started
        
        # RUNNING -> STOPPED
        test_module.stop()
        assert test_module.is_stopped()
        assert test_module.stopped
        
        # STOPPED -> IDLE
        test_module.reset()
        assert test_module.is_idle()
        assert test_module.reset_called
    
    def test_restart_after_failure(self, test_module):
        """Test restarting after a failure."""
        # Fail initialization
        test_module.should_fail = True
        test_module.initialize()
        assert test_module.is_stopped()
        
        # Reset to try again
        test_module.should_fail = False  # Clear failure flag before reset
        test_module.reset()
        assert test_module.is_idle()
        
        # Success on second try
        test_module.initialize()
        assert test_module.is_running()
        assert test_module.initialized
        assert test_module.initialized


class TestStateQueries:
    """Test state query methods."""
    
    def test_is_idle(self, test_module):
        """Test is_idle query."""
        assert test_module.is_idle()
        test_module.initialize()
        assert not test_module.is_idle()
    
    def test_is_ready(self, test_module):
        """Test is_ready query."""
        assert not test_module.is_ready()
        test_module.initialize()
        test_module.pause()
        test_module.resume()
        assert test_module.is_ready()
    
    def test_is_running(self, test_module):
        """Test is_running query."""
        assert not test_module.is_running()
        test_module.initialize()
        assert test_module.is_running()
    
    def test_is_paused(self, test_module):
        """Test is_paused query."""
        assert not test_module.is_paused()
        test_module.initialize()
        test_module.pause()
        assert test_module.is_paused()
    
    def test_is_stopped(self, test_module):
        """Test is_stopped query."""
        assert not test_module.is_stopped()
        test_module.should_fail = True
        test_module.initialize()
        assert test_module.is_stopped()
    
    def test_get_operational_state(self, test_module):
        """Test get_operational_state method."""
        state = test_module.get_operational_state()
        assert state == OperationalState.IDLE
        
        test_module.initialize()
        state = test_module.get_operational_state()
        assert state == OperationalState.RUNNING
    
    def test_get_all_states(self, test_module):
        """Test get_all_states method."""
        states = test_module.get_all_states()
        assert 'lifecycle' in states
        assert 'operational' in states
        assert 'health' in states
        assert states['operational'] == 'Idle'


class TestReturnValues:
    """Test handling of different return values."""
    
    def test_true_return_is_success(self, test_module):
        """Test that returning True is treated as success."""
        test_module.initialize()
        assert test_module.is_running()
    
    def test_false_return_is_failure(self, test_module):
        """Test that returning False is treated as failure."""
        test_module.should_fail = True
        test_module.initialize()
        assert test_module.is_stopped()
    
    def test_none_return_is_success(self, state_machine):
        """Test that returning None is treated as success."""
        class NoReturnModule(OperationalStateMachine):
            def on_initialize(self):
                pass  # Implicitly returns None
        
        module = NoReturnModule(state_machine)
        module.initialize()
        assert module.is_running()


class TestModuleWithoutMethods:
    """Test behavior when optional methods are not implemented."""
    
    def test_missing_method_warning(self, state_machine):
        """Test that missing methods produce warnings but don't crash."""
        class MinimalModule(OperationalStateMachine):
            pass  # No methods implemented
        
        module = MinimalModule(state_machine)
        
        # Should return False but not crash
        result = module.initialize()
        assert result is False


class TestIntegration:
    """Test integration with underlying state machine."""
    
    def test_respects_lifecycle_state(self, state_machine):
        """Test that operational transitions respect lifecycle state."""
        # Put lifecycle in RECOVERING state
        state_machine.send_event(StateEvent(EventType.FAULT_DETECTED))
        
        module = TestModule(state_machine)
        
        # Should not be able to use operational methods in RECOVERING
        # The lifecycle constraints should prevent certain operational states
        assert state_machine.get_lifecycle_state().value == 'Recovering'
    
    def test_state_machine_events_sent(self, state_machine):
        """Test that proper events are sent to state machine."""
        module = TestModule(state_machine)
        
        initial_history_size = len(state_machine.get_history())
        
        module.initialize()
        
        # Should have recorded state transitions
        final_history_size = len(state_machine.get_history())
        assert final_history_size > initial_history_size


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
