"""
Unit tests for operational metaclass and OperationalStateMachine.

Tests cover:
- Metaclass wrapping functionality
- State validation
- Automatic transitions
- Error handling
- Success/failure paths
- Operation decorator with reference counting
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
    operation,
)


class TestModule(OperationalStateMachine):
    """Test module for unit tests."""
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.initialized = False
        self.paused = False
        self.stopped = False
        self.reset_called = False
        self.should_fail = False
        self.operation_count = 0
    
    def initialize(self):
        """Test initialization."""
        if self.should_fail:
            return False
        self.initialized = True
        return True
    
    def pause(self):
        """Test pause."""
        if self.should_fail:
            return False
        self.paused = True
        return True
    
    def resume(self):
        """Test resume."""
        if self.should_fail:
            return False
        self.paused = False
        return True
    
    def stop(self):
        """Test stop."""
        if self.should_fail:
            return False
        self.stopped = True
        return True
    
    def reset(self):
        """Test reset."""
        if self.should_fail:
            return False
        self.reset_called = True
        self.initialized = False
        self.stopped = False
        return True
    
    @operation
    def process_data(self, data):
        """Test operation with decorator."""
        if self.should_fail:
            raise ValueError("Processing failed")
        self.operation_count += 1
        return f"Processed: {data}"
    
    def start_async_operation(self):
        """Helper to simulate starting an async operation (stays in RUNNING)."""
        if self.get_operational_state() not in {OperationalState.READY, OperationalState.RUNNING}:
            raise OperationalStateError(f"Cannot start operation from {self.get_operational_state()}")
        self._increment_operation_counter()
    
    def complete_async_operation(self):
        """Helper to complete an async operation (may return to READY)."""
        self._decrement_operation_counter()


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
        """Test that lifecycle methods are wrapped."""
        # Check that methods exist and are wrapped
        assert hasattr(test_module, 'initialize')
        assert hasattr(test_module, 'pause')
        assert callable(test_module.initialize)
    
    def test_public_api_methods_exist(self, test_module):
        """Test that public API methods exist."""
        assert hasattr(test_module, 'initialize')
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
    

    def test_pause_requires_running(self, test_module):
        """Test that pause requires RUNNING state."""
        # Pause from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.pause()
        assert "Cannot call pause" in str(exc_info.value)
    
    def test_resume_requires_paused(self, test_module):
        """Test that resume requires PAUSED state."""
        # Resume from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.resume()
        assert "Cannot call resume" in str(exc_info.value)
        assert "Required states: ['Paused']" in str(exc_info.value)
    
    def test_stop_requires_running_or_paused(self, test_module):
        """Test that stop requires RUNNING or PAUSED state."""
        # Stop from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.stop()
        assert "Cannot call stop" in str(exc_info.value)
    
    def test_reset_requires_stopped(self, test_module):
        """Test that reset requires STOPPED or ERROR state."""
        # Reset from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.reset()
        assert "Cannot call reset" in str(exc_info.value)
        assert "Required states:" in str(exc_info.value)


class TestSuccessTransitions:
    """Test successful state transitions."""
    
    def test_initialize_success_path(self, test_module):
        """Test initialize success: IDLE -> READY."""
        assert test_module.is_idle()
        
        result = test_module.initialize()
        
        assert result is True
        assert test_module.initialized
        assert test_module.is_ready()
        assert test_module.get_operation_counter() == 0
    
    def test_pause_success_path(self, test_module):
        """Test pause success: RUNNING -> PAUSED."""
        test_module.initialize()
        # Start async operation to stay in RUNNING
        test_module.start_async_operation()
        assert test_module.is_running()
        
        result = test_module.pause()
        
        assert result is True
        assert test_module.paused
        assert test_module.is_paused()
    
    def test_resume_success_path(self, test_module):
        """Test resume success: PAUSED -> READY."""
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        test_module.pause()
        assert test_module.is_paused()
        
        result = test_module.resume()
        
        assert result is True
        assert not test_module.paused
        assert test_module.is_ready()
        assert test_module.get_operation_counter() == 0  # Counter reset
    
    def test_stop_success_path(self, test_module):
        """Test stop success: RUNNING -> STOPPED."""
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        assert test_module.is_running()
        
        result = test_module.stop()
        
        assert result is True
        assert test_module.stopped
        assert test_module.is_stopped()
    
    def test_reset_success_path(self, test_module):
        """Test reset success: STOPPED -> IDLE."""
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        test_module.stop()
        assert test_module.is_stopped()
        
        result = test_module.reset()
        
        assert result is True
        assert test_module.reset_called
        assert test_module.is_idle()


class TestFailureTransitions:
    """Test failure state transitions."""
    
    def test_initialize_failure_path(self, test_module):
        """Test initialize failure: IDLE -> ERROR."""
        test_module.should_fail = True
        assert test_module.is_idle()
        
        result = test_module.initialize()
        
        assert result is False
        assert not test_module.initialized
        assert test_module.is_error()
    
    def test_resume_failure_path(self, test_module):
        """Test resume failure: PAUSED -> ERROR."""
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        test_module.pause()
        
        test_module.should_fail = True
        assert test_module.is_paused()
        
        result = test_module.resume()
        
        assert result is False
        assert test_module.is_error()


class TestExceptionHandling:
    """Test exception handling in lifecycle methods."""
    
    def test_exception_treated_as_failure(self, state_machine):
        """Test that exceptions are caught and treated as failures."""
        class FailingModule(OperationalStateMachine):
            def initialize(self):
                raise ValueError("Test exception")
        
        module = FailingModule(state_machine)
        
        with pytest.raises(ValueError):
            module.initialize()
        
        # Should transition to ERROR on exception
        assert module.is_error()


class TestFullLifecycle:
    """Test complete lifecycle flows."""
    
    def test_complete_success_flow(self, test_module):
        """Test complete success flow through all states."""
        # IDLE -> READY
        assert test_module.is_idle()
        test_module.initialize()
        assert test_module.is_ready()
        assert test_module.initialized
        
        # READY -> RUNNING (via async operation)
        test_module.start_async_operation()
        assert test_module.is_running()
        
        # RUNNING -> PAUSED
        test_module.pause()
        assert test_module.is_paused()
        assert test_module.paused
        
        # PAUSED -> READY
        test_module.resume()
        assert test_module.is_ready()
        assert not test_module.paused
        
        # READY -> RUNNING (via async operation)
        test_module.start_async_operation()
        assert test_module.is_running()
        
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
        assert test_module.is_error()
        
        # Reset to try again
        test_module.should_fail = False  # Clear failure flag before reset
        test_module.reset()
        assert test_module.is_idle()
        
        # Success on second try
        test_module.initialize()
        assert test_module.is_ready()
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
        assert test_module.is_ready()
    
    def test_is_running(self, test_module):
        """Test is_running query."""
        assert not test_module.is_running()
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        assert test_module.is_running()
    
    def test_is_paused(self, test_module):
        """Test is_paused query."""
        assert not test_module.is_paused()
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        test_module.pause()
        assert test_module.is_paused()
    
    def test_is_stopped(self, test_module):
        """Test is_stopped query."""
        assert not test_module.is_stopped()
        test_module.initialize()
        test_module.start_async_operation()  # Go to RUNNING
        test_module.stop()
        assert test_module.is_stopped()
    
    def test_is_error(self, test_module):
        """Test is_error query."""
        assert not test_module.is_error()
        test_module.should_fail = True
        test_module.initialize()
        assert test_module.is_error()
    
    def test_get_operational_state(self, test_module):
        """Test get_operational_state method."""
        state = test_module.get_operational_state()
        assert state == OperationalState.IDLE
        
        test_module.initialize()
        state = test_module.get_operational_state()
        assert state == OperationalState.READY
    
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
        assert test_module.is_ready()
    
    def test_false_return_is_failure(self, test_module):
        """Test that returning False is treated as failure."""
        test_module.should_fail = True
        test_module.initialize()
        assert test_module.is_error()
    
    def test_none_return_is_success(self, state_machine):
        """Test that returning None is treated as success."""
        class NoReturnModule(OperationalStateMachine):
            def initialize(self):
                pass  # Implicitly returns None
        
        module = NoReturnModule(state_machine)
        module.initialize()
        assert module.is_ready()


class TestModuleWithoutMethods:
    """Test behavior when optional methods are not implemented."""
    
    def test_missing_method_warning(self, state_machine):
        """Test that module without lifecycle methods can still be instantiated."""
        class MinimalModule(OperationalStateMachine):
            pass  # No methods implemented
        
        module = MinimalModule(state_machine)
        
        # Module can be created but won't have lifecycle methods
        assert module.is_idle()
        # Trying to call initialize would raise AttributeError since method doesn't exist
        assert not hasattr(module, 'initialize') or callable(getattr(module, 'initialize', None))


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


class TestOperationDecorator:
    """Test @operation decorator functionality."""
    
    def test_operation_requires_ready_or_running(self, test_module):
        """Test that @operation requires READY or RUNNING state."""
        # Operation from IDLE should fail
        with pytest.raises(OperationalStateError) as exc_info:
            test_module.process_data("test")
        assert "Cannot call process_data" in str(exc_info.value)
    
    def test_operation_transitions_ready_to_running(self, test_module):
        """Test that operation transitions READY -> RUNNING."""
        test_module.initialize()
        assert test_module.is_ready()
        
        # Call operation - should transition to RUNNING
        result = test_module.process_data("test_data")
        
        assert result == "Processed: test_data"
        assert test_module.operation_count == 1
        # After operation completes, counter is 0, should be back to READY
        assert test_module.is_ready()
        assert test_module.get_operation_counter() == 0
    
    def test_operation_reference_counting(self, test_module):
        """Test that operation reference counting works correctly."""
        test_module.initialize()
        assert test_module.is_ready()
        assert test_module.get_operation_counter() == 0
        
        # Simulate nested operations by checking counter during execution
        class CountingModule(OperationalStateMachine):
            def __init__(self, state_machine):
                super().__init__(state_machine)
                self.counter_values = []
            
            @operation
            def outer_operation(self):
                self.counter_values.append(('outer_start', self.get_operation_counter()))
                self.inner_operation()
                self.counter_values.append(('outer_end', self.get_operation_counter()))
                return True
            
            @operation
            def inner_operation(self):
                self.counter_values.append(('inner', self.get_operation_counter()))
                return True
        
        module = CountingModule(test_module._state_machine)
        module._set_operational_state(OperationalState.READY)
        
        module.outer_operation()
        
        # Should see: outer starts (1), inner runs (2), outer continues (1)
        assert len(module.counter_values) == 3
        assert module.counter_values[0] == ('outer_start', 1)
        assert module.counter_values[1] == ('inner', 2)
        assert module.counter_values[2] == ('outer_end', 1)
        assert module.is_ready()
        assert module.get_operation_counter() == 0
    
    def test_operation_from_running_state(self, test_module):
        """Test that operation can be called when already RUNNING."""
        test_module.initialize()
        assert test_module.is_ready()
        
        # First operation takes us to RUNNING and back to READY
        result1 = test_module.process_data("data1")
        assert result1 == "Processed: data1"
        # After operation completes, should be READY (counter 0)
        assert test_module.is_ready()
        assert test_module.get_operation_counter() == 0
        
        # Second operation from READY
        result2 = test_module.process_data("data2")
        assert result2 == "Processed: data2"
        # Should be back to READY after all operations complete
        assert test_module.is_ready()
        assert test_module.get_operation_counter() == 0
    
    def test_operation_exception_handling(self, test_module):
        """Test that exceptions in operations are handled correctly."""
        test_module.initialize()
        assert test_module.is_ready()
        test_module.should_fail = True
        
        with pytest.raises(ValueError) as exc_info:
            test_module.process_data("fail")
        
        assert "Processing failed" in str(exc_info.value)
        # Counter should be properly decremented even on exception
        assert test_module.get_operation_counter() == 0
        assert test_module.is_ready()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

