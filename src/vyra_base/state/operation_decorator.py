"""
Operation decorator for automatic state management with reference counting.

This module provides a decorator that can be applied to any method to automatically
manage operational state transitions based on active operation count.
"""
import functools
import logging
from typing import Callable, Any, Optional, Set

from .state_types import OperationalState
from .operational_metaclass import OperationalStateError


logger = logging.getLogger(__name__)


class OperationConfig:
    """Configuration for operation decorator behavior."""
    
    def __init__(
        self,
        required_states: Optional[Set[OperationalState]] = None,
        pre_transition: Optional[OperationalState] = None,
        success_transition: Optional[OperationalState] = None,
        failure_transition: Optional[OperationalState] = None,
        use_reference_counting: bool = True,
    ):
        """
        Initialize operation configuration.
        
        Args:
            required_states: Set of states required before operation can execute
            pre_transition: State to transition to before execution (if not using ref counting)
            success_transition: State to transition to on success (if not using ref counting)
            failure_transition: State to transition to on failure (if not using ref counting)
            use_reference_counting: If True, use reference counting for READY<->RUNNING transitions
        """
        # Default required states: READY or RUNNING
        self.required_states = required_states or {OperationalState.READY, OperationalState.RUNNING}
        self.pre_transition = pre_transition
        self.success_transition = success_transition
        self.failure_transition = failure_transition
        self.use_reference_counting = use_reference_counting


def operation(
    func: Optional[Callable] = None,
    *,
    required_states: Optional[Set[OperationalState]] = None,
    pre_transition: Optional[OperationalState] = None,
    success_transition: Optional[OperationalState] = None,
    failure_transition: Optional[OperationalState] = None,
    use_reference_counting: bool = True,
) -> Callable:
    """
    Decorator for methods that need automatic operational state management.
    
    This decorator wraps a method with state validation and transition logic.
    When reference counting is enabled (default), it automatically manages
    READY <-> RUNNING transitions based on the number of active operations.
    
    Default behavior with reference counting:
    - Pre-condition: READY or RUNNING state
    - Before execution: If in READY, transition to RUNNING and increment counter
    - After execution: Decrement counter; if counter reaches 0, transition to READY
    
    Custom behavior (use_reference_counting=False):
    - Use custom required_states, pre_transition, success/failure transitions
    
    Args:
        func: Function to decorate (provided automatically when used without parentheses)
        required_states: States that must be active before operation can run
        pre_transition: State to transition to before execution
        success_transition: State to transition to on success
        failure_transition: State to transition to on failure  
        use_reference_counting: Enable automatic reference counting for READY<->RUNNING
        
    Example:
        >>> from vyra_base import state
        >>>
        >>> class MyModule(state.OperationalStateMachine):
        ...     @state.operation
        ...     def process_data(self, data):
        ...         # This method is automatically wrapped with state management
        ...         # State will be RUNNING while executing
        ...         result = self._do_processing(data)
        ...         return result
        ...
        ...     @state.operation(required_states={OperationalState.RUNNING})
        ...     def critical_task(self):
        ...         # This can only run when already in RUNNING state
        ...         pass
    
    Returns:
        Decorated function with state management
    """
    def decorator_impl(f: Callable) -> Callable:
        config = OperationConfig(
            required_states=required_states,
            pre_transition=pre_transition,
            success_transition=success_transition,
            failure_transition=failure_transition,
            use_reference_counting=use_reference_counting,
        )
        
        @functools.wraps(f)
        def wrapper(self, *args, **kwargs) -> Any:
            # Get current operational state
            current_state = self.get_operational_state()
            
            # Validate pre-condition
            if current_state not in config.required_states:
                error_msg = (
                    f"Cannot call {f.__name__} in state {current_state.value}. "
                    f"Required states: {[s.value for s in config.required_states]}"
                )
                logger.error(error_msg)
                raise OperationalStateError(error_msg)
            
            logger.info(f"Executing operation {f.__name__} from state {current_state.value}")
            
            # Handle reference counting mode
            if config.use_reference_counting:
                # Increment operation counter and transition to RUNNING if needed
                self._increment_operation_counter()
            elif config.pre_transition is not None:
                # Manual pre-transition without ref counting
                logger.debug(f"Pre-transition: {current_state.value} -> {config.pre_transition.value}")
                self._set_operational_state(config.pre_transition)
            
            # Execute user method
            success = False
            result = None
            error = None
            
            try:
                result = f(self, *args, **kwargs)
                
                # Determine success based on return value
                if isinstance(result, bool):
                    success = result
                else:
                    success = True
                    
            except Exception as e:
                logger.error(f"{f.__name__} failed with exception: {e}", exc_info=True)
                success = False
                error = e
            finally:
                # Handle reference counting mode
                if config.use_reference_counting:
                    # Decrement operation counter and transition to READY if needed
                    self._decrement_operation_counter()
            
            # Apply post-transition based on success/failure (only if not using ref counting)
            if not config.use_reference_counting:
                if success and config.success_transition is not None:
                    logger.debug(f"Success transition: {self.get_operational_state().value} -> {config.success_transition.value}")
                    self._set_operational_state(config.success_transition)
                elif not success and config.failure_transition is not None:
                    logger.debug(f"Failure transition: {self.get_operational_state().value} -> {config.failure_transition.value}")
                    self._set_operational_state(config.failure_transition)
            
            # Re-raise exception if occurred
            if error is not None:
                raise error
            
            logger.info(f"{f.__name__} completed. Current state: {self.get_operational_state().value}")
            return result
        
        # Mark function as vyra operation for potential metaclass processing
        setattr(wrapper, '_is_vyra_operation', True)
        return wrapper
    
    # Support both @operation and @operation(...) syntax
    if func is None:
        # Called with arguments: @operation(...)
        return decorator_impl
    else:
        # Called without arguments: @operation
        return decorator_impl(func)
