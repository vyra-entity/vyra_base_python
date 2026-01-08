"""
Metaclass for automatic operational state management.

This module provides a metaclass that automatically wraps lifecycle methods
with state validation and transition logic, following industrial automation
best practices.
"""

import functools
import logging
from typing import Callable, Any, Optional

from .state_types import OperationalState


logger = logging.getLogger(__name__)


class OperationalStateError(Exception):
    """Raised when an operation is called in an invalid operational state."""
    pass


class MetaOperationalState(type):
    """
    Metaclass that automatically manages operational state transitions.
    
    This metaclass wraps user-defined lifecycle methods and automatically handles:
    
    - Pre-condition state validation
    - State transitions before method execution
    - Success/failure state transitions after method execution
    - Error handling and logging
    
    Supported lifecycle methods and their state transitions:
    
    ``initialize()``:
        Pre-condition: IDLE
        
        On success: IDLE -> READY (resets operation counter)
        
        On failure: IDLE -> ERROR
    
    ``pause()``:
        Pre-condition: RUNNING
        
        On success: RUNNING -> PAUSED
        
        On failure: No state change
    
    ``resume()``:
        Pre-condition: PAUSED
        
        On success: PAUSED -> READY (resets operation counter)
        
        On failure: PAUSED -> ERROR
    
    ``stop()``:
        Pre-condition: RUNNING, PAUSED
        
        On success: (current) -> STOPPED
        
        On failure: (current) -> ERROR
    
    ``reset()``:
        Pre-condition: STOPPED, ERROR
        
        On success: (current) -> IDLE
        
        On failure: No state change
    
    Example:
        >>> class MyModule(OperationalStateMachine):
        ...     def initialize(self):
        ...         # Setup hardware, load config, etc.
        ...         print("Initializing...")
        ...         return True
        ...
        >>>
        >>> module = MyModule(state_machine)
        >>> module.initialize()  # Automatically: IDLE -> READY on success
    """
    
    # Mapping of lifecycle methods to their state transition rules
    STATE_RULES = {
        'initialize': {
            'required_states': {OperationalState.IDLE},
            'pre_transition': None,
            'success_transition': OperationalState.READY,
            'failure_transition': OperationalState.ERROR,
            'reset_counter': True,  # Reset operation counter on success
        },
        'pause': {
            'required_states': {OperationalState.RUNNING},
            'pre_transition': None,
            'success_transition': OperationalState.PAUSED,
            'failure_transition': OperationalState.ERROR,
            'reset_counter': False,
        },
        'resume': {
            'required_states': {OperationalState.PAUSED},
            'pre_transition': None,
            'success_transition': OperationalState.READY,
            'failure_transition': OperationalState.ERROR,
            'reset_counter': True,  # Reset operation counter on success
        },
        'stop': {
            'required_states': {OperationalState.RUNNING, OperationalState.PAUSED},
            'pre_transition': None,
            'success_transition': OperationalState.STOPPED,
            'failure_transition': OperationalState.ERROR,
            'reset_counter': False,
        },
        'reset': {
            'required_states': {OperationalState.STOPPED, OperationalState.ERROR},
            'pre_transition': None,
            'success_transition': OperationalState.IDLE,
            'failure_transition': None,
            'reset_counter': False,
        },
    }
    
    def __new__(cls, name, bases, attrs):
        """
        Create a new class with wrapped lifecycle methods.
        
        Args:
            name: Class name
            bases: Base classes
            attrs: Class attributes and methods
            
        Returns:
            New class with wrapped methods
        """
        # Find and wrap all lifecycle methods that have state rules
        for method_name, rules in cls.STATE_RULES.items():
            if method_name in attrs:
                original_method = attrs[method_name]
                attrs[method_name] = cls._wrap_method(original_method, method_name, rules)
                logger.debug(f"Wrapped method '{method_name}' with state transitions")
        
        return super().__new__(cls, name, bases, attrs)
    
    @staticmethod
    def _wrap_method(original_method: Callable, method_name: str, rules: dict) -> Callable:
        """
        Wrap a lifecycle method with state validation and transition logic.
        
        Args:
            original_method: Original user-defined method
            method_name: Name of the method
            rules: State transition rules
            
        Returns:
            Wrapped method with state management
        """
        @functools.wraps(original_method)
        def wrapper(self, *args, **kwargs) -> Any:
            # Get current operational state
            current_state = self.get_operational_state()
            
            # Validate pre-condition
            required_states = rules['required_states']
            if current_state not in required_states:
                error_msg = (
                    f"Cannot call {method_name} in state {current_state.value}. "
                    f"Required states: {[s.value for s in required_states]}"
                )
                logger.error(error_msg)
                raise OperationalStateError(error_msg)
            
            logger.info(f"Executing {method_name} from state {current_state.value}")
            
            # Apply pre-transition if defined
            if rules['pre_transition'] is not None:
                logger.debug(f"Pre-transition: {current_state.value} -> {rules['pre_transition'].value}")
                self._set_operational_state(rules['pre_transition'])
            
            # Execute user method
            success = False
            result = None
            error = None
            
            try:
                result = original_method(self, *args, **kwargs)
                
                # Determine success based on return value
                # If method returns bool, use it; otherwise assume success
                if isinstance(result, bool):
                    success = result
                else:
                    success = True
                    
            except Exception as e:
                logger.error(f"{method_name} failed with exception: {e}", exc_info=True)
                success = False
                error = e
            
            # Apply post-transition based on success/failure
            if success and rules['success_transition'] is not None:
                new_state = rules['success_transition']
                logger.info(f"Success transition: {self.get_operational_state().value} -> {new_state.value}")
                
                # Reset operation counter if specified
                if rules.get('reset_counter', False):
                    logger.debug(f"Resetting operation counter (was {self.get_operation_counter()})")
                    self._reset_operation_counter()
                
                self._set_operational_state(new_state)
            elif not success and rules['failure_transition'] is not None:
                new_state = rules['failure_transition']
                logger.warning(f"Failure transition: {self.get_operational_state().value} -> {new_state.value}")
                self._set_operational_state(new_state)
            
            # Re-raise exception if occurred
            if error is not None:
                raise error
            
            logger.info(f"{method_name} completed. Current state: {self.get_operational_state().value}")
            return result
        
        return wrapper
