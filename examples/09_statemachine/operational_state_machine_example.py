"""
Example: Using the Operational State Machine with Metaclass

This example demonstrates how to use the OperationalStateMachine base class
with automatic state management through lifecycle methods.
"""

from vyra_base.state import (
    StateMachine,
    StateMachineConfig,
    OperationalStateMachine,
    OperationalState,
    LifecycleState,
)


# =============================================================================
# Example 1: Simple Module with Initialize and Start
# =============================================================================

class SimpleModule(OperationalStateMachine):
    """
    A simple module that demonstrates basic lifecycle methods.
    """
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.data = None
        self.is_processing = False
    
    def initialize(self):
        """
        Initialize the module.
        
        Automatic state transitions:
        - Pre-condition: Must be in IDLE state
        - Before execution: IDLE -> READY
        - On success: READY -> RUNNING
        - On failure: READY -> STOPPED
        """
        print("SimpleModule: Initializing...")
        self.data = []
        print("SimpleModule: Hardware initialized")
        return True  # Success
    
    def start(self):
        """
        Start main processing.
        
        Automatic state transitions:
        - Pre-condition: Must be in READY state
        - On success: READY -> RUNNING
        - On failure: READY -> STOPPED
        """
        print("SimpleModule: Starting processing...")
        self.is_processing = True
        print("SimpleModule: Now processing")
        return True  # Success


def example_simple_module():
    """Example usage of SimpleModule."""
    print("\n" + "=" * 70)
    print("EXAMPLE 1: Simple Module")
    print("=" * 70 + "\n")
    
    # Create state machine
    config = StateMachineConfig()
    fsm = StateMachine(config)
    
    # Start lifecycle - required for operational state transitions
    from vyra_base.state import StateEvent, EventType
    fsm.send_event(StateEvent(EventType.START))  # OFFLINE -> INITIALIZING
    fsm.send_event(StateEvent(EventType.INIT_SUCCESS))  # INITIALIZING -> ACTIVE
    
    # Create module
    module = SimpleModule(fsm)
    print(f"Initial operational state: {module.get_operational_state().value}")
    print(f"Lifecycle state: {fsm.get_lifecycle_state().value}\n")
    
    # Initialize - automatic state management
    print("Calling initialize()...")
    module.initialize()
    print(f"After initialize: {module.get_operational_state().value}\n")
    
    print(f"Module data: {module.data}")
    print(f"Is processing: {module.is_processing}")


# =============================================================================
# Example 2: Module with Full Lifecycle
# =============================================================================

class FullLifecycleModule(OperationalStateMachine):
    """
    A module that implements the full operational lifecycle.
    """
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.config = {}
        self.tasks = []
        self.paused_at = None
    
    def initialize(self):
        """Initialize with configuration."""
        print("FullLifecycle: Loading configuration...")
        self.config = {"max_tasks": 10, "timeout": 30}
        print(f"FullLifecycle: Config loaded: {self.config}")
        return True
    
    def start(self):
        """Start task processing."""
        print("FullLifecycle: Starting task processing...")
        self.tasks = []
        return True
    
    def pause(self):
        """
        Pause current processing.
        
        Automatic state transitions:
        - Pre-condition: RUNNING
        - On success: (current) -> PAUSED
        """
        print("FullLifecycle: Pausing...")
        import time
        self.paused_at = time.time()
        return True
    
    def resume(self):
        """
        Resume paused processing.
        
        Automatic state transitions:
        - Pre-condition: PAUSED
        - On success: PAUSED -> READY
        - On failure: PAUSED -> STOPPED
        """
        print("FullLifecycle: Resuming...")
        if self.paused_at:
            import time
            pause_duration = time.time() - self.paused_at
            print(f"FullLifecycle: Was paused for {pause_duration:.2f} seconds")
        self.paused_at = None
        return True
    
    def stop(self):
        """
        Stop processing.
        
        Automatic state transitions:
        - Pre-condition: RUNNING or PAUSED
        - On success: (current) -> STOPPED
        """
        print("FullLifecycle: Stopping...")
        print(f"FullLifecycle: Processed {len(self.tasks)} tasks")
        return True
    
    def reset(self):
        """
        Reset to initial state.
        
        Automatic state transitions:
        - Pre-condition: STOPPED
        - On success: STOPPED -> IDLE
        """
        print("FullLifecycle: Resetting...")
        self.config = {}
        self.tasks = []
        self.paused_at = None
        print("FullLifecycle: Reset complete")
        return True


def example_full_lifecycle():
    """Example usage of FullLifecycleModule."""
    print("\n" + "=" * 70)
    print("EXAMPLE 2: Full Lifecycle Module")
    print("=" * 70 + "\n")
    
    # Create state machine
    config = StateMachineConfig()
    fsm = StateMachine(config)
    
    # Start lifecycle
    from vyra_base.state import StateEvent, EventType
    fsm.send_event(StateEvent(EventType.START))
    fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
    
    # Create module
    module = FullLifecycleModule(fsm)
    print(f"Initial state: {module.get_operational_state().value}\n")
    
    # Initialize (IDLE -> READY)
    print("1. Initializing...")
    module.initialize()
    print(f"   State: {module.get_operational_state().value}\n")
    
    # Start an operation to go READY -> RUNNING
    print("2. Starting operation (READY -> RUNNING)...")
    module._increment_operation_counter()  # Simulate operation start
    print(f"   State: {module.get_operational_state().value}\n")
    
    # Pause (RUNNING -> PAUSED)
    print("3. Pausing...")
    module.pause()
    print(f"   State: {module.get_operational_state().value}\n")
    
    # Resume (PAUSED -> READY)
    print("4. Resuming...")
    module.resume()
    print(f"   State: {module.get_operational_state().value}\n")
    
    # Start another operation (READY -> RUNNING)
    print("5. Starting operation again...")
    module._increment_operation_counter()
    print(f"   State: {module.get_operational_state().value}\n")
    
    # Stop (RUNNING -> STOPPED)
    print("6. Stopping...")
    module.stop()
    print(f"   State: {module.get_operational_state().value}\n")
    
    # Reset (STOPPED -> IDLE)
    print("7. Resetting...")
    module.reset()
    print(f"   State: {module.get_operational_state().value}\n")


# =============================================================================
# Example 3: Error Handling
# =============================================================================

class ErrorHandlingModule(OperationalStateMachine):
    """
    A module that demonstrates error handling in lifecycle methods.
    """
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.fail_initialization = False
    
    def initialize(self):
        """Initialize with potential failure."""
        print("ErrorHandling: Attempting initialization...")
        
        if self.fail_initialization:
            print("ErrorHandling: Initialization FAILED!")
            return False  # Return False to indicate failure
        
        print("ErrorHandling: Initialization SUCCESS!")
        return True
    
    def start(self):
        """Start with exception handling."""
        print("ErrorHandling: Starting...")
        # Exceptions are automatically caught and treated as failures
        return True


def example_error_handling():
    """Example of error handling in lifecycle methods."""
    print("\n" + "=" * 70)
    print("EXAMPLE 3: Error Handling")
    print("=" * 70 + "\n")
    
    from vyra_base.state import StateEvent, EventType
    
    # Successful initialization
    print("Scenario A: Successful Initialization")
    print("-" * 40)
    config = StateMachineConfig()
    fsm = StateMachine(config)
    fsm.send_event(StateEvent(EventType.START))
    fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
    
    module1 = ErrorHandlingModule(fsm)
    module1.fail_initialization = False
    module1.initialize()
    print(f"Final state: {module1.get_operational_state().value}")
    print(f"Expected: READY (success path: IDLE -> READY)\n")
    
    # Failed initialization
    print("Scenario B: Failed Initialization")
    print("-" * 40)
    config2 = StateMachineConfig()
    fsm2 = StateMachine(config2)
    fsm2.send_event(StateEvent(EventType.START))
    fsm2.send_event(StateEvent(EventType.INIT_SUCCESS))
    
    module2 = ErrorHandlingModule(fsm2)
    module2.fail_initialization = True
    module2.initialize()
    print(f"Final state: {module2.get_operational_state().value}")
    print(f"Expected: ERROR (failure path: IDLE -> ERROR)\n")


# =============================================================================
# Example 4: State Validation
# =============================================================================

def example_state_validation():
    """Example of automatic state validation."""
    print("\n" + "=" * 70)
    print("EXAMPLE 4: State Validation")
    print("=" * 70 + "\n")
    
    from vyra_base.state import OperationalStateError, StateEvent, EventType
    
    # Create state machine and module
    config = StateMachineConfig()
    fsm = StateMachine(config)
    fsm.send_event(StateEvent(EventType.START))
    fsm.send_event(StateEvent(EventType.INIT_SUCCESS))
    
    module = SimpleModule(fsm)
    
    print(f"Current state: {module.get_operational_state().value}")
    print("Module is in IDLE state\n")
    
    # Try to call start() when in IDLE (should fail - requires READY)
    print("Attempting to start() without initializing first...")
    try:
        module.start()
        print("ERROR: Should have raised exception!")
    except OperationalStateError as e:
        print(f"✓ Correctly caught error: {e}\n")
    
    # Now do it correctly
    print("Doing it correctly: initialize() first, then start()...")
    module.initialize()
    print(f"After initialize: {module.get_operational_state().value}")
    print("✓ Can now call other lifecycle methods\n")


# =============================================================================
# Example 5: Integration with UnifiedStateMachine
# =============================================================================

class IntegratedModule(OperationalStateMachine):
    """Module that integrates with full 3-layer state machine."""
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
    
    def initialize(self):
        """Initialize and check all layers."""
        print("Integrated: Initializing...")
        
        # Can access all state machine layers
        all_states = self.get_all_states()
        print(f"  Lifecycle: {all_states['lifecycle']}")
        print(f"  Operational: {all_states['operational']}")
        print(f"  Health: {all_states['health']}")
        
        return True
    
    def start(self):
        """Start processing."""
        print("Integrated: Starting...")
        return True


def example_integration():
    """Example of integration with UnifiedStateMachine."""
    print("\n" + "=" * 70)
    print("EXAMPLE 5: Integration with UnifiedStateMachine")
    print("=" * 70 + "\n")
    
    from vyra_base.state import UnifiedStateMachine
    
    # Create unified state machine
    usm = UnifiedStateMachine()
    
    # Start lifecycle layer
    usm.start()  # Lifecycle: OFFLINE -> INITIALIZING
    usm.complete_initialization()  # Lifecycle: INITIALIZING -> ACTIVE
    
    print(f"Lifecycle ready. All states: {usm.get_all_states()}\n")
    
    # Create operational module using the same FSM
    module = IntegratedModule(usm.fsm)
    
    print("Initializing operational module...")
    module.initialize()
    
    print(f"\nFinal states: {module.get_all_states()}")


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    # Run all examples
    example_simple_module()
    example_full_lifecycle()
    example_error_handling()
    example_state_validation()
    example_integration()
    
    print("\n" + "=" * 70)
    print("All examples completed!")
    print("=" * 70)
