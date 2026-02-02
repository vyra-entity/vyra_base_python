==========================
Quick Reference Guide
==========================

A quick reference for common State Machine operations.

State Machine Setup
===================

.. code-block:: python

    from vyra_base.state import UnifiedStateMachine
    
    # Basic setup
    sm = UnifiedStateMachine(
        module_name="my_module",
        enable_lifecycle=True,
        enable_health=True
    )

Common Operations
=================

Startup Sequence
----------------

.. code-block:: python

    from vyra_base.state.state_events import EventType
    
    # 1. Start module
    sm.handle_event(EventType.START)
    
    # 2. Initialize
    sm.handle_event(EventType.INIT_SUCCESS)
    
    # 3. Set ready
    sm.handle_event(EventType.SET_READY)

Task Execution
--------------

.. code-block:: python

    # Start task
    sm.handle_event(EventType.TASK_START)
    
    # Complete task
    sm.handle_event(EventType.TASK_COMPLETE)

Task Pause/Resume
-----------------

.. code-block:: python

    # Pause
    sm.handle_event(EventType.TASK_PAUSE)
    
    # Resume
    sm.handle_event(EventType.TASK_RESUME)

Shutdown
--------

.. code-block:: python

    # 1. Stop tasks if running
    if sm.get_operational_state() == OperationalState.RUNNING:
        sm.handle_event(EventType.TASK_STOP)
    
    # 2. Shutdown
    sm.handle_event(EventType.SHUTDOWN)
    
    # 3. Finish
    sm.handle_event(EventType.FINISHED)

State Queries
=============

Get Current States
------------------

.. code-block:: python

    # Individual states
    lifecycle = sm.get_lifecycle_state()
    operational = sm.get_operational_state()
    health = sm.get_health_state()
    
    # Full state report
    report = sm.get_full_state_report()
    # Returns: {
    #   "lifecycle": "Active",
    #   "operational": "Running",
    #   "health": "Healthy",
    #   "timestamp": "2025-01-19T10:30:00"
    # }

Check State
-----------

.. code-block:: python

    from vyra_base.state.state_types import (
        LifecycleState, 
        OperationalState, 
        HealthState
    )
    
    # Check specific state
    if sm.get_operational_state() == OperationalState.READY:
        sm.handle_event(EventType.TASK_START)
    
    if sm.get_health_state() == HealthState.CRITICAL:
        handle_critical_error()

Event Callbacks
===============

Register Callbacks
------------------

.. code-block:: python

    def on_state_change(old_state, new_state, event):
        print(f"{old_state} → {new_state}")
    
    # Register for specific layer
    sm.register_lifecycle_callback(on_state_change)
    sm.register_operational_callback(on_state_change)
    sm.register_health_callback(on_state_change)

Unregister Callbacks
--------------------

.. code-block:: python

    # Unregister specific callback
    sm.unregister_lifecycle_callback(on_state_change)

Error Handling
==============

Handle Invalid Transitions
---------------------------

.. code-block:: python

    from vyra_base.state.state_types import InvalidTransitionError
    
    try:
        sm.handle_event(EventType.TASK_START)
    except InvalidTransitionError as e:
        print(f"Invalid transition: {e}")

Health Monitoring
-----------------

.. code-block:: python

    # Report warning
    sm.handle_event(EventType.WARN)
    
    # Clear warning
    sm.handle_event(EventType.CLEAR_WARNING)
    
    # Report critical error
    sm.handle_event(EventType.FAULT)
    
    # Recover
    sm.handle_event(EventType.RECOVER)

Emergency Stop
--------------

.. code-block:: python

    # Immediate emergency stop
    sm.handle_event(EventType.EMERGENCY_STOP)
    
    # This will:
    # - Set Health to Critical
    # - Set Operational to Stopped
    # - Set Lifecycle to ShuttingDown

Component Integration
=====================

Using OperationalStateMachine
------------------------------

.. code-block:: python

    from vyra_base.state import (
        UnifiedStateMachine, 
        OperationalStateMachine
    )
    from vyra_base.com import remote_callable
    
    class MyComponent(OperationalStateMachine):
        def __init__(self, unified_sm: UnifiedStateMachine):
            super().__init__(unified_sm)
        
        @remote_callable
        def initialize(self, request=None, response=None):
            # State transition handled automatically
            # IDLE → READY
            return True
        
        @remote_callable
        def start_task(self, request=None, response=None):
            # READY → RUNNING
            return self.execute_task()
        
        @remote_callable
        def stop_task(self, request=None, response=None):
            # RUNNING → STOPPED
            return True

Event Types Quick Reference
============================

Lifecycle Events
----------------

.. code-block:: python

    EventType.START              # Offline → Initializing
    EventType.INIT_SUCCESS       # Initializing → Active
    EventType.INIT_FAILURE       # Initializing → Recovering
    EventType.SHUTDOWN           # Active → ShuttingDown
    EventType.FINISHED           # ShuttingDown → Offline
    EventType.FAULT_DETECTED     # Active → Recovering
    EventType.RECOVERY_SUCCESS   # Recovering → Active
    EventType.RECOVERY_FAILED    # Recovering → ShuttingDown

Operational Events
------------------

.. code-block:: python

    EventType.SET_READY          # Idle → Ready
    EventType.TASK_START         # Ready → Running
    EventType.TASK_COMPLETE      # Running → Ready
    EventType.TASK_PAUSE         # Running → Paused
    EventType.TASK_RESUME        # Paused → Ready
    EventType.TASK_STOP          # Running → Stopped
    EventType.TASK_RESET         # Stopped → Idle
    EventType.SET_BACKGROUND     # Running → BackgroundRunning
    EventType.SET_FOREGROUND     # BackgroundRunning → Running

Health Events
-------------

.. code-block:: python

    EventType.WARN               # Healthy → Warning
    EventType.CLEAR_WARNING      # Warning → Healthy
    EventType.FAULT              # Any → Critical
    EventType.RECOVER            # Critical → Healthy/Warning
    EventType.RESET              # Any → Healthy

Special Events
--------------

.. code-block:: python

    EventType.INTERRUPT          # Interrupt current operation
    EventType.EMERGENCY_STOP     # Emergency stop all layers
    EventType.PRIORITY_OVERRIDE  # Admin override

State Diagrams Cheat Sheet
===========================

Lifecycle States
----------------

.. code-block:: text

    Offline → Initializing → Active → ShuttingDown → Offline
                    ↓           ↓
                Recovering ←────┘

Operational States
------------------

.. code-block:: text

    Idle → Ready → Running → Ready
                      ↓
                   Paused → Ready
                      ↓
                   Stopped → Idle

Health States
-------------

.. code-block:: text

    Healthy ⇄ Warning → Critical → Healthy/Warning

Common Patterns
===============

Standard Task Pattern
---------------------

.. code-block:: python

    # 1. Prepare
    sm.handle_event(EventType.SET_READY)
    
    # 2. Execute
    sm.handle_event(EventType.TASK_START)
    execute_task()
    
    # 3. Complete
    sm.handle_event(EventType.TASK_COMPLETE)

Error Recovery Pattern
----------------------

.. code-block:: python

    try:
        execute_risky_operation()
    except Exception as e:
        sm.handle_event(EventType.FAULT_DETECTED)
        try:
            perform_recovery()
            sm.handle_event(EventType.RECOVERY_SUCCESS)
        except:
            sm.handle_event(EventType.RECOVERY_FAILED)

Background Processing Pattern
------------------------------

.. code-block:: python

    # Switch to background
    sm.handle_event(EventType.SET_BACKGROUND)
    
    # Process in background
    while background_task_active:
        process_background_task()
    
    # Return to foreground
    sm.handle_event(EventType.SET_FOREGROUND)

Troubleshooting
===============

State Won't Change
------------------

Check:

1. Is the transition allowed? See :doc:`transitions`
2. Is lifecycle state blocking? Check ``sm.get_lifecycle_state()``
3. Is health critical? Check ``sm.get_health_state()``

.. code-block:: python

    # Debug state issues
    print(f"Lifecycle: {sm.get_lifecycle_state()}")
    print(f"Operational: {sm.get_operational_state()}")
    print(f"Health: {sm.get_health_state()}")

Invalid Transition Error
-------------------------

.. code-block:: python

    try:
        sm.handle_event(EventType.TASK_START)
    except InvalidTransitionError as e:
        # Check current state
        current = sm.get_operational_state()
        print(f"Cannot start task from state: {current}")
        
        # Transition to correct state first
        if current == OperationalState.IDLE:
            sm.handle_event(EventType.SET_READY)
            sm.handle_event(EventType.TASK_START)

State Stuck
-----------

.. code-block:: python

    # Reset to known state
    if sm.get_operational_state() == OperationalState.STOPPED:
        sm.handle_event(EventType.TASK_RESET)
    
    # Or use emergency procedures
    sm.handle_event(EventType.EMERGENCY_STOP)

See Also
========

- :doc:`overview` - Full architecture documentation
- :doc:`states` - Complete state definitions
- :doc:`transitions` - All transition rules
- :doc:`events` - Detailed event documentation
- :doc:`api_reference` - API reference
