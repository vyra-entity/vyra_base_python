==================
Event System
==================

The State Machine uses an **event-driven architecture** pattern. All state transitions 
are triggered by events.

Event Structure
===============

Each event is an immutable ``StateEvent`` instance:

.. code-block:: python

    from dataclasses import dataclass
    from datetime import datetime
    from typing import Optional, Dict, Any
    
    @dataclass(frozen=True)
    class StateEvent:
        event_type: EventType              # Event type
        timestamp: datetime                # Timestamp
        origin_layer: Optional[str]        # Origin layer (lifecycle/operational/health)
        payload: Optional[Dict[str, Any]]  # Event-specific data
        event_id: Optional[str]            # Unique event ID

Example
-------

.. code-block:: python

    from vyra_base.state.state_events import StateEvent, EventType
    
    event = StateEvent(
        event_type=EventType.TASK_START,
        origin_layer="operational",
        payload={"task_id": "task_123", "priority": "high"}
    )
    
    # Handle the event
    state_machine.handle_event(event)

Event Types Overview
====================

Lifecycle Events
----------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Event
     - Transition
     - Description
   * - START
     - Offline → Initializing
     - Start module initialization
   * - INIT_SUCCESS
     - Initializing → Active
     - Initialization successful
   * - INIT_FAILURE
     - Initializing → Recovering
     - Initialization failed
   * - SHUTDOWN
     - Active → ShuttingDown
     - Controlled shutdown
   * - FINISHED
     - ShuttingDown → Offline
     - Shutdown completed
   * - FAULT_DETECTED
     - Active → Recovering
     - Error detected, recovery needed
   * - RECOVERY_SUCCESS
     - Recovering → Active
     - Recovery successful
   * - RECOVERY_FAILED
     - Recovering → ShuttingDown
     - Recovery failed

Operational Events
------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Event
     - Transition
     - Description
   * - SET_READY
     - Idle → Ready
     - Signal readiness for tasks
   * - TASK_START
     - Ready → Running
     - Start task execution
   * - SET_BACKGROUND
     - Running → BackgroundRunning
     - Switch to background processing
   * - SET_FOREGROUND
     - BackgroundRunning → Running
     - Return to foreground execution
   * - TASK_PAUSE
     - Running → Paused
     - Pause task
   * - TASK_RESUME
     - Paused → Ready
     - Resume paused task
   * - TASK_COMPLETE
     - Running → Ready
     - Task completed successfully
   * - TASK_STOP
     - Running → Stopped
     - Stop task (requires reset)
   * - TASK_RESET
     - Stopped → Idle
     - Reset operational state

Health Events
-------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Event
     - Transition
     - Description
   * - WARN
     - Healthy → Warning
     - Report non-critical warning
   * - CLEAR_WARNING
     - Warning → Healthy
     - Warning resolved
   * - FAULT
     - Any → Critical
     - Critical error detected
   * - RECOVER
     - Critical → Healthy/Warning
     - Recovery from critical error
   * - RESET
     - Any → Healthy
     - Reset health state

Cross-Layer Events
------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Event
     - Impact
   * - INTERRUPT
     - Interrupts current operation, can affect all layers
   * - EMERGENCY_STOP
     - Emergency stop: Health→Critical, Operational→Stopped, Lifecycle→ShuttingDown
   * - PRIORITY_OVERRIDE
     - Admin/Debug override for special system states

Event Handling
==============

Basic Usage
-----------

.. code-block:: python

    from vyra_base.state import UnifiedStateMachine
    from vyra_base.state.state_events import EventType
    
    # Create state machine
    state_machine = UnifiedStateMachine(module_name="my_module")
    
    # Handle events using EventType
    state_machine.handle_event(EventType.START)
    state_machine.handle_event(EventType.INIT_SUCCESS)
    state_machine.handle_event(EventType.SET_READY)

With Payload
------------

Events can carry additional data in the payload:

.. code-block:: python

    from vyra_base.state.state_events import StateEvent, EventType
    
    # Create event with payload
    event = StateEvent(
        event_type=EventType.TASK_START,
        origin_layer="operational",
        payload={
            "task_id": "task_123",
            "priority": "high",
            "parameters": {"speed": 100}
        }
    )
    
    # Handle the event
    state_machine.handle_event(event)

Event Callbacks
---------------

Register callbacks to react to events:

.. code-block:: python

    def on_lifecycle_event(old_state, new_state, event):
        print(f"Lifecycle: {old_state} → {new_state}")
        print(f"Event: {event.event_type}")
        if event.payload:
            print(f"Payload: {event.payload}")
    
    def on_operational_event(old_state, new_state, event):
        print(f"Operational: {old_state} → {new_state}")
    
    def on_health_event(old_state, new_state, event):
        print(f"Health: {old_state} → {new_state}")
    
    # Register callbacks
    state_machine.register_lifecycle_callback(on_lifecycle_event)
    state_machine.register_operational_callback(on_operational_event)
    state_machine.register_health_callback(on_health_event)

Event Validation
================

Invalid Events
--------------

The state machine validates events before processing:

.. code-block:: python

    from vyra_base.state.state_types import InvalidTransitionError
    
    try:
        # Try to start task when not ready
        state_machine.handle_event(EventType.TASK_START)
    except InvalidTransitionError as e:
        print(f"Invalid event: {e}")
        # Output: "Cannot transition from Idle to Running with event TASK_START"

Event Sequence Validation
--------------------------

Events must follow valid transition sequences:

.. code-block:: python

    # ✅ Valid sequence
    state_machine.handle_event(EventType.START)        # Offline → Initializing
    state_machine.handle_event(EventType.INIT_SUCCESS) # Initializing → Active
    state_machine.handle_event(EventType.SET_READY)    # Idle → Ready
    state_machine.handle_event(EventType.TASK_START)   # Ready → Running
    
    # ❌ Invalid sequence
    state_machine.handle_event(EventType.TASK_START)   # Error: Not in Ready state

Event Priority
==============

Events have different priorities:

High Priority (Immediate)
-------------------------

- ``EMERGENCY_STOP``
- ``FAULT``
- ``INTERRUPT``

These events are processed immediately and can override current operations.

Normal Priority
---------------

- Most lifecycle events (``START``, ``INIT_SUCCESS``, ``SHUTDOWN``)
- Most operational events (``TASK_START``, ``TASK_STOP``)
- Health monitoring events (``WARN``, ``CLEAR_WARNING``)

Low Priority (Queued)
---------------------

- Status updates
- Monitoring events
- Non-critical notifications

Event Queue
===========

Events are processed through an internal queue:

.. code-block:: python

    # Events are queued and processed in order
    state_machine.handle_event(EventType.SET_READY)
    state_machine.handle_event(EventType.TASK_START)
    state_machine.handle_event(EventType.TASK_COMPLETE)
    
    # High priority events jump the queue
    state_machine.handle_event(EventType.EMERGENCY_STOP)  # Processed immediately

Async Event Handling
--------------------

For async operations:

.. code-block:: python

    async def handle_events_async():
        # Handle event asynchronously
        await state_machine.handle_event_async(EventType.TASK_START)
        
        # Wait for task completion
        await task_execution()
        
        await state_machine.handle_event_async(EventType.TASK_COMPLETE)

Event Patterns
==============

Task Execution Pattern
----------------------

.. code-block:: python

    # Standard task execution pattern
    state_machine.handle_event(EventType.SET_READY)
    state_machine.handle_event(EventType.TASK_START)
    
    try:
        # Execute task
        execute_task()
        state_machine.handle_event(EventType.TASK_COMPLETE)
    except Exception as e:
        # Handle error
        state_machine.handle_event(EventType.TASK_STOP)
        state_machine.handle_event(EventType.FAULT)

Error Recovery Pattern
----------------------

.. code-block:: python

    # Error recovery pattern
    def handle_error():
        # Detect error
        state_machine.handle_event(EventType.FAULT_DETECTED)
        
        # Attempt recovery
        try:
            perform_recovery()
            state_machine.handle_event(EventType.RECOVERY_SUCCESS)
        except Exception:
            state_machine.handle_event(EventType.RECOVERY_FAILED)

Graceful Shutdown Pattern
--------------------------

.. code-block:: python

    # Graceful shutdown pattern
    def shutdown():
        # Stop current tasks
        if state_machine.get_operational_state() == OperationalState.RUNNING:
            state_machine.handle_event(EventType.TASK_STOP)
        
        # Initiate shutdown
        state_machine.handle_event(EventType.SHUTDOWN)
        
        # Cleanup
        cleanup_resources()
        
        # Finish shutdown
        state_machine.handle_event(EventType.FINISHED)

Event Logging
=============

All events are automatically logged:

.. code-block:: python

    import logging
    
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    
    # Events will be logged automatically
    state_machine.handle_event(EventType.TASK_START)
    # Output: INFO: Handling event TASK_START: Ready → Running

Custom Event Logging
--------------------

Add custom logging via callbacks:

.. code-block:: python

    def log_event(old_state, new_state, event):
        logging.info(
            f"State change: {old_state} → {new_state}",
            extra={
                "event_type": event.event_type,
                "event_id": event.event_id,
                "timestamp": event.timestamp,
                "payload": event.payload
            }
        )
    
    state_machine.register_lifecycle_callback(log_event)

Best Practices
==============

1. **Always use EventType enum**

   ❌ Don't use strings:
   
   .. code-block:: python

       state_machine.handle_event("task_start")  # Wrong
   
   ✅ Use EventType:
   
   .. code-block:: python

       state_machine.handle_event(EventType.TASK_START)  # Correct

2. **Handle exceptions**

   .. code-block:: python

       try:
           state_machine.handle_event(EventType.TASK_START)
       except InvalidTransitionError as e:
           logger.error(f"Transition failed: {e}")

3. **Use payload for context**

   .. code-block:: python

       event = StateEvent(
           event_type=EventType.FAULT,
           payload={"error": str(exception), "component": "database"}
       )
       state_machine.handle_event(event)

4. **Register callbacks for monitoring**

   .. code-block:: python

       state_machine.register_lifecycle_callback(monitor_lifecycle)
       state_machine.register_operational_callback(monitor_operations)
       state_machine.register_health_callback(monitor_health)

5. **Use emergency stop for critical situations**

   .. code-block:: python

       if critical_error_detected():
           state_machine.handle_event(EventType.EMERGENCY_STOP)

See Also
========

- :doc:`overview` - Architecture overview
- :doc:`states` - State definitions
- :doc:`transitions` - Transition rules
- :doc:`api_reference` - Complete API reference
