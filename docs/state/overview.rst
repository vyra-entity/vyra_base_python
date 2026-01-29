========================================
3-Layer State Machine Architecture
========================================

The State Machine consists of three hierarchical layers that work together to manage
the module's lifecycle, operations, and health status.

Quick Overview: The Three Layers
=================================

1. **Lifecycle Layer**
   
   *When* a module exists, initializes, activates, or shuts down.
   
   → Corresponds to Node Lifecycle in ROS2, ECS Lifecycle in Unreal, Driver Lifecycle in Linux.

2. **Operational Layer**
   
   *What* the module is currently doing.
   
   → Runtime behavior (Idle, Ready, Running, Paused, Stopped ...)

3. **Health Layer**
   
   *How well* the module is functioning.
   
   → Diagnostics: Healthy, Warning, Critical

Layer Hierarchy
================

The layers are **nested**, not parallel:

.. code-block:: text

    ┌─────────────────────────────────────────┐
    │         Lifecycle Layer                 │
    │  (Controls module existence)            │
    │                                         │
    │  ┌───────────────────────────────────┐  │
    │  │    Operational Layer              │  │
    │  │  (Controls runtime behavior)      │  │
    │  │                                   │  │
    │  │  ┌─────────────────────────────┐  │  │
    │  │  │   Health Layer              │  │  │
    │  │  │ (Monitors system health)    │  │  │
    │  │  └─────────────────────────────┘  │  │
    │  └───────────────────────────────────┘  │
    └─────────────────────────────────────────┘

Lifecycle → Operational
========================

The Lifecycle determines **which Operational states are allowed**.

.. code-block:: text

    Initializing: No operational states allowed (FSM locked)
    Active: Operational FSM fully active (all states allowed)
    Recovering: Operational FSM paused/limited (only Idle, Paused, Stopped)
    ShuttingDown: Operational FSM frozen (only Idle allowed)
    Offline: Operational only Idle or Stopped

Rules from Code
---------------

.. code-block:: python

    LIFECYCLE_OPERATIONAL_RULES = {
        LifecycleState.INITIALIZING: set(),  # No operational states
        LifecycleState.ACTIVE: {             # All operational states allowed
            IDLE, READY, RUNNING, BACKGROUND_RUNNING, PAUSED, STOPPED
        },
        LifecycleState.RECOVERING: {         # Limited states
            IDLE, PAUSED, STOPPED
        },
        LifecycleState.SHUTTING_DOWN: {      # Only Idle
            IDLE
        },
        LifecycleState.OFFLINE: {            # Only Idle/Stopped
            IDLE, STOPPED
        }
    }

Health → Operational
====================

Health acts as a **regulation layer** that corrects Operational behavior.

Rules
-----

- **Warning** → Operational remains active, but with restrictions (e.g., reduced load)
- **Critical** → Operational is **immediately interrupted**, jumps to Stopped

.. note::
   The concrete implementation is done via event handlers in the State Machine.

Health → Lifecycle
==================

Only Health may **escalate from bottom to top** to Lifecycle:

- **Warning** → Lifecycle remains Active (monitoring only)
- **Critical** + fault → Lifecycle jumps from Active → Recovering
- **Critical** + emergency_stop → Lifecycle → ShuttingDown (emergency stop)

.. important::
   "Safety health states override everything."

Events for Escalation
---------------------

.. code-block:: python

    EventType.FAULT_DETECTED      # Health Critical → Lifecycle Recovering
    EventType.EMERGENCY_STOP      # Health Critical → Lifecycle ShuttingDown
    EventType.RECOVERY_SUCCESS    # Lifecycle Recovering → Active
    EventType.RECOVERY_FAILED     # Lifecycle Recovering → ShuttingDown

UnifiedStateMachine API
=======================

The ``UnifiedStateMachine`` class manages all three layers.

Initialization
--------------

.. code-block:: python

    from vyra_base.state import UnifiedStateMachine
    
    state_machine = UnifiedStateMachine(
        module_name="my_module",
        enable_lifecycle=True,    # Enable lifecycle layer
        enable_health=True        # Enable health monitoring
    )

Event Handling
--------------

.. code-block:: python

    # Lifecycle events
    state_machine.handle_event(EventType.START)
    state_machine.handle_event(EventType.INIT_SUCCESS)
    
    # Operational events
    state_machine.handle_event(EventType.SET_READY)
    state_machine.handle_event(EventType.TASK_START)
    
    # Health events
    state_machine.handle_event(EventType.WARN)
    state_machine.handle_event(EventType.CLEAR_WARNING)

State Queries
-------------

.. code-block:: python

    # Get current states
    lifecycle = state_machine.get_lifecycle_state()
    operational = state_machine.get_operational_state()
    health = state_machine.get_health_state()
    
    # Get full state report
    report = state_machine.get_full_state_report()
    # Returns: {
    #   "lifecycle": "Active",
    #   "operational": "Running",
    #   "health": "Healthy",
    #   "timestamp": "2025-01-19T10:30:00"
    # }

Event Callbacks
---------------

Register callbacks for state changes:

.. code-block:: python

    def on_lifecycle_change(old_state, new_state, event):
        print(f"Lifecycle: {old_state} → {new_state}")
    
    def on_operational_change(old_state, new_state, event):
        print(f"Operational: {old_state} → {new_state}")
    
    state_machine.register_lifecycle_callback(on_lifecycle_change)
    state_machine.register_operational_callback(on_operational_change)

Best Practices
==============

1. **Always use events for transitions**
   
   ❌ Don't manually set states
   
   ✅ Use ``handle_event()`` with appropriate EventType

2. **Check state before operations**

   .. code-block:: python

       if state_machine.get_operational_state() == OperationalState.READY:
           state_machine.handle_event(EventType.TASK_START)

3. **Handle Health escalations**

   .. code-block:: python

       def on_health_critical():
           # Health → Critical will trigger Lifecycle → Recovering
           # Implement recovery logic here
           pass

4. **Use callbacks for monitoring**

   Register callbacks to monitor all state changes for logging and diagnostics.

5. **Respect layer hierarchy**

   Don't try to force transitions that violate layer rules. The state machine
   will reject invalid transitions.

Implementation Details
======================

Layer Communication
-------------------

The layers communicate through:

1. **Event propagation**: Events flow through all layers
2. **State validation**: Each layer validates transitions
3. **Callbacks**: Registered callbacks notify about state changes

Thread Safety
-------------

The UnifiedStateMachine is **thread-safe**:

- All state modifications are protected by internal locks
- Event handling is atomic
- Callbacks are executed in the calling thread

Error Handling
--------------

Invalid transitions are handled gracefully:

.. code-block:: python

    try:
        state_machine.handle_event(EventType.TASK_START)
    except InvalidTransitionError as e:
        print(f"Transition not allowed: {e}")

See Also
========

- :doc:`states` - All state definitions
- :doc:`transitions` - Allowed transitions
- :doc:`events` - Event system documentation
- :doc:`api_reference` - Complete API reference
