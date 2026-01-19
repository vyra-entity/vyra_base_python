=====================
State Machine
=====================

The VYRA Base State Machine follows industrial standards (IEC 61508, IEC 61131-3, ISO 13849) 
and consists of three hierarchical layers:

- **Lifecycle States** (Existence / startup / shutdown)
- **Operational States** (Activity states)
- **Health States** (Functional and error status)

.. toctree::
   :maxdepth: 2
   :caption: State Machine Documentation

   statemachine/overview
   statemachine/states
   statemachine/transitions
   statemachine/events
   statemachine/api_reference
   statemachine/quick_reference

Overview
========

The three layers are **not parallel**, but **nested within each other**:

.. code-block:: text

    Lifecycle controls Operational
    Operational is regulated by Health
    Health can override Lifecycle (escalation)

Layer Influence Rules
---------------------

Each layer may only influence in one direction:

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Source
     - May Influence
     - May NOT Influence
   * - **Lifecycle**
     - Operational + Health
     - —
   * - **Operational**
     - —
     - Lifecycle, Health
   * - **Health**
     - Lifecycle (hard) + Operational (soft)
     - —

Quick Example
=============

.. code-block:: python

    from vyra_base.state import UnifiedStateMachine
    
    # Initialize state machine
    state_machine = UnifiedStateMachine(
        module_name="my_module",
        enable_lifecycle=True,
        enable_health=True
    )
    
    # Handle lifecycle events
    state_machine.handle_event(EventType.START)
    state_machine.handle_event(EventType.INIT_SUCCESS)
    
    # Handle operational events
    state_machine.handle_event(EventType.SET_READY)
    state_machine.handle_event(EventType.TASK_START)
    
    # Check current states
    print(state_machine.get_lifecycle_state())    # Active
    print(state_machine.get_operational_state())  # Running
    print(state_machine.get_health_state())       # Healthy

See Also
========

- :doc:`statemachine/overview` - Complete architecture and layer interactions
- :doc:`statemachine/states` - All state definitions
- :doc:`statemachine/transitions` - Allowed state transitions
- :doc:`statemachine/events` - Event system documentation
- :doc:`statemachine/api_reference` - Complete API reference
- :doc:`statemachine/quick_reference` - Quick reference guide
