==================
State Transitions
==================

This document defines all allowed state transitions for each layer.

Lifecycle Transitions
=====================

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 40

   * - From
     - Event
     - To
     - Description
   * - Initializing
     - init_success
     - Active
     - Initialization completed successfully.
   * - Initializing
     - init_failure
     - Recovering
     - Initialization failed, recovery needed.
   * - Active
     - shutdown
     - ShuttingDown
     - Controlled shutdown initiated.
   * - Active
     - fault_detected
     - Recovering
     - Error detected, recovery process starts.
   * - Recovering
     - recovery_success
     - Active
     - Recovery successful, back to active.
   * - Recovering
     - recovery_failed
     - ShuttingDown
     - Recovery failed, shutdown required.
   * - ShuttingDown
     - finished
     - Offline
     - Shutdown completed, module offline.

Lifecycle State Diagram
------------------------

.. code-block:: text

              START
                │
                ▼
         ┌─────────────┐
         │Initializing │
         └─────────────┘
          │          │
    init_ │          │ init_
   success│          │ failure
          │          │
          ▼          ▼
      ┌──────┐   ┌───────────┐
      │Active│◄──│ Recovering│
      └──────┘   └───────────┘
          │           │
    shutdown     recovery_
          │        failed
          │           │
          ▼           ▼
     ┌──────────────┐
     │ShuttingDown  │
     └──────────────┘
          │
       finished
          │
          ▼
      ┌────────┐
      │Offline │
      └────────┘

Operational Transitions
=======================

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 40

   * - From
     - Event
     - To
     - Description
   * - Idle
     - set_ready
     - Ready
     - Module ready for tasks.
   * - Ready
     - task_start
     - Running
     - Task execution started.
   * - Running
     - task_pause
     - Paused
     - Task paused, can be resumed.
   * - Running
     - task_stop
     - Stopped
     - Task stopped, reset required.
   * - Running
     - set_background
     - BackgroundRunning
     - Switch to background processing.
   * - Running
     - task_complete
     - Ready
     - Task completed successfully.
   * - BackgroundRunning
     - set_foreground
     - Running
     - Return to foreground task execution.
   * - BackgroundRunning
     - task_pause
     - Paused
     - Background processing paused.
   * - BackgroundRunning
     - task_stop
     - Stopped
     - Background processing stopped.
   * - Paused
     - task_resume
     - Ready
     - Task resumed, back to Ready.
   * - Stopped
     - task_reset
     - Idle
     - Operational state reset to Idle.

Operational State Diagram
--------------------------

.. code-block:: text

         ┌──────┐
         │ Idle │
         └──────┘
            │
       set_ready
            │
            ▼
         ┌──────┐     task_start       ┌─────────┐
         │Ready │◄─────────────────────┤ Running │
         └──────┘                      └─────────┘
            ▲                           │  │  │
            │                           │  │  │ set_
            │                      task_│  │  │background
            │                     complete │  │
            │                           │  │  ▼
            │                           │  │ ┌──────────────────┐
            │                           │  │ │BackgroundRunning │
            │                           │  │ └──────────────────┘
            │                           │  │      │
            │                      task_│  │ task_│
         task_                     pause│  │pause │
         resume                         │  │      │
            │                           ▼  ▼      ▼
            │                        ┌────────┐
            └────────────────────────┤ Paused │
                                     └────────┘
                                         
         ┌─────────┐                      
         │ Stopped │◄─────────────────────┘
         └─────────┘      task_stop
            │
         task_
         reset
            │
            ▼
         ┌──────┐
         │ Idle │
         └──────┘

Health Transitions
==================

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 40

   * - From
     - Event
     - To
     - Description
   * - Healthy
     - warn
     - Warning
     - Warning detected (non-critical).
   * - Healthy
     - fault
     - Critical
     - Critical error directly detected.
   * - Warning
     - clear_warning
     - Healthy
     - Warning resolved, back to healthy.
   * - Warning
     - fault
     - Critical
     - Warning escalated to critical error.
   * - Critical
     - recover
     - Healthy
     - Critical error resolved, back to healthy.
   * - Critical
     - recover
     - Warning
     - Partial recovery, warnings still active.

Health State Diagram
--------------------

.. code-block:: text

         ┌─────────┐
         │ Healthy │◄───────────────┐
         └─────────┘                │
            │    ▲              recover
            │    │                  │
        warn│    │clear_warning     │
            │    │                  │
            ▼    │                  │
         ┌─────────┐                │
         │ Warning │                │
         └─────────┘                │
            │                       │
         fault                      │
            │                       │
            ▼                       │
         ┌──────────┐               │
         │ Critical │───────────────┘
         └──────────┘

Cross-Layer Transitions
=======================

Special Events
--------------

These events can affect multiple layers simultaneously:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Event
     - Impact
   * - interrupt
     - Interrupts current operation, can affect all layers.
   * - emergency_stop
     - Emergency stop: Health → Critical, Lifecycle → ShuttingDown, Operational → Stopped (highest priority).
   * - priority_override
     - Override event for special system states (Admin/Debug).

Emergency Stop Sequence
-----------------------

When ``EMERGENCY_STOP`` event is triggered:

.. code-block:: text

    1. Health → Critical
    2. Operational → Stopped
    3. Lifecycle → ShuttingDown
    4. Module safely shuts down

Example:

.. code-block:: python

    # Emergency stop sequence
    state_machine.handle_event(EventType.EMERGENCY_STOP)
    
    # Results in:
    # Health: Critical
    # Operational: Stopped
    # Lifecycle: ShuttingDown

Transition Validation
=====================

Rules
-----

1. **Lifecycle State Controls Operational States**

   .. code-block:: python

       # If Lifecycle is Recovering:
       # Only Idle, Paused, Stopped are allowed in Operational
       
       # This will be rejected:
       state_machine.handle_event(EventType.TASK_START)  # ❌

2. **Health Critical Forces Operational Stop**

   .. code-block:: python

       # If Health becomes Critical:
       # Operational is forced to Stopped
       
       state_machine.handle_event(EventType.FAULT)
       # Operational automatically → Stopped

3. **Invalid Transitions Are Rejected**

   .. code-block:: python

       # Try invalid transition
       try:
           state_machine.handle_event(EventType.TASK_START)
           state_machine.handle_event(EventType.TASK_START)  # ❌ Already Running
       except InvalidTransitionError as e:
           print(f"Transition rejected: {e}")

Transition Guards
-----------------

Before executing a transition, the state machine checks:

1. **Current state allows this transition**
2. **Event is valid for current state**
3. **Cross-layer rules are respected**
4. **No locks prevent the transition**

Example:

.. code-block:: python

    # Transition guard example
    def can_transition(from_state, event, to_state):
        # Check lifecycle rules
        if not lifecycle_allows_operational_state(to_state):
            return False
        
        # Check health escalation
        if health_requires_escalation():
            return False
        
        return True

Atomic Transitions
------------------

All transitions are **atomic** and **thread-safe**:

.. code-block:: python

    # Multiple threads can safely call handle_event
    import threading
    
    def thread_task():
        state_machine.handle_event(EventType.TASK_START)
    
    threads = [threading.Thread(target=thread_task) for _ in range(10)]
    for t in threads:
        t.start()
    
    # Only one thread will successfully transition
    # Others will receive appropriate exceptions

See Also
========

- :doc:`overview` - Layer architecture
- :doc:`states` - State definitions
- :doc:`events` - Event system
