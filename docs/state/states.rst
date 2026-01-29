==================
State Definitions
==================

This document defines all states across the three layers of the State Machine.

Lifecycle States
================

Lifecycle states control the existence and initialization of a module.

Initializing
------------

**Description**: Module is currently initializing.

**Allowed transitions**:

- → **Active** (via ``INIT_SUCCESS``)
- → **Recovering** (via ``INIT_FAILURE``)

**Operational states allowed**: None (FSM locked)

**Use cases**:

- Loading configuration
- Establishing connections
- Initializing resources

Active
------

**Description**: Module is active and operational.

**Allowed transitions**:

- → **Recovering** (via ``FAULT_DETECTED``)
- → **ShuttingDown** (via ``SHUTDOWN``)

**Operational states allowed**: All (Idle, Ready, Running, BackgroundRunning, Paused, Stopped)

**Use cases**:

- Normal operation
- Processing tasks
- Serving requests

Recovering
----------

**Description**: Module is recovering from an error.

**Allowed transitions**:

- → **Active** (via ``RECOVERY_SUCCESS``)
- → **ShuttingDown** (via ``RECOVERY_FAILED``)

**Operational states allowed**: Limited (Idle, Paused, Stopped)

**Use cases**:

- Error recovery procedures
- Reconnecting to services
- Resetting internal state

ShuttingDown
------------

**Description**: Module is shutting down gracefully.

**Allowed transitions**:

- → **Offline** (via ``FINISHED``)

**Operational states allowed**: Idle only

**Use cases**:

- Cleanup operations
- Saving state
- Closing connections

Offline
-------

**Description**: Module is offline and not operational.

**Allowed transitions**:

- → **Initializing** (via ``START``)

**Operational states allowed**: Idle, Stopped

**Use cases**:

- Before startup
- After shutdown
- Maintenance mode

Operational States
==================

Operational states define what the module is currently doing.

Idle
----

**Description**: Module is idle, not performing any tasks.

**Allowed transitions**:

- → **Ready** (via ``SET_READY``)

**Use cases**:

- Waiting for commands
- Initial state after initialization

Ready
-----

**Description**: Module is ready to execute tasks.

**Allowed transitions**:

- → **Running** (via ``TASK_START``)

**Use cases**:

- Ready to process requests
- Waiting for task assignment

Running
-------

**Description**: Module is actively executing a task.

**Allowed transitions**:

- → **Ready** (via ``TASK_COMPLETE``)
- → **Paused** (via ``TASK_PAUSE``)
- → **Stopped** (via ``TASK_STOP``)
- → **BackgroundRunning** (via ``SET_BACKGROUND``)

**Use cases**:

- Processing data
- Executing commands
- Active task execution

BackgroundRunning
-----------------

**Description**: Module is executing a task in background mode.

**Allowed transitions**:

- → **Running** (via ``SET_FOREGROUND``)
- → **Paused** (via ``TASK_PAUSE``)
- → **Stopped** (via ``TASK_STOP``)

**Use cases**:

- Low-priority background processing
- Monitoring tasks
- Non-interactive operations

Paused
------

**Description**: Module has paused task execution.

**Allowed transitions**:

- → **Ready** (via ``TASK_RESUME``)

**Use cases**:

- Temporary pause requested by user
- Waiting for resources
- Coordinated multi-module operations

Stopped
-------

**Description**: Module has stopped and requires reset.

**Allowed transitions**:

- → **Idle** (via ``TASK_RESET``)

**Use cases**:

- Task cancelled
- Error requiring manual intervention
- Requested stop by user

Health States
=============

Health states monitor the functional status of the module.

Healthy
-------

**Description**: Module is functioning normally.

**Allowed transitions**:

- → **Warning** (via ``WARN``)
- → **Critical** (via ``FAULT``)

**Impact on other layers**: None

**Use cases**:

- Normal operation
- All systems nominal

Warning
-------

**Description**: Module has non-critical warnings.

**Allowed transitions**:

- → **Healthy** (via ``CLEAR_WARNING``)
- → **Critical** (via ``FAULT``)

**Impact on other layers**: 

- Operational: May continue with restrictions
- Lifecycle: Remains Active (monitoring)

**Use cases**:

- High CPU usage
- Low memory
- Slow response times
- Deprecated API usage

Critical
--------

**Description**: Module has critical errors.

**Allowed transitions**:

- → **Healthy** (via ``RECOVER``)
- → **Warning** (via ``RECOVER``)

**Impact on other layers**: 

- Operational: Forced to Stopped
- Lifecycle: Forced to Recovering or ShuttingDown (escalation)

**Use cases**:

- Connection lost
- Critical resource unavailable
- Data corruption detected
- Security violation

State Transition Rules
======================

Cross-Layer Rules
-----------------

1. **Lifecycle overrides Operational**
   
   If Lifecycle is not Active, Operational states are restricted.

2. **Health escalates to Lifecycle**
   
   Critical health state can force Lifecycle to Recovering or ShuttingDown.

3. **Operational cannot affect Lifecycle**
   
   Operational states cannot change Lifecycle states directly.

State Persistence
-----------------

States are persisted in Redis for recovery after restarts:

.. code-block:: python

    # States are automatically saved
    state_machine.handle_event(EventType.TASK_START)
    
    # States can be restored after restart
    state_machine.restore_state()

State Validation
----------------

All state transitions are validated before execution:

.. code-block:: python

    # Valid transition
    state_machine.handle_event(EventType.TASK_START)  # ✅ Allowed
    
    # Invalid transition (will raise exception)
    state_machine.handle_event(EventType.TASK_START)  # ❌ Already Running

See Also
========

- :doc:`overview` - Layer architecture and hierarchy
- :doc:`transitions` - Detailed transition rules
- :doc:`events` - Event types and usage
