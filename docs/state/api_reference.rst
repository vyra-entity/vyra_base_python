==================
API Reference
==================

Complete API reference for the VYRA State Machine system.

UnifiedStateMachine
===================

The main class that manages all three state layers.

.. autoclass:: vyra_base.state.unified.UnifiedStateMachine
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

OperationalStateMachine
=======================

Base class for components that use operational state management.

.. autoclass:: vyra_base.state.operational_state_machine.OperationalStateMachine
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

State Types
===========

LifecycleState
--------------

.. autoclass:: vyra_base.state.state_types.LifecycleState
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

OperationalState
----------------

.. autoclass:: vyra_base.state.state_types.OperationalState
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

HealthState
-----------

.. autoclass:: vyra_base.state.state_types.HealthState
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

Event System
============

StateEvent
----------

.. autoclass:: vyra_base.state.state_events.StateEvent
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

EventType
---------

.. autoclass:: vyra_base.state.state_events.EventType
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

State Machines
==============

The state machine implementation consists of three layer classes that work together
within the UnifiedStateMachine. These are internal implementation details.

For usage, refer to the :class:`~vyra_base.state.unified.UnifiedStateMachine` class above.

Decorators
==========

The ``@remote_service`` decorator is used to mark methods that should be exposed as ROS2 services.
It automatically handles state transitions and validation.

Usage:

.. code-block:: python

    from vyra_base.com import remote_service
    
    class MyComponent(OperationalStateMachine):
        @remote_service
        def initialize(self, request=None, response=None):
            return True

Exceptions
==========

InvalidTransitionError
----------------------

Raised when an invalid state transition is attempted.

.. code-block:: python

    from vyra_base.state.state_types import InvalidTransitionError
    
    try:
        state_machine.handle_event(EventType.TASK_START)
    except InvalidTransitionError as e:
        print(f"Invalid transition: {e}")

OperationalStateError
---------------------

Raised when an operational state error occurs.

.. code-block:: python

    from vyra_base.state.state_types import OperationalStateError
    
    try:
        component.start()
    except OperationalStateError as e:
        print(f"Operational error: {e}")

Complete Module Reference
==========================

state.unified
-------------

.. automodule:: vyra_base.state.unified
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.lifecycle_layer
---------------------

.. automodule:: vyra_base.state.lifecycle_layer
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.operational_layer
-----------------------

.. automodule:: vyra_base.state.operational_layer
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.health_layer
------------------

.. automodule:: vyra_base.state.health_layer
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.state_machine
-------------------

.. automodule:: vyra_base.state.state_machine
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.state_events
------------------

.. automodule:: vyra_base.state.state_events
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.state_types
-----------------

.. automodule:: vyra_base.state.state_types
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.operational_metaclass
---------------------------

.. automodule:: vyra_base.state.operational_metaclass
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:

state.operation_decorator
-------------------------

.. automodule:: vyra_base.state.operation_decorator
   :members:
   :undoc-members:
   :show-inheritance:
   :no-index:
