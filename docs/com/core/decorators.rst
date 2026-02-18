Communication Decorators
========================

Protocol-agnostic decorators that transform component methods into communication interfaces.

.. automodule:: vyra_base.com.core.decorators
   :members:
   :undoc-members:
   :show-inheritance:

Overview
--------

All decorators follow the **two-phase initialization** pattern:

1. **Phase 1** — The decorator runs at class-definition time and creates a :class:`.Blueprint`.
   No network connection is made yet.
2. **Phase 2** — Calling :func:`bind_decorated_callbacks` attaches your instance methods
   to the blueprints. After that, ``InterfaceFactory`` can create live interfaces.

Decorator Reference
-------------------

@remote_service
~~~~~~~~~~~~~~~

Expose a method as a request/response service.

.. code-block:: python

   from vyra_base.com import remote_service, ProtocolType, AccessLevel

   class MyComponent:
       @remote_service(
           name="calculate",
           protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
           namespace="my_module",
           access_level=AccessLevel.PUBLIC,
       )
       async def calculate(self, request, response=None):
           return {"result": request["x"] + request["y"]}

Parameters:

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Parameter
     - Type
     - Description
   * - ``name``
     - ``str``
     - Service name (defaults to function name)
   * - ``protocols``
     - ``list[ProtocolType]``
     - Preferred protocols in fallback order
   * - ``namespace``
     - ``str``
     - Topic namespace prefix (usually your module name)
   * - ``access_level``
     - ``AccessLevel``
     - Who can call this service (default: ``PUBLIC``)

@remote_publisher
~~~~~~~~~~~~~~~~~

Expose a method as a message publisher.

.. code-block:: python

   from vyra_base.com import remote_publisher, ProtocolType

   class SensorComponent:
       @remote_publisher(
           name="temperature",
           protocols=[ProtocolType.ZENOH],
           namespace="sensors",
       )
       async def publish_temperature(self, value: float):
           return {"value": value, "unit": "°C"}

The ``return`` value becomes the message payload published to subscribers.

@remote_subscriber
~~~~~~~~~~~~~~~~~~

Register a method as a subscriber callback.

.. code-block:: python

   from vyra_base.com import remote_subscriber, ProtocolType

   class DashboardComponent:
       @remote_subscriber(
           name="temperature",
           protocols=[ProtocolType.ZENOH],
           namespace="sensors",
       )
       async def on_temperature(self, message: dict):
           print(f"Temperature: {message['value']} {message['unit']}")

@remote_actionServer
~~~~~~~~~~~~~~~~~~~~

Register a group of methods as an action server.
Your class **must** inherit from :class:`IActionHandler`.

.. code-block:: python

   from vyra_base.com import remote_actionServer, IActionHandler, IGoalHandle, ProtocolType

   class ProcessingComponent(IActionHandler):

       @remote_actionServer.on_goal(name="run", protocols=[ProtocolType.ZENOH])
       async def on_goal(self, goal_request: dict) -> bool:
           """Accept (True) or reject (False) the incoming goal."""
           return goal_request.get("dataset") is not None

       @remote_actionServer.execute(name="run")
       async def execute(self, goal_handle: IGoalHandle):
           """Run the action; publish feedback; set result."""
           for i in range(100):
               await goal_handle.publish_feedback({"progress": i})
           goal_handle.succeed()
           return {"status": "done"}

       @remote_actionServer.on_cancel(name="run")
       async def on_cancel(self, goal_handle: IGoalHandle) -> bool:
           """Return True to accept cancellation."""
           return True

Helper Functions
----------------

.. code-block:: python

   from vyra_base.com import get_decorated_methods, bind_decorated_callbacks

   # Inspect all decorated methods on a component instance
   methods = get_decorated_methods(component)

   # Phase 2: attach instance callbacks to blueprints
   bind_decorated_callbacks(component, namespace="my_module")

.. seealso::

   :doc:`../../quickstart` — Step 2 shows full usage in a module context
