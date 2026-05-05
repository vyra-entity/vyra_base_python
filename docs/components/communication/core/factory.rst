Interface Factory
=================

Unified interface creation with automatic protocol selection and fallback.

Overview
--------

The ``InterfaceFactory`` provides a high-level API for creating communication interfaces
with automatic protocol selection and graceful fallback. It supports both server-side
(responding to requests) and client-side (making requests) patterns.

**Key Features:**

- Automatic protocol selection (Zenoh → ROS2 → Redis → UDS)  
- Server/Client pattern with explicit methods
- Publisher/Subscriber pattern support
- Action Server/Client for long-running tasks
- Protocol availability checking
- Graceful degradation
- Customizable fallback chains

Factory Methods
---------------

Server-Side Methods
^^^^^^^^^^^^^^^^^^^

These methods create interfaces that respond to requests or execute tasks:

**create_callable(name, callback, protocols=None, \*\*kwargs)**

Creates a service server (responds to service calls).

:param name: Service name
:param callback: Server callback function ``(request, response) -> response``
:param protocols: Optional list of protocols to try (defaults to CALLABLE_FALLBACK)
:param kwargs: Additional protocol-specific parameters
:returns: ``VyraCallable`` instance
:raises InterfaceError: If no protocol is available

.. code-block:: python

   # Service Server
   server = await InterfaceFactory.create_callable(
       "calculate",
       callback=lambda req, res: setattr(res, 'result', req.a + req.b) or res
   )

**create_speaker(name, callback=None, protocols=None, \*\*kwargs)**

Creates a publisher (publishes messages to topics).

:param name: Topic name
:param callback: Optional callback (unused for publisher)
:param protocols: Optional list of protocols to try (defaults to SPEAKER_FALLBACK)
:param kwargs: Additional protocol-specific parameters
:returns: ``VyraSpeaker`` instance
:raises InterfaceError: If no protocol is available

.. code-block:: python

   # Publisher
   speaker = await InterfaceFactory.create_speaker("events")
   await speaker.shout({"event": "update"})

**create_job(name, callback, protocols=None, \*\*kwargs)**

Creates an action server (executes long-running tasks).

:param name: Action name
:param callback: Action execution callback
:param protocols: Optional list of protocols to try (defaults to JOB_FALLBACK)
:param kwargs: Additional protocol-specific parameters
:returns: ``VyraJob`` instance
:raises InterfaceError: If no protocol is available

.. code-block:: python

   # Action Server
   async def execute_task(goal, feedback_callback=None):
       for i in range(100):
           if feedback_callback:
               await feedback_callback({"progress": i})
       return {"status": "completed"}
   
   job = await InterfaceFactory.create_job("process", callback=execute_task)

Client-Side Methods
^^^^^^^^^^^^^^^^^^^

These methods create interfaces that make requests or send goals:

**create_caller(name, protocols=None, \*\*kwargs)** *(NEW!)*

Creates a service client (calls remote services).

:param name: Service name
:param protocols: Optional list of protocols to try (defaults to CALLABLE_FALLBACK)
:param kwargs: Additional protocol-specific parameters
:returns: ``VyraCallable`` instance configured as client
:raises InterfaceError: If no protocol is available

.. code-block:: python

   # Service Client
   caller = await InterfaceFactory.create_caller("calculate")
   response = await caller.call(request, timeout=5.0)

**create_listener(name, callback, protocols=None, \*\*kwargs)** *(NEW!)*

Creates a subscriber (receives messages from topics).

:param name: Topic name
:param callback: Message callback function ``(message) -> None``
:param protocols: Optional list of protocols to try (defaults to SPEAKER_FALLBACK)
:param kwargs: Additional protocol-specific parameters
:returns: ``VyraSpeaker`` instance configured as subscriber
:raises InterfaceError: If no protocol is available

.. code-block:: python

   # Subscriber
   listener = await InterfaceFactory.create_listener(
       "events",
       callback=lambda msg: print(f"Received: {msg}")
   )

**create_dispatcher(name, protocols=None, \*\*kwargs)** *(NEW!)*

Creates an action client (sends goals to action servers).

:param name: Action name
:param protocols: Optional list of protocols to try (defaults to JOB_FALLBACK)
:param kwargs: Additional protocol-specific parameters (e.g., feedback_callback)
:returns: ``VyraJob`` instance configured as client
:raises InterfaceError: If no protocol is available

.. code-block:: python

   # Action Client
   dispatcher = await InterfaceFactory.create_dispatcher(
       "process",
       feedback_callback=lambda fb: print(f"Progress: {fb}")
   )
   result = await dispatcher.execute(goal)

Configuration Methods
^^^^^^^^^^^^^^^^^^^^^

**register_provider(provider)**

Register a protocol provider with the factory.

:param provider: ``AbstractProtocolProvider`` instance or list of providers

.. code-block:: python

   from vyra_base.com.transport.t_ros2 import ROS2Provider
   
   provider = ROS2Provider("my_module", "id123")
   await provider.initialize()
   InterfaceFactory.register_provider(provider)

**unregister_provider(protocol)**

Unregister a protocol provider.

:param protocol: ``ProtocolType`` enum value

**set_fallback_chain(interface_type, protocols)**

Customize the protocol fallback order for a specific interface type.

:param interface_type: "callable", "speaker", or "job"
:param protocols: Ordered list of ``ProtocolType`` to try

.. code-block:: python

   # Prioritize UDS for callables
   InterfaceFactory.set_fallback_chain(
       "callable",
       [ProtocolType.UDS, ProtocolType.REDIS, ProtocolType.ROS2]
   )

**get_available_protocols()**

Returns a list of currently available protocols.

:returns: ``List[ProtocolType]``

.. code-block:: python

   protocols = InterfaceFactory.get_available_protocols()
   if ProtocolType.ROS2 in protocols:
       print("ROS2 is available")

Usage Examples
--------------

Complete Server-Client Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   from vyra_base.com.core.types import ProtocolType
   
   # Initialize provider
   from vyra_base.com.transport.t_ros2 import ROS2Provider
   provider = ROS2Provider("my_module", "id123")
   await provider.initialize()
   InterfaceFactory.register_provider(provider)
   
   # SERVER: Create service server
   async def handle_request(request, response):
       response.result = request.a + request.b
       return response
   
   server = await InterfaceFactory.create_callable(
       "add_service",
       callback=handle_request,
       service_type=AddTwoInts
   )
   
   # CLIENT: Call the service
   caller = await InterfaceFactory.create_caller(
       "add_service",
       service_type=AddTwoInts
   )
   
   request = AddTwoInts.Request()
   request.a = 5
   request.b = 3
   response = await caller.call(request)
   print(f"Result: {response.sum}")  # 8

Publisher-Subscriber Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # PUBLISHER
   speaker = await InterfaceFactory.create_speaker(
       "temperature",
       message_type=Temperature
   )
   await speaker.shout(Temperature(value=23.5))
   
   # SUBSCRIBER
   def on_temperature(msg):
       print(f"Temperature: {msg.value}°C")
   
   listener = await InterfaceFactory.create_listener(
       "temperature",
       callback=on_temperature,
       message_type=Temperature
   )

Action Server-Client Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # ACTION SERVER
   async def execute_fibonacci(goal, feedback_callback=None):
       sequence = [0, 1]
       for i in range(1, goal.order):
           sequence.append(sequence[-1] + sequence[-2])
           if feedback_callback:
               await feedback_callback({"sequence": sequence})
       return {"sequence": sequence}
   
   job = await InterfaceFactory.create_job(
       "fibonacci",
       callback=execute_fibonacci,
       action_type=Fibonacci
   )
   
   # ACTION CLIENT
   def on_feedback(fb):
       print(f"Current sequence: {fb['sequence']}")
   
   dispatcher = await InterfaceFactory.create_dispatcher(
       "fibonacci",
       feedback_callback=on_feedback,
       action_type=Fibonacci
   )
   
   goal = Fibonacci.Goal(order=10)
   result = await dispatcher.execute(goal)
   print(f"Final: {result.result.sequence}")

Explicit Protocol Selection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Try specific protocols only
   caller = await InterfaceFactory.create_caller(
       "fast_service",
       protocols=[ProtocolType.UDS, ProtocolType.SHARED_MEMORY]
   )

Fallback Chain Customization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Prioritize Zenoh for all speakers
   InterfaceFactory.set_fallback_chain(
       "speaker",
       [ProtocolType.ZENOH, ProtocolType.REDIS, ProtocolType.MQTT]
   )

Default Fallback Chains
------------------------

**CALLABLE_FALLBACK:**
  - Zenoh → ROS2 → Redis → UDS

**SPEAKER_FALLBACK:**
  - Zenoh → ROS2 → Redis → UDS

**JOB_FALLBACK:**
  - Zenoh → ROS2 → Redis → UDS

Server vs Client Behavior
--------------------------

The factory automatically determines server/client behavior based on the method used:

.. list-table::
   :header-rows: 1
   :widths: 25 20 20 35

   * - Method
     - Role
     - Flag
     - Behavior
   * - ``create_callable()``
     - Server
     - ``is_callable=True``
     - Responds to service calls
   * - ``create_caller()``
     - Client
     - ``is_callable=False``
     - Makes service calls
   * - ``create_speaker()``
     - Publisher
     - ``is_publisher=True``
     - Publishes messages
   * - ``create_listener()``
     - Subscriber
     - ``is_publisher=False``
     - Receives messages
   * - ``create_job()``
     - Server
     - ``is_job=True``
     - Executes actions
   * - ``create_dispatcher()``
     - Client
     - ``is_job=False``
     - Sends goals

Error Handling
--------------

.. code-block:: python

   from vyra_base.com.core.exceptions import (
       ProtocolUnavailableError,
       InterfaceError
   )
   
   try:
       caller = await InterfaceFactory.create_caller(
           "service",
           protocols=[ProtocolType.ROS2]
       )
   except ProtocolUnavailableError:
       # ROS2 not available, try fallback
       caller = await InterfaceFactory.create_caller(
           "service",
           protocols=[ProtocolType.UDS]
       )
   except InterfaceError as e:
       logger.error(f"Failed to create interface: {e}")

Best Practices
--------------

1. **Use explicit server/client methods** for clarity:

   .. code-block:: python

      # ✅ Good - Clear intent
      server = await InterfaceFactory.create_callable(...)
      client = await InterfaceFactory.create_caller(...)
      
      # ❌ Avoid - Ambiguous
      interface = await InterfaceFactory.create_callable(...)  # Server or client?

2. **Register providers once at startup**:

   .. code-block:: python

      # ✅ Good - Register once
      provider = ROS2Provider(...)
      await provider.initialize()
      InterfaceFactory.register_provider(provider)

3. **Check availability before explicit protocol selection**:

   .. code-block:: python

      # ✅ Good - Check first
      if ProtocolType.ROS2 in InterfaceFactory.get_available_protocols():
          interface = await InterfaceFactory.create_caller(
              protocols=[ProtocolType.ROS2]
          )

4. **Use type hints and docstrings**:

   .. code-block:: python

      # ✅ Good - Clear types
      async def handle_request(
          request: AddTwoInts.Request,
          response: AddTwoInts.Response
      ) -> AddTwoInts.Response:
          """Add two integers."""
          response.sum = request.a + request.b
          return response

See Also
--------

- :doc:`types` - Interface types and enums
- :doc:`exceptions` - Communication exceptions
- :doc:`decorators` - Protocol-agnostic decorators
- :doc:`../transport/ros2` - ROS2 transport provider
- :doc:`../transport/redis` - Redis transport provider

.. automodule:: vyra_base.com.core.factory
   :members:
   :undoc-members:
   :show-inheritance:
