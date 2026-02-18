COM API Overview
================

Complete reference of all public classes, decorators, and helpers in ``vyra_base.com``.

.. contents::
   :local:
   :depth: 2

---

Decorators
----------

Use these on component methods to expose them over the network.

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Decorator
     - Type
     - Description
   * - ``@remote_service``
     - Function
     - Expose a method as a request/response service server
   * - ``@remote_publisher``
     - Function
     - Expose a method as a message publisher
   * - ``@remote_subscriber``
     - Function
     - Register a method as a message subscriber callback
   * - ``@remote_actionServer.on_goal``
     - Method on class
     - Accept/reject incoming action goals
   * - ``@remote_actionServer.execute``
     - Method on class
     - Execute the action (main loop)
   * - ``@remote_actionServer.on_cancel``
     - Method on class
     - Handle cancellation requests

All decorators require your component class to call ``bind_decorated_callbacks()``
during Phase 2 initialization.

.. seealso:: :class:`~vyra_base.com.core.decorators`

---

Abstract Handlers
-----------------

Implement these interfaces in your component class.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Description
   * - ``IServiceHandler``
     - Marker interface for classes that expose ``@remote_service`` methods
   * - ``IActionHandler``
     - Required base for classes using ``@remote_actionServer``
   * - ``IGoalHandle``
     - Passed to ``execute()`` — use to publish feedback and set result

.. code-block:: python

   from vyra_base.com import IActionHandler, IGoalHandle

   class MyComponent(IActionHandler):
       @remote_actionServer.execute(name="task")
       async def execute(self, goal_handle: IGoalHandle):
           await goal_handle.publish_feedback({"progress": 50})
           goal_handle.succeed()
           return {"done": True}

---

InterfaceFactory
----------------

Creates live communication objects (servers, clients, publishers, subscribers, action objects).

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Method
     - Description
   * - ``create_server(name, callback, protocols)``
     - Create a service server
   * - ``create_client(name, protocols)``
     - Create a service client
   * - ``create_publisher(name, protocols)``
     - Create a publisher
   * - ``create_subscriber(name, callback, protocols)``
     - Create a subscriber
   * - ``create_action_client(name, protocols, feedback_callback)``
     - Create an action client
   * - ``create_from_blueprint(blueprint)``
     - Create interface from a Blueprint object
   * - ``get_available_protocols()``
     - Returns list of currently available ``ProtocolType`` values

.. seealso:: :class:`~vyra_base.com.core.factory.InterfaceFactory`

---

Blueprints
----------

Blueprints describe an interface *before* it is created (Phase 1 of two-phase init).
Created automatically by decorators; rarely needed manually.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Blueprint
     - Description
   * - ``ServiceBlueprint``
     - Describes a service server
   * - ``PublisherBlueprint``
     - Describes a publisher
   * - ``SubscriberBlueprint``
     - Describes a subscriber
   * - ``ActionBlueprint``
     - Describes an action server (groups goal/execute/cancel blueprints)
   * - ``CallbackRegistry``
     - Global registry — stores and retrieves all blueprints by name

---

Transport Layer
---------------

All four transports are **fully implemented**. Import from ``com.transport.t_*``.

.. list-table::
   :header-rows: 1
   :widths: 15 20 30 35

   * - Protocol
     - Module
     - Key Classes
     - Notes
   * - **Zenoh** (default)
     - ``transport.t_zenoh``
     - ``ZenohProvider``, ``ZenohSession``
     - Requires ``eclipse-zenoh``
   * - **ROS2**
     - ``transport.t_ros2``
     - ``VyraNode``, ``ROS2Publisher``, ``ROS2Subscriber``, ``ROS2ServiceServer``, ``ROS2ServiceClient``, ``ROS2ActionServer``, ``ROS2ActionClient``
     - Requires ROS2 + ``rclpy``
   * - **Redis**
     - ``transport.t_redis``
     - ``RedisProvider``, ``RedisClient``
     - Requires ``redis``
   * - **UDS**
     - ``transport.t_uds``
     - ``UDSProvider``
     - No extra dependencies

Fallback chain: ``Zenoh → ROS2 → Redis → UDS``

---

External Layer
--------------

.. list-table::
   :header-rows: 1
   :widths: 20 20 30 30

   * - Protocol
     - Module
     - Key Classes
     - Notes
   * - **gRPC**
     - ``external.grpc``
     - ``GrpcServer``, ``GrpcClient``
     - Requires ``grpcio``
   * - **MQTT**
     - ``external.mqtt``
     - ``MqttClient``
     - Requires ``paho-mqtt``
   * - **REST**
     - ``external.rest``
     - ``RestClient``
     - Requires ``aiohttp``
   * - **WebSocket**
     - ``external.websocket``
     - ``WebSocketClient``
     - Requires ``websockets``
   * - **Shared Memory**
     - ``external.shared_memory``
     - ``SharedMemorySegment``, ``SharedMemorySerializer``, ``SharedMemoryDiscovery``
     - Linux only; no extra deps

---

Industrial Layer
----------------

.. list-table::
   :header-rows: 1
   :widths: 20 20 30 30

   * - Protocol
     - Module
     - Key Classes
     - Notes
   * - **Modbus**
     - ``industrial.modbus``
     - TCP + RTU sub-modules
     - Requires ``pymodbus``
   * - **OPC UA**
     - ``industrial.opcua``
     - ``OpcuaClient``, ``OpcuaServer``
     - Requires ``asyncua``

---

Feeders
-------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Class
     - Description
   * - ``BaseFeeder``
     - Abstract base for all feeders
   * - ``StateFeeder``
     - Publishes lifecycle + operational state changes
   * - ``NewsFeeder``
     - Publishes informational messages
   * - ``ErrorFeeder``
     - Publishes error reports
   * - ``AvailableModuleFeeder``
     - Publishes module availability (heartbeat/discovery)

Use feeders via the entity:

.. code-block:: python

   entity.publish_news("Ready")
   entity.publish_error("Storage unavailable")

---

Type System
-----------

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Type / Enum
     - Description
   * - ``ProtocolType``
     - All supported protocols (``ROS2``, ``ZENOH``, ``REDIS``, ``UDS``, ``MQTT``, ``GRPC``, ``REST``, ``WEBSOCKET``, ``SHARED_MEMORY``, ``MODBUS``, ``OPCUA``)
   * - ``InterfaceType``
     - Interface kinds: ``SERVER``, ``CLIENT``, ``PUBLISHER``, ``SUBSCRIBER``, ``ACTION_SERVER``, ``ACTION_CLIENT``
   * - ``AccessLevel``
     - Security access control: ``PUBLIC``, ``PROTECTED``, ``PRIVATE``, ``INTERNAL``
   * - ``ActionStatus``
     - Action result status: ``SUCCEEDED``, ``ABORTED``, ``CANCELED``, ``EXECUTING``
   * - ``VyraServer``
     - Type alias for a running service server
   * - ``VyraClient``
     - Type alias for a service client
   * - ``VyraPublisher``
     - Type alias for a publisher (has ``.shout()`` method)
   * - ``VyraSubscriber``
     - Type alias for a subscriber
   * - ``VyraActionServer``
     - Type alias for an action server
   * - ``VyraActionClient``
     - Type alias for an action client

---

Topic Builder
-------------

Generates consistent topic / service names following VYRA naming conventions.

.. code-block:: python

   from vyra_base.com import build_topic, parse_topic, InterfaceType

   # Build a topic name
   name = build_topic(namespace="my_module", name="temperature", interface_type=InterfaceType.PUBLISHER)
   # → "my_module/temperature"

   # Parse an existing topic back to its components
   components = parse_topic("my_module/temperature")
   print(components.namespace, components.name)

---

Exceptions
----------

All exceptions live in ``vyra_base.com.core.exceptions``.

.. code-block:: python

   from vyra_base.com import (
       CommunicationError,          # Base
       ProtocolUnavailableError,    # Protocol not installed / unreachable
       ProtocolNotInitializedError, # Protocol not set up yet
       TransportError,              # Low-level transport failure
       InterfaceError,              # Interface creation failed
       TServerError,                # Server runtime error
       TSubscriberError,            # Subscriber error
       ActionServerError,           # Action server error
   )

---

Further Reading
---------------

* :doc:`../com` — Transport, External, Industrial — full module reference
* :doc:`core/factory` — InterfaceFactory API
* :doc:`core/decorators` — Decorator API
* :doc:`core/types` — Type system
* :doc:`../quickstart` — Step-by-step module building guide
