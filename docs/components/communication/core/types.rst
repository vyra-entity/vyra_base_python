Core Communication Types
========================

Enums, type aliases, and metadata classes used across the entire ``vyra_base.com`` system.

.. automodule:: vyra_base.com.core.types
   :members:
   :undoc-members:
   :show-inheritance:

ProtocolType
------------

Identifies which communication protocol to use.

.. code-block:: python

   from vyra_base.com import ProtocolType

   class ProtocolType(str, Enum):
       # Transport Layer — in-process / low-latency
       ROS2   = "ros2"          # ROS2/DDS
       ZENOH  = "zenoh"         # Zenoh (DEFAULT fallback chain position 1)
       REDIS  = "redis"         # Redis pub/sub + KV
       UDS    = "uds"           # Unix Domain Sockets

       # External Layer — cross-service
       SHARED_MEMORY = "sharedmemory"
       MQTT          = "mqtt"
       GRPC          = "grpc"
       REST          = "rest"
       WEBSOCKET     = "websocket"

       # Industrial Layer
       MODBUS = "modbus"
       OPCUA  = "opcua"

**Fallback chain:** ``ZENOH → ROS2 → REDIS → UDS``

InterfaceType
-------------

Identifies the kind of communication interface.

.. code-block:: python

   from vyra_base.com import InterfaceType

   class InterfaceType(str, Enum):
       PUBLISHER     = "publisher"     # Publish-only
       SUBSCRIBER    = "subscriber"    # Subscribe with callback
       SERVER        = "server"        # Request-Response server
       CLIENT        = "client"        # Request-Response client
       ACTION_SERVER = "actionServer"  # Long-running task server
       ACTION_CLIENT = "actionClient"  # Long-running task client

AccessLevel
-----------

Controls who can access a service or topic.

.. code-block:: python

   from vyra_base.com import AccessLevel

   class AccessLevel(str, Enum):
       PUBLIC    = "public"    # All modules
       PROTECTED = "protected" # Authorized modules only
       PRIVATE   = "private"   # Same module only
       INTERNAL  = "internal"  # Framework internal

Type Aliases (returned by InterfaceFactory)
-------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Alias
     - Description
   * - ``VyraServer``
     - A running service server (has request/response handling)
   * - ``VyraClient``
     - A service client (can call a remote server)
   * - ``VyraPublisher``
     - A publisher — call ``.shout(data)`` to publish
   * - ``VyraSubscriber``
     - A subscriber — callback is invoked on each message
   * - ``VyraActionServer``
     - An action server (goal/feedback/result lifecycle)
   * - ``VyraActionClient``
     - An action client (``send_goal()``, feedback callback)
