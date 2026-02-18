Communication (com)
====================

The ``com`` module is the multi-protocol communication layer of the VYRA framework.
It provides a unified API for **Transport** (Zenoh, ROS2, Redis, UDS),
**External** (gRPC, MQTT, REST, WebSocket, Shared Memory), and
**Industrial** (Modbus, OPC UA) protocols with automatic protocol selection and fallback.

.. tip::

   For a step-by-step guide on using communication in your own module see
   :doc:`quickstart` — especially steps 2 and 7.

.. toctree::
   :maxdepth: 2
   :caption: Core Components

   com/core/factory
   com/core/types
   com/core/exceptions
   com/core/decorators
   com/core/registry

.. toctree::
   :maxdepth: 2
   :caption: Transport Layer

   com/transport/ros2
   com/transport/zenoh
   com/transport/redis
   com/transport/uds

.. toctree::
   :maxdepth: 2
   :caption: External Protocols

   com/external/grpc
   com/external/mqtt
   com/external/rest
   com/external/websocket
   com/external/shared_memory
   com/external/registry

.. toctree::
   :maxdepth: 2
   :caption: Industrial Protocols

   com/industrial/modbus
   com/industrial/opcua

.. toctree::
   :maxdepth: 2
   :caption: Providers

   com/providers/protocol_provider
   com/providers/provider_registry

.. toctree::
   :maxdepth: 2
   :caption: Features

   com/feeders
   com/monitoring

.. toctree::
   :maxdepth: 2
   :caption: Reference

   com/overview

Architecture
------------

::

    ┌─────────────────────────────────────────────────────────────┐
    │                    Your Component                           │
    │   @remote_service  @remote_publisher  @remote_subscriber   │
    │   @remote_actionServer    InterfaceFactory                  │
    └────────────────────────┬────────────────────────────────────┘
                             │
    ┌────────────────────────▼────────────────────────────────────┐
    │                     Core Layer                              │
    │   Decorators · Blueprints · Factory · Types · Exceptions    │
    └────┬──────────────────┬─────────────────┬───────────────────┘
         │                  │                 │
    ┌────▼────┐     ┌────────▼────┐    ┌──────▼──────────┐
    │Transport│     │  External   │    │   Industrial    │
    ├─────────┤     ├─────────────┤    ├─────────────────┤
    │ Zenoh   │     │ gRPC        │    │ Modbus          │
    │ ROS2    │     │ MQTT        │    │ OPC UA          │
    │ Redis   │     │ REST        │    └─────────────────┘
    │ UDS     │     │ WebSocket   │
    └─────────┘     │ Shared Mem  │
                    └─────────────┘

**Protocol fallback (automatic):** ``Zenoh → ROS2 → Redis → UDS``

Quick Reference
---------------

Expose a service (server side):

.. code-block:: python

   from vyra_base.com import remote_service, ProtocolType, bind_decorated_callbacks

   class MyComponent:
       @remote_service(name="ping", protocols=[ProtocolType.ZENOH], namespace="my_module")
       async def ping(self, request, response=None):
           return {"pong": True}

   component = MyComponent()
   bind_decorated_callbacks(component, namespace="my_module")

Call a service (client side):

.. code-block:: python

   from vyra_base.com import InterfaceFactory, ProtocolType

   client = await InterfaceFactory.create_client("ping", namespace="my_module",
                                                  protocols=[ProtocolType.ZENOH])
   response = await client.call({}, timeout=5.0)

Publish messages:

.. code-block:: python

   from vyra_base.com import remote_publisher, ProtocolType

   class MyComponent:
       @remote_publisher(name="events", protocols=[ProtocolType.ZENOH])
       async def publish_event(self, data: dict):
           return data

Subscribe to messages:

.. code-block:: python

   from vyra_base.com import remote_subscriber, ProtocolType

   class MyComponent:
       @remote_subscriber(name="events", protocols=[ProtocolType.ZENOH])
       async def on_event(self, message: dict):
           print(message)

Migration from Legacy API
-------------------------

If you have code that uses the old API (ROS2-only ``datalayer`` module), migrate as follows:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Old (removed)
     - New
   * - ``@remote_callable``
     - ``@remote_service``
   * - ``@remote_speaker``
     - ``@remote_publisher``
   * - ``@remote_listener``
     - ``@remote_subscriber``
   * - ``@remote_job``
     - ``@remote_actionServer``
   * - ``VyraCallable``
     - ``VyraServer``
   * - ``VyraSpeaker``
     - ``VyraPublisher``
   * - ``VyraJob``
     - ``VyraActionServer``
   * - ``from vyra_base.com.datalayer…``
     - ``from vyra_base.com…``
   * - ``InterfaceType.CALLABLE``
     - ``InterfaceType.SERVER``
   * - ``InterfaceType.SPEAKER``
     - ``InterfaceType.PUBLISHER``
   * - ``InterfaceType.JOB``
     - ``InterfaceType.ACTION_SERVER``
