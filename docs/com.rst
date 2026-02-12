Communication
=============

The COM-Module represents the multi-protocol communication layer of the VYRA framework.
It provides unified interfaces for ROS2, Redis, Zenoh, UDS, and other protocols with
automatic protocol selection and fallback.

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
   com/transport/redis
   com/transport/uds

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
   :caption: Handlers

   com/handler/ros2
   com/handler/ipc
   com/handler/database
   com/handler/communication

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
   :caption: Reference

   com/overview
   com/ros2_communication
   com/ipc_communication

Overview
---------

The COM-Module provides a unified communication API with three main layers:

1. **Core Layer**: Protocol-agnostic interfaces and factory
   
   * **InterfaceFactory**: Creates interfaces with automatic protocol selection
   * **Server/Client Pattern**: Explicit methods for clarity
     
     - ``create_callable()`` / ``create_caller()`` - Service server/client
     - ``create_speaker()`` / ``create_listener()`` - Publisher/subscriber  
     - ``create_job()`` / ``create_dispatcher()`` - Action server/client
   
   * **Type System**: Base types (VyraCallable, VyraSpeaker, VyraJob)
   * **Decorators**: @remote_callable, @remote_speaker, @remote_job

2. **Transport Layer**: Protocol implementations
   
   * **ROS2**: Distributed robotics (DDS-based)
   * **Redis**: Pub/Sub and key-value store
   * **Zenoh**: High-performance pub/sub
   * **UDS**: Low-latency local IPC

3. **Provider Layer**: Protocol providers and registry
   
   * **AbstractProtocolProvider**: Base provider interface
   * **ProviderRegistry**: Manages available protocols
   * **Automatic fallback**: Zenoh → ROS2 → Redis → UDS
Quick Start
-----------

New Unified API (Recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Call Remote Service
"""""""""""""""""""

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   
   # Service Client (NEW!)
   caller = await InterfaceFactory.create_caller(
       "calculate",
       service_type=AddTwoInts
   )
   
   # Call service
   request = AddTwoInts.Request(a=5, b=3)
   response = await caller.call(request, timeout=5.0)
   print(f"Result: {response.sum}")

Provide Service
"""""""""""""""

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   
   # Service Server callback
   async def handle_calculate(request, response):
       response.sum = request.a + request.b
       return response
   
   # Create service server
   server = await InterfaceFactory.create_callable(
       "calculate",
       callback=handle_calculate,
       service_type=AddTwoInts
   )

Publish Messages
""""""""""""""""

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   
   # Publisher
   speaker = await InterfaceFactory.create_speaker(
       "events",
       message_type=String
   )
   
   # Publish message
   await speaker.shout(String(data="Hello World"))

Subscribe to Messages
"""""""""""""""""""""

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   
   # Subscriber callback
   def on_message(msg):
       print(f"Received: {msg.data}")
   
   # Create subscriber (NEW!)
   listener = await InterfaceFactory.create_listener(
       "events",
       callback=on_message,
       message_type=String
   )

Send Action Goals
"""""""""""""""""

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   
   # Feedback callback
   def on_feedback(feedback):
       print(f"Progress: {feedback.progress}%")
   
   # Action Client (NEW!)
   dispatcher = await InterfaceFactory.create_dispatcher(
       "process_data",
       feedback_callback=on_feedback,
       action_type=ProcessData
   )
   
   # Send goal
   goal = ProcessData.Goal(dataset="data.csv")
   result = await dispatcher.execute(goal)

Execute Actions
"""""""""""""""

.. code-block:: python

   from vyra_base.com import InterfaceFactory
   
   # Action Server callback
   async def execute_task(goal, feedback_callback=None):
       for i in range(100):
           if feedback_callback:
               await feedback_callback({"progress": i})
       return {"status": "completed"}
   
   # Create action server
   job = await InterfaceFactory.create_job(
       "process_data",
       callback=execute_task,
       action_type=ProcessData
   )

Using Decorators
""""""""""""""""

.. code-block:: python

   from vyra_base.com import remote_callable, remote_speaker
   
   class MyComponent:
       @remote_callable
       async def calculate(self, request, response):
           """Automatically creates service server"""
           response.result = request.value * 2
           return response
       
       @remote_speaker
       async def notify(self, message):
           """Automatically creates publisher"""
           pass  # Publishing handled automatically

Legacy API (ROS2-only)
^^^^^^^^^^^^^^^^^^^^^^

Call ROS2 Service
^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_job
   
   # Job (Service Client) create
   job = create_vyra_job(
       node=entity.node,
       service_name="other_module/get_status",
       service_type=GetStatus
   )
   
   # Call Service
   response = await job.call_async(request)

Provide ROS2 Service
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com import remote_callable
   
   class MyComponent:
       @remote_callable
       async def my_service(self, request, response):
           # Service-Logic
           response.result = "Success"
           return response

Topic publish
^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_speaker
   
   # Speaker (Publisher) create
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="sensor_data",
       topic_type=SensorData
   )
   
   # Publish message
   speaker.shout(message)

Further Information
-------------------

Core Components
^^^^^^^^^^^^^^^

* :doc:`com/core/factory` - **InterfaceFactory** - Unified interface creation
* :doc:`com/core/types` - Base types and enums (VyraCallable, VyraSpeaker, VyraJob)
* :doc:`com/core/exceptions` - Communication exceptions
* :doc:`com/core/decorators` - Protocol-agnostic decorators
* :doc:`com/core/registry` - Provider registry management

Transport Layer
^^^^^^^^^^^^^^^

* :doc:`com/transport/ros2` - ROS2 transport implementation
* :doc:`com/transport/redis` - Redis transport implementation
* :doc:`com/transport/uds` - Unix Domain Socket transport

Providers
^^^^^^^^^

* :doc:`com/providers/protocol_provider` - AbstractProtocolProvider interface
* :doc:`com/providers/provider_registry` - ProviderRegistry singleton

Features
^^^^^^^^

* :doc:`com/feeders` - Automatic data publication (StateFeeder, NewsFeeder, ErrorFeeder)
* :doc:`com/monitoring` - Prometheus metrics integration

Legacy Documentation
^^^^^^^^^^^^^^^^^^^^

* :doc:`com/ros2_communication` - Legacy ROS2 Details (Job, Callable, Speaker)
* :doc:`com/ipc_communication` - Legacy IPC via gRPC
* :doc:`com/overview` - Complete API Overview
* :doc:`vyra_base.com.datalayer` - Datalayer API Reference
* :doc:`vyra_base.com.feeder` - Feeder API Reference

Migration Guide
^^^^^^^^^^^^^^^

**From Legacy to New API:**

.. code-block:: python

   # OLD (ROS2-only)
   from vyra_base.com.datalayer.interface_factory import create_vyra_job
   job = create_vyra_job(node=node, service_name="service", service_type=Type)
   
   # NEW (Multi-protocol)
   from vyra_base.com import InterfaceFactory
   caller = await InterfaceFactory.create_caller("service", service_type=Type)

**Key Differences:**

1. **No node required** - Protocols managed by providers
2. **Async initialization** - ``await`` for create methods
3. **Explicit server/client** - ``create_callable()`` vs ``create_caller()``
4. **Automatic protocol fallback** - Tries multiple protocols
5. **Multi-protocol support** - Works with ROS2, Redis, Zenoh, UDS