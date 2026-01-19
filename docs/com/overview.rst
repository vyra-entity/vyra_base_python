COM API Overview
=================

This page provides a complete overview of all communication APIs in vyra_base.com.

ROS2 Communication (Datalayer)
-------------------------------

Services
^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Class
     - Usage
     - Reference
   * - **VyraCallable**
     - Service Server (provides service)
     - :class:`~vyra_base.com.datalayer.callable.VyraCallable`
   * - **VyraJob** (via create_vyra_job)
     - Service Client (calls service)
     - :doc:`../vyra_base.com.datalayer`
   * - **@remote_callable**
     - Decorato for automatic service regisration
     - :func:`~vyra_base.com.datalayer.interface_factoy.remote_callable`

Topics
^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Class
     - Usage
     - Reference
   * - **VyraSpeaker**
     - Publisher (publishes)
     - :class:`~vyra_base.com.datalayer.speaker.VyraSpeaker`
   * - **VyraSpeakerListener**
     - Subscriber (receives)
     - :class:`~vyra_base.com.datalayer.speaker.VyraSpeakerListener`
   * - **VyraPublisher**
     - Low-Level Publisher
     - :class:`~vyra_base.com.datalayer.publisher.VyraPublisher`
   * - **VyraSubscriber**
     - Low-Level Subscriber
     - :class:`~vyra_base.com.datalayer.subscriber.VyraSubscriber`

Actions
^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Class
     - Usage
     - Reference
   * - **VyraActionServer**
     - Action Server (executes)
     - :class:`~vyra_base.com.datalayer.action_server.VyraActionServer`
   * - **VyraActionClient**
     - Action Client (starts action)
     - :class:`~vyra_base.com.datalayer.action_client.VyraActionClient`

Node Management
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Class
     - Usage
     - Reference
   * - **VyraNode**
     - Main ROS2 node for modules
     - :class:`~vyra_base.com.datalayer.node.VyraNode`
   * - **CheckerNode**
     - Helper node for node availability checking
     - :class:`~vyra_base.com.datalayer.node.CheckerNode`

Feeder
------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Class
     - Usage
     - Reference
   * - **BaseFeeder**
     - Base class for all feeders
     - :class:`~vyra_base.com.feeder.feeder.BaseFeeder`
   * - **StateFeeder**
     - Automatic state publication
     - :class:`~vyra_base.com.feeder.state_feeder.StateFeeder`
   * - **NewsFeeder**
     - Automatic news publication
     - :class:`~vyra_base.com.feeder.news_feeder.NewsFeeder`
   * - **ErrorFeeder**
     - Automatic error publication
     - :class:`~vyra_base.com.feeder.error_feeder.ErrorFeeder`

IPC (Inter-Process Communication)
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Class
     - Usage
     - Reference
   * - **GrpcUdsServer**
     - gRPC server via Unix Domain Socket
     - :class:`~vyra_base.com.handler.ipc.GrpcUdsServer`
   * - **GrpcUdsClient**
     - gRPC client via Unix Domain Socket
     - :class:`~vyra_base.com.handler.ipc.GrpcUdsClient`

Factoy Functions
-----------------

Interface Creation
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import (
       create_vyra_job,      # Service Client
       create_vyra_callable,  # Service Server
       create_vyra_speaker,   # Publisher
       DataSpace             # Interface-Registry
   )

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Function
     - Description
   * - ``create_vyra_job(node, service_name, service_type)``
     - Creates service client
   * - ``create_vyra_callable(node, service_name, service_type, callback)``
     - Creates service server
   * - ``create_vyra_speaker(node, topic_name, topic_type, qos_profile)``
     - Creates publisher
   * - ``@remote_callable``
     - Decorato for automatic service regisration

Usage Examples
--------------------

Service Call (Job)
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_job
   from example_interfaces.srv import AddTwoInts
   
   job = create_vyra_job(
       node=entity.node,
       service_name="/calculato/add_two_ints",
       service_type=AddTwoInts
   )
   response = await job.call_async(request)

Provide Service (Callable)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import remote_callable
   
   class MyService:
       @remote_callable
       async def my_service_method(self, request, response):
           response.result = "success"
           return response

Publish Topic (Speaker)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_speaker
   from std_msgs.msg import String
   
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/my_topic",
       topic_type=String
   )
   speaker.shout(message)

Use Feeders
^^^^^^^^^^^^^

.. code-block:: python

   # Via Entity (easiest way)
   entity.publish_state()
   entity.publish_news("Module started")
   entity.publish_error("Connection failed")

Use IPC
^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.handler.ipc import GrpcUdsClient
   
   client = GrpcUdsClient(socket_path="/tmp/my_module.sock")
   await client.connect()
   response = await client.call_unary("MethodName", request)

Complete API Reference
--------------------------

* :doc:`../vyra_base.com.datalayer` - Datalayer (ROS2) API
* :doc:`../vyra_base.com.feeder` - Feeder API
* :doc:`../vyra_base.com` - Complete COM API

Further Documentation
-----------------------------

* :doc:`ros2_communication` - ROS2 Details
* :doc:`ipc_communication` - IPC Details
* :doc:`feeders` - Feeder Details
