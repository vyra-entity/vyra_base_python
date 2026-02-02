Communication
=============

The COM-Module represents the communication layer of the VYRA framework.
It enables ROS2 communication, IPC via gRPC, and automatic data feeds.

.. toctree::
   :maxdepth: 2
   :caption: COM Components

   com/ros2_communication
   com/ipc_communication
   com/feeders
   com/overview

Overview
---------

The COM-Module consists of three main areas:

1. **ROS2 Communication**: Services, Topics, Actions
   
   * **Job** (Service Client): Calls services on other modules
   * **Callable** (Service Server): Provides services for other modules
   * **Speaker** (Publisher): Publishes data on Topics
   * **Listener** (Subscriber): Receives data from Topics

2. **IPC (Inter-Process Communication)**: Unix Domain Socket via gRPC
   
   * Fast communication **within** a module
   * Cross-process calls without network overhead

3. **Feeder**: Automatic Data Publication
   
   * **StateFeeder**: Automatic State Updates
   * **NewsFeeder**: Automatic News Notifications
   * **ErrorFeeder**: Automatic Error Messages
Quick Start
-----------

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

* :doc:`com/ros2_communication` - ROS2 Details (Job, Callable, Speaker)
* :doc:`com/ipc_communication` - IPC via gRPC
* :doc:`com/feeders` - Automatic Feeders
* :doc:`com/overview` - Complete API Overview
* :doc:`vyra_base.com.datalayer` - Datalayer API Reference
* :doc:`vyra_base.com.feeder` - Feeder API Reference