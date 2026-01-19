Kommunikation (COM)
===================

The COM-Modul bildet die Kommunikationsschicht des VYRA-Frameworks ab.
Es enables ROS2 Communication, IPC via gRPC and automaticallye Daten-Feeds.

.. toctree::
   :maxdepth: 2
   :caption: COM Komponenten

   com/ros2_communication
   com/ipc_communication
   com/feeders
   com/overview

Overview
---------

The COM-Modul besteht from drei Hauptbereichen:

1. **ROS2 Kommunikation**: Services, Topics, Actions
   
   * **Job** (Service Client): Ruft Services on underen Modulen on
   * **Callable** (Service Server): Stellt Services for undere Module bereit
   * **Speaker** (Publisher): Publishes Daten on Topics
   * **Listener** (Subscriber): Empfängt Daten of Topics

2. **IPC (Inter-Process Communication)**: Unix Domain Socket via gRPC
   
   * Schnelle Kommunikation **innerhalb** eines Moduls
   * Prozess-crossing Aufrufe without Netzwerk-Overhead

3. **Feeder**: Automatische Daten-Publikation
   
   * **StateFeeder**: Automatische State-Updates
   * **NewsFeeder**: Automatische News-Notifications
   * **ErrorFeeder**: Automatische Error-Meldungen

Quick Start
---------------

ROS2 Service onrufen
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_job
   
   # Job (Service Client) create
   job = create_vyra_job(
       node=entity.node,
       service_name="other_module/get_status",
       service_type=GetStatus
   )
   
   # Service onrufen
   response = await job.call_async(request)

ROS2 Service bereitstellen
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import remote_callable
   
   class MyComponent:
       @remote_callable
       async def my_service(self, request, response):
           # Service-Logik
           response.result = "Success"
           return response

Topic publish
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_speaker
   
   # Speaker (Publisher) create
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="sensor_data",
       topic_type=SensorData
   )
   
   # Nachricht publish
   speaker.shout(message)

Further Information
-----------------------------

* :doc:`com/ros2_communication` - ROS2 Details (Job, Callable, Speaker)
* :doc:`com/ipc_communication` - IPC via gRPC
* :doc:`com/feeders` - Automatische Feeder
* :doc:`com/overview` - Complete API-Overview
* :doc:`vyra_base.com.datalayer` - Datalayer API-Referenz
* :doc:`vyra_base.com.feeder` - Feeder API-Referenz
