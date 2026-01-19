Kommunikation (COM)
===================

Das COM-Modul bildet die Kommunikationsschicht des VYRA-Frameworks ab.
Es ermöglicht ROS2-Kommunikation, IPC über gRPC und automatische Daten-Feeds.

.. toctree::
   :maxdepth: 2
   :caption: COM Komponenten

   com/ros2_communication
   com/ipc_communication
   com/feeders
   com/overview

Übersicht
---------

Das COM-Modul besteht aus drei Hauptbereichen:

1. **ROS2 Kommunikation**: Services, Topics, Actions
   
   * **Job** (Service Client): Ruft Services auf anderen Modulen auf
   * **Callable** (Service Server): Stellt Services für andere Module bereit
   * **Speaker** (Publisher): Veröffentlicht Daten auf Topics
   * **Listener** (Subscriber): Empfängt Daten von Topics

2. **IPC (Inter-Process Communication)**: Unix Domain Socket über gRPC
   
   * Schnelle Kommunikation **innerhalb** eines Moduls
   * Prozess-übergreifende Aufrufe ohne Netzwerk-Overhead

3. **Feeder**: Automatische Daten-Publikation
   
   * **StateFeeder**: Automatische State-Updates
   * **NewsFeeder**: Automatische News-Benachrichtigungen
   * **ErrorFeeder**: Automatische Fehler-Meldungen

Schnelleinstieg
---------------

ROS2 Service aufrufen
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_job
   
   # Job (Service Client) erstellen
   job = create_vyra_job(
       node=entity.node,
       service_name="other_module/get_status",
       service_type=GetStatus
   )
   
   # Service aufrufen
   response = await job.call_async(request)

ROS2 Service bereitstellen
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import remote_callable
   
   class MyComponent:
       @remote_callable
       async def my_service(self, request, response):
           # Service-Logik
           response.result = "Success"
           return response

Topic veröffentlichen
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
   
   # Speaker (Publisher) erstellen
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="sensor_data",
       topic_type=SensorData
   )
   
   # Nachricht veröffentlichen
   speaker.shout(message)

Weiterführende Informationen
-----------------------------

* :doc:`com/ros2_communication` - ROS2 Details (Job, Callable, Speaker)
* :doc:`com/ipc_communication` - IPC über gRPC
* :doc:`com/feeders` - Automatische Feeder
* :doc:`com/overview` - Vollständige API-Übersicht
* :doc:`vyra_base.com.datalayer` - Datalayer API-Referenz
* :doc:`vyra_base.com.feeder` - Feeder API-Referenz
