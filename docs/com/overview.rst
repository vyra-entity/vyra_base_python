COM API-Übersicht
=================

Diese Seite bietet eine vollständige Übersicht über alle Kommunikations-APIs in vyra_base.com.

ROS2 Kommunikation (Datalayer)
-------------------------------

Services
^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Klasse
     - Verwendung
     - Referenz
   * - **VyraCallable**
     - Service Server (wird aufgerufen)
     - :class:`~vyra_base.com.datalayer.callable.VyraCallable`
   * - **VyraJob** (via create_vyra_job)
     - Service Client (ruft auf)
     - :doc:`../vyra_base.com.datalayer`
   * - **@remote_callable**
     - Decorator für automatische Service-Registrierung
     - :func:`~vyra_base.com.datalayer.interface_factory.remote_callable`

Topics
^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Klasse
     - Verwendung
     - Referenz
   * - **VyraSpeaker**
     - Publisher (veröffentlicht)
     - :class:`~vyra_base.com.datalayer.speaker.VyraSpeaker`
   * - **VyraSpeakerListener**
     - Subscriber (empfängt)
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

   * - Klasse
     - Verwendung
     - Referenz
   * - **VyraActionServer**
     - Action Server (führt aus)
     - :class:`~vyra_base.com.datalayer.action_server.VyraActionServer`
   * - **VyraActionClient**
     - Action Client (startet Aktion)
     - :class:`~vyra_base.com.datalayer.action_client.VyraActionClient`

Node Management
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Klasse
     - Verwendung
     - Referenz
   * - **VyraNode**
     - Haupt-ROS2-Node für Module
     - :class:`~vyra_base.com.datalayer.node.VyraNode`
   * - **CheckerNode**
     - Hilfsnodeode für Node-Verfügbarkeitsprüfung
     - :class:`~vyra_base.com.datalayer.node.CheckerNode`

Feeder
------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Klasse
     - Verwendung
     - Referenz
   * - **BaseFeeder**
     - Basis-Klasse für alle Feeder
     - :class:`~vyra_base.com.feeder.feeder.BaseFeeder`
   * - **StateFeeder**
     - Automatische State-Publikation
     - :class:`~vyra_base.com.feeder.state_feeder.StateFeeder`
   * - **NewsFeeder**
     - Automatische News-Publikation
     - :class:`~vyra_base.com.feeder.news_feeder.NewsFeeder`
   * - **ErrorFeeder**
     - Automatische Error-Publikation
     - :class:`~vyra_base.com.feeder.error_feeder.ErrorFeeder`

IPC (Inter-Process Communication)
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Klasse
     - Verwendung
     - Referenz
   * - **GrpcUdsServer**
     - gRPC Server über Unix Domain Socket
     - :class:`~vyra_base.com.handler.ipc.GrpcUdsServer`
   * - **GrpcUdsClient**
     - gRPC Client über Unix Domain Socket
     - :class:`~vyra_base.com.handler.ipc.GrpcUdsClient`

Factory Functions
-----------------

Interface-Erstellung
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import (
       create_vyra_job,      # Service Client
       create_vyra_callable,  # Service Server
       create_vyra_speaker,   # Publisher
       DataSpace             # Interface-Registry
   )

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Funktion
     - Beschreibung
   * - ``create_vyra_job(node, service_name, service_type)``
     - Erstellt Service Client
   * - ``create_vyra_callable(node, service_name, service_type, callback)``
     - Erstellt Service Server
   * - ``create_vyra_speaker(node, topic_name, topic_type, qos_profile)``
     - Erstellt Publisher
   * - ``@remote_callable``
     - Decorator für automatische Service-Registrierung

Verwendungsbeispiele
--------------------

Service-Aufruf (Job)
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_job
   from example_interfaces.srv import AddTwoInts
   
   job = create_vyra_job(
       node=entity.node,
       service_name="/calculator/add_two_ints",
       service_type=AddTwoInts
   )
   response = await job.call_async(request)

Service bereitstellen (Callable)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import remote_callable
   
   class MyService:
       @remote_callable
       async def my_service_method(self, request, response):
           response.result = "success"
           return response

Topic veröffentlichen (Speaker)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
   from std_msgs.msg import String
   
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/my_topic",
       topic_type=String
   )
   speaker.shout(message)

Feeder nutzen
^^^^^^^^^^^^^

.. code-block:: python

   # Über Entity (einfachster Weg)
   entity.publish_state()
   entity.publish_news("Module started")
   entity.publish_error("Connection failed")

IPC verwenden
^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.handler.ipc import GrpcUdsClient
   
   client = GrpcUdsClient(socket_path="/tmp/my_module.sock")
   await client.connect()
   response = await client.call_unary("MethodName", request)

Vollständige API-Referenz
--------------------------

* :doc:`../vyra_base.com.datalayer` - Datalayer (ROS2) API
* :doc:`../vyra_base.com.feeder` - Feeder API
* :doc:`../vyra_base.com` - Vollständige COM API

Weiterführende Dokumentation
-----------------------------

* :doc:`ros2_communication` - ROS2 Details
* :doc:`ipc_communication` - IPC Details
* :doc:`feeders` - Feeder Details
