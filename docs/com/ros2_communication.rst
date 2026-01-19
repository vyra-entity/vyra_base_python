ROS2 Kommunikation
==================

VYRA nutzt ROS2 für die Kommunikation zwischen Modulen. Das Framework abstrahiert die ROS2-API
und bietet einfache Schnittstellen für Services, Topics und Actions.

Konzepte
--------

ROS2 kennt drei Hauptkommunikationsmuster:

1. **Services (Request/Response)**:
   
   * **Job** = Service Client (ruft auf)
   * **Callable** = Service Server (wird aufgerufen)
   * Synchrone Anfrage-Antwort-Kommunikation

2. **Topics (Publish/Subscribe)**:
   
   * **Speaker** = Publisher (veröffentlicht)
   * **Listener** = Subscriber (empfängt)
   * Asynchrone Broadcast-Kommunikation

3. **Actions (Langläufige Operationen)**:
   
   * **Action Client** (startet Aktion)
   * **Action Server** (führt Aktion aus)
   * Mit Feedback und Cancel-Möglichkeit

Services: Job & Callable
-------------------------

Job - Service Client
^^^^^^^^^^^^^^^^^^^^

Ein **Job** ruft Services auf anderen Modulen auf:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_job
   from example_interfaces.srv import AddTwoInts
   
   # Job erstellen
   job = create_vyra_job(
       node=entity.node,
       service_name="/calculator/add_two_ints",
       service_type=AddTwoInts
   )
   
   # Request vorbereiten
   request = AddTwoInts.Request()
   request.a = 5
   request.b = 7
   
   # Service asynchron aufrufen
   response = await job.call_async(request)
   print(f"Ergebnis: {response.sum}")

**Parameter:**

* ``node``: Die ROS2-Node (typischerweise ``entity.node``)
* ``service_name``: Name des Services (mit Namespace)
* ``service_type``: ROS2 Service-Typ (Import aus ``*_interfaces.srv``)

Callable - Service Server
^^^^^^^^^^^^^^^^^^^^^^^^^^

Ein **Callable** stellt einen Service für andere Module bereit.
Verwenden Sie den ``@remote_callable`` Decorator:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import remote_callable
   from vyra_base.state import OperationalStateMachine
   
   class Calculator(OperationalStateMachine):
       def __init__(self, unified_state_machine):
           super().__init__(unified_state_machine)
       
       @remote_callable
       async def add_two_ints(self, request, response):
           """Addiert zwei Zahlen"""
           response.sum = request.a + request.b
           return response
       
       @remote_callable
       async def get_status(self, request, response):
           """Gibt den aktuellen Status zurück"""
           response.status = "operational"
           return response

**Automatische Registrierung:**

Methoden mit ``@remote_callable`` werden automatisch als ROS2-Services registriert,
wenn die JSON-Metadaten korrekt definiert sind (siehe :doc:`../interfaces`).

**Parameter:**

* ``request``: Eingehende Service-Anfrage (vom Service-Typ definiert)
* ``response``: Ausgehende Service-Antwort (wird zurückgegeben)

Topics: Speaker & Listener
---------------------------

Speaker - Publisher
^^^^^^^^^^^^^^^^^^^

Ein **Speaker** veröffentlicht Nachrichten auf einem Topic:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
   from std_msgs.msg import String
   
   # Speaker erstellen
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/status_updates",
       topic_type=String,
       qos_profile=10  # Queue-Size
   )
   
   # Nachricht veröffentlichen
   msg = String()
   msg.data = "System running"
   speaker.shout(msg)

**Parameter:**

* ``node``: Die ROS2-Node
* ``topic_name``: Name des Topics
* ``topic_type``: ROS2 Message-Typ (Import aus ``*_interfaces.msg``)
* ``qos_profile``: Quality-of-Service (Queue-Size oder QoS-Objekt)

Listener - Subscriber
^^^^^^^^^^^^^^^^^^^^^^

Ein **Listener** empfängt Nachrichten von einem Topic:

.. code-block:: python

   from vyra_base.com.datalayer.subscriber import VyraSubscriber
   from std_msgs.msg import String
   
   # Callback-Funktion definieren
   def on_message_received(msg):
       print(f"Empfangen: {msg.data}")
   
   # Listener erstellen
   listener = VyraSubscriber(
       node=entity.node,
       topic_name="/status_updates",
       topic_type=String,
       callback=on_message_received,
       qos_profile=10
   )
   
   # Subscription erstellen
   listener.create_subscription()

**Parameter:**

* ``callback``: Funktion, die bei jeder Nachricht aufgerufen wird
* Andere Parameter wie beim Speaker

Praktische Beispiele
--------------------

Inter-Modul-Service-Aufruf
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Modul A (Server)**:

.. code-block:: python

   class RobotController(OperationalStateMachine):
       @remote_callable
       async def move_to_position(self, request, response):
           """Bewegt Roboter zur Position"""
           x, y = request.target_x, request.target_y
           
           # Bewegung ausführen
           success = await self.robot.move_to(x, y)
           
           response.success = success
           response.message = "Movement complete"
           return response

**Modul B (Client)**:

.. code-block:: python

   # Job erstellen
   move_job = create_vyra_job(
       node=entity.node,
       service_name="/robot_controller/move_to_position",
       service_type=MoveToPosition
   )
   
   # Bewegung anfordern
   request = MoveToPosition.Request()
   request.target_x = 10.5
   request.target_y = 5.3
   
   response = await move_job.call_async(request)
   if response.success:
       print("Roboter erfolgreich bewegt")

Topic-basierte Statusmeldungen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Publisher (Modul A)**:

.. code-block:: python

   # Speaker für Status-Updates
   status_speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/system_status",
       topic_type=SystemStatus
   )
   
   # Regelmäßig Status senden
   async def publish_status_loop():
       while True:
           status = SystemStatus()
           status.cpu_usage = get_cpu_usage()
           status.memory_usage = get_memory_usage()
           status.timestamp = get_current_time()
           
           status_speaker.shout(status)
           await asyncio.sleep(1.0)  # Jede Sekunde

**Subscriber (Modul B)**:

.. code-block:: python

   # Callback für Status-Empfang
   def on_status_received(msg):
       if msg.cpu_usage > 80:
           logger.warning(f"Hohe CPU-Last: {msg.cpu_usage}%")
   
   # Listener erstellen
   status_listener = VyraSubscriber(
       node=entity.node,
       topic_name="/system_status",
       topic_type=SystemStatus,
       callback=on_status_received
   )
   status_listener.create_subscription()

Interface-Definition
--------------------

ROS2-Interfaces werden über JSON-Metadaten definiert.

**Beispiel** (``config/service_interfaces.json``):

.. code-block:: json

   {
       "services": [
           {
               "name": "add_two_ints",
               "type": "example_interfaces/srv/AddTwoInts",
               "description": "Addiert zwei Ganzzahlen"
           },
           {
               "name": "get_status",
               "type": "std_srvs/srv/Trigger",
               "description": "Gibt aktuellen Status zurück"
           }
       ]
   }

Diese Metadaten werden beim Entity-Start geladen und automatisch registriert:

.. code-block:: python

   # In _base_.py
   interfaces = await entity.load_interfaces_from_config()
   await entity.set_interfaces(interfaces)

Weitere Details siehe :doc:`../interfaces`.

Quality of Service (QoS)
------------------------

ROS2 nutzt QoS-Profile zur Konfiguration der Kommunikation:

.. code-block:: python

   from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
   
   # Custom QoS-Profil
   qos = QoSProfile(
       depth=10,  # Queue-Size
       reliability=ReliabilityPolicy.RELIABLE,  # oder BEST_EFFORT
       durability=DurabilityPolicy.VOLATILE  # oder TRANSIENT_LOCAL
   )
   
   # Mit Speaker verwenden
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/important_data",
       topic_type=Data,
       qos_profile=qos
   )

**Typische Profile:**

* **Sensor-Daten**: ``BEST_EFFORT`` (schnell, Verlust akzeptabel)
* **Befehle**: ``RELIABLE`` (garantierte Zustellung)
* **Persistente Daten**: ``TRANSIENT_LOCAL`` (neue Subscriber erhalten letzte Nachricht)

Best Practices
--------------

✅ **Empfohlen:**

* Verwenden Sie aussagekräftige Service-/Topic-Namen mit Namespace
* Definieren Sie Interfaces in JSON-Metadaten
* Nutzen Sie ``@remote_callable`` für automatische Registrierung
* Implementieren Sie Timeouts für Service-Calls
* Verwenden Sie QoS-Profile passend zum Anwendungsfall

❌ **Vermeiden:**

* Manuelle ROS2-Node-Erstellung (nutzen Sie ``entity.node``)
* Sehr große Nachrichten (> 1 MB) über Topics
* Hochfrequente Service-Calls (> 100 Hz, nutzen Sie Topics)
* Blocking-Aufrufe ohne Timeout

Fehlerbehandlung
----------------

.. code-block:: python

   from rclpy.task import Future
   
   try:
       # Service-Call mit Timeout
       response = await asyncio.wait_for(
           job.call_async(request),
           timeout=5.0  # 5 Sekunden
       )
   except asyncio.TimeoutError:
       logger.error("Service-Aufruf timeout")
   except Exception as e:
       logger.error(f"Service-Fehler: {e}")

Weiterführende Informationen
-----------------------------

* :doc:`ipc_communication` - IPC innerhalb eines Moduls
* :doc:`feeders` - Automatische Feeder
* :doc:`../interfaces` - Interface-Definitionen
* :doc:`../vyra_base.com.datalayer` - API-Referenz
* ROS2 Dokumentation: https://docs.ros.org/en/kilted/
