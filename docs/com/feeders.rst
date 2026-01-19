Feeder - Automatische Daten-Publikation
========================================

Feeder sind spezielle Publisher, die automatisch Daten auf ROS2-Topics veröffentlichen.
VYRA bietet vorgefertigte Feeder für States, News und Errors.

Konzept
-------

Feeder automatisieren die Publikation von standardisierten Daten:

* **StateFeeder**: Veröffentlicht State Machine Zustandsänderungen
* **NewsFeeder**: Veröffentlicht informative Nachrichten (Logs, Events)
* **ErrorFeeder**: Veröffentlicht Fehlermeldungen und Warnungen

**Vorteile:**

* ✅ Automatische Topic-Verwaltung
* ✅ Konsistentes Message-Format
* ✅ Integriert mit Entity und State Machine
* ✅ Kein manuelles Topic-Setup nötig

StateFeeder
-----------

Der StateFeeder veröffentlicht automatisch Zustandsänderungen der State Machine:

Verwendung
^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.feeder.state_feeder import StateFeeder
   from vyra_base.defaults.entries import StateEntry
   
   # StateEntry konfigurieren
   state_entry = StateEntry(
       namespace="/my_module",
       topic_name="state",
       message_type="vyra_interfaces/msg/State"
   )
   
   # StateFeeder erstellen (typischerweise in Entity)
   state_feeder = StateFeeder(
       node=entity.node,
       state_entry=state_entry
   )
   state_feeder.create()
   
   # State veröffentlichen (automatisch durch State Machine)
   entity.publish_state()

Integration mit State Machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Die State Machine nutzt den StateFeeder automatisch:

.. code-block:: python

   # Bei State-Transition
   await state_machine.trigger_event(StateEvent.INITIALIZE)
   # ➡️ StateFeeder veröffentlicht neuen State automatisch

**Veröffentlichte Informationen:**

* Lifecycle State (IDLE, READY, RUNNING, etc.)
* Operational State (IDLE, INITIALIZING, OPERATIONAL, etc.)
* Health State (HEALTHY, DEGRADED, UNHEALTHY, etc.)
* Timestamp
* Module-Namespace

NewsFeeder
----------

Der NewsFeeder veröffentlicht informative Nachrichten und Events:

Verwendung
^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.feeder.news_feeder import NewsFeeder
   from vyra_base.defaults.entries import NewsEntry
   
   # NewsEntry konfigurieren
   news_entry = NewsEntry(
       namespace="/my_module",
       topic_name="news",
       message_type="vyra_interfaces/msg/News"
   )
   
   # NewsFeeder erstellen
   news_feeder = NewsFeeder(
       node=entity.node,
       news_entry=news_entry
   )
   news_feeder.create()

Nachrichten veröffentlichen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Über Entity (empfohlen)
   entity.publish_news("Modul erfolgreich initialisiert")
   entity.publish_news("Sensor-Kalibrierung abgeschlossen")
   entity.publish_news("Verbindung zu externem System hergestellt")
   
   # Direkt über Feeder
   from vyra_base.defaults.entries import NewsEntry
   news_msg = NewsEntry(
       namespace="/my_module",
       message="Operation erfolgreich",
       timestamp=datetime.now()
   )
   news_feeder.feed(news_msg)

**Anwendungsfälle:**

* Statusmeldungen für Monitoring
* Wichtige Events (Start, Stop, Kalibrierung)
* Fortschrittsberichte bei langläufigen Operationen
* Informative Logs für andere Module

ErrorFeeder
-----------

Der ErrorFeeder veröffentlicht Fehler und Warnungen:

Verwendung
^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.feeder.error_feeder import ErrorFeeder
   from vyra_base.defaults.entries import ErrorEntry
   
   # ErrorEntry konfigurieren
   error_entry = ErrorEntry(
       namespace="/my_module",
       topic_name="error",
       message_type="vyra_interfaces/msg/Error"
   )
   
   # ErrorFeeder erstellen
   error_feeder = ErrorFeeder(
       node=entity.node,
       error_entry=error_entry
   )
   error_feeder.create()

Fehler veröffentlichen
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Über Entity (empfohlen)
   entity.publish_error("Verbindung zu Sensor verloren")
   entity.publish_error("Timeout bei Service-Aufruf")
   entity.publish_error("Konfigurationsdatei nicht gefunden")
   
   # Mit Severity-Level (wenn unterstützt)
   error_msg = ErrorEntry(
       namespace="/my_module",
       message="Kritischer Fehler",
       severity="ERROR",  # INFO, WARNING, ERROR, CRITICAL
       timestamp=datetime.now()
   )
   error_feeder.feed(error_msg)

**Anwendungsfälle:**

* Fehlerbenachrichtigungen für Monitoring-Systeme
* Warn-Meldungen bei kritischen Zuständen
* Exception-Logging über ROS2
* Zentrale Fehlersammlung über Topics

Praktische Beispiele
--------------------

Monitoring-Dashboard
^^^^^^^^^^^^^^^^^^^^

Ein zentrales Dashboard kann alle Feeder-Topics abonnieren:

.. code-block:: python

   # Dashboard-Modul (Subscriber)
   from vyra_base.com.datalayer.subscriber import VyraSubscriber
   
   class MonitoringDashboard:
       def __init__(self, entity):
           self.entity = entity
           
           # State-Listener
           self.state_listener = VyraSubscriber(
               node=entity.node,
               topic_name="/+/state",  # Alle Module
               topic_type=State,
               callback=self.on_state_update
           )
           
           # News-Listener
           self.news_listener = VyraSubscriber(
               node=entity.node,
               topic_name="/+/news",
               topic_type=News,
               callback=self.on_news_received
           )
           
           # Error-Listener
           self.error_listener = VyraSubscriber(
               node=entity.node,
               topic_name="/+/error",
               topic_type=Error,
               callback=self.on_error_received
           )
       
       def on_state_update(self, msg):
           module = msg.namespace
           state = msg.lifecycle_state
           print(f"[STATE] {module}: {state}")
       
       def on_news_received(self, msg):
           print(f"[NEWS] {msg.namespace}: {msg.message}")
       
       def on_error_received(self, msg):
           print(f"[ERROR] {msg.namespace}: {msg.message}")

Automatisches Error-Logging
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Feeder können mit dem Standard-Logger integriert werden:

.. code-block:: python

   import logging
   
   class ErrorLoggingHandler(logging.Handler):
       def __init__(self, entity):
           super().__init__()
           self.entity = entity
       
       def emit(self, record):
           if record.levelno >= logging.ERROR:
               # Fehler über ErrorFeeder veröffentlichen
               self.entity.publish_error(
                   f"{record.name}: {record.getMessage()}"
               )
   
   # Handler zum Logger hinzufügen
   logger = logging.getLogger()
   logger.addHandler(ErrorLoggingHandler(entity))
   
   # Jetzt werden alle Fehler automatisch veröffentlicht
   logger.error("Dies erscheint auf /my_module/error Topic")

Custom Feeder
-------------

Sie können auch eigene Feeder erstellen:

.. code-block:: python

   from vyra_base.com.feeder.feeder import BaseFeeder
   
   class MetricsFeeder(BaseFeeder):
       """Feeder für Performance-Metriken"""
       
       def __init__(self, node, topic_name="metrics"):
           super().__init__(
               node=node,
               topic_name=topic_name,
               topic_type=Metrics  # Ihr Message-Typ
           )
       
       def feed(self, metrics_data):
           """Veröffentlicht Metrik-Daten"""
           msg = Metrics()
           msg.cpu_usage = metrics_data["cpu"]
           msg.memory_usage = metrics_data["memory"]
           msg.timestamp = self.get_timestamp()
           self.publisher.publish(msg)
   
   # Verwendung
   metrics_feeder = MetricsFeeder(node=entity.node)
   metrics_feeder.create()
   
   # Metriken regelmäßig veröffentlichen
   async def publish_metrics_loop():
       while True:
           metrics = get_system_metrics()
           metrics_feeder.feed(metrics)
           await asyncio.sleep(5.0)

Best Practices
--------------

✅ **Empfohlen:**

* Verwenden Sie Feeder für standardisierte Daten (State, News, Error)
* Integrieren Sie Feeder in die Entity-Initialisierung
* Nutzen Sie ``entity.publish_*()`` für einfachen Zugriff
* Abonnieren Sie Feeder-Topics für zentrales Monitoring
* Verwenden Sie aussagekräftige Nachrichten

❌ **Vermeiden:**

* Hochfrequente Publikation (> 10 Hz, nutzen Sie normale Topics)
* Sehr große Nachrichten über Feeder
* Sensible Daten ohne Verschlüsselung
* Feeder für binäre Daten (nutzen Sie normale Publisher)

Performance-Hinweise
--------------------

**Typische Publikationsraten:**

* StateFeeder: Bei Zustandsänderung (~0.1 - 1 Hz)
* NewsFeeder: Bei Events (~0.01 - 0.1 Hz)
* ErrorFeeder: Bei Fehlern (~0.001 - 0.01 Hz)

.. tip::
   Feeder sind nicht für hochfrequente Daten optimiert.
   Für Sensor-Daten mit > 10 Hz nutzen Sie normale ROS2 Publisher.

Weiterführende Informationen
-----------------------------

* :doc:`ros2_communication` - ROS2 Publisher/Subscriber Details
* :doc:`../vyra_base.com.feeder` - API-Referenz
* :class:`~vyra_base.com.feeder.feeder.BaseFeeder` - Basis-Klasse
* :class:`~vyra_base.com.feeder.state_feeder.StateFeeder` - State Feeder
* :class:`~vyra_base.com.feeder.news_feeder.NewsFeeder` - News Feeder
* :class:`~vyra_base.com.feeder.error_feeder.ErrorFeeder` - Error Feeder
