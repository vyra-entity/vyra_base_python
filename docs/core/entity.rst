VyraEntity - Das Herzstück
===========================

Die :class:`~vyra_base.core.entity.VyraEntity` ist die zentrale Verwaltungseinheit jedes VYRA-Moduls.
Sie orchestriert alle Komponenten wie ROS2-Kommunikation, State Management, Storage und Sicherheit.

Initialisierung
---------------

Die Entity wird typischerweise in der ``_base_.py`` eines Moduls erstellt:

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   from vyra_base.defaults.entries import StateEntry, NewsEntry, ErrorEntry, ModuleEntry
   
   # Konfiguration vorbereiten
   state_entry = StateEntry(...)
   news_entry = NewsEntry(...)
   error_entry = ErrorEntry(...)
   module_entry = ModuleEntry(...)
   module_config = {...}
   
   # Entity erstellen
   entity = VyraEntity(
       state_entry=state_entry,
       news_entry=news_entry,
       error_entry=error_entry,
       module_entry=module_entry,
       module_config=module_config
   )
   
   # Entity starten
   await entity.startup_entity()

Hauptfunktionen
---------------

Parameter-Verwaltung
^^^^^^^^^^^^^^^^^^^^

Zugriff auf persistente Konfigurationsdaten über die Parameter-Komponente:

.. code-block:: python

   # Parameter lesen
   value = await entity.parameter.get_param(request, response)
   
   # Parameter setzen
   await entity.parameter.set_param(request, response)
   
   # Alle Parameter auslesen
   all_params = await entity.parameter.read_all_params(request, response)

Parameter werden in einer **SQLite-Datenbank** im Modul gespeichert unter ``/workspace/storage/data/``.
Dies ermöglicht persistente Datenspeicherung zwischen Neustarts.

.. note::
   Parameter eignen sich für Konfigurationsdaten, die dauerhaft gespeichert werden müssen.
   Die Datenbankzugriffe sind relativ langsam, daher nicht für Echtzeitdaten geeignet.

Volatile-Verwaltung
^^^^^^^^^^^^^^^^^^^

Zugriff auf flüchtige, schnelle Daten über die Volatile-Komponente:

.. code-block:: python

   # Volatile-Wert setzen
   await entity.volatile.set_volatile_value("sensor_data", temperature)
   
   # Volatile-Wert lesen
   value = await entity.volatile.get_volatile_value("sensor_data")
   
   # Alle Volatile-Namen auslesen
   all_keys = await entity.volatile.read_all_volatile_names()
   
   # Event-Listener für Änderungen registrieren
   await entity.volatile.add_change_event("sensor_data")

Volatiles nutzen **Redis** für schnelle In-Memory-Datenspeicherung.
Diese Daten sind **flüchtig** - sie gehen bei einem Neustart verloren.

.. tip::
   Verwenden Sie Volatiles für:
   
   * Sensordaten und Echtzeitinformationen
   * Temporäre Zwischenergebnisse
   * Schnelle Inter-Modul-Kommunikation
   * Event-basierte Trigger

ROS2-Kommunikation
^^^^^^^^^^^^^^^^^^

Die Entity stellt automatisch ROS2-Schnittstellen bereit:

.. code-block:: python

   # News veröffentlichen
   entity.publish_news("Modul erfolgreich gestartet")
   
   # Error veröffentlichen
   entity.publish_error("Fehler beim Verbindungsaufbau")
   
   # State veröffentlichen
   entity.publish_state()

State Machine
^^^^^^^^^^^^^

Jede Entity verfügt über eine integrierte State Machine:

.. code-block:: python

   # Zugriff auf State Machine
   state_machine = entity.state_machine
   
   # Aktuellen Status abfragen
   current_state = state_machine.get_current_state()
   
   # State-Events triggern
   await state_machine.trigger_event(StateEvent(...))

Interfaces registrieren
^^^^^^^^^^^^^^^^^^^^^^^

ROS2-Interfaces werden über JSON-Metadaten definiert und automatisch registriert:

.. code-block:: python

   # Interfaces aus JSON laden und registrieren
   interfaces = await entity.load_interfaces_from_config()
   await entity.set_interfaces(interfaces)

Lifecycle Management
--------------------

Startup
^^^^^^^

.. code-block:: python

   # Entity hochfahren
   success = await entity.startup_entity()
   if success:
       print("Entity erfolgreich gestartet")

Shutdown
^^^^^^^^

.. code-block:: python

   # Entity herunterfahren
   await entity.shutdown_entity()

Storage-Zugriff
---------------

Die Entity verwaltet verschiedene Storage-Backends:

.. code-block:: python

   # SQLite-Datenbank-Zugriff
   db_access = entity.storage.db_access
   
   # Redis-Client-Zugriff
   redis_client = entity.storage.redis_client
   
   # Custom Storage registrieren
   await entity.setup_storage(
       db_path="/workspace/storage/data/module.db",
       redis_host="redis",
       redis_port=6379
   )

Wichtige Hinweise
-----------------

.. warning::
   Die Entity sollte nur einmal pro Modul initialisiert werden.
   Mehrfache Initialisierungen können zu Konflikten führen.

.. important::
   Alle Datenbankoperationen sind asynchron (``async/await``).
   Vergessen Sie nicht, ``await`` zu verwenden!

Weiterführende Informationen
-----------------------------

* :doc:`parameter` - Detaillierte Parameter-Dokumentation
* :doc:`volatile` - Detaillierte Volatile-Dokumentation
* :class:`~vyra_base.core.entity.VyraEntity` - API-Referenz
