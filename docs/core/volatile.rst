Volatile - Flüchtige Echtzeitdaten
===================================

Die :class:`~vyra_base.core.volatile.Volatile`-Klasse verwaltet flüchtige Daten,
die schnell gelesen und geschrieben werden müssen - ideal für Echtzeitanwendungen.

Konzept
-------

Volatiles sind **temporäre, schnelle Datenspeicher**, die:

* In Redis (In-Memory-Datenbank) gespeichert werden
* Sehr schnellen Zugriff bieten (~0.1ms)
* Bei Neustart verloren gehen (flüchtig)
* Change-Events unterstützen
* Pub/Sub-Mechanismen nutzen

Wann Volatiles nutzen?
----------------------

✅ **Ideal für:**

* Sensordaten in Echtzeit
* Statusvariablen
* Temporäre Berechnungsergebnisse
* Event-basierte Kommunikation zwischen Modulen
* Caching von Daten

❌ **Nicht geeignet für:**

* Persistente Konfiguration (nutzen Sie :doc:`parameter`)
* Daten, die Neustarts überleben müssen
* Sehr große Datenmengen (> GB)

Zugriff über Entity
-------------------

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   
   entity = VyraEntity(...)
   
   # Volatile-Zugriff
   volatile = entity.volatile

Werte setzen
------------

.. code-block:: python

   # Einfachen Wert setzen
   await entity.volatile.set_volatile_value("temperature", 23.5)
   
   # Komplexe Daten (werden automatisch serialisiert)
   await entity.volatile.set_volatile_value("sensor_data", {
       "temperature": 23.5,
       "humidity": 60.2,
       "timestamp": "2026-01-19T10:30:00"
   })
   
   # Listen
   await entity.volatile.set_volatile_value("measurements", [1, 2, 3, 4, 5])

Werte lesen
-----------

.. code-block:: python

   # Einzelnen Wert lesen
   temperature = await entity.volatile.get_volatile_value("temperature")
   print(f"Aktuelle Temperatur: {temperature}°C")
   
   # Komplexe Daten
   sensor_data = await entity.volatile.get_volatile_value("sensor_data")
   print(f"Temperatur: {sensor_data['temperature']}")
   
   # Alle vorhandenen Keys auflisten
   all_keys = await entity.volatile.read_all_volatile_names()
   for key in all_keys:
       value = await entity.volatile.get_volatile_value(key)
       print(f"{key}: {value}")

Change-Events
-------------

Überwachen Sie Änderungen an Volatiles in Echtzeit:

Event registrieren
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Change-Event für einen Key aktivieren
   await entity.volatile.add_change_event("temperature")

Event-Callback definieren
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   async def on_temperature_change(message: dict, callback_context):
       """Wird aufgerufen, wenn sich 'temperature' ändert"""
       new_value = message.get("value")
       print(f"Temperatur geändert: {new_value}°C")
       
       # Reaktion auf Änderung
       if new_value > 30:
           print("⚠️ Temperatur zu hoch!")
   
   # Callback registrieren
   await entity.volatile.transient_event_callback(
       message={"key": "temperature"},
       callback_context=on_temperature_change
   )

Event entfernen
^^^^^^^^^^^^^^^

.. code-block:: python

   # Change-Event deaktivieren
   await entity.volatile.remove_change_event("temperature")

Channel-Listener
----------------

Für fortgeschrittene Pub/Sub-Kommunikation:

.. code-block:: python

   # Listener für einen Channel aktivieren
   await entity.volatile.activate_listener("sensor_channel")
   
   # Jetzt werden alle Nachrichten auf 'sensor_channel' empfangen

Praktisches Beispiel
--------------------

Sensor-Daten mit Event-Reaktion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   async def sensor_monitoring_example(entity: VyraEntity):
       """Überwacht Sensordaten und reagiert auf Änderungen"""
       
       # Change-Event aktivieren
       await entity.volatile.add_change_event("temperature")
       await entity.volatile.add_change_event("humidity")
       
       # Callback für Temperaturänderungen
       async def on_temp_change(message: dict, context):
           temp = await entity.volatile.get_volatile_value("temperature")
           if temp > 30:
               # Warnung veröffentlichen
               entity.publish_error("Temperatur kritisch!")
       
       # Sensordaten aktualisieren (z.B. in einer Schleife)
       while True:
           # Daten vom Sensor lesen (simuliert)
           current_temp = read_sensor_temperature()
           current_humidity = read_sensor_humidity()
           
           # In Volatiles speichern (triggert Change-Events)
           await entity.volatile.set_volatile_value("temperature", current_temp)
           await entity.volatile.set_volatile_value("humidity", current_humidity)
           
           await asyncio.sleep(1)  # Jede Sekunde aktualisieren

Inter-Modul-Kommunikation
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Modul A: Daten bereitstellen
   await entity_a.volatile.set_volatile_value("shared_status", "running")
   
   # Modul B: Daten lesen
   status = await entity_b.volatile.get_volatile_value("shared_status")
   print(f"Status von Modul A: {status}")

.. note::
   Beide Module müssen denselben Redis-Server verwenden (standardmäßig der Fall im VYRA-System).

Redis-Backend
-------------

**Verbindung**: Automatisch über ``entity.storage.redis_client``

**Speicherort**: In-Memory (RAM des Redis-Servers)

**Persistenz**: Keine - Daten gehen bei Redis-Neustart verloren

.. tip::
   Redis kann optional konfiguriert werden, um Snapshots zu erstellen.
   Dies erfolgt über die Redis-Konfiguration, nicht in VYRA selbst.
   Weitere Informationen: https://redis.io/docs/management/persistence/

Performance
-----------

**Typische Zugriffszeiten:**

* Schreiben: ~0.1 - 0.5 ms
* Lesen: ~0.1 - 0.3 ms
* Event-Trigger: ~1 - 5 ms

.. important::
   Volatiles sind **100x schneller** als Parameter (SQLite).
   Nutzen Sie Volatiles für hochfrequente Datenoperationen!

Best Practices
--------------

✅ **Empfohlen:**

.. code-block:: python

   # Kurze, aussagekräftige Keys
   await volatile.set_volatile_value("temp_sensor_1", 23.5)
   
   # Strukturierte Daten für zusammenhängende Werte
   await volatile.set_volatile_value("robot_pose", {
       "x": 1.5, "y": 2.3, "theta": 0.78
   })
   
   # Change-Events für kritische Werte
   await volatile.add_change_event("emergency_stop")

❌ **Vermeiden:**

.. code-block:: python

   # Sehr lange Keys
   await volatile.set_volatile_value("this_is_a_very_long_key_name...", value)
   
   # Zu große Datenstrukturen (> 1 MB)
   await volatile.set_volatile_value("huge_data", very_large_object)
   
   # Hochfrequente Events ohne Bedarf
   for i in range(10000):
       await volatile.add_change_event(f"sensor_{i}")

Fehlerbehandlung
----------------

.. code-block:: python

   try:
       value = await entity.volatile.get_volatile_value("nonexistent_key")
   except KeyError:
       print("Key existiert nicht")
       # Standardwert setzen
       await entity.volatile.set_volatile_value("nonexistent_key", 0)
   except Exception as e:
       print(f"Redis-Verbindungsfehler: {e}")

Weiterführende Informationen
-----------------------------

* :doc:`entity` - Entity-Dokumentation
* :doc:`parameter` - Persistente Alternative
* :doc:`../storage` - Storage-Backend Details
* :class:`~vyra_base.core.volatile.Volatile` - API-Referenz
* Redis Dokumentation: https://redis.io/docs/
