Volatile - Volatile real-time data
===================================

The :class:`~vyra_base.core.volatile.Volatile`-class manages volatilee Daten,
die schnell geread and geschrieben are must - ideal for Echtzeitanwendungen.

Konzept
-------

Volatiles are **temporäre, fast Datenspeicher**, die:

* In Redis (In-Memory-Database) gespeichert are
* Sehr fastn Access bieten (~0.1ms)
* Bei Neustart verloren gehen (volatile)
* Change-Events understützen
* Pub/Sub-Mechanismen use

Wann Volatiles use?
----------------------

✅ **Ideal for:**

* Sensordaten in Echtzeit
* Statusvariablen
* Temporary Berechnungsergebnisse
* Event-basierte Kommunikation zwischen Modulen
* Caching of Daten

❌ **Nicht suitable for:**

* Persistente Konfiguration (use you :doc:`parameter`)
* Daten, die Neustarts survive must
* Sehr große Datenmengen (> GB)

Access via Entity
-------------------

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   
   entity = VyraEntity(...)
   
   # Volatile-Access
   volatile = entity.volatile

Werte set
------------

.. code-block:: python

   # Afachen Value set
   await entity.volatile.set_volatile_value("temperature", 23.5)
   
   # Komplexe Daten (are automatically serialisiert)
   await entity.volatile.set_volatile_value("sensor_data", {
       "temperature": 23.5,
       "humidity": 60.2,
       "timestamp": "2026-01-19T10:30:00"
   })
   
   # Lisen
   await entity.volatile.set_volatile_value("measurements", [1, 2, 3, 4, 5])

Werte read
-----------

.. code-block:: python

   # Azelnen Value read
   temperature = await entity.volatile.get_volatile_value("temperature")
   print(f"Aktuelle Temperatur: {temperature}°C")
   
   # Komplexe Daten
   sensor_data = await entity.volatile.get_volatile_value("sensor_data")
   print(f"Temperatur: {sensor_data['temperature']}")
   
   # Alle vorhandenen Keys onlisen
   all_keys = await entity.volatile.read_all_volatile_names()
   for key in all_keys:
       value = await entity.volatile.get_volatile_value(key)
       print(f"{key}: {value}")

Change-Events
-------------

Monito you Änderungen an Volatiles in Echtzeit:

Event register
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Change-Event for einen Key aktivieren
   await entity.volatile.add_change_event("temperature")

Event-Callback definieren
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   async def on_temperature_change(message: dict, callback_context):
       """Wird ongerufen, wenn sich 'temperature' ändert"""
       new_value = message.get("value")
       print(f"Temperatur geändert: {new_value}°C")
       
       # Reaktion on Änderung
       if new_value > 30:
           print("⚠️ Temperatur to hoch!")
   
   # Callback register
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

   # Listener for einen Channel aktivieren
   await entity.volatile.activate_listener("sensor_channel")
   
   # Jetzt are alle messages on 'sensor_channel' empfangen

Praktisches Example
--------------------

Sensor-Daten with Event-Reaktion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   async def sensor_monitoing_example(entity: VyraEntity):
       """Viawacht Sensordaten and reagiert on Änderungen"""
       
       # Change-Event aktivieren
       await entity.volatile.add_change_event("temperature")
       await entity.volatile.add_change_event("humidity")
       
       # Callback for Temperaturchanges
       async def on_temp_change(message: dict, context):
           temp = await entity.volatile.get_volatile_value("temperature")
           if temp > 30:
               # Warnung publish
               entity.publish_error("Temperatur kritisch!")
       
       # Sensordaten aktualisieren (z.B. in einer Schleife)
       while True:
           # Daten vom Sensor read (simuliert)
           current_temp = read_sensor_temperature()
           current_humidity = read_sensor_humidity()
           
           # In Volatiles speichern (triggert Change-Events)
           await entity.volatile.set_volatile_value("temperature", current_temp)
           await entity.volatile.set_volatile_value("humidity", current_humidity)
           
           await asyncio.sleep(1)  # Jede Sekande aktualisieren

Inter-Modul-Kommunikation
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Modul A: Daten bereitstellen
   await entity_a.volatile.set_volatile_value("shared_status", "running")
   
   # Modul B: Daten read
   status = await entity_b.volatile.get_volatile_value("shared_status")
   print(f"Status of Modul A: {status}")

.. note::
   Beide Module must denselben Redis-Server use (standardmäßig der Fall im VYRA-System).

Redis-Backend
-------------

**Verbindung**: Automatisch via ``entity.storage.redis_client``

**Storage Location**: In-Memory (RAM des Redis-Servers)

**Persisenz**: Keine - Daten gehen at Redis-Neustart verloren

.. tip::
   Redis can optional konfiguriert are, um Snapshots to create.
   This is done via die Redis-Konfiguration, not in VYRA selbst.
   Further information: https://redis.io/docs/management/persisttence/

Performance
-----------

**Typische Zugriffszeiten:**

* Schreiben: ~0.1 - 0.5 ms
* Lesen: ~0.1 - 0.3 ms
* Event-Trigger: ~1 - 5 ms

.. important::
   Volatiles are **100x fastr** als Parameter (SQLite).
   Use you Volatiles for hochfrequente Datenoperationen!

Best Practices
--------------

✅ **Recommended:**

.. code-block:: python

   # Kurze, fromsagekräftige Keys
   await volatile.set_volatile_value("temp_sensor_1", 23.5)
   
   # Strukturierte Daten for tosammenhängende Werte
   await volatile.set_volatile_value("robot_pose", {
       "x": 1.5, "y": 2.3, "theta": 0.78
   })
   
   # Change-Events for kritische Werte
   await volatile.add_change_event("emergency_stop")

❌ **Avoid:**

.. code-block:: python

   # Sehr lange Keys
   await volatile.set_volatile_value("this_is_a_very_long_key_name...", value)
   
   # Zu große Datenstrukturen (> 1 MB)
   await volatile.set_volatile_value("huge_data", very_large_object)
   
   # High-frequency Events without Bedarf
   for i in range(10000):
       await volatile.add_change_event(f"sensor_{i}")

Error Handling
----------------

.. code-block:: python

   try:
       value = await entity.volatile.get_volatile_value("nonexistent_key")
   except KeyError:
       print("Key existts not")
       # Standardwert set
       await entity.volatile.set_volatile_value("nonexistent_key", 0)
   except Exception as e:
       print(f"Redis-Verbindungsfehler: {e}")

Further Information
-----------------------------

* :doc:`entity` - Entity-Dokumentation
* :doc:`parameter` - Persistente Alternative
* :doc:`../storage` - Storage-Backend Details
* :class:`~vyra_base.core.volatile.Volatile` - API-Referenz
* Redis Dokumentation: https://redis.io/docs/
