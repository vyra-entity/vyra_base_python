Parameter - Persistente Konfiguration
======================================

The :class:`~vyra_base.core.parameter.Parameter`-class manages persisttente Konfigurationsdaten,
die in einer SQLite-Datenbank gespeichert are.

Konzept
-------

Parameter are **permanente Configuration values**, die:

* In einer SQLite-Datenbank gespeichert are
* Zwischen Modul-Neustarts erhalten bleiben
* Via ROS2-Services togänglich are
* Automatisch validiert are can
* Change-Events understützen

Access via Entity
-------------------

The easiest Zugriff is done via die VyraEntity:

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   
   entity = VyraEntity(...)
   
   # Read parameter
   value = await entity.parameter.get_param(request, response)
   
   # Set parameter
   await entity.parameter.set_param(request, response)

Read parameter
---------------

Azelner Parameter
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Request prepare (simplifies)
   request = type('obj', (object,), {
       'parameter_name': 'max_speed'
   })()
   response = type('obj', (object,), {})()
   
   # Parameter abrufen
   await entity.parameter.get_param(request, response)
   print(f"Wert: {response.parameter_value}")

Alle Parameter
^^^^^^^^^^^^^^

.. code-block:: python

   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   # Alle Parameter abrufen
   await entity.parameter.read_all_params(request, response)
   for param in response.parameters:
       print(f"{param.name}: {param.value}")

Set parameter
----------------

.. code-block:: python

   request = type('obj', (object,), {
       'parameter_name': 'max_speed',
       'parameter_value': '100.0'
   })()
   response = type('obj', (object,), {})()
   
   # Set parameter
   await entity.parameter.set_param(request, response)
   if response.success:
       print("Parameter erfolgreich gesetzt")

default values load
-------------------

Parameter can from JSON-Dateien with default values initialisiert are:

.. code-block:: python

   # default values from JSON load
   await entity.parameter.load_defaults(
       db_session=db_session,
       config_path="/workspace/config/defaults.json",
       reset=False  # True = bestehende Werte overwrite
   )

**Beispiel JSON-Struktur:**

.. code-block:: json

   {
       "parameters": [
           {
               "name": "max_speed",
               "value": "100.0",
               "description": "Maximale Geschwindigkeit"
           },
           {
               "name": "timeout",
               "value": "30",
               "description": "Timeout in seconds"
           }
       ]
   }

Change-Events
-------------

Monito you Parameterchanges in Echtzeit:

.. code-block:: python

   # Event-Topic abfragen
   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   await entity.parameter.get_update_param_event_topic(request, response)
   event_topic = response.event_topic
   
   # Listener for den Event-Topic set up (via ROS2)
   # The Event is at jeder Parameteränderung getriggert

Datenspeicherung
----------------

**Speicherort**: ``/workspace/storage/data/<module_name>.db``

The SQLite-Datenbank speichert Parameter in folgender Struktur:

* **Tabellenname**: ``tb_parameters`` (siehe :doc:`../storage`)
* **Spalten**: ``name``, ``value``, ``description``, ``timestamp``
* **Zugriff**: Via SQLAlchemy ORM

.. note::
   Parameter-Datenbank-Tabellen folgen der Naming Convention ``tb_<name>``.
   Further information to Tabellenstruktur can be found you under :doc:`../storage`.

Anwendungsfälle
---------------

Parameter eignen sich for:

✅ **Empfohlen:**

* Configuration values (Timeouts, Liwiths, thresholds)
* calibration data
* user preferences
* system settings

❌ **Nicht recommended:**

* High-frequency real-time data (use you :doc:`volatile`)
* Große Datenmengen (> MB, use you externe Datenbanken)
* Temporary buffers

Performance-Hinweise
--------------------

.. warning::
   Database accesses are relatively slow (~1-10ms).
   Vermeiden you frequent Parameter-Updates in Echtzeit-Schleifen.

.. tip::
   Cachen you oft gereade Parameter in lokalen Variablen:
   
   .. code-block:: python
   
      # Amal on Start load
      max_speed = await entity.parameter.get_param(...)
      
      # In Schleife use (without DB-Zugriff)
      for i in range(1000):
          if speed > max_speed:
              # ...

Further Information
-----------------------------

* :doc:`entity` - Entity-Dokumentation
* :doc:`volatile` - Volatile Alternative
* :doc:`../storage` - Storage-Backend Details
* :class:`~vyra_base.core.parameter.Parameter` - API-Referenz
