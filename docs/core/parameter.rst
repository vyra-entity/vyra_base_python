Parameter - Persistente Konfiguration
======================================

Die :class:`~vyra_base.core.parameter.Parameter`-Klasse verwaltet persistente Konfigurationsdaten,
die in einer SQLite-Datenbank gespeichert werden.

Konzept
-------

Parameter sind **dauerhafte Konfigurationswerte**, die:

* In einer SQLite-Datenbank gespeichert werden
* Zwischen Modul-Neustarts erhalten bleiben
* Über ROS2-Services zugänglich sind
* Automatisch validiert werden können
* Change-Events unterstützen

Zugriff über Entity
-------------------

Der einfachste Zugriff erfolgt über die VyraEntity:

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   
   entity = VyraEntity(...)
   
   # Parameter lesen
   value = await entity.parameter.get_param(request, response)
   
   # Parameter setzen
   await entity.parameter.set_param(request, response)

Parameter lesen
---------------

Einzelner Parameter
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Request vorbereiten (vereinfacht)
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

Parameter setzen
----------------

.. code-block:: python

   request = type('obj', (object,), {
       'parameter_name': 'max_speed',
       'parameter_value': '100.0'
   })()
   response = type('obj', (object,), {})()
   
   # Parameter setzen
   await entity.parameter.set_param(request, response)
   if response.success:
       print("Parameter erfolgreich gesetzt")

Standardwerte laden
-------------------

Parameter können aus JSON-Dateien mit Standardwerten initialisiert werden:

.. code-block:: python

   # Standardwerte aus JSON laden
   await entity.parameter.load_defaults(
       db_session=db_session,
       config_path="/workspace/config/defaults.json",
       reset=False  # True = bestehende Werte überschreiben
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
               "description": "Timeout in Sekunden"
           }
       ]
   }

Change-Events
-------------

Überwachen Sie Parameteränderungen in Echtzeit:

.. code-block:: python

   # Event-Topic abfragen
   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   await entity.parameter.get_update_param_event_topic(request, response)
   event_topic = response.event_topic
   
   # Listener für den Event-Topic einrichten (über ROS2)
   # Der Event wird bei jeder Parameteränderung getriggert

Datenspeicherung
----------------

**Speicherort**: ``/workspace/storage/data/<module_name>.db``

Die SQLite-Datenbank speichert Parameter in folgender Struktur:

* **Tabellenname**: ``tb_parameters`` (siehe :doc:`../storage`)
* **Spalten**: ``name``, ``value``, ``description``, ``timestamp``
* **Zugriff**: Über SQLAlchemy ORM

.. note::
   Parameter-Datenbank-Tabellen folgen der Namenskonvention ``tb_<name>``.
   Weitere Informationen zur Tabellenstruktur finden Sie unter :doc:`../storage`.

Anwendungsfälle
---------------

Parameter eignen sich für:

✅ **Empfohlen:**

* Konfigurationswerte (Timeouts, Limits, Schwellwerte)
* Kalibrationsdaten
* Benutzerpräferenzen
* Systemeinstellungen

❌ **Nicht empfohlen:**

* Hochfrequente Echtzeitdaten (nutzen Sie :doc:`volatile`)
* Große Datenmengen (> MB, nutzen Sie externe Datenbanken)
* Temporäre Zwischenspeicher

Performance-Hinweise
--------------------

.. warning::
   Datenbankzugriffe sind relativ langsam (~1-10ms).
   Vermeiden Sie häufige Parameter-Updates in Echtzeit-Schleifen.

.. tip::
   Cachen Sie oft gelesene Parameter in lokalen Variablen:
   
   .. code-block:: python
   
      # Einmal beim Start laden
      max_speed = await entity.parameter.get_param(...)
      
      # In Schleife nutzen (ohne DB-Zugriff)
      for i in range(1000):
          if speed > max_speed:
              # ...

Weiterführende Informationen
-----------------------------

* :doc:`entity` - Entity-Dokumentation
* :doc:`volatile` - Flüchtige Alternative
* :doc:`../storage` - Storage-Backend Details
* :class:`~vyra_base.core.parameter.Parameter` - API-Referenz
