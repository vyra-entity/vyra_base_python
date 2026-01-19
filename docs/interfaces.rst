ROS2 Interfaces
===============

Das ``vyra_base/interfaces`` Paket enthält alle Basis-ROS2-Interfaces für das VYRA-Framework.
Diese Interfaces definieren standardisierte Nachrichten, Services und Actions.

Übersicht
---------

ROS2-Interfaces in VYRA bestehen aus drei Typen:

1. **Messages (.msg)**: Datenstrukturen für Topics
2. **Services (.srv)**: Request/Response-Definitionen
3. **Actions (.action)**: Langläufige Operationen mit Feedback

Alle VYRA-Module können diese Basis-Interfaces nutzen, um standardisierte Kommunikation zu gewährleisten.

Verfügbare Interfaces
---------------------

Messages (msg/)
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Beschreibung
   * - **VBASEUpdateParamEvent.msg**
     - Event-Nachricht für Parameter-Änderungen (wird von Parameter-System genutzt)
   * - **VBASEVolatileList.msg**
     - Liste aller Volatile-Keys
   * - **VBASEVolatileHash.msg**
     - Hash-Wert für Volatile-Daten

Services (srv/)
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Beschreibung
   * - **VBASESetParam.srv**
     - Parameter setzen (Request: name, value / Response: success)
   * - **VBASEReadAllParams.srv**
     - Alle Parameter lesen (Response: Liste aller Parameter)
   * - **VBASEStateResume.srv**
     - State Machine fortsetzen (für Pause/Resume-Funktionalität)
   * - **VBASESecurityRequestAccess.srv**
     - Sicherheits-Zugriff anfordern (Security Framework)

Actions (action/)
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Beschreibung
   * - **VBASEInitiateUpdate.action**
     - Software-Update initiieren (langläufige Operation mit Fortschritts-Feedback)

Interface-Metadaten
-------------------

Interfaces werden über JSON-Konfigurationsdateien beschrieben.
Diese befinden sich in ``config/*.json`` und definieren, wie Interfaces verwendet werden.

Beispiel-Metadaten
^^^^^^^^^^^^^^^^^^

**Parameter-Service-Definition** (``config/parameter_metadata.json``):

.. code-block:: json

   {
       "services": [
           {
               "name": "set_param",
               "type": "vyra_base_interfaces/srv/VBASESetParam",
               "description": "Setzt einen Parameter-Wert",
               "implementation": "vyra_base.core.parameter.Parameter.set_param"
           },
           {
               "name": "read_all_params",
               "type": "vyra_base_interfaces/srv/VBASEReadAllParams",
               "description": "Liest alle Parameter",
               "implementation": "vyra_base.core.parameter.Parameter.read_all_params"
           }
       ]
   }

**Verwendungszweck:**

* Automatische Service-Registrierung durch ``entity.set_interfaces()``
* Dokumentation der verfügbaren Schnittstellen
* Mapping von Service-Namen zu Implementierungen

Interface-Definition
--------------------

Beispiel: Service definieren
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**VBASESetParam.srv**:

.. code-block:: text

   # Request
   string parameter_name
   string parameter_value
   ---
   # Response
   bool success
   string message

**Bedeutung:**

* Oberhalb von ``---``: Request-Struktur
* Unterhalb von ``---``: Response-Struktur
* Kommentare mit ``#``

Beispiel: Message definieren
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**VBASEUpdateParamEvent.msg**:

.. code-block:: text

   # Parameter-Update-Event
   string parameter_name
   string old_value
   string new_value
   builtin_interfaces/Time timestamp

Eigene Module-Interfaces
-------------------------

Jedes VYRA-Modul sollte sein eigenes Interface-Paket haben:

.. code-block:: text

   v2_modulemanager_*_interfaces/
   ├── msg/
   │   └── CustomMessage.msg
   ├── srv/
   │   └── CustomService.srv
   ├── action/
   │   └── CustomAction.action
   ├── config/
   │   └── metadata.json
   ├── CMakeLists.txt
   └── package.xml

**Namenskonvention:**

* Modulname + ``_interfaces`` (z.B. ``v2_modulemanager_interfaces``)
* Prefix für Interface-Namen mit Modul-Kürzel (z.B. ``MM`` für ModuleManager)

Interface-Verwendung
--------------------

Im Python-Code
^^^^^^^^^^^^^^

.. code-block:: python

   # Basis-Interfaces importieren
   from vyra_base_interfaces.srv import VBASESetParam
   from vyra_base_interfaces.msg import VBASEUpdateParamEvent
   
   # Service Client erstellen
   job = create_vyra_job(
       node=entity.node,
       service_name="/my_module/set_param",
       service_type=VBASESetParam
   )
   
   # Request erstellen
   request = VBASESetParam.Request()
   request.parameter_name = "max_speed"
   request.parameter_value = "100.0"
   
   # Service aufrufen
   response = await job.call_async(request)

In JSON-Metadaten
^^^^^^^^^^^^^^^^^

.. code-block:: json

   {
       "services": [
           {
               "name": "my_custom_service",
               "type": "my_module_interfaces/srv/MyService",
               "description": "Mein eigener Service"
           }
       ],
       "topics": [
           {
               "name": "sensor_data",
               "type": "sensor_msgs/msg/Temperature",
               "qos": "sensor_data"
           }
       ]
   }

Automatische Registrierung
---------------------------

Entity lädt und registriert Interfaces automatisch:

.. code-block:: python

   # In _base_.py
   async def build_base():
       entity = await build_entity(project_settings)
       
       # Interfaces laden (aus JSON-Metadaten)
       base_interfaces = await _create_base_interfaces()
       
       # Automatische ROS2-Service-Registrierung
       await entity.set_interfaces(base_interfaces)
       
       return entity

**Prozess:**

1. JSON-Metadaten lesen (``config/*.json``)
2. Interface-Typen importieren
3. Methoden mit ``@remote_callable`` finden
4. ROS2-Services registrieren
5. DataSpace aktualisieren

Best Practices
--------------

✅ **Empfohlen:**

* Verwenden Sie VYRA-Basis-Interfaces wo möglich
* Erstellen Sie eigene Interface-Pakete für modulspezifische Typen
* Dokumentieren Sie Interfaces in JSON-Metadaten
* Nutzen Sie Namespaces für Interface-Namen (z.B. ``VBASE``, ``MM``)
* Versionieren Sie Interface-Änderungen

❌ **Vermeiden:**

* Direkte Verwendung von ROS2-Standard-Interfaces ohne Wrapper
* Übermäßig komplexe Nachrichten (> 1 KB)
* Interfaces ohne Metadaten-Dokumentation
* Breaking Changes an bestehenden Interfaces

Colcon Build
------------

Interfaces müssen mit ``colcon build`` kompiliert werden:

.. code-block:: bash

   # Im Modul-Root
   source /opt/ros/kilted/setup.bash
   colcon build --packages-select vyra_base_interfaces
   source install/setup.bash

Nach dem Build sind die Interfaces in Python verfügbar:

.. code-block:: python

   from vyra_base_interfaces.srv import VBASESetParam
   # ✅ Funktioniert nach colcon build

Interface-Pfade
---------------

**Quelle**: ``src/vyra_base/interfaces/``

**Nach Build**: ``install/vyra_base_interfaces/``

**Python-Import**: ``from vyra_base_interfaces.srv import *``

**NFS-Share**: ``/nfs/ros_interfaces/`` (für alle Module zugänglich)

Weiterführende Informationen
-----------------------------

* :doc:`com/ros2_communication` - Verwendung von Interfaces
* :doc:`vyra_base.com.datalayer` - Interface Factory
* ROS2 Interface-Dokumentation: https://docs.ros.org/en/kilted/Concepts/About-ROS-Interfaces.html
* ``package.xml`` - ROS2 Package-Konfiguration
* ``CMakeLists.txt`` - Build-Konfiguration
