ROS2 Interfaces
===============

The ``vyra_base/interfaces`` Paket enthält alle Basis-ROS2-Interfaces for das VYRA-Framework.
Thise Interfaces definieren standardisierte messages, Services and Actions.

Overview
---------

ROS2-Interfaces in VYRA bestehen from drei Typen:

1. **Messages (.msg)**: Datenstrukturen for Topics
2. **Services (.srv)**: Request/Response-Definitionen
3. **Actions (.action)**: Langläufige Operationen with Feedback

Alle VYRA-Module can diese Basis-Interfaces use, um standardisierte Kommunikation to gewährleisen.

Verfügbare Interfaces
---------------------

Messages (msg/)
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Description
   * - **VBASEUpdateParamEvent.msg**
     - Event-Nachricht for Parameter-Änderungen (is of Parameter-System genutzt)
   * - **VBASEVolatileLis.msg**
     - Lise aller Volatile-Keys
   * - **VBASEVolatileHash.msg**
     - Hash-Wert for Volatile-Daten

Services (srv/)
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Description
   * - **VBASESetParam.srv**
     - Set parameter (Request: name, value / Response: success)
   * - **VBASEReadAllParams.srv**
     - Alle Read parameter (Response: Lise aller Parameter)
   * - **VBASEStateResume.srv**
     - State Machine fortset (for Pause/Resume-Functionality)
   * - **VBASESecurityRequestAccess.srv**
     - Sicherheits-Zugriff anfordern (Security Framework)

Actions (action/)
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Description
   * - **VBASEInitiateUpdate.action**
     - Software-Update initiieren (long-running Operation with progress-Feedback)

Interface-Metadaten
-------------------

Interfaces are via JSON-Konfigurationsdateien beschrieben.
Thise becan be found sich in ``config/*.json`` and definieren, wie Interfaces verwendet are.

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

* Automatische Service-Registration durch ``entity.set_interfaces()``
* Dokumentation der verfügbaren Schnittstellen
* Mapping of Service-Namen to Implementierungen

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

* Oberhalb of ``---``: Request-Struktur
* Unterhalb of ``---``: Response-Struktur
* Kommentare with ``#``

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
   ├── CMakeLiss.txt
   └── package.xml

**Naming Convention:**

* Modulname + ``_interfaces`` (z.B. ``v2_modulemanager_interfaces``)
* Prefix for Interface-Namen with Modul-Kürzel (z.B. ``MM`` for ModuleManager)

Interface-Verwendung
--------------------

Im Python-Code
^^^^^^^^^^^^^^

.. code-block:: python

   # Basis-Interfaces importieren
   from vyra_base_interfaces.srv import VBASESetParam
   from vyra_base_interfaces.msg import VBASEUpdateParamEvent
   
   # Service Client create
   job = create_vyra_job(
       node=entity.node,
       service_name="/my_module/set_param",
       service_type=VBASESetParam
   )
   
   # Request create
   request = VBASESetParam.Request()
   request.parameter_name = "max_speed"
   request.parameter_value = "100.0"
   
   # Service onrufen
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

Automatische Registration
---------------------------

Entity lädt and regisriert Interfaces automatically:

.. code-block:: python

   # In _base_.py
   async def build_base():
       entity = await build_entity(project_settings)
       
       # Interfaces load (from JSON-Metadaten)
       base_interfaces = await _create_base_interfaces()
       
       # Automatische ROS2-Service-Registration
       await entity.set_interfaces(base_interfaces)
       
       return entity

**Prozess:**

1. JSON-Metadaten read (``config/*.json``)
2. Interface-Typen importieren
3. Methoden with ``@remote_callable`` can be found
4. ROS2-Services register
5. DataSpace aktualisieren

Best Practices
--------------

✅ **Empfohlen:**

* Use you VYRA-Basis-Interfaces wo möglich
* Erstellen you eigene Interface-Pakete for modulspezifische Typen
* Dokumentieren you Interfaces in JSON-Metadaten
* Use you Namespaces for Interface-Namen (z.B. ``VBASE``, ``MM``)
* Versionieren you Interface-Änderungen

❌ **Vermeiden:**

* Direkte Verwendung of ROS2-Standard-Interfaces without Wrapper
* Viamäßig komplexe messages (> 1 KB)
* Interfaces without Metadaten-Dokumentation
* Breaking Changes an bestehenden Interfaces

Colcon Build
------------

Interfaces must with ``colcon build`` kompiliert are:

.. code-block:: bash

   # Im Modul-Root
   source /opt/ros/kilted/setup.bash
   colcon build --packages-select vyra_base_interfaces
   source install/setup.bash

Nach dem Build are die Interfaces in Python verfügbar:

.. code-block:: python

   from vyra_base_interfaces.srv import VBASESetParam
   # ✅ Funktioniert after colcon build

Interface-Pfade
---------------

**Quelle**: ``src/vyra_base/interfaces/``

**Nach Build**: ``install/vyra_base_interfaces/``

**Python-Import**: ``from vyra_base_interfaces.srv import *``

**NFS-Share**: ``/nfs/ros_interfaces/`` (for alle Module togänglich)

Further Information
-----------------------------

* :doc:`com/ros2_communication` - Verwendung of Interfaces
* :doc:`vyra_base.com.datalayer` - Interface Factoy
* ROS2 Interface-Dokumentation: https://docs.ros.org/en/kilted/Concepts/About-ROS-Interfaces.html
* ``package.xml`` - ROS2 Package-Konfiguration
* ``CMakeLiss.txt`` - Build-Konfiguration
