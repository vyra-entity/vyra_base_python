ROS2 Interfaces
===============

The ``vyra_base/interfaces`` Paket enthält alle Basis-ROS2-Interfaces for das VYRA-Framework.
Thise Interfaces definieren standardisierte messages, Services and Actions.

Overview
--------

ROS2-Interfaces in VYRA bestehen from drei Typen:

1. **Messages (.msg)**: Datenstrukturen for Topics
2. **Services (.srv)**: Request/Response-Definitionen
3. **Actions (.action)**: Langläufige Operationen with Feedback

Alle VYRA-Module can diese Basis-Interfaces use, um standardisierte Kommunikation to gewährleisen.

Available Interfaces
--------------------

Messages (msg/)
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Description
   * - **VBASEUpdateParamEvent.msg**
     - Event message for parameter changes (used by the parameter system)
   * - **VBASEVolatileLis.msg**
     - List of all volatile keys
   * - **VBASEVolatileHash.msg**
     - Hash value for volatile data

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
     - Read all parameters (Response: list of all parameters)
   * - **VBASEStateResume.srv**
     - State Machine resume (for Pause/Resume functionality)
   * - **VBASESecurityRequestAccess.srv**
     - Request security access (Security Framework)
Actions (action/)
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Interface
     - Description
   * - **VBASEInitiateUpdate.action**
     - Initiate software update (long-running operation with progress feedback)

Interface-Metadata
------------------

Interfaces are described via JSON configuration files.
These can be found in ``config/*.json`` and define how interfaces are used.

Example Metadata
^^^^^^^^^^^^^^^^^^

**Parameter-Service-Definition** (``config/parameter_metadata.json``):

.. code-block:: json

   {
       "services": [
           {
               "name": "set_parameter",
               "type": "vyra_base_interfaces/srv/VBASESetParam",
               "description": "Set a parameter value",
               "implementation": "vyra_base.core.parameter.Parameter.set_parameter"
           },
           {
               "name": "read_all_params",
               "type": "vyra_base_interfaces/srv/VBASEReadAllParams",
               "description": "Read all parameters",
               "implementation": "vyra_base.core.parameter.Parameter.read_all_params"
           }
       ]
   }

**Purpose:**

* Automatic service registration through ``entity.set_interfaces()``
* Documentation of available interfaces
* Mapping of service names to implementations
Interface-Definition
--------------------

Example: Define Service
^^^^^^^^^^^^^^^^^^^^^^^

**VBASESetParam.srv**:

.. code-block:: text

   # Request
   string parameter_name
   string parameter_value
   ---
   # Response
   bool success
   string message

**Meaning:**

* Above ``---``: Request structure
* Below ``---``: Response structure
* Comments with ``#``
Example: Define Message
^^^^^^^^^^^^^^^^^^^^^^^

**VBASEUpdateParamEvent.msg**:

.. code-block:: text

   # Parameter-Update-Event
   string parameter_name
   string old_value
   string new_value
   builtin_interfaces/Time timestamp

Own Module Interfaces
---------------------

Each VYRA module should have its own interface package:

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

* Module name + ``_interfaces`` (e.g. ``v2_modulemanager_interfaces``)
* Prefix for interface names with module abbreviation (e.g. ``MM`` for ModuleManager)

Interface Usage
---------------

In Python Code
^^^^^^^^^^^^^^

.. code-block:: python

   # Import base interfaces
   from vyra_base_interfaces.srv import VBASESetParam
   from vyra_base_interfaces.msg import VBASEUpdateParamEvent
   
   # Create service client
   job = create_vyra_job(
       node=entity.node,
       service_name="/my_module/set_parameter",
       service_type=VBASESetParam
   )
   
   # Create request
   request = VBASESetParam.Request()
   request.parameter_name = "max_speed"
   request.parameter_value = "100.0"
   
   # Call service
   response = await job.call_async(request)

In JSON Metadata
^^^^^^^^^^^^^^^^

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

Automatic Registration
----------------------

Entity loads and registers interfaces automatically:

.. code-block:: python

   # In _base_.py
   async def build_base():
       entity = await build_entity(project_settings)
       
       # Interfaces load (from JSON-Metadaten)
       base_interfaces = await _create_base_interfaces()
       
       # Automatische ROS2-Service-Registration
       await entity.set_interfaces(base_interfaces)
       
       return entity

**Process:**

1. JSON metadata read (``config/*.json``)
2. mport interface types
3. Methods with ``@remote_callable`` can be found
4. Register ROS2 services
5. Update DataSpace

Best Practices
--------------

✅ **Recommended:**

* Use your VYRA base interfaces where possible
* Create your own interface packages for module-specific types
* Document your interfaces in JSON metadata
* Use your namespaces for interface names (e.g. ``VBASE``, ``MM``)
* Version your interface changes

❌ **Avoid:**

* Direct use of ROS2 standard interfaces without wrapper
* Excessively complex messages (> 1 KB)
* Interfaces without metadata documentation
* Breaking changes to existing interfaces

Colcon Build
------------

Interfaces must be compiled with ``colcon build``:

.. code-block:: bash

   # In the module root
   source /opt/ros/kilted/setup.bash
   colcon build --packages-select vyra_base_interfaces
   source install/setup.bash

After the build, the interfaces are available in Python:

.. code-block:: python

   from vyra_base_interfaces.srv import VBASESetParam
   # ✅ Works after colcon build

Interface-Pfade
---------------

**Source**: ``src/vyra_base/interfaces/``

**After Build**: ``install/vyra_base_interfaces/``

**Python Import**: ``from vyra_base_interfaces.srv import *``
**NFS-Share**: ``/nfs/vyra_interfaces/`` (for all modules accessible)

Further Information
-----------------------------

* :doc:`com/ros2_communication` - Usage of Interfaces
* :doc:`vyra_base.com.datalayer` - Interface Factory
* ROS2 Interface Documentation: https://docs.ros.org/en/kilted/Concepts/About-ROS-Interfaces.html
* ``package.xml`` - ROS2 Package Configuration
* ``CMakeLists.txt`` - Build Configuration