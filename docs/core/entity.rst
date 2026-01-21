VyraEntity - The Core
=====================

The :class:`~vyra_base.core.entity.VyraEntity` is the central management unit of each VYRA module.
It orchestrates all components such as ROS2 communication, state management, storage, and security.

Initialization
--------------

The entity is typically created in a module's ``_base_.py``:

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   from vyra_base.defaults.entries import StateEntry, NewsEntry, ErrorEntry, ModuleEntry
   
   # Prepare configuration
   state_entry = StateEntry(...)
   news_entry = NewsEntry(...)
   error_entry = ErrorEntry(...)
   module_entry = ModuleEntry(...)
   module_config = {...}
   
   # Create entity
   entity = VyraEntity(
       state_entry=state_entry,
       news_entry=news_entry,
       error_entry=error_entry,
       module_entry=module_entry,
       module_config=module_config
   )
   
   # Start entity
   await entity.startup_entity()

Main Functions
---------------

Parameter Management
^^^^^^^^^^^^^^^^^^^^

Access to persistent configuration data via the Parameter component.

**Internal API** (``_impl`` methods):

For use **within your module**, call the ``_impl`` methods directly:

.. code-block:: python

   # Read parameter (internal)
   result = await entity.param_manager.get_parameter_impl("max_speed")
   if result and result["success"]:
       param_data = json.loads(result["value"])
       print(f"Max Speed: {param_data['value']}")
   
   # Set parameter (internal)
   result = await entity.param_manager.set_parameter_impl(
       key="max_speed",
       value="120.0"
   )
   if result and result["success"]:
       print(f"✅ {result['message']}")
   
   # Read all parameters (internal)
   result = await entity.param_manager.read_all_params_impl()
   if result:
       params = json.loads(result["all_params_json"])
       for param in params:
           print(f"{param['name']}: {param['value']}")

**External API** (ROS2 services):

For access from **other modules** or via ROS2:

.. code-block:: python

   # Read parameter (ROS2 service)
   await entity.param_manager.get_parameter(request, response)
   
   # Set parameter (ROS2 service)
   await entity.param_manager.set_parameter(request, response)
   
   # Read all parameters (ROS2 service)
   await entity.param_manager.read_all_params(request, response)

Parameters are stored in a **SQLite database** in the module under ``/workspace/storage/data/``.
This enables persistent data storage between restarts.

.. note::
   Parameters are suitable for configuration data that must be stored permanently.
   Database accesses are relatively slow, therefore not suitable for real-time data.
   
.. tip::
   **Performance:** Use ``_impl`` methods for internal calls to avoid ROS2 serialization overhead.
   This is significantly faster (~10x) than going through the ROS2 service layer.

Volatile Management
^^^^^^^^^^^^^^^^^^^

Access to volatile, fast data via the Volatile component.

**Note:** Volatile methods are **already internal methods** - they don't have
a separate ROS2 service interface. All volatile operations are designed for
internal module use:

.. code-block:: python

   # Set volatile value (internal only)
   await entity.volatile.set_volatile_value("sensor_data", temperature)
   
   # Read volatile value (internal only)
   value = await entity.volatile.get_volatile_value("sensor_data")
   
   # Read all volatile names (internal only)
   all_keys = await entity.volatile.read_all_volatile_names()
   
   # Subscribe to changes -> creates ROS2 topic for external access
   await entity.volatile.subscribe_to_changes("sensor_data")
   await entity.volatile.activate_listener("sensor_data")

Volatiles use **Redis** for fast in-memory data storage.
This data is **volatile** - it is lost on restart.

.. tip::
   Use Volatiles for:
   
   * Sensor data and real-time information
   * Temporary intermediate results
   * Fast inter-module communication via ROS2 topics
   * Event-based triggers
   
   **Inter-module communication:** While volatiles are set internally,
   you can publish changes to ROS2 topics using ``subscribe_to_changes()``.
   Other modules can then subscribe to these topics.

ROS2 Communication
^^^^^^^^^^^^^^^^^^

The entity automatically provides ROS2 interfaces:

.. code-block:: python

   # Publish news
   entity.publish_news("Module started successfuly")
   
   # Publish error
   entity.publish_error("Connection error")
   
   # Publish state
   entity.publish_state()

State Machine
^^^^^^^^^^^^^

Each entity has an integrated state machine:

.. code-block:: python

   # Access state machine
   state_machine = entity.state_machine
   
   # Query current status
   current_state = state_machine.get_current_state()
   
   # Trigger state events
   await state_machine.trigger_event(StateEvent(...))

Regiser Interfaces
^^^^^^^^^^^^^^^^^^

ROS2 interfaces are defined via JSON metadata and registered automatically:

.. code-block:: python

   # Load and register interfaces from JSON
   interfaces = await entity.load_interfaces_from_config()
   await entity.set_interfaces(interfaces)
Interface Introspection
^^^^^^^^^^^^^^^^^^^^^^^

Query available interfaces programmatically:

**Internal API:**

.. code-block:: python

   # Get interface list (internal)
   result = await entity.get_interface_list_impl()
   
   if result:
       for interface_json in result["interface_list"]:
           interface = json.loads(interface_json)
           print(f"Interface: {interface['functionname']}")
           print(f"  Type: {interface['type']}")
           print(f"  Visible: {interface['displaystyle']['visible']}")

**External API (ROS2 service):**

.. code-block:: python

   # Get interface list (ROS2 service)
   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   await entity.get_interface_list(request, response)
   
   for interface_json in response.interface_list:
       interface = json.loads(interface_json)
       print(f"Interface: {interface['functionname']}")

**ROS2 CLI:**

.. code-block:: bash

   ros2 service call /module_name/get_interface_list \\
       vyra_base_interfaces/srv/GetInterfaceList "{}"
Lifecycle Management
--------------------

Startup
^^^^^^^
.. code-block:: python

   # Start up entity
   success = await entity.startup_entity()
   if success:
       print("Entity started successfully")

Shutdown
^^^^^^^^

.. code-block:: python

   # Shut down entity
   await entity.shutdown_entity()

Storage Access
---------------

The entity manages various storage backends:

.. code-block:: python

   # SQLite database access
   db_access = entity.storage.db_access
   
   # Redis client access
   redis_client = entity.storage.redis_client
   
   # Regiser custom storage
   await entity.setup_storage(
       db_path="/workspace/storage/data/module.db",
       redis_host="redis",
       redis_port=6379
   )

Important Notes
---------------

.. warning::
   The entity should only be initialized once per module.
   Multiple initializations can lead to conflicts.

.. important::
   All database operations are asynchronous (``async/await``).
   Don't forget to use ``await``!

Further Information
-----------------------------

* :doc:`parameter` - Detailed parameter documentation
* :doc:`volatile` - Detailed volatile documentation
* :class:`~vyra_base.core.entity.VyraEntity` - API Reference