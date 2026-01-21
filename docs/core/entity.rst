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

Access to persistent configuration data via the Parameter component:

.. code-block:: python

   # Read parameter
   value = await entity.parameter.get_parameter(request, response)
   
   # Set parameter
   await entity.parameter.set_parameter(request, response)
   
   # Read all parameters
   all_params = await entity.parameter.read_all_params(request, response)

Parameters are stored in a **SQLite database** in the module under ``/workspace/storage/data/``.
This enables persistent data storage between restarts.

.. note::
   Parameters are suitable for configuration data that must be stored permanently.
   Database accesses are relatively slow, therefore not suitable for real-time data.

Volatile Management
^^^^^^^^^^^^^^^^^^^

Access to volatile, fast data via the Volatile component:

.. code-block:: python

   # Set volatile value
   await entity.volatile.set_volatile_value("sensor_data", temperature)
   
   # Read volatile value
   value = await entity.volatile.get_volatile_value("sensor_data")
   
   # Read all volatile names
   all_keys = await entity.volatile.read_all_volatile_names()
   
   # Regiser event listener for changes
   await entity.volatile.add_change_event("sensor_data")

Volatiles use **Redis** for fast in-memory data storage.
This data is **volatile** - it is lost on restart.

.. tip::
   Use Volatiles for:
   
   * Sensor data and real-time information
   * Temporary intermediate results
   * Fast inter-module communication
   * Event-based triggers

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