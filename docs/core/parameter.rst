Parameter - Persistent Configuration
====================================

The :class:`~vyra_base.core.parameter.Parameter`-class manages persistent configuration data,
which is stored in a SQLite database.
Concept
-------

Parameter are **permanent configuration values**, which:

* Are stored in a SQLite database
* Persist between module restarts
* Are accessible via ROS2 services
* Are automatically validated
* Support change events

Access via Entity
-------------------

The easiest access is done via the VyraEntity:

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
   
   # Read parameter
   await entity.parameter.get_param(request, response)
   print(f"Value: {response.parameter_value}")

All parameters
^^^^^^^^^^^^^^

.. code-block:: python

   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   # Read all parameters
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
       print("Parameter set successfully")

default values load
-------------------

Parameter can be initialized from JSON files with default values:

.. code-block:: python

   # default values from JSON load
   await entity.parameter.load_defaults(
       db_session=db_session,
       config_path="/workspace/config/defaults.json",
       reset=False  # True = overwrite existing values
   )

**Example JSON structure:**

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

Monitor your parameter changes in real-time:

.. code-block:: python

   # Query event topic
   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   await entity.parameter.get_update_param_event_topic(request, response)
   event_topic = response.event_topic
   
   # Listener for the event topic set up (via ROS2)
   # The event is triggered on every parameter change

Storage
-------

**Storage Location**: ``/workspace/storage/data/<module_name>.db``

The SQLite database stores parameters in the following structure:

* **Table name**: ``tb_parameters`` (See :doc:`../storage`)
* **Columns**: ``name``, ``value``, ``description``, ``timestamp``
* **Access**: Via SQLAlchemy ORM

.. note::
   Parameter database tables follow the naming convention ``tb_<name>``.
   Further information about table structure can be found under :doc:`../storage`.

Use Cases
---------

Parameter are suitable for:

✅ **Recommended:**

* Configuration values (Timeouts, Limits, thresholds)
* calibration data
* user preferences
* system settings

❌ **Not recommended:**

* High-frequency real-time data (use you :doc:`volatile`)
* Large data volumes (> MB, use you external databases)
* Temporary buffers

Performance Notes
-----------------
.. warning::
   Database accesses are relatively slow (~1-10ms).
   Avoid frequent parameter updates in real-time loops.

.. tip::
   Cache frequently read parameters in local variables:
   
   .. code-block:: python
   
      max_speed = await entity.parameter.get_param(...)
      
      # In loop use (without DB-Access)
      for i in range(1000):
          if speed > max_speed:
              # ...

Further Information
-----------------------------

* :doc:`entity` - Entity Documentation
* :doc:`volatile` - Volatile Alternative
* :doc:`../storage` - Storage Backend Details
* :class:`~vyra_base.core.parameter.Parameter` - API Reference