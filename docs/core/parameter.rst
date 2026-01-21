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
   
   # External access via ROS2 services (requires request/response objects)
   value = await entity.param_manager.get_parameter(request, response)
   
   # Internal access via _impl methods (direct, without ROS2 overhead)
   result = await entity.param_manager.get_parameter_impl("max_speed")

Internal vs. External API
^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameter functions are available in **two variants**:

**1. ROS2 Service Interface** (``@remote_callable``)
   - For external access via ROS2 services
   - Requires ``request`` and ``response`` objects
   - Used by other modules or external systems
   - Example: ``get_parameter(request, response)``

**2. Internal Implementation** (``_impl`` suffix)
   - For internal module use (Python code)
   - Direct function calls without ROS2 overhead
   - Returns Python dictionaries
   - Example: ``get_parameter_impl(key)``

.. tip::
   **When to use which?**
   
   - Use ``_impl`` methods when calling from **within your module**
   - Use ROS2 services when calling from **other modules** or **external systems**
   
   The ``_impl`` methods are **faster** as they skip the ROS2 serialization layer.

Read parameter
---------------

Single Parameter (Internal API)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For use **within your module**, use the ``_impl`` method:

.. code-block:: python

   # Direct internal call
   result = await entity.param_manager.get_parameter_impl("max_speed")
   
   if result and result["success"]:
       param_data = json.loads(result["value"])
       print(f"Max Speed: {param_data['value']}")
   else:
       print(f"Error: {result['message'] if result else 'Unknown error'}")

Single Parameter (ROS2 Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For external access via ROS2 services:

.. code-block:: python

   # Request prepare
   request = type('obj', (object,), {
       'key': 'max_speed'
   })()
   response = type('obj', (object,), {})()
   
   # Call ROS2 service
   await entity.param_manager.get_parameter(request, response)
   
   if response.success:
       print(f"Value: {response.json_value}")

**ROS2 CLI:**

.. code-block:: bash

   ros2 service call /module_name/get_parameter \\
       vyra_base_interfaces/srv/GetParameter \\
       "{key: 'max_speed'}"

All parameters (Internal API)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Direct internal call
   result = await entity.param_manager.read_all_params_impl()
   
   if result:
       params = json.loads(result["all_params_json"])
       for param in params:
           print(f"{param['name']}: {param['value']}")

All parameters (ROS2 Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   # Call ROS2 service
   await entity.param_manager.read_all_params(request, response)
   
   # Parse JSON response
   params = json.loads(response.all_params_json)
   for param in params:
       print(f"{param['name']}: {param['value']}")

Set parameter
----------------

Set Parameter (Internal API)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Direct internal call
   result = await entity.param_manager.set_parameter_impl(
       key="max_speed",
       value="120.0"
   )
   
   if result and result["success"]:
       print(f"✅ {result['message']}")
   else:
       print(f"❌ Error: {result['message'] if result else 'Unknown error'}")

Set Parameter (ROS2 Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   request = type('obj', (object,), {
       'key': 'max_speed',
       'value': '120.0'
   })()
   response = type('obj', (object,), {})()
   
   # Call ROS2 service
   await entity.param_manager.set_parameter(request, response)
   
   if response.success:
       print(f"✅ {response.message}")
   else:
       print(f"❌ {response.message}")

**ROS2 CLI:**

.. code-block:: bash

   ros2 service call /module_name/set_parameter \\
       vyra_base_interfaces/srv/SetParameter \\
       "{key: 'max_speed', value: '120.0'}"

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

Monitor parameter changes in real-time:

Get Event Topic (Internal API)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Direct internal call
   result = await entity.param_manager.param_changed_topic_impl()
   
   if result:
       event_topic = result["topic"]
       print(f"Parameter change topic: {event_topic}")
       
       # Subscribe to topic
       entity.node.create_subscription(
           UpdateParamEvent,
           event_topic,
           callback=on_param_changed,
           qos_profile=10
       )

Get Event Topic (ROS2 Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Query event topic via ROS2 service
   request = type('obj', (object,), {})()
   response = type('obj', (object,), {})()
   
   await entity.param_manager.param_changed_topic(request, response)
   event_topic = response.topic
   
   # Subscribe to the topic (via ROS2)
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
   
      max_speed = await entity.parameter.get_parameter(...)
      
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