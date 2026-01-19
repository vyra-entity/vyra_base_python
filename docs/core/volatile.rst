Volatile - Volatile real-time data
===================================

The :class:`~vyra_base.core.volatile.Volatile`-class manages volatile data via a 
Redis backend., which must be read and written quickly - ideal for real-time applications.

Concept
-------

Volatiles are **temporary, fast data stores** that:

* Are stored in Redis (In-Memory Database)
* Provide very fast access (~0.1ms)
* Are lost on restart (volatile)
* Support change events
* Use Pub/Sub mechanisms

When to use Volatiles?
----------------------

✅ **Ideal for:**

* Real-time sensor data
* Status variables
* Temporary calculation results
* Event-based communication between modules
* Caching of data

❌ **Not suitable for:**

* Persistent configuration (use you :doc:`parameter`)
* Data that must survive restarts
* Very large data volumes (> GB)
Access via Entity
-------------------

.. code-block:: python

   from vyra_base.core.entity import VyraEntity
   
   entity = VyraEntity(...)
   
   # Volatile-Access
   volatile = entity.volatile

Set value
------------

.. code-block:: python

   # Value set
   await entity.volatile.set_volatile_value("temperature", 23.5)
   
   # Complex data (are automatically serialized)
   await entity.volatile.set_volatile_value("sensor_data", {
       "temperature": 23.5,
       "humidity": 60.2,
       "timestamp": "2026-01-19T10:30:00"
   })
   
   # Lists/Arrays
   await entity.volatile.set_volatile_value("measurements", [1, 2, 3, 4, 5])

Read values
-----------

.. code-block:: python

   # Read single value
   temperature = await entity.volatile.get_volatile_value("temperature")
   print(f"Current temperature: {temperature}°C")
   
   # Complex data
   sensor_data = await entity.volatile.get_volatile_value("sensor_data")
   print(f"Temperature: {sensor_data['temperature']}")
   
   # Read all existing keys
   all_keys = await entity.volatile.read_all_volatile_names()
   for key in all_keys:
       value = await entity.volatile.get_volatile_value(key)
       print(f"{key}: {value}")

Change-Events
-------------

Monitor your changes to volatiles in real-time:

Event register
^^^^^^^^^^^^^^

.. code-block:: python

   # Change-Event for a key activate
   await entity.volatile.add_change_event("temperature")

Event-Callback define
^^^^^^^^^^^^^^^^^^^^^
.. code-block:: python

   async def on_temperature_change(message: dict, callback_context):
       """Called when 'temperature' changes"""
       new_value = message.get("value")
       print(f"Temperature changed: {new_value}°C")
       
       # Reaction to change
       if new_value > 30:
           print("⚠️ Temperature too high!")
   
   # Callback register
   await entity.volatile.transient_event_callback(
       message={"key": "temperature"},
       callback_context=on_temperature_change
   )

Event remove
^^^^^^^^^^^^

.. code-block:: python

   # Change-Event deactivate
   await entity.volatile.remove_change_event("temperature")

Channel-Listener
----------------

For advanced Pub/Sub communication:

.. code-block:: python

   # Listener for a channel activate
   await entity.volatile.activate_listener("sensor_channel")
   
   # Now all messages on 'sensor_channel' are received

Practical Example
----------------

Sensor Data with Event Reaction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   async def sensor_monitoring_example(entity: VyraEntity):
       """Monitor sensor data and react to changes"""
       
       # Activate change events
       await entity.volatile.add_change_event("temperature")
       await entity.volatile.add_change_event("humidity")
       
       # Callback for temperature changes
       async def on_temp_change(message: dict, context):
           temp = await entity.volatile.get_volatile_value("temperature")
           if temp > 30:
               # Publish warning
               entity.publish_error("Temperature critical!")
       
       # Update sensor data (e.g., in a loop)
       while True:
           # Read data from sensor (simulated)
           current_temp = read_sensor_temperature()
           current_humidity = read_sensor_humidity()
           
           # Store in volatiles (triggers change events)
           await entity.volatile.set_volatile_value("temperature", current_temp)
           await entity.volatile.set_volatile_value("humidity", current_humidity)
           
           await asyncio.sleep(1)  # Update every second
Inter-Module Communication (IMC)
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Module A: Provide data
   await entity_a.volatile.set_volatile_value("shared_status", "running")
   
   # Module B: Read data
   status = await entity_b.volatile.get_volatile_value("shared_status")
   print(f"Status of Module A: {status}")
.. note::
   Both modules must use the same Redis server (default in the VYRA system).

Redis-Backend
-------------

**Connection**: Automatic via ``entity.storage.redis_client``

**Storage Location**: In-Memory (RAM of the Redis server)

**Persistence**: None - Data is lost on Redis restart
.. tip::
   Redis can optionally be configured to create snapshots.
   This is done via the Redis configuration, not in VYRA itself.
   Further information: https://redis.io/docs/management/persistence/

Performance
-----------

**Typical access times:**

* Writing: ~0.1 - 0.5 ms
* Reading: ~0.1 - 0.3 ms
* Event Trigger: ~1 - 5 ms
.. important::
   Volatiles are **100x faster** than Parameters (SQLite).
   Use your Volatiles for high-frequency data operations!

Best Practices
--------------

✅ **Recommended:**

.. code-block:: python

   # Short, meaningful keys
   await volatile.set_volatile_value("temp_sensor_1", 23.5)
   
   # Structured data for related values
   await volatile.set_volatile_value("robot_pose", {
       "x": 1.5, "y": 2.3, "theta": 0.78
   })
   
   # Change-Events for critical values
   await volatile.add_change_event("emergency_stop")

❌ **Avoid:**

.. code-block:: python

   # Very long keys
   await volatile.set_volatile_value("this_is_a_very_long_key_name...", value)
   
   # Very large data structures (> 1 MB)
   await volatile.set_volatile_value("huge_data", very_large_object)
   
   # High-frequency Events without need
   for i in range(10000):
       await volatile.add_change_event(f"sensor_{i}")

Error Handling
----------------

.. code-block:: python

   try:
       value = await entity.volatile.get_volatile_value("nonexistent_key")
   except KeyError:
       print("Key does not exist")
       # Set default value
       await entity.volatile.set_volatile_value("nonexistent_key", 0)
   except Exception as e:
       print(f"Redis connection error: {e}")

Further Information
-----------------------------

* :doc:`entity` - Entity Documentation
* :doc:`parameter` - Persistent Alternative
* :doc:`../storage` - Storage-Backend Details
* :class:`~vyra_base.core.volatile.Volatile` - API Reference
* Redis Documentation: https://redis.io/docs/