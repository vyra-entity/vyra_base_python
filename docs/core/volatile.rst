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
   
   # Volatile-Access (all methods are internal - no ROS2 services)
   volatile = entity.volatile

.. note::
   **Important:** Unlike Parameters, Volatile methods do **not** have a separate
   ROS2 service interface. All volatile operations (``set_volatile_value``,
   ``get_volatile_value``, etc.) are **internal methods** designed for use
   within your module's Python code.
   
   However, you can **publish volatile changes to ROS2 topics** using
   ``subscribe_to_changes()`` - see :ref:`ros2-topic-mapping` for details.

Set value
------------

All volatile operations are **internal methods** - they are called directly
from your Python code without going through ROS2 services:

.. code-block:: python

   # Set value (internal method)
   await entity.volatile.set_volatile_value("temperature", 23.5)
   
   # Complex data (automatically serialized)
   await entity.volatile.set_volatile_value("sensor_data", {
       "temperature": 23.5,
       "humidity": 60.2,
       "timestamp": "2026-01-19T10:30:00"
   })
   
   # Lists/Arrays
   await entity.volatile.set_volatile_value("measurements", [1, 2, 3, 4, 5])

All read operations are **internal methods**:

.. code-block:: python

   # Read single value (internal method)
   temperature = await entity.volatile.get_volatile_value("temperature")
   print(f"Current temperature: {temperature}°C")
   
   # Complex data
   sensor_data = await entity.volatile.get_volatile_value("sensor_data")
   print(f"Temperature: {sensor_data['temperature']}")
   
   # Read all existing keys (internal method)ty.volatile.get_volatile_value("temperature")
   print(f"Current temperature: {temperature}°C")
   
   # Complex data
   sensor_data = await entity.volatile.get_volatile_value("sensor_data")
   print(f"Temperature: {sensor_data['temperature']}")
   
   # Read all existing keys
   all_keys = await entity.volatile.read_all_volatile_names()
   for key in all_keys:
       value = await entity.volatile.get_volatile_value(key)
.. _ros2-topic-mapping:

Change-Events & ROS2 Topic Mapping
-----------------------------------

While volatile methods themselves are internal, you can **publish volatile changes
to ROS2 topics** for inter-module communication. This allows other modules to
.. important::
   **Internal vs. External:**
    (Internal)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, create a volatile parameter with an initial value using the **internal method**:

.. code-block:: python

   # Create volatile with initial value (internal method)

Monitor changes to volatiles in real-time and publish them automatically to ROS2 topics.

**Complete workflow to map a volatile to a ROS2 topic:**

Step 1: Create the Volatile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^ (Setup ROS2 Publishing)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Subscribe to change notifications for the volatile. This creates a ROS2 speaker
and sets up Redis key-space notifications:

.. code-block:: python

   # Subscribe to changes -> creates ROS2 topic (internal method
Step 2: Subscribe to Changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Subscribe to change notifications for the volatile. This creates a ROS2 speaker
and sets up Redis key-space notifications:
 (Internal)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Activate the Redis pub/sub listener to receive change notifications:

.. code-block:: python

   # Activate the listener to receive notifications (internal method)

Step 3: Activate the Listener
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Activate the Redis pub/sub listener to receive change notifications:

.. code-block:: python

   # Activate the listener to receive notifications
   await entity.volatile.activate_listener("temperature")

Step 4: Changes Automatically Published (Internal → External)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now, whenever the volatile value changes internally, it's automatically published
to the ROS2 topic for external subscribers:

.. code-block:: python

   # Any update now triggers ROS2 topic publication (internal method)
   await entity.volatile.set_volatile_value("temperature", 24.1)
   # -> Automatically published to /module_name/volatile/temperature!
   
   # You update internally, other modules receive via ROS2 topic subscription

Complete Example
^^^^^^^^^^^^^^^^

.. code-block:: python

   async def setup_volatile_ros2_mapping(entity: VyraEntity):
       \"\"\"Complete example: Map volatile to ROS2 topic\"\"\"
       
       # Step 1: Create volatile
       await entity.volatile.set_volatile_value("sensor_data", {
           "temperature": 23.5,
           "humidity": 60.2,
           "timestamp": "2026-01-21T10:30:00"
       })
       
       # Step 2: Subscribe to changes
       await entity.volatile.subscribe_to_changes("sensor_data")
       
       # Step 3: Activate listener
       await entity.volatile.activate_listener("sensor_data")
       
       # Step 4: Updates are now published to ROS2
       while True:
           # Read sensor (simulated)
           new_data = read_sensor()
           
           # Update volatile -> triggers ROS2 publication
           await entity.volatile.set_volatile_value("sensor_data", new_data)
           
           await asyncio.sleep(1)  # Update every second

Unsubscribe from Changes
^^^^^^^^^^^^^^^^^^^^^^^^^

To stop monitoring and remove the ROS2 topic:

.. code-block:: python

   # Stop publishing changes to ROS2
   await entity.volatile.unsubscribe_from_changes("temperature")

Channel-Listener
----------------

For advanced Pub/Sub communication:

.. code-block:: python

   # Listener for a channel activate
   await entity.volatile.activate_listener("sensor_channel")
   
   # Now all messages on 'sensor_channel' are received

Practical Examples
------------------

Sensor Data with ROS2 Publishing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   async def sensor_monitoring_example(entity: VyraEntity):
       \"\"\"Monitor sensor data and publish changes to ROS2 topics\"\"\"
       
       # Setup: Create volatiles and subscribe to changes
       await entity.volatile.set_volatile_value("temperature", 20.0)
       await entity.volatile.set_volatile_value("humidity", 50.0)
       
       await entity.volatile.subscribe_to_changes("temperature")
       await entity.volatile.subscribe_to_changes("humidity")
       await entity.volatile.activate_listener("temperature")
       
       # Monitor and publish sensor data
       while True:
           # Read data from sensor (simulated)
           current_temp = read_sensor_temperature()
           current_humidity = read_sensor_humidity()
           
           # Update volatiles -> automatically published to ROS2 topics
           await entity.volatile.set_volatile_value("temperature", current_temp)
           await entity.volatile.set_volatile_value("humidity", current_humidity)
           
           # React to critical values
           if current_temp > 30:
               await entity.news_feeder.feed("⚠️ Temperature critical!")
           
           await asyncio.slvia ROS2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Module A: Publish status via volatile -> ROS2 topic
   async def module_a_publisher(entity_a: VyraEntity):
       # Setup volatile with ROS2 publishing
       await entity_a.volatile.set_volatile_value("module_a_status", "idle")
       await entity_a.volatile.subscribe_to_changes("module_a_status")
       await entity_a.volatile.activate_listener("module_a_status")
       
       # Status updates are now published to ROS2
       await entity_a.volatile.set_volatile_value("module_a_status", "running")
       await entity_a.volatile.set_volatile_value("module_a_status", "completed")
   
   # Module B: Subscribe to Module A's ROS2 topic
   async def module_b_subscriber(entity_b: VyraNode):
       # Subscribe to the ROS2 topic created by Module A
       # Topic name: /module_a/volatile/module_a_status
       
       def on_status_received(msg):
           print(f"Module A status: {msg}")
       
       entity_b.create_subscription(
           msg_type=String,  # or appropriate type
           topic='/module_a/volatile/module_a_status',
           callback=on_status_received,
           qos_profile=10
       volatile_value("shared_status")
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
* EveSubscribe to changes for values that other modules need
   await volatile.subscribe_to_changes("robot_pose")
   await volatile.activate_listener("robot_pose")
   
   # Always create the volatile before subscribing
   await volatile.set_volatile_value("status", "idle")  # First!
   await volatile.subscribe_to_changes("status")        # Then!

❌ **Avoid:**

.. code-block:: python

   # Don't subscribe before creating the volatile
   await volatile.subscribe_to_changes("status")  # ❌ KeyError!
   await volatile.set_volatile_value("status", "idle")
   
   # Very long keys
   await volatile.set_volatile_value("this_is_a_very_long_key_name...", value)
   
   # Very large data structures (> 1 MB)
   await volatile.set_volatile_value("huge_data", very_large_object)
   
   # High-frequency subscriptions without need
   for i in range(10000):
       await volatile.subscribe_to_changess
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