Feeders - Automatic Data Publication
========================================

Feeders are special publishers that automatically publish data on ROS2 topics.
VYRA provides pre-built feeders for States, News, and Errors.

Concept
-------

Feeders automate the publication of standardized data:

* **StateFeeder**: Publishes state machine state changes
* **NewsFeeder**: Publishes informative messages (logs, events)
* **ErrorFeeder**: Publishes error messages and warnings

**Advantages:**

* ✅ Automatic topic management
* ✅ Consistent message format
* ✅ Integrated with Entity and State Machine
* ✅ No manual topic setup required

StateFeeder
-----------

The StateFeeder automatically publishes state machine state changes:

Usage
^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.feeder.state_feeder import StateFeeder
   from vyra_base.defaults.entries import StateEntry
   
   # Configure StateEntry
   state_entry = StateEntry(
       namespace="/my_module",
       topic_name="state",
       message_type="vyra_interfaces/msg/State"
   )
   
   # Create StateFeeder (typically in Entity)
   state_feeder = StateFeeder(
       node=entity.node,
       state_entry=state_entry
   )
   state_feeder.create()
   
   # Publish state (automatically via State Machine)
   entity.publish_state()

Integration with State Machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The State Machine uses the StateFeeder automatically:

.. code-block:: python

   # On state transition
   await state_machine.trigger_event(StateEvent.INITIALIZE)
   # ➡️ StateFeeder publishes new state automatically

**Published Information:**

* Lifecycle State (IDLE, READY, RUNNING, etc.)
* Operational State (IDLE, INITIALIZING, OPERATIONAL, etc.)
* Health State (HEALTHY, DEGRADED, UNHEALTHY, etc.)
* Timestamp
* Module-Namespace

NewsFeeder
----------

The NewsFeeder publishes informative messages and events:

Usage
^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.feeder.news_feeder import NewsFeeder
   from vyra_base.defaults.entries import NewsEntry
   
   # Configure NewsEntry
   news_entry = NewsEntry(
       namespace="/my_module",
       topic_name="news",
       message_type="vyra_interfaces/msg/News"
   )
   
   # Create NewsFeeder
   news_feeder = NewsFeeder(
       node=entity.node,
       news_entry=news_entry
   )
   news_feeder.create()

Publish Messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Via Entity (recommended)
   entity.publish_news("Module initialized successfuly")
   entity.publish_news("Sensor calibration completed")
   entity.publish_news("Connection to external system established")
   
   # Directly via Feeder
   from vyra_base.defaults.entries import NewsEntry
   news_msg = NewsEntry(
       namespace="/my_module",
       message="Operation successful",
       timestamp=datetime.now()
   )
   news_feeder.feed(news_msg)

**Use Cases:**

* Status messages for monitoing
* Important events (Start, Stop, Calibration)
* Progress reports for long-running operations
* Informative logs for other modules

ErrorFeeder
-----------

The ErrorFeeder publishes errors and warnings:

Usage
^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.feeder.error_feeder import ErrorFeeder
   from vyra_base.defaults.entries import ErrorEntry
   
   # Configure ErrorEntry
   error_entry = ErrorEntry(
       namespace="/my_module",
       topic_name="error",
       message_type="vyra_interfaces/msg/Error"
   )
   
   # Create ErrorFeeder
   error_feeder = ErrorFeeder(
       node=entity.node,
       error_entry=error_entry
   )
   error_feeder.create()

Publish Errors
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Via Entity (recommended)
   entity.publish_error("Connection to Sensor lost")
   entity.publish_error("Timeout on service call")
   entity.publish_error("Configuration file not foand")
   
   # With Severity-Level (if supported)
   error_msg = ErrorEntry(
       namespace="/my_module",
       message="Critical error",
       severity="ERROR",  # INFO, WARNING, ERROR, CRITICAL
       timestamp=datetime.now()
   )
   error_feeder.feed(error_msg)

**Use Cases:**

* Error notifications for monitoing systems
* Warning messages for critical states
* Exception logging via ROS2
* Central error collection via topics

Practical Examples
--------------------

Monitoing Dashboard
^^^^^^^^^^^^^^^^^^^^

A central dashboard can subscribe to all feeder topics:

.. code-block:: python

   # Dashboard Module (Subscriber)
   from vyra_base.com.datalayer.subscriber import VyraSubscriber
   
   class MonitoingDashboard:
       def __init__(self, entity):
           self.entity = entity
           
           # State Listener
           self.state_listener = VyraSubscriber(
               node=entity.node,
               topic_name="/+/state",  # All modules
               topic_type=State,
               callback=self.on_state_update
           )
           
           # News-Listener
           self.news_listener = VyraSubscriber(
               node=entity.node,
               topic_name="/+/news",
               topic_type=News,
               callback=self.on_news_received
           )
           
           # Error-Listener
           self.error_listener = VyraSubscriber(
               node=entity.node,
               topic_name="/+/error",
               topic_type=Error,
               callback=self.on_error_received
           )
       
       def on_state_update(self, msg):
           module = msg.namespace
           state = msg.lifecycle_state
           print(f"[STATE] {module}: {state}")
       
       def on_news_received(self, msg):
           print(f"[NEWS] {msg.namespace}: {msg.message}")
       
       def on_error_received(self, msg):
           print(f"[ERROR] {msg.namespace}: {msg.message}")

Automatic Error Logging
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Feeders can be integrated with the standard logger:

.. code-block:: python

   import logging
   
   class ErrorLoggingHandler(logging.Handler):
       def __init__(self, entity):
           super().__init__()
           self.entity = entity
       
       def ewith(self, record):
           if record.levelno >= logging.ERROR:
               # Publish error via ErrorFeeder
               self.entity.publish_error(
                   f"{record.name}: {record.getMessage()}"
               )
   
   # Add handler to logger
   logger = logging.getLogger()
   logger.addHandler(ErrorLoggingHandler(entity))
   
   # Now all errors are automatically published
   logger.error("This appears on /my_module/error topic")

Custom Feeder
-------------

You can also create your own feeders:

.. code-block:: python

   from vyra_base.com.feeder.feeder import BaseFeeder
   
   class MetricsFeeder(BaseFeeder):
       """Feeder for performance metrics"""
       
       def __init__(self, node, topic_name="metrics"):
           super().__init__(
               node=node,
               topic_name=topic_name,
               topic_type=Metrics  # Your message type
           )
       
       def feed(self, metrics_data):
           """Publishes metric data"""
           msg = Metrics()
           msg.cpu_usage = metrics_data["cpu"]
           msg.memory_usage = metrics_data["memory"]
           msg.timestamp = self.get_timestamp()
           self.publisher.publish(msg)
   
   # Usage
   metrics_feeder = MetricsFeeder(node=entity.node)
   metrics_feeder.create()
   
   # Publish metrics regularly
   async def publish_metrics_loop():
       while True:
           metrics = get_system_metrics()
           metrics_feeder.feed(metrics)
           await asyncio.sleep(5.0)

Best Practices
--------------

✅ **Recommended:**

* Use feeders for standardized data (State, News, Error)
* Integrate feeders in Entity initialization
* Use ``entity.publish_*()`` for easy access
* Subscribe to feeder topics for central monitoing
* Use meaningful messages

❌ **Avoid:**

* High-frequency publication (> 10 Hz, use normal topics)
* Very large messages via feeders
* Sensitive data without encryption
* Feeders for binary data (use normal publishers)

Performance Notes
--------------------

**Typical Publication Rates:**

* StateFeeder: On state change (~0.1 - 1 Hz)
* NewsFeeder: On events (~0.01 - 0.1 Hz)
* ErrorFeeder: On errors (~0.001 - 0.01 Hz)

.. tip::
   Feeders are not optimized for high-frequency data.
   For Sensor data with > 10 Hz, use normal ROS2 publishers.

Further Information
-----------------------------

* :doc:`ros2_communication` - ROS2 Publisher/Subscriber Details
* :doc:`../vyra_base.com.feeder` - API Reference
* :class:`~vyra_base.com.feeder.feeder.BaseFeeder` - Base Class
* :class:`~vyra_base.com.feeder.state_feeder.StateFeeder` - State Feeder
* :class:`~vyra_base.com.feeder.news_feeder.NewsFeeder` - News Feeder
* :class:`~vyra_base.com.feeder.error_feeder.ErrorFeeder` - Error Feeder
