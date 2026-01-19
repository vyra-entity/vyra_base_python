ROS2 Communication
==================

VYRA uses ROS2 for inter-module communication. The framework abstracts the ROS2 API
and provides simple interfaces for Services, Topics, and Actions.

Concepts
--------

ROS2 has three main communication patterns:

1. **Services (Request/Response)**:
   
   * **Job** = Service Client (calls service)
   * **Callable** = Service Server (provides service)
   * Synchronous request-response communication

2. **Topics (Publish/Subscribe)**:
   
   * **Speaker** = Publisher (publishes)
   * **Listener** = Subscriber (receives)
   * Asynchronous broadcast communication

3. **Actions (Long-running operations)**:
   
   * **Action Client** (starts action)
   * **Action Server** (executes action)
   * With feedback and cancel capability

Services: Job & Callable
-------------------------

Job - Service Client
^^^^^^^^^^^^^^^^^^^^

A **Job** calls services on other modules:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_job
   from example_interfaces.srv import AddTwoInts
   
   # Create job
   job = create_vyra_job(
       node=entity.node,
       service_name="/calculato/add_two_ints",
       service_type=AddTwoInts
   )
   
   # Prepare request
   request = AddTwoInts.Request()
   request.a = 5
   request.b = 7
   
   # Call service asynchronously
   response = await job.call_async(request)
   print(f"Result: {response.sum}")

**Parameters:**

* ``node``: The ROS2 node (typically ``entity.node``)
* ``service_name``: Service name (with namespace)
* ``service_type``: ROS2 service type (imported from ``*_interfaces.srv``)

Callable - Service Server
^^^^^^^^^^^^^^^^^^^^^^^^^^

A **Callable** provides a service for other modules.
Use the ``@remote_callable`` decorato:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import remote_callable
   from vyra_base.state import OperationalStateMachine
   
   class Calculato(OperationalStateMachine):
       def __init__(self, unified_state_machine):
           super().__init__(unified_state_machine)
       
       @remote_callable
       async def add_two_ints(self, request, response):
           """Adds two numbers"""
           response.sum = request.a + request.b
           return response
       
       @remote_callable
       async def get_status(self, request, response):
           """Returns the current status"""
           response.status = "operational"
           return response

**Automatic Regisration:**

Methods with ``@remote_callable`` are automatically registered as ROS2 services,
when the JSON metadata is correctly defined (see :doc:`../interfaces`).

**Parameters:**

* ``request``: Incoming service request (defined by service type)
* ``response``: Outgoing service response (returned)

Topics: Speaker & Listener
---------------------------

Speaker - Publisher
^^^^^^^^^^^^^^^^^^^

A **Speaker** publishes messages on a topic:

.. code-block:: python

   from vyra_base.com.datalayer.interface_factoy import create_vyra_speaker
   from std_msgs.msg import String
   
   # Create speaker
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/status_updates",
       topic_type=String,
       qos_profile=10  # Queue-Size
   )
   
   # Publish message
   msg = String()
   msg.data = "System running"
   speaker.shout(msg)

**Parameters:**

* ``node``: The ROS2 node
* ``topic_name``: Topic name
* ``topic_type``: ROS2 message type (imported from ``*_interfaces.msg``)
* ``qos_profile``: Quality of Service (queue size or QoS object)

Listener - Subscriber
^^^^^^^^^^^^^^^^^^^^^^

A **Listener** receives messages from a topic:

.. code-block:: python

   from vyra_base.com.datalayer.subscriber import VyraSubscriber
   from std_msgs.msg import String
   
   # Define callback function
   def on_message_received(msg):
       print(f"Received: {msg.data}")
   
   # Create listener
   listener = VyraSubscriber(
       node=entity.node,
       topic_name="/status_updates",
       topic_type=String,
       callback=on_message_received,
       qos_profile=10
   )
   
   # Subscription create
   listener.create_subscription()

**Parameters:**

* ``callback``: Function called for each message
* Other parameters same as Speaker

Practical Examples
--------------------

Inter-Module Service Call
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Module A (Server)**:

.. code-block:: python

   class RobotController(OperationalStateMachine):
       @remote_callable
       async def move_to_position(self, request, response):
           """Moves robot to position"""
           x, y = request.target_x, request.target_y
           
           # Execute movement
           success = await self.robot.move_to(x, y)
           
           response.success = success
           response.message = "Movement complete"
           return response

**Module B (Client)**:

.. code-block:: python

   # Create job
   move_job = create_vyra_job(
       node=entity.node,
       service_name="/robot_controller/move_to_position",
       service_type=MoveToPosition
   )
   
   # Request movement
   request = MoveToPosition.Request()
   request.target_x = 10.5
   request.target_y = 5.3
   
   response = await move_job.call_async(request)
   if response.success:
       print("Robot moved successfuly")

Topic-based Status Messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Publisher (Module A)**:

.. code-block:: python

   # Speaker for status updates
   status_speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/system_status",
       topic_type=SystemStatus
   )
   
   # Publish status regularly
   async def publish_status_loop():
       while True:
           status = SystemStatus()
           status.cpu_usage = get_cpu_usage()
           status.memory_usage = get_memory_usage()
           status.timestamp = get_current_time()
           
           status_speaker.shout(status)
           await asyncio.sleep(1.0)  # Every second

**Subscriber (Module B)**:

.. code-block:: python

   # Callback for status reception
   def on_status_received(msg):
       if msg.cpu_usage > 80:
           logger.warning(f"High CPU load: {msg.cpu_usage}%")
   
   # Create listener
   status_listener = VyraSubscriber(
       node=entity.node,
       topic_name="/system_status",
       topic_type=SystemStatus,
       callback=on_status_received
   )
   status_listener.create_subscription()

Interface Definition
--------------------

ROS2 interfaces are defined via JSON metadata.

**Example** (``config/service_interfaces.json``):

.. code-block:: json

   {
       "services": [
           {
               "name": "add_two_ints",
               "type": "example_interfaces/srv/AddTwoInts",
               "description": "Adds two integers"
           },
           {
               "name": "get_status",
               "type": "std_srvs/srv/Trigger",
               "description": "Returns current status"
           }
       ]
   }

Thise metadata are loaded at entity start and automatically registered:

.. code-block:: python

   # In _base_.py
   interfaces = await entity.load_interfaces_from_config()
   await entity.set_interfaces(interfaces)

For more Details see :doc:`../interfaces`.

Quality of Service (QoS)
------------------------

ROS2 uses QoS profiles to configure communication:

.. code-block:: python

   from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
   
   # Custom QoS profile
   qos = QoSProfile(
       depth=10,  # Queue size
       reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
       durability=DurabilityPolicy.VOLATILE  # or TRANSIENT_LOCAL
   )
   
   # Use with Speaker
   speaker = create_vyra_speaker(
       node=entity.node,
       topic_name="/important_data",
       topic_type=Data,
       qos_profile=qos
   )

**Typical Profiles:**

* **Sensor Data**: ``BEST_EFFORT`` (fast, loss acceptable)
* **Commands**: ``RELIABLE`` (guaranteed delivery)
* **Persistent Data**: ``TRANSIENT_LOCAL`` (new subscribers receive last message)

Best Practices
--------------

✅ **Recommended:**

* Use meaningful service/topic names with namespace
* Define interfaces in JSON metadata
* Use ``@remote_callable`` for automatic regisration
* Implement timeouts for service calls
* Use QoS profiles appropriate for the use case

❌ **Avoid:**

* Manual ROS2 node creation (use ``entity.node``)
* Very large messages (> 1 MB) via topics
* High-frequency service calls (> 100 Hz, use topics)
* Blocking calls without timeout

Error Handling
----------------

.. code-block:: python

   from rclpy.task import Future
   
   try:
       # Service call with timeout
       response = await asyncio.wait_for(
           job.call_async(request),
           timeout=5.0  # 5 seconds
       )
   except asyncio.TimeoutError:
       logger.error("Service call timeout")
   except Exception as e:
       logger.error(f"Service error: {e}")

Further Information
-----------------------------

* :doc:`ipc_communication` - IPC within a module
* :doc:`feeders` - Automatic feeders
* :doc:`../interfaces` - Interface definitions
* :doc:`../vyra_base.com.datalayer` - API Reference
* ROS2 Documentation: https://docs.ros.org/en/kilted/
