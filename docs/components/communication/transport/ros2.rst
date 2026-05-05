ROS2 Transport Module
=====================

Provides ROS2/DDS-based transport implementation with layered architecture.

Overview
--------

The ROS2 transport module implements the ``AbstractProtocolProvider`` interface
for Robot Operating System 2 (ROS2) communication using DDS middleware.

**Architecture Layers:**

- **communication/**: Core ROS2functionality (Services, Topics, Actions)
- **vyra_models/**: VYRA abstractions (ROS2Callable, ROS2Speaker, ROS2Job)
- **node.py**: ROS2 node management and lifecycle
- **provider.py**: Protocol provider implementation

**Features:**

- ✅ Service-based request-response (ROS2Callable)
- ✅ Topic-based Pub/Sub (ROS2Speaker)
- ✅ Action-based long-running tasks (ROS2Job)
- ✅ Server/Client pattern with ``is_callable`` and ``is_job`` flags
- ✅ Type conversion utilities
- ✅ Node lifecycle management
- ✅ QoS policy configuration
- ✅ SROS2 security support
- ✅ Dynamic interface loading

Usage
-----

Basic Provider Setup
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from vyra_base.com.transport.t_ros2 import ROS2Provider, ROS2_AVAILABLE
   from vyra_base.com.core.types import ProtocolType
   
   if ROS2_AVAILABLE:
       # Create provider
       provider = ROS2Provider(
           module_name="my_module",
           module_id="abc123",
           protocol=ProtocolType.ROS2
       )
       
       # Initialize with config
       await provider.initialize(config={
           "node_name": "my_node",
           "namespace": "/my_namespace"
       })

Create Service Server
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from example_interfaces.srv import AddTwoInts
   
   # Server callback
   async def handle_request(request, response):
       response.sum = request.a + request.b
       return response
   
   # Create service server (is_callable=True)
   server = await provider.create_callable(
       name="add_service",
       callback=handle_request,
       service_type=AddTwoInts
   )

Create Service Client
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Create service client (is_callable=False)
   client = await provider.create_callable(
       name="add_service",
       callback=None,  # No callback for client
       service_type=AddTwoInts,
       is_callable=False
   )
   
   # Call service
   request = AddTwoInts.Request(a=5, b=3)
   response = await client.call(request, timeout=5.0)

Create Publisher
^^^^^^^^^^^^^^^^

.. code-block:: python

   from std_msgs.msg import String
   
   # Create publisher (is_publisher=True)
   publisher = await provider.create_speaker(
       name="my_topic",
       message_type=String,
       is_publisher=True,
       qos_profile=10
   )
   
   # Publish message
   await publisher.shout(String(data="Hello"))

Create Subscriber
^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Subscriber callback
   def on_message(msg):
       print(f"Received: {msg.data}")
   
   # Create subscriber (is_publisher=False)
   subscriber = await provider.create_speaker(
       name="my_topic",
       message_type=String,
       is_publisher=False,
       qos_profile=10
   )
   
   # Start listening
   await subscriber.listen(on_message)

Create Action Server
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from example_interfaces.action import Fibonacci
   
   # Action server callback
   async def execute_fibonacci(goal, feedback_callback=None):
       sequence = [0, 1]
       for i in range(1, goal.order):
           sequence.append(sequence[-1] + sequence[-2])
           if feedback_callback:
               await feedback_callback({"sequence": sequence})
       return {"sequence": sequence}
   
   # Create action server (is_job=True)
   action_server = await provider.create_job(
       name="fibonacci",
       callback=execute_fibonacci,
       action_type=Fibonacci
   )

Create Action Client
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Feedback callback
   def on_feedback(feedback):
       print(f"Progress: {feedback['sequence']}")
   
   # Create action client (is_job=False)
   action_client = await provider.create_job(
       name="fibonacci",
       callback=None,  # No callback for client
       action_type=Fibonacci,
       is_job=False,
       feedback_callback=on_feedback
   )
   
   # Send goal
   goal = Fibonacci.Goal(order=10)
   result = await action_client.execute(goal)

Configuration
-------------

Node Configuration
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   config = {
       "node_name": "my_node",
       "namespace": "/my_namespace",
       "use_sim_time": False,
       "enable_rosout": True,
       "parameter_overrides": {
           "param1": "value1",
           "param2": 42
       }
   }
   
   await provider.initialize(config=config)

QoS Profiles
^^^^^^^^^^^^

.. code-block:: python

   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
   
   # Custom QoS
   qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=10
   )
   
   speaker = await provider.create_speaker(
       name="reliable_topic",
       message_type=String,
       qos_profile=qos
   )

Server/Client Flags
^^^^^^^^^^^^^^^^^^^

The provider automatically detects server/client mode based on callback presence,
but you can explicitly set the flags:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Interface Type
     - Flag
     - Behavior
   * - Service Server
     - ``is_callable=True``
     - Responds to service calls (callback required)
   * - Service Client
     - ``is_callable=False``
     - Makes service calls (no callback)
   * - Publisher
     - ``is_publisher=True``
     - Publishes messages
   * - Subscriber
     - ``is_publisher=False``
     - Receives messages (callback required)
   * - Action Server
     - ``is_job=True``
     - Executes actions (callback required)
   * - Action Client
     - ``is_job=False``
     - Sends goals (no callback)

API Reference
-------------

.. automodule:: vyra_base.com.transport.t_ros2.provider
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: vyra_base.com.transport.t_ros2.vyra_models
   :members:
   :undoc-members:
   :show-inheritance:

Dependencies
------------

- ``rclpy`` (ROS2 Python client library)
- ``rclpy.node`` (Node management)
- ``rclpy.qos`` (Quality of Service)
- ``rclpy.action`` (Action interfaces)

See Also
--------

- :doc:`../core/factory` - InterfaceFactory for protocol-agnostic usage
- :doc:`redis` - Redis transport provider
- :doc:`uds` - Unix Domain Socket transport provider
