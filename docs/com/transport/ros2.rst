# vyra_base.com.transport.ros2

ROS2 Transport Module
=====================

Provides ROS2/DDS-based transport implementation with layered architecture:

Layers:
    - communication/: Core ROS2 functionality (Services, Topics, Actions)
    - vyra_models/: VYRA abstractions (ROS2Callable, ROS2Speaker, ROS2Job)
    - node.py: ROS2 node management and lifecycle
    - provider.py: Interface layer for VYRA integration

**Features:**
- Service-based request-response (ROS2Callable)
- Topic-based Pub/Sub (ROS2Speaker)
- Action-based long-running tasks (ROS2Job)
- Type conversion utilities
- Node lifecycle management

**Usage:**

.. code-block:: python

    from vyra_base.com.transport.ros2 import ROS2Provider, ROS2_AVAILABLE
    
    if ROS2_AVAILABLE:
        provider = ROS2Provider(node_name="my_node")
        await provider.initialize()
        callable = await provider.create_callable("/service", callback)
        speaker = await provider.create_speaker("/topic")
