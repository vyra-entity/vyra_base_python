Core Communication Types
=======================

Defines base data structures for all communication protocols.
VYRA uses three communication paradigms:

- **Callable**: Request-Response (e.g. ROS2 Service, gRPC Unary)
- **Speaker**: Publish-Subscribe (e.g. ROS2 Topic, MQTT, Redis Pub/Sub)
- **Job**: Long-Running Task (e.g. ROS2 Action, async Task)

**Docstring:**

"""
Core Communication Types

Defines base data structures for all communication protocols.
VYRA uses three communication paradigms:
- Callable: Request-Response (ROS2 Service, gRPC Unary, etc.)
- Speaker: Publish-Subscribe (ROS2 Topic, MQTT, Redis Pub/Sub)
- Job: Long-Running Task (ROS2 Action, async Task)
"""

**Enums:**
- ProtocolType: Supported protocols (ROS2, SHARED_MEMORY, UDS, REDIS, MQTT, GRPC, REST, WEBSOCKET, MODBUS, OPCUA)
- InterfaceType: Interface types (CALLABLE, SPEAKER, JOB)
