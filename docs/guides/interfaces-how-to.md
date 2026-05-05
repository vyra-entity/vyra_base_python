# Interfaces How-To Guide

## What are Interfaces?

Interfaces are the communication contracts between ROS2 nodes in VYRA.
They define what messages can be sent and received.

## Basic Types

### Service

Request-reply pattern:

```python
interface = ServiceInterface(
    service_name="my_service",
    request_type=MyRequest,
    response_type=MyResponse
)
```

### Topic Publisher

One-way publish:

```python
interface = TopicPublisher(
    topic_name="my_topic",
    message_type=MyMessage
)
```

### Topic Subscriber

Listen for messages:

```python
interface = TopicSubscriber(
    topic_name="my_topic",
    message_type=MyMessage,
    callback=my_handler
)
```

## Registration

```python
entity = VyraEntity("module_name")
entity.register_interface(interface)
await entity.set_interfaces(transport)
```

## Example

```python
from vyra_base import VyraEntity
from std_msgs.msg import String

entity = VyraEntity("publisher")

# Create publisher interface
pub_interface = TopicPublisher(
    topic_name="messages",
    message_type=String
)

entity.register_interface(pub_interface)
pub_interface.publish(String(data="Hello"))
```

## Debugging

```bash
# List all interfaces
ros2 interface list

# Check interface definition  
ros2 interface show <interface_name>

# Monitor topic traffic
ros2 topic echo <topic_name>
```

---

See [Dynamic Interface Loading](../guides/dynamic-interface-loading.md) for advanced topics.
