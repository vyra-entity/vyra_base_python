"""
Example: Using VyraSpeaker as Listener (Subscriber)

This example demonstrates how to create and use a VyraSpeaker
with is_publisher=False to subscribe to ROS2 topics and receive messages.

Run example_speaker.py in another terminal to publish messages to this listener,
or use: ros2 topic pub /vyra/example_topic std_msgs/msg/String "{data: 'Hello'}"

Usage:
  python examples/example_listener.py
"""

import asyncio
import rclpy
from std_msgs.msg import String
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider
from vyra_base.com.transport.t_ros2.node import VyraNode, NodeSettings


async def main():
    """Main function demonstrating VyraSpeaker as subscriber/listener."""
    rclpy.init()

    node_settings = NodeSettings(name="listener_example_node")
    node = VyraNode(node_settings=node_settings)

    provider = ROS2Provider(
        protocol=ProtocolType.ROS2,
        node_name="listener_example_provider"
    )
    await provider.initialize(config={
        "node_name": "vyra_listener_example",
        "namespace": "/vyra"
    })

    # Create a Listener (Subscriber) – same topic as example_speaker
    listener = await provider.create_speaker(
        name="example_topic",
        message_type=String,
        is_publisher=False,  # Subscriber
        description="Example listener for string messages",
        ident_name="example_listener",
        qos_profile=10
    )

    received_count = [0]  # use list so callback can modify

    def on_message(msg):
        received_count[0] += 1
        print(f"Received #{received_count[0]}: {msg.data}")

    # Register callback and ensure subscription is created
    await listener.listen(on_message)
    sub = listener.get_subscriber()
    if sub is not None and sub._subscription_info.subscription is None:
        sub._subscription_info.callback = on_message
        sub.create_subscription()

    print("Listener created! Waiting for messages on /vyra/example_topic ...")
    print("Run example_speaker.py in another terminal or:")
    print("  ros2 topic pub /vyra/example_topic std_msgs/msg/String \"{data: 'Hello'}\"")
    print("(Ctrl+C to stop)\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")

    await listener.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("Listener example completed!")


if __name__ == "__main__":
    asyncio.run(main())
