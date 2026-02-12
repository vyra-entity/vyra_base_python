"""
Example: Using VyraSpeaker as Listener (Subscriber)

This example demonstrates how to create and use a VyraSpeaker
with is_publisher=False to subscribe to ROS2 topics and receive messages.

Run example_speaker.py in another terminal to publish messages to this listener,
or use: ros2 topic pub /examples/example_listener_demo123/example_topic std_msgs/msg/String "{data: 'Hello'}"

Usage:
  python examples/example_listener.py
"""

import asyncio
import rclpy
from std_msgs.msg import String
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider


async def main():
    """Main function demonstrating VyraSpeaker as subscriber/listener."""
    rclpy.init()

    # Create ROS2 provider with module info
    provider = ROS2Provider(
        module_name="example_listener",
        module_id="demo123",
        protocol=ProtocolType.ROS2
    )
    
    # Initialize provider with config
    await provider.initialize(config={
        "node_name": "listener_example_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")

    received_count = [0]  # use list so callback can modify

    def on_message(msg):
        received_count[0] += 1
        print(f"📥 Received #{received_count[0]}: {msg.data}")

    # Create a Listener (Subscriber) – same topic as example_speaker
    listener = await provider.create_speaker(
        name="example_topic",
        message_type=String,
        is_publisher=False,  # Subscriber
        qos_profile=10
    )

    # Register callback and ensure subscription is created
    await listener.listen(on_message)

    print("✅ Listener created! Waiting for messages on topic...")
    print("\nPublish with:")
    print("  ros2 topic pub /examples/example_listener_demo123/example_topic std_msgs/msg/String '{data: \"Hello\"}'")
    print("  or run example_speaker.py in another terminal")
    print("\n(Press Ctrl+C to stop)\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(provider.get_node(), timeout_sec=0.5)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")

    await listener.shutdown()
    await provider.shutdown()
    print("✅ Listener example completed!")


if __name__ == "__main__":
    asyncio.run(main())
