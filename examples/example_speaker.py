"""
Example: Using VyraSpeaker (Publisher)

This example demonstrates how to create and use a VyraSpeaker
to publish messages on ROS2 topics.
"""

import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.ros2 import ROS2Provider
from vyra_base.com.transport.ros2.node import VyraNode, NodeSettings

async def main():
    """Main function demonstrating VyraSpeaker usage."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create a VyraNode
    node_settings = NodeSettings(name="speaker_example_node")
    node = VyraNode(node_settings=node_settings)
    
    provider = ROS2Provider(
        protocol=ProtocolType.ROS2,
        node_name="speaker_example_provider"
    )

    await provider.initialize(config={
        "node_name": "vyra_speaker_example",
        "namespace": "/vyra"
    })

    # Create a Speaker (Publisher)
    speaker = await provider.create_speaker(
        name="example_topic",
        message_type=String,
        is_publisher=True,
        description="Example speaker for publishing string messages",
        ident_name="example_speaker",
        qos_profile=10  # Queue size
    )
    
    print("Speaker created! Publishing messages...")
    
    # Publish messages
    for i in range(10):
        msg = String()
        msg.data = f"Hello from VyraSpeaker! Message #{i}"
        
        # Use 'shout' method to publish
        await speaker.shout(msg)
        print(f"Published: {msg.data}")
        
        # Sleep for 1 second
        rclpy.spin_once(node, timeout_sec=1.0)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    print("Speaker example completed!")


if __name__ == '__main__':
    asyncio.run(main())