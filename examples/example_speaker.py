"""
Example: Using VyraSpeaker (Publisher)

This example demonstrates how to create and use a VyraSpeaker
to publish messages on ROS2 topics using ROS2Provider.
"""

import asyncio
import rclpy
from std_msgs.msg import String
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider

async def main():
    """Main function demonstrating VyraSpeaker usage."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 provider with module info
    provider = ROS2Provider(
        module_name="example_speaker",
        module_id="demo123",
        protocol=ProtocolType.ROS2
    )
    
    # Initialize provider with config
    await provider.initialize(config={
        "node_name": "speaker_example_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")

    # Create a Speaker (Publisher)
    speaker = await provider.create_speaker(
        name="example_topic",
        message_type=String,
        is_publisher=True,
        qos_profile=10  # Queue size
    )
    
    print("✅ Speaker created! Publishing messages...")
    print("\nListen with:")
    print("  ros2 topic echo /examples/example_speaker_demo123/example_topic")
    print("\n(Press Ctrl+C to stop)\n")
    
    # Publish messages
    try:
        for i in range(10):
            msg = String()
            msg.data = f"Hello from VyraSpeaker! Message #{i}"
            
            # Use 'shout' method to publish
            await speaker.shout(msg)
            print(f"📤 Published: {msg.data}")
            
            # Sleep for 1 second
            await asyncio.sleep(1.0)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    
    # Cleanup
    await speaker.shutdown()
    await provider.shutdown()
    print("✅ Speaker example completed!")


if __name__ == '__main__':
    asyncio.run(main())