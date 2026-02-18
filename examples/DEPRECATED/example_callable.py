"""
Example: Using VyraCallable (Service Server)

This example demonstrates how to create and use a VyraCallable
to provide ROS2 services using ROS2Provider.
"""

import asyncio
import rclpy
from example_interfaces.srv import AddTwoInts
from vyra_base.com import remote_service
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider
from vyra_base.com.transport.t_ros2.node import VyraNode, NodeSettings


# Callback function for AddTwoInts service
def add_two_ints_callback(request, response):
    """
    Callback function for AddTwoInts service.
    
    :param request: Service request containing two integers (a, b)
    :param response: Service response containing sum
    :return: Service response
    """
    response.sum = request.a + request.b
    print(f"Received request: {request.a} + {request.b} = {response.sum}")
    return response


async def main():
    """Main function demonstrating VyraCallable usage."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 provider with module info
    provider = ROS2Provider(
        module_name="example_callable",
        module_id="demo123",
        protocol=ProtocolType.ROS2
    )
    
    # Initialize provider with config
    await provider.initialize(config={
        "node_name": "callable_example_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")
    
    # Create a Callable (Service Server) with explicit callback
    callable_add = await provider.create_callable(
        name="add_service",
        service_type=AddTwoInts,
        callback=add_two_ints_callback
    )
    
    print("\n✅ Callable service 'add_service' created!")
    print("📞 Waiting for service requests...")
    print("\nTest with:")
    print("  ros2 service call /examples/example_callable_demo123/add_service example_interfaces/srv/AddTwoInts '{a: 5, b: 3}'")
    print("\n(Press Ctrl+C to stop)\n")
    
    # Spin to handle service requests
    try:
        rclpy.spin(provider.get_node())
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    
    # Cleanup
    await callable_add.shutdown()
    await provider.shutdown()
    print("✅ Callable example completed!")


if __name__ == '__main__':
    asyncio.run(main())
