"""
Example: Using VyraCallable (Service Server)

This example demonstrates how to create and use a VyraCallable
to provide ROS2 services.
"""

import asyncio
import rclpy
from example_interfaces.srv import AddTwoInts
from vyra_base.com import remote_callable
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider
from vyra_base.com.transport.t_ros2.node import VyraNode, NodeSettings


# Method 1: Using create_vyra_callable with explicit callback
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


# Method 2: Using @remote_callable decorator (for class methods)
class Calculator:
    """Example class using @remote_callable decorator."""
    
    @remote_callable()
    async def multiply_two_ints(self, request, response):
        """
        Service to multiply two integers.
        
        :param request: Service request (a, b)
        :param response: Service response (product)
        :return: Service response
        """
        response.product = request.a * request.b
        print(f"Multiplication: {request.a} * {request.b} = {response.product}")
        return response


async def main():
    """Main function demonstrating VyraCallable usage."""
    
    # Initialize ROS2
    rclpy.init()
    
    provider = ROS2Provider(
        protocol=ProtocolType.ROS2,
        node_name="callable_example_provider"
    )
    # Method 1: Create a Callable with explicit callback
    callable_add = await provider.create_callable(
        name="add_service",
        message_type=AddTwoInts,
        callback=add_two_ints_callback,
        ident_name="add_service",
        description="Service to add two integers"
    )
    
    print("Callable service 'add_service' created!")
    print("Waiting for service requests...")
    print("Call with: ros2 service call /callable_example_node/callable/add_service example_interfaces/srv/AddTwoInts \"{a: 5, b: 3}\"")
    
    # Spin to handle service requests
    try:
        rclpy.spin(provider.get_node())
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    # Cleanup
    await callable_add.shutdown()
    rclpy.shutdown()
    print("Callable example completed!")


if __name__ == '__main__':
    asyncio.run(main())
