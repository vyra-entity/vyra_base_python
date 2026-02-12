"""
Example: Using create_caller and create_dispatcher (Client-side)

This example demonstrates the new client-side methods:
- create_caller: Service Client (counterpart to create_callable)
- create_dispatcher: Action Client (counterpart to create_job)

These methods automatically set is_callable=False and is_job=False flags.

Prerequisites:
  Run these servers first:
  1. python examples/example_callable.py (for service)
  2. python examples/example_job.py server (for action)

Usage:
  python examples/example_caller_dispatcher.py
"""

import asyncio
import rclpy
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci
from vyra_base.com import InterfaceFactory
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider


async def demo_caller():
    """Demonstrate create_caller (Service Client)."""
    print("=" * 60)
    print("DEMO 1: create_caller (Service Client)")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create and register ROS2 provider
    provider = ROS2Provider(
        module_name="example_caller",
        module_id="demo123",
        protocol=ProtocolType.ROS2
    )
    
    await provider.initialize(config={
        "node_name": "caller_example_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")
    
    # Register provider with InterfaceFactory
    InterfaceFactory.register_provider(provider)
    
    # Create caller (client-side) - automatically sets is_callable=False
    caller = await InterfaceFactory.create_caller(
        name="add_service",
        service_type=AddTwoInts,
        protocols=[ProtocolType.ROS2]
    )
    
    print("✅ Caller (Service Client) created!\n")
    
    # Make service calls
    try:
        print("📞 Calling service: 5 + 3")
        request = AddTwoInts.Request()
        request.a = 5
        request.b = 3
        
        response = await caller.call(request, timeout=5.0)
        print(f"✅ Response: {response.sum}\n")
        
        print("📞 Calling service: 10 + 20")
        request.a = 10
        request.b = 20
        response = await caller.call(request, timeout=5.0)
        print(f"✅ Response: {response.sum}\n")
        
    except Exception as e:
        print(f"❌ Service call failed: {e}")
        print("💡 Make sure example_callable.py server is running!\n")
    
    # Cleanup
    await caller.shutdown()
    await provider.shutdown()
    rclpy.shutdown()


async def demo_dispatcher():
    """Demonstrate create_dispatcher (Action Client)."""
    print("\n" + "=" * 60)
    print("DEMO 2: create_dispatcher (Action Client)")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create and register ROS2 provider
    provider = ROS2Provider(
        module_name="example_dispatcher",
        module_id="demo456",
        protocol=ProtocolType.ROS2
    )
    
    await provider.initialize(config={
        "node_name": "dispatcher_example_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")
    
    # Register provider with InterfaceFactory
    InterfaceFactory.register_provider(provider)
    
    # Define feedback callback
    def on_feedback(feedback_msg):
        print(f"📊 Feedback: {feedback_msg.feedback.sequence}")
    
    # Create dispatcher (client-side) - automatically sets is_job=False
    dispatcher = await InterfaceFactory.create_dispatcher(
        name="fibonacci_job",
        action_type=Fibonacci,
        feedback_callback=on_feedback,
        protocols=[ProtocolType.ROS2]
    )
    
    print("✅ Dispatcher (Action Client) created!\n")
    
    # Send goal
    try:
        print("🚀 Sending goal: Calculate Fibonacci sequence (order=10)")
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        
        result = await dispatcher.execute(goal_msg, feedback_callback=on_feedback)
        print(f"✅ Final result: {result.result.sequence}\n")
        
    except Exception as e:
        print(f"❌ Action failed: {e}")
        print("💡 Make sure example_job.py server is running!\n")
    
    # Cleanup
    await dispatcher.shutdown()
    await provider.shutdown()
    rclpy.shutdown()


async def main():
    """Main function demonstrating both caller and dispatcher."""
    print("\n" + "🔷" * 30)
    print("Client-Side Methods Demo")
    print("create_caller & create_dispatcher")
    print("🔷" * 30 + "\n")
    
    # Demo 1: Caller (Service Client)
    await demo_caller()
    
    # Demo 2: Dispatcher (Action Client)
    await demo_dispatcher()
    
    print("=" * 60)
    print("✅ Demo completed!")
    print("=" * 60)


if __name__ == '__main__':
    asyncio.run(main())
