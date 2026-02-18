"""
Example: Using VyraJob (Action Server) and Action Client

This example demonstrates how to create and use Jobs for long-running operations
with feedback support using ROS2Provider.

Jobs are based on ROS2 actions and provide:
- Goal acceptance/rejection
- Periodic feedback during execution
- Final result upon completion
"""

import asyncio
import rclpy
import time
from example_interfaces.action import Fibonacci
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.transport.t_ros2 import ROS2Provider


# ============================================================================
# SERVER SIDE: VyraJob (Action Server)
# ============================================================================

def fibonacci_goal_callback(goal_handle):
    """
    Goal callback for Fibonacci action server.
    
    This is a long-running task that:
    1. Accepts a goal (number of Fibonacci sequence elements)
    2. Provides periodic feedback (current progress)
    3. Returns final result (complete sequence)
    
    :param goal_handle: ROS2 action goal handle
    :return: Result message
    """
    print(f"Executing Fibonacci sequence for order: {goal_handle.request.order}")
    
    # Initialize feedback and result
    feedback_msg = Fibonacci.Feedback()
    result = Fibonacci.Result()
    
    # Calculate Fibonacci sequence
    fibonacci_sequence = [0, 1]
    
    for i in range(1, goal_handle.request.order):
        # Calculate next number
        fibonacci_sequence.append(
            fibonacci_sequence[i] + fibonacci_sequence[i-1]
        )
        
        # Provide feedback (current sequence)
        feedback_msg.sequence = fibonacci_sequence
        goal_handle.publish_feedback(feedback_msg)
        print(f"Feedback: {feedback_msg.sequence}")
        
        # Simulate work
        time.sleep(0.5)
    
    # Mark goal as succeeded
    goal_handle.succeed()
    
    # Return final result
    result.sequence = fibonacci_sequence
    print(f"Final result: {result.sequence}")
    return result


async def run_job_server():
    """Run the VyraJob (action server)."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 provider
    provider = ROS2Provider(
        module_name="example_job_server",
        module_id="demo123",
        protocol=ProtocolType.ROS2
    )
    
    await provider.initialize(config={
        "node_name": "job_server_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")
    
    # Create a Job (Action Server)
    job = await provider.create_job(
        name="fibonacci_job",
        action_type=Fibonacci,
        callback=fibonacci_goal_callback
    )
    
    print("✅ VyraJob (Action Server) created!")
    print("📞 Waiting for goal requests...")
    print("\nTest with:")
    print("  ros2 action send_goal /examples/example_job_server_demo123/fibonacci_job example_interfaces/action/Fibonacci '{order: 10}'")
    print("\n(Press Ctrl+C to stop)\n")
    
    # Spin to handle action requests
    try:
        rclpy.spin(provider.get_node())
    except KeyboardInterrupt:
        print("\n🛑 Shutting down server...")
    
    # Cleanup
    await job.shutdown()
    await provider.shutdown()
# ============================================================================
# CLIENT SIDE: Action Client
# ============================================================================

async def run_job_client():
    """Run the Action Client to send goals to the action server."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 provider
    provider = ROS2Provider(
        module_name="example_job_client",
        module_id="demo456",
        protocol=ProtocolType.ROS2
    )
    
    await provider.initialize(config={
        "node_name": "job_client_node",
        "namespace": "/examples"
    })
    
    print("✅ ROS2Provider initialized")
    
    # Define feedback callback
    def on_feedback(feedback_msg):
        print(f"📊 Feedback: {feedback_msg.feedback.sequence}")
    
    # Create Job Client (no callback = client side)
    job_client = await provider.create_job(
        name="fibonacci_job",
        action_type=Fibonacci,
        feedback_callback=on_feedback  # Only feedback, no result_callback = client
    )
    
    print("✅ Action Client created!")
    print("📤 Sending goal to action server...")
    
    # Prepare goal message
    goal_msg = Fibonacci.Goal()
    goal_msg.order = 10  # Calculate first 10 Fibonacci numbers
    
    # Send goal (async)
    try:
        result = await job_client.execute(goal_msg, feedback_callback=on_feedback)
        print(f"✅ Goal completed! Result: {result.result.sequence}")
    except Exception as e:
        print(f"❌ Error sending goal: {e}")
    
    # Cleanup
    await job_client.shutdown()
    await provider.shutdown()
    print("✅ Job client example completed!")


# ============================================================================
# COMBINED EXAMPLE: Server + Client
# ============================================================================

def main():
    """
    Main function - choose to run server or client.
    
    For a complete demo:
    1. Run server in one terminal: python example_job.py server
    2. Run client in another terminal: python example_job.py client
    """
    import sys
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python example_job.py server  # Run action server")
        print("  python example_job.py client  # Run action client")
        return
    
    mode = sys.argv[1].lower()
    
    if mode == "server":
        asyncio.run(run_job_server())
    elif mode == "client":
        asyncio.run(run_job_client())
    else:
        print(f"Unknown mode: {mode}")
        print("Use 'server' or 'client'")


if __name__ == '__main__':
    main()
