"""
Example: Using VyraJob (Action Server) and VyraJobRunner (Action Client)

This example demonstrates how to create and use Jobs for long-running operations
with feedback support.

Jobs are based on ROS2 actions and provide:
- Goal acceptance/rejection
- Periodic feedback during execution
- Final result upon completion
"""

import rclpy
import time
from rclpy.node import Node
from example_interfaces.action import Fibonacci
from vyra_base.com import create_vyra_job, create_vyra_job_runner
from vyra_base.com.transport.t_ros2.node import VyraNode, NodeSettings


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


def run_job_server():
    """Run the VyraJob (action server)."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create a VyraNode
    node_settings = NodeSettings(name="job_server_node")
    node = VyraNode(node_settings=node_settings)
    
    # Create a Job (Action Server)
    job = create_vyra_job(
        type=Fibonacci,
        node=node,
        goal_callback=fibonacci_goal_callback,
        ident_name="fibonacci_job",
    )
    
    print("VyraJob (Action Server) created!")
    print("Waiting for goal requests...")
    print("Call with: ros2 action send_goal /job_server_node/job/fibonacci_job example_interfaces/action/Fibonacci \"{order: 10}\"")
    
    # Spin to handle action requests
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down server...")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


# ============================================================================
# CLIENT SIDE: VyraJobRunner (Action Client)
# ============================================================================

def on_feedback_received(feedback_msg):
    """
    Callback for receiving feedback from action server.
    
    :param feedback_msg: Feedback message from action
    """
    print(f"Received feedback: {feedback_msg.feedback.sequence}")


def on_result_received(result):
    """
    Callback for receiving final result from action server.
    
    :param result: Result message from action
    """
    print(f"Final result received: {result.result.sequence}")
    print(f"Status: {result.status}")


async def run_job_client():
    """Run the VyraJobRunner (action client)."""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create a VyraNode
    node_settings = NodeSettings(name="job_client_node")
    node = VyraNode(node_settings=node_settings)
    
    # Create a Job Runner (Action Client)
    job_runner = create_vyra_job_runner(
        type=Fibonacci,
        node=node,
        feedback_callback=on_feedback_received,
        result_callback=on_result_received,
        ident_name="fibonacci_client",
        timeout_sec=5.0
    )
    
    print("VyraJobRunner (Action Client) created!")
    print("Sending goal to action server...")
    
    # Prepare goal message
    goal_msg = Fibonacci.Goal()
    goal_msg.order = 10  # Calculate first 10 Fibonacci numbers
    
    # Send goal (async)
    try:
        result = await job_runner.send_goal(goal_msg)
        print(f"Goal completed! Result: {result.sequence}")
    except Exception as e:
        print(f"Error sending goal: {e}")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    print("Job client example completed!")


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
        run_job_server()
    elif mode == "client":
        import asyncio
        asyncio.run(run_job_client())
    else:
        print(f"Unknown mode: {mode}")
        print("Use 'server' or 'client'")


if __name__ == '__main__':
    main()
