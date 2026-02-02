"""
Transport Layer Implementation Examples

Demonstrates usage of all three transport protocols:
- Shared Memory (zero-copy local IPC)
- ROS2 (distributed DDS communication)
- UDS (Unix domain sockets)
"""
import asyncio
import logging

from vyra_base.com.core.types import ProtocolType, VyraSpeaker
from vyra_base.com.transport import (
    get_available_transports,
    print_transport_status,
    SHARED_MEMORY_AVAILABLE,
    ROS2_AVAILABLE,
    UDS_AVAILABLE,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# =============================================================================
# SHARED MEMORY EXAMPLE
# =============================================================================

async def shared_memory_example():
    """Demonstrate shared memory transport."""
    if not SHARED_MEMORY_AVAILABLE:
        logger.warning("⚠️ Shared Memory not available")
        return
    
    from vyra_base.com.transport.shared_memory import SharedMemoryProvider
    
    logger.info("=" * 60)
    logger.info("SHARED MEMORY TRANSPORT EXAMPLE")
    logger.info("=" * 60)
    
    # Initialize provider
    provider = SharedMemoryProvider(
        protocol=ProtocolType.SHARED_MEMORY,
        module_name="example_module"
    )
    
    if not await provider.check_availability():
        logger.error("❌ Shared memory not available")
        return
    
    await provider.initialize(config={
        "segment_size": 8192,  # 8KB segments
        "serialization_format": "json"
    })
    
    # Example 1: Callable (Request-Response)
    logger.info("\n--- Example 1: Callable (Request-Response) ---")
    
    async def calculate_handler(request):
        """Server-side handler."""
        result = request["a"] + request["b"]
        return {"result": result}
    
    # Create server
    server_callable = await provider.create_callable(
        "add_service",
        calculate_handler,
        module_name="math_module"
    )
    
    # Create client
    client_callable = await provider.create_callable(
        "add_service",
        lambda x:x,  # No callback for client
        module_name="math_module"
    )
    
    # Make call
    result = await client_callable.call({"a": 10, "b": 32}, timeout=5.0)
    logger.info(f"Result: {result}")  # {"result": 42}
    
    # Cleanup
    await server_callable.shutdown()
    await client_callable.shutdown()
    
    # Example 2: Speaker (Pub/Sub)
    logger.info("\n--- Example 2: Speaker (Pub/Sub) ---")
    
    # Create publisher
    publisher = await provider.create_speaker(
        "sensor_data",
        module_name="robot",
        is_publisher=True
    )
    
    # Create subscriber
    subscriber: VyraSpeaker = await provider.create_speaker(
        "sensor_data",
        module_name="robot",
        is_publisher=False
    )
    
    # Subscribe with callback
    messages_received = []
    
    async def on_message(data):
        messages_received.append(data)
        logger.info(f"Received: {data}")
    
    await subscriber.listen(on_message)
    
    # Publish messages
    for i in range(3):
        await publisher.shout({"temperature": 20.0 + i, "humidity": 50.0})
        await asyncio.sleep(0.1)
    
    # Wait for messages
    await asyncio.sleep(0.5)
    
    logger.info(f"Total messages received: {len(messages_received)}")
    
    # Cleanup
    await subscriber.stop_listening()
    await publisher.shutdown()
    await subscriber.shutdown()
    
    # Show discovery statistics
    stats = provider.get_discovery_statistics()
    logger.info(f"\nDiscovery Statistics: {stats}")
    
    # Shutdown provider
    await provider.shutdown()
    
    logger.info("\n✅ Shared Memory example completed")


# =============================================================================
# ROS2 EXAMPLE
# =============================================================================

async def ros2_example():
    """Demonstrate ROS2 transport."""
    if not ROS2_AVAILABLE:
        logger.warning("⚠️ ROS2 not available")
        return
    
    from vyra_base.com.transport.ros2 import ROS2Provider
    
    logger.info("=" * 60)
    logger.info("ROS2 TRANSPORT EXAMPLE")
    logger.info("=" * 60)
    
    # Initialize provider
    provider = ROS2Provider(
        protocol=ProtocolType.ROS2,
        node_name="example_node"
    )
    
    if not await provider.check_availability():
        logger.error("❌ ROS2 not available")
        return
    
    await provider.initialize(config={
        "node_name": "vyra_example",
        "namespace": "/vyra"
    })
    
    logger.info("✅ ROS2 provider initialized")
    
    # Note: Full ROS2 examples require service/message types
    # This is a basic initialization example
    
    # Shutdown
    await provider.shutdown()
    
    logger.info("\n✅ ROS2 example completed")


# =============================================================================
# UDS EXAMPLE
# =============================================================================

async def uds_example():
    """Demonstrate Unix Domain Socket transport."""
    if not UDS_AVAILABLE:
        logger.warning("⚠️ UDS not available")
        return
    
    from vyra_base.com.transport.uds import UDSProvider
    
    logger.info("=" * 60)
    logger.info("UNIX DOMAIN SOCKET TRANSPORT EXAMPLE")
    logger.info("=" * 60)
    
    # Initialize provider
    provider = UDSProvider(
        protocol=ProtocolType.UDS,
        module_name="example_module"
    )
    
    if not await provider.check_availability():
        logger.error("❌ UDS not available")
        return
    
    await provider.initialize()
    
    # Example: Callable (Request-Response)
    logger.info("\n--- Example: Callable (Request-Response) ---")
    
    async def echo_handler(request):
        """Server-side echo handler."""
        return {"echo": request.get("message", "")}
    
    # Create server
    server_callable = await provider.create_callable(
        "echo_service",
        echo_handler,
        module_name="echo_module"
    )
    
    # Create client
    client_callable = await provider.create_callable(
        "echo_service",
        None,  # No callback for client
        module_name="echo_module"
    )
    
    # Make call
    result = await client_callable.call(
        {"message": "Hello UDS!"},
        timeout=5.0
    )
    logger.info(f"Result: {result}")  # {"echo": "Hello UDS!"}
    
    # Cleanup
    await server_callable.shutdown()
    await client_callable.shutdown()
    
    # Shutdown provider
    await provider.shutdown()
    
    logger.info("\n✅ UDS example completed")


# =============================================================================
# MAIN
# =============================================================================

async def main():
    """Run all examples."""
    logger.info("\n" + "=" * 60)
    logger.info("VYRA TRANSPORT LAYER EXAMPLES")
    logger.info("=" * 60)
    
    # Print transport status
    print_transport_status()
    
    # Run examples for available transports
    await shared_memory_example()
    await ros2_example()
    await uds_example()
    
    logger.info("\n" + "=" * 60)
    logger.info("ALL EXAMPLES COMPLETED")
    logger.info("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
