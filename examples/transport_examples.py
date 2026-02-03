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
    print_transport_status,
    ROS2_AVAILABLE,
    UDS_AVAILABLE,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


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
        name="echo_service",
        callback=echo_handler,
        module_name="echo_module"
    )
    
    # Create client
    client_callable = await provider.create_callable(
        name="echo_service",
        callback=lambda request: None,  # No callback for client
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
    await ros2_example()
    await uds_example()
    
    logger.info("\n" + "=" * 60)
    logger.info("ALL EXAMPLES COMPLETED")
    logger.info("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
