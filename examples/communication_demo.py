#!/usr/bin/env python3
"""
Communication Layer Demo

Demonstrates the new layered architecture, thread safety, and transaction support.
"""
import asyncio
import logging
import sys
from pathlib import Path

# Add vyra_base to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def demo_redis_provider():
    """Demonstrate Redis provider with layered architecture."""
    logger.info("=" * 60)
    logger.info("DEMO 1: Redis Provider (Layered Architecture)")
    logger.info("=" * 60)
    
    try:
        from vyra_base.com.transport.redis import create_redis_provider
        
        # Create provider (communication layer + VYRA models)
        provider = await create_redis_provider(
            host="localhost",
            port=6379,
            entity_name="demo_module"
        )
        
        # Speaker (publish)
        speaker = provider.create_speaker("demo_events")
        logger.info("✅ Created Speaker on 'demo_events'")
        
        # Listener (subscribe)
        listener = provider.create_listener("demo_events")
        logger.info("✅ Created Listener on 'demo_events'")
        
        # Publish message
        await speaker.publish({"event": "system_started", "timestamp": 1234567890})
        logger.info("📤 Published: system_started")
        
        # Receive message (with timeout)
        async def receive_one():
            async for msg in listener.receive():
                return msg
        
        message = await asyncio.wait_for(receive_one(), timeout=2.0)
        logger.info(f"📥 Received: {message}")
        
        # Callable (request-response pattern)
        callable_obj = provider.create_callable("demo_rpc")
        logger.info("✅ Created Callable 'demo_rpc'")
        
        await provider.disconnect()
        logger.info("✅ Disconnected")
        
    except Exception as e:
        logger.error(f"❌ Redis demo failed: {e}")
        logger.info("💡 Make sure Redis is running: docker run -p 6379:6379 redis")


async def demo_uds_provider():
    """Demonstrate UDS provider with request-response."""
    logger.info("\n" + "=" * 60)
    logger.info("DEMO 2: UDS Provider (Request-Response)")
    logger.info("=" * 60)
    
    try:
        from vyra_base.com.transport.uds import create_uds_provider
        import tempfile
        import os
        
        # Create temp socket
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "demo.sock")
            
            # Server provider
            server = await create_uds_provider(
                socket_path=socket_path,
                is_server=True,
                entity_name="server"
            )
            logger.info(f"✅ Server created on {socket_path}")
            
            # Client provider
            client = await create_uds_provider(
                socket_path=socket_path,
                is_server=False,
                entity_name="client"
            )
            logger.info(f"✅ Client connected to {socket_path}")
            
            # Server handles requests
            server_callable = server.create_callable("calculator")
            
            async def handle_calculation(request):
                operation = request.get("operation")
                a = request.get("a", 0)
                b = request.get("b", 0)
                
                if operation == "add":
                    result = a + b
                elif operation == "multiply":
                    result = a * b
                else:
                    result = None
                
                logger.info(f"🔧 Server processing: {operation}({a}, {b}) = {result}")
                return {"result": result}
            
            # Start server in background
            server_task = asyncio.create_task(
                server_callable.start_server(handle_calculation)
            )
            await asyncio.sleep(0.1)  # Let server start
            
            # Client makes requests
            client_callable = client.create_callable("calculator")
            
            # Request 1: Addition
            result1 = await client_callable.call({"operation": "add", "a": 5, "b": 3})
            logger.info(f"📥 Client received: 5 + 3 = {result1['result']}")
            
            # Request 2: Multiplication
            result2 = await client_callable.call({"operation": "multiply", "a": 7, "b": 6})
            logger.info(f"📥 Client received: 7 * 6 = {result2['result']}")
            
            # Cleanup
            server_task.cancel()
            try:
                await server_task
            except asyncio.CancelledError:
                pass
            
            await server.disconnect()
            await client.disconnect()
            logger.info("✅ Cleanup complete")
            
    except Exception as e:
        logger.error(f"❌ UDS demo failed: {e}")


async def demo_shared_memory_safety():
    """Demonstrate shared memory with thread safety and transactions."""
    logger.info("\n" + "=" * 60)
    logger.info("DEMO 3: Shared Memory (Thread Safety & Transactions)")
    logger.info("=" * 60)
    
    try:
        from vyra_base.com.external.shared_memory.segment import SharedMemorySegment
        from vyra_base.com.external.shared_memory.safety import get_global_safety_manager
        
        # Create shared memory segment
        with SharedMemorySegment("/demo_shm", 1024, create=True) as segment:
            logger.info("✅ Created shared memory segment '/demo_shm'")
            
            # Thread-safe write (automatic locking)
            success = segment.write({"counter": 0, "name": "demo"})
            logger.info(f"📤 Write: counter=0 (success={success})")
            
            # Thread-safe read
            msg_type, data, timestamp = segment.read()
            logger.info(f"📥 Read: {data}")
            
            # Transaction support
            logger.info("\n🔄 Starting transaction...")
            try:
                with segment.transaction() as (current_value, commit):
                    logger.info(f"   Current value: {current_value}")
                    
                    # Modify value
                    new_value = current_value.copy()
                    new_value["counter"] = current_value["counter"] + 10
                    
                    logger.info(f"   New value: {new_value}")
                    
                    # Commit changes
                    commit(new_value)
                    logger.info("   ✅ Transaction committed")
                    
            except Exception as e:
                logger.error(f"   ❌ Transaction failed: {e}")
            
            # Verify update
            msg_type, data, timestamp = segment.read()
            logger.info(f"📥 After transaction: {data}")
            
            # Lock statistics
            safety_manager = get_global_safety_manager()
            stats = safety_manager.get_lock_statistics("/demo_shm")
            if stats:
                logger.info(f"\n📊 Lock Statistics:")
                logger.info(f"   Read operations: {stats.get('read_count', 0)}")
                logger.info(f"   Write operations: {stats.get('write_count', 0)}")
                logger.info(f"   Total wait time: {stats.get('wait_time_total', 0)*1000:.2f}ms")
            
            logger.info("✅ Segment will auto-unlink on exit")
            
    except Exception as e:
        logger.error(f"❌ Shared memory demo failed: {e}")
        logger.info("💡 Make sure posix-ipc is installed: pip install posix-ipc")


async def demo_modbus_split():
    """Demonstrate Modbus split architecture."""
    logger.info("\n" + "=" * 60)
    logger.info("DEMO 4: Modbus Split Architecture")
    logger.info("=" * 60)
    
    try:
        from vyra_base.com.industrial.modbus.tcp import ModbusTCPClient
        
        # Create Modbus TCP client (inherits from ModbusBaseClient)
        client = ModbusTCPClient(
            host="localhost",
            port=502
        )
        logger.info("✅ Created ModbusTCPClient (uses ModbusBaseClient)")
        
        # Try to connect
        try:
            await client.connect()
            logger.info("✅ Connected to Modbus server")
            
            # Read registers
            values = await client.read_holding_registers(address=0, count=5)
            logger.info(f"📥 Read registers: {values}")
            
            await client.disconnect()
            logger.info("✅ Disconnected")
            
        except Exception as e:
            logger.warning(f"⚠️  No Modbus server running: {e}")
            logger.info("💡 Architecture demonstration complete (no server needed)")
            
    except Exception as e:
        logger.error(f"❌ Modbus demo failed: {e}")


async def demo_external_registry():
    """Demonstrate external registry."""
    logger.info("\n" + "=" * 60)
    logger.info("DEMO 5: External Registry")
    logger.info("=" * 60)
    
    try:
        from vyra_base.com.external.registry import ExternalRegistry
        
        # Create registry
        registry = ExternalRegistry()
        logger.info("✅ Created ExternalRegistry")
        
        # Register multiple clients
        registry.register_client("mqtt", "sensor_client", {"status": "active", "port": 1883})
        registry.register_client("mqtt", "actuator_client", {"status": "active", "port": 1883})
        registry.register_client("grpc", "api_server", {"status": "running", "port": 50051})
        logger.info("✅ Registered 3 clients")
        
        # Get all clients for a protocol
        mqtt_clients = registry.get_all_clients("mqtt")
        logger.info(f"📋 MQTT clients: {list(mqtt_clients.keys())}")
        
        # Get health status
        health = registry.get_health_status()
        logger.info(f"🏥 Health status:")
        for protocol, clients in health.items():
            logger.info(f"   {protocol}: {len(clients)} clients")
            for name, info in clients.items():
                logger.info(f"      - {name}: {info['status']}")
        
        # Update client status
        registry.update_client_status("mqtt", "sensor_client", "reconnecting")
        logger.info("✅ Updated sensor_client status")
        
        # Get updated health
        health = registry.get_health_status()
        sensor_status = health["mqtt"]["sensor_client"]["status"]
        logger.info(f"📊 sensor_client status: {sensor_status}")
        
        # Unregister client
        registry.unregister_client("mqtt", "actuator_client")
        logger.info("✅ Unregistered actuator_client")
        
        # Final count
        mqtt_clients = registry.get_all_clients("mqtt")
        logger.info(f"📋 Final MQTT clients: {list(mqtt_clients.keys())}")
        
    except Exception as e:
        logger.error(f"❌ Registry demo failed: {e}")


async def main():
    """Run all demos."""
    logger.info("\n" + "🚀" * 30)
    logger.info("VYRA Communication Layer Demo")
    logger.info("Layered Architecture | Thread Safety | Transaction Support")
    logger.info("🚀" * 30)
    
    # Demo 1: Redis (optional - needs server)
    await demo_redis_provider()
    
    # Demo 2: UDS (always works)
    await demo_uds_provider()
    
    # Demo 3: Shared Memory (needs posix-ipc)
    await demo_shared_memory_safety()
    
    # Demo 4: Modbus (always works - architecture demo)
    await demo_modbus_split()
    
    # Demo 5: Registry (always works)
    await demo_external_registry()
    
    logger.info("\n" + "=" * 60)
    logger.info("✅ All demos complete!")
    logger.info("=" * 60)
    logger.info("\nKey Features Demonstrated:")
    logger.info("  1. Layered architecture (communication + vyra_models)")
    logger.info("  2. Provider pattern (unified interface)")
    logger.info("  3. Thread-safe operations (automatic locking)")
    logger.info("  4. Transaction support (ACID-like guarantees)")
    logger.info("  5. Modbus split (base/tcp/rtu)")
    logger.info("  6. External registry (multi-protocol management)")


if __name__ == '__main__':
    asyncio.run(main())
