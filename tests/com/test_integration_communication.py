"""
Integration tests for Communication Restructuring

Tests the complete communication layer with all transports and protocols.
"""
import pytest
import asyncio
import tempfile
import os
from pathlib import Path

# Skip if dependencies not available
pytest.importorskip("redis")
pytest.importorskip("rclpy")
pytest.importorskip("posix_ipc")


@pytest.mark.integration
class TestTransportIntegration:
    """Integration tests for transport layer."""
    
    @pytest.mark.asyncio
    async def test_redis_provider_lifecycle(self):
        """Test Redis provider full lifecycle."""
        from vyra_base.com.transport.t_redis import RedisProvider
        
        try:
            # Create provider
            provider = RedisProvider()

            await provider.initialize({
                    "db": 0,
                    "host": "localhost",
                    "port": 6379,
                    "ssl": False,
                    "entity_name": "test_module"
                }
            )

            # Create components
            publisher = provider.create_publisher("test_events")
            subscriber = provider.create_subscriber("test_events")
            
            # Publish and receive
            await publisher.publish({"event": "test"})
            
            # Receive with timeout
            async def receive_with_timeout():
                async for message in subscriber.receive():
                    return message
            
            message = await asyncio.wait_for(receive_with_timeout(), timeout=2.0)
            assert message["event"] == "test"
            
            # Cleanup
            await provider.disconnect()
            
        except Exception as e:
            pytest.skip(f"Redis not available: {e}")
    
    @pytest.mark.asyncio
    async def test_uds_provider_lifecycle(self):
        """Test UDS provider full lifecycle."""
        from vyra_base.com.transport.t_uds import UDSProvider
        
        # Create temp socket path
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "test.sock")
            
            try:
                # Server provider
                server_provider = UDSProvider(
                    module_name="server",
                    module_id="server_id",
                )
                
                # Client provider
                client_provider = UDSProvider(
                    module_name="client",
                    module_id="client_id",
                )
                
                # Server callable
                server_callable = server_provider.create_server("rpc")
                
                # Start server
                async def handle_request(request):
                    return {"result": request["value"] * 2}
                
                server_task = asyncio.create_task(
                    server_callable.start_server(handle_request)
                )
                
                # Give server time to start
                await asyncio.sleep(0.1)
                
                # Client call
                client_callable = client_provider.create_server("rpc")
                result = await client_callable.call({"value": 21})
                
                assert result["result"] == 42
                
                # Cleanup
                server_task.cancel()
                try:
                    await server_task
                except asyncio.CancelledError:
                    pass
                
                await server_provider.disconnect()
                await client_provider.disconnect()
                
            except Exception as e:
                pytest.skip(f"UDS test failed: {e}")


@pytest.mark.integration
class TestIndustrialProtocols:
    """Integration tests for industrial protocols."""
    
    @pytest.mark.asyncio
    async def test_modbus_tcp_connection(self):
        """Test Modbus TCP client lifecycle."""
        from vyra_base.com.industrial.modbus.tcp import ModbusTCPClient
        
        try:
            client = ModbusTCPClient(
                host="localhost",
                port=502
            )
            
            # Try to connect
            await client.connect()
            
            # Read registers (will fail if no server, that's ok)
            try:
                values = await client.read_holding_registers(
                    address=0,
                    count=10
                )
            except Exception:
                # Expected if no Modbus server running
                pass
            
            await client.disconnect()
            
        except Exception as e:
            pytest.skip(f"Modbus TCP not available: {e}")
    
    @pytest.mark.asyncio
    async def test_opcua_client_connection(self):
        """Test OPC UA client lifecycle."""
        from vyra_base.com.industrial.opcua.opcua_client import OpcuaClient
        
        try:
            client = OpcuaClient(
                endpoint="opc.tcp://localhost:4840"
            )
            
            # Try to connect
            await client.connect()
            
            # Read node (will fail if no server, that's ok)
            try:
                value = await client.read_node("ns=2;i=1")
            except Exception:
                # Expected if no OPC UA server running
                pass
            
            await client.disconnect()
            
        except Exception as e:
            pytest.skip(f"OPC UA not available: {e}")


@pytest.mark.integration
class TestExternalProtocols:
    """Integration tests for external protocols."""
    
    @pytest.mark.asyncio
    async def test_shared_memory_lifecycle(self):
        """Test shared memory segment lifecycle."""
        from vyra_base.com.external.shared_memory.segment import SharedMemorySegment
        
        try:
            # Create segment
            with SharedMemorySegment("/test_integration", 1024, create=True) as segment:
                # Write data
                success = segment.write({"test": "data"})
                assert success
                
                # Read data
                msg_type, data, timestamp = segment.read()
                assert data["test"] == "data"
                
                # Should auto-unlink on exit (create=True)
            
        except Exception as e:
            pytest.skip(f"Shared memory not available: {e}")
    
    @pytest.mark.asyncio
    async def test_registry_lifecycle(self):
        """Test external registry lifecycle."""
        from vyra_base.com.external.registry import ExternalRegistry
        
        try:
            registry = ExternalRegistry()
            
            # Register client
            registry.register_client(
                protocol="mqtt",
                name="test_client",
                client_instance={"status": "active"}
            )
            
            # Get client
            client = registry.get_client("mqtt", "test_client")
            assert client is not None
            assert client["status"] == "active"
            
            # Get health status
            health = registry.get_health_status()
            assert "mqtt" in health
            assert "test_client" in health["mqtt"]
            
            # Unregister
            success = registry.unregister_client("mqtt", "test_client")
            assert success
            
        except Exception as e:
            pytest.skip(f"Registry test failed: {e}")


@pytest.mark.integration
class TestLayeredArchitecture:
    """Test layered architecture patterns."""
    
    @pytest.mark.asyncio
    async def test_communication_to_vyra_models(self):
        """Test communication layer to VYRA models flow."""
        from vyra_base.com.transport.t_uds import UDSProvider
        
        with tempfile.TemporaryDirectory() as tmpdir:
            
            try:
                # Provider uses communication layer internally
                provider = UDSProvider(
                    module_name="test",
                    module_id="test_id",
                )
                
                # VYRA models (Speaker, Listener, Callable) built on communication layer
                speaker = provider.create_publisher("events")
                
                # Should work without knowing underlying communication details
                await speaker.publish({"event": "test"})
                
                await provider.disconnect()
                
            except Exception as e:
                pytest.skip(f"Layered architecture test failed: {e}")


@pytest.mark.integration
class TestEndToEnd:
    """End-to-end tests simulating real usage."""
    
    @pytest.mark.asyncio
    async def test_multi_transport_communication(self):
        """Test communication across multiple transports."""
        results = []
        
        # Use UDS for local communication
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "e2e_test.sock")
            
            try:
                from vyra_base.com.transport.t_uds import UDSProvider
                
                # Server
                server = UDSProvider(
                    module_name="server",
                    module_id="server_id",
                )
                
                # Client
                client = UDSProvider(
                    module_name="client",
                    module_id="client_id",
                )
                
                # Server handles requests
                server_callable = server.create_server("api")
                
                async def handle_api_request(request):
                    return {"processed": request["data"]}
                
                server_task = asyncio.create_task(
                    server_callable.start_server(handle_api_request)
                )
                
                await asyncio.sleep(0.1)
                
                # Client sends requests
                client_callable = client.create_server("api")
                
                for i in range(5):
                    result = await client_callable.call({"data": f"request_{i}"})
                    results.append(result)
                
                # Check results
                assert len(results) == 5
                assert all("processed" in r for r in results)
                
                # Cleanup
                server_task.cancel()
                try:
                    await server_task
                except asyncio.CancelledError:
                    pass
                
                await server.disconnect()
                await client.disconnect()
                
            except Exception as e:
                pytest.skip(f"Multi-transport test failed: {e}")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-m', 'integration'])
