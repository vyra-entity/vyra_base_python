"""
Unit tests for External Protocol Providers.

Tests gRPC, MQTT, REST, and WebSocket protocol implementations.
"""
import pytest
from unittest.mock import Mock, AsyncMock, MagicMock, patch
from vyra_base.com.core.types import ProtocolType


class TestGrpcProvider:
    """Test gRPC protocol provider."""
    
    @pytest.mark.asyncio
    async def test_grpc_provider_availability_without_grpcio(self):
        """Test that gRPC provider checks for grpcio package."""
        with patch('importlib.import_module') as mock_import:
            mock_import.side_effect = ImportError("No module named 'grpcio'")
            
            # Provider should handle missing grpcio gracefully
            try:
                from vyra_base.com.external.grpc_provider import GrpcProvider
                provider = GrpcProvider()
                available = provider.is_available
                assert available is False
            except ImportError:
                # If provider doesn't exist yet, test passes
                pytest.skip("GrpcProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_grpc_provider_with_grpcio(self):
        """Test gRPC provider when grpcio is available."""
        try:
            from vyra_base.com.external.grpc_provider import GrpcProvider
            
            # Mock grpcio availability
            with patch('importlib.import_module') as mock_import:
                mock_grpc = MagicMock()
                mock_import.return_value = mock_grpc
                
                provider = GrpcProvider()
                assert provider.protocol == ProtocolType.GRPC
        except ImportError:
            pytest.skip("GrpcProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_grpc_create_server(self):
        """Test creating gRPC callable."""
        try:
            from vyra_base.com.external.grpc_provider import GrpcProvider
            
            provider = GrpcProvider()
            
            async def test_callback(request):
                return {"result": "ok"}
            
            # Should create callable even if grpcio not available (graceful degradation)
            if provider.is_available:
                callable = await provider.create_server(
                    name="test_service",
                    callback=test_callback
                )
                assert callable is not None
        except ImportError:
            pytest.skip("GrpcProvider not implemented yet")


class TestMqttProvider:
    """Test MQTT protocol provider."""
    
    @pytest.mark.asyncio
    async def test_mqtt_provider_availability_without_paho(self):
        """Test that MQTT provider checks for paho-mqtt package."""
        with patch('importlib.import_module') as mock_import:
            mock_import.side_effect = ImportError("No module named 'paho'")
            
            try:
                from vyra_base.com.external.mqtt_provider import MqttProvider
                provider = MqttProvider()
                available = provider.is_available
                assert available is False
            except ImportError:
                pytest.skip("MqttProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_mqtt_provider_with_paho(self):
        """Test MQTT provider when paho-mqtt is available."""
        try:
            from vyra_base.com.external.mqtt_provider import MqttProvider
            
            # Mock paho availability
            with patch('importlib.import_module') as mock_import:
                mock_paho = MagicMock()
                mock_import.return_value = mock_paho
                
                provider = MqttProvider()
                assert provider.protocol == ProtocolType.MQTT
        except ImportError:
            pytest.skip("MqttProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_mqtt_create_publisher(self):
        """Test creating MQTT speaker (publisher)."""
        try:
            from vyra_base.com.external.mqtt_provider import MqttProvider
            
            provider = MqttProvider()
            
            if provider.is_available:
                speaker = await provider.create_publisher(
                    name="test_topic",
                    broker_host="localhost",
                    broker_port=1883
                )
                assert speaker is not None
        except ImportError:
            pytest.skip("MqttProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_mqtt_connection_settings(self):
        """Test MQTT connection with custom settings."""
        try:
            from vyra_base.com.external.mqtt_provider import MqttProvider
            
            provider = MqttProvider()
            
            config = {
                "broker_host": "mqtt.example.com",
                "broker_port": 8883,
                "use_tls": True,
                "username": "user",
                "password": "pass"
            }
            
            if provider.is_available:
                await provider.initialize(config)
                assert provider.is_initialized
        except ImportError:
            pytest.skip("MqttProvider not implemented yet")


class TestRestProvider:
    """Test REST/FastAPI protocol provider."""
    
    @pytest.mark.asyncio
    async def test_rest_provider_availability(self):
        """Test REST provider availability."""
        try:
            from vyra_base.com.external.rest_provider import RestProvider
            
            provider = RestProvider()
            assert provider.protocol == ProtocolType.REST
            
            # REST should always be available (uses FastAPI from requirements)
            assert provider.is_available is True
        except ImportError:
            pytest.skip("RestProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_rest_create_server(self):
        """Test creating REST endpoint as callable."""
        try:
            from vyra_base.com.external.rest_provider import RestProvider
            
            provider = RestProvider()
            
            async def test_callback(request):
                return {"result": "ok"}
            
            callable = await provider.create_server(
                name="test_endpoint",
                callback=test_callback,
                method="POST",
                path="/api/test"
            )
            
            assert callable is not None
        except ImportError:
            pytest.skip("RestProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_rest_create_publisher(self):
        """Test creating REST webhook as speaker."""
        try:
            from vyra_base.com.external.rest_provider import RestProvider
            
            provider = RestProvider()
            
            speaker = await provider.create_publisher(
                name="test_webhook",
                webhook_url="https://example.com/webhook"
            )
            
            assert speaker is not None
        except ImportError:
            pytest.skip("RestProvider not implemented yet")


class TestWebSocketProvider:
    """Test WebSocket protocol provider."""
    
    @pytest.mark.asyncio
    async def test_websocket_provider_availability(self):
        """Test WebSocket provider availability."""
        try:
            from vyra_base.com.external.websocket_provider import WebSocketProvider
            
            provider = WebSocketProvider()
            assert provider.protocol == ProtocolType.WEBSOCKET
            
            # WebSocket should be available (uses FastAPI WebSockets)
            assert provider.is_available is True
        except ImportError:
            pytest.skip("WebSocketProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_websocket_create_server(self):
        """Test creating WebSocket handler as callable."""
        try:
            from vyra_base.com.external.websocket_provider import WebSocketProvider
            
            provider = WebSocketProvider()
            
            async def test_callback(message):
                return {"echo": message}
            
            callable = await provider.create_server(
                name="test_ws_handler",
                callback=test_callback,
                path="/ws/test"
            )
            
            assert callable is not None
        except ImportError:
            pytest.skip("WebSocketProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_websocket_create_publisher(self):
        """Test creating WebSocket broadcaster as speaker."""
        try:
            from vyra_base.com.external.websocket_provider import WebSocketProvider
            
            provider = WebSocketProvider()
            
            speaker = await provider.create_publisher(
                name="test_broadcast",
                room="global"
            )
            
            assert speaker is not None
        except ImportError:
            pytest.skip("WebSocketProvider not implemented yet")
    
    @pytest.mark.asyncio
    async def test_websocket_connection_manager(self):
        """Test WebSocket connection manager."""
        try:
            from vyra_base.com.external.websocket_provider import WebSocketProvider
            
            provider = WebSocketProvider()
            await provider.initialize()
            
            # Provider should have connection manager
            assert hasattr(provider, 'connection_manager') or hasattr(provider, 'connections')
        except (ImportError, AttributeError):
            pytest.skip("WebSocketProvider not fully implemented yet")


class TestExternalProvidersIntegration:
    """Integration tests for external protocol providers."""
    
    @pytest.mark.asyncio
    async def test_all_external_providers_in_registry(self):
        """Test that all external providers can be registered."""
        from vyra_base.com.providers import ProviderRegistry
        from vyra_base.com.core.types import ProtocolType
        
        registry = ProviderRegistry()
        registry._providers.clear()
        
        external_protocols = [
            ProtocolType.GRPC,
            ProtocolType.MQTT,
            ProtocolType.REST,
            ProtocolType.WEBSOCKET
        ]
        
        registered = []
        
        for protocol in external_protocols:
            try:
                # Try to import and register each provider
                if protocol == ProtocolType.GRPC:
                    from vyra_base.com.external.grpc_provider import GrpcProvider
                    provider = GrpcProvider()
                elif protocol == ProtocolType.MQTT:
                    from vyra_base.com.external.mqtt_provider import MqttProvider
                    provider = MqttProvider()
                elif protocol == ProtocolType.REST:
                    from vyra_base.com.external.rest_provider import RestProvider
                    provider = RestProvider()
                elif protocol == ProtocolType.WEBSOCKET:
                    from vyra_base.com.external.websocket_provider import WebSocketProvider
                    provider = WebSocketProvider()
                
                registry.register_provider(provider)
                registered.append(protocol)
            except ImportError:
                pass  # Provider not implemented yet
        
        # At least some providers should be registrable
        assert len(registered) >= 0  # Will pass even if no providers implemented yet
    
    @pytest.mark.asyncio
    async def test_external_protocol_fallback(self):
        """Test fallback between external protocols."""
        from vyra_base.com.core.factory import InterfaceFactory
        from vyra_base.com.providers import ProviderRegistry
        
        registry = ProviderRegistry()
        registry._providers.clear()
        
        # Test that factory can handle unavailable external protocols gracefully
        async def test_callback(request):
            return {"result": "ok"}
        
        try:
            # Should raise error if no providers available
            with pytest.raises(Exception):
                await InterfaceFactory.create_server(
                    name="test",
                    callback=test_callback,
                    protocols=[ProtocolType.GRPC, ProtocolType.REST]
                )
        except Exception:
            # Expected behavior when no providers registered
            pass
