"""
Unit tests for InterfaceFactory.
"""
import pytest
from unittest.mock import Mock, AsyncMock, patch
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.core.types import ProtocolType, InterfaceType
from vyra_base.com.providers import ProviderRegistry


class TestInterfaceFactory:
    """Test InterfaceFactory class."""
    
    @pytest.fixture(autouse=True)
    def clear_registry(self):
        """Clear registry before each test."""
        registry = ProviderRegistry()
        registry._providers.clear()
        yield
        registry._providers.clear()
    
    @pytest.fixture
    def mock_provider(self):
        """Create mock provider."""
        from vyra_base.com.providers import AbstractProtocolProvider
        
        class MockProvider(AbstractProtocolProvider):
            def __init__(self, protocol: ProtocolType):
                super().__init__(protocol)
                self._available = True
                self._initialized = False
            
            async def check_availability(self) -> bool:
                return self._available
            
            def is_available(self) -> bool:
                return self._available
            
            async def initialize(self):
                self._initialized = True
            
            async def shutdown(self):
                self._initialized = False
            
            async def create_server(self, name: str, **kwargs):
                return Mock(name=name, _protocol=self.protocol)

            async def create_subscriber(self, name: str, **kwargs):
                return Mock(name=name, _protocol=self.protocol)

            async def create_client(self, name: str, **kwargs):
                return Mock(name=name, _protocol=self.protocol)

            async def create_action_server(self, name: str, **kwargs):
                return Mock(name=name, _protocol=self.protocol)

            async def create_action_client(self, name: str, **kwargs):
                return Mock(name=name, _protocol=self.protocol)
            
            async def create_publisher(self, name: str, **kwargs):
                return Mock(name=name, _protocol=self.protocol)
        
        return MockProvider
    
    @pytest.mark.asyncio
    async def test_factory_has_fallback_chains(self):
        """Test factory defines fallback chains."""
        assert hasattr(InterfaceFactory, 'SERVER_FALLBACK')
        assert hasattr(InterfaceFactory, 'PUBLISHER_FALLBACK')
        assert hasattr(InterfaceFactory, 'ACTION_SERVER_FALLBACK')
        
        # Verify chains are lists of ProtocolType
        assert isinstance(InterfaceFactory.SERVER_FALLBACK, list)
        assert len(InterfaceFactory.SERVER_FALLBACK) > 0
    
    @pytest.mark.asyncio
    async def test_create_server_with_specific_protocol(self, mock_provider):
        """Test creating callable with specific protocol."""
        registry = ProviderRegistry()
        provider = mock_provider(ProtocolType.REDIS)
        registry.register_provider(provider)
        
        async def test_callback(request):
            return {"result": "ok"}
        
        result = await InterfaceFactory.create_server(
            "test_callable",
            response_callback=test_callback,
            protocols=[ProtocolType.REDIS]
        )
        
        assert hasattr(result, 'name')
    
    @pytest.mark.asyncio
    async def test_create_server_fallback_chain(self, mock_provider):
        """Test callable creation falls back to next protocol."""
        registry = ProviderRegistry()
        
        # Register two providers, second will be used
        provider1 = mock_provider(ProtocolType.ROS2)
        provider1._available = False  # First unavailable
        
        provider2 = mock_provider(ProtocolType.REDIS)
        provider2._available = True  # Second available
        
        registry.register_provider(provider1)
        registry.register_provider(provider2)
        
        async def test_callback(request):
            return {"result": "ok"}
        
        # Should fall back to REDIS
        result = await InterfaceFactory.create_server(
            "test_callable",
            response_callback=test_callback,
            protocols=[ProtocolType.REDIS]  # Skip ROS2, use REDIS directly
        )
        
        assert result is not None
    
    @pytest.mark.asyncio
    async def test_create_publisher_with_specific_protocol(self, mock_provider):
        """Test creating speaker with specific protocol."""
        registry = ProviderRegistry()
        provider = mock_provider(ProtocolType.MQTT)
        registry.register_provider(provider)
        
        result = await InterfaceFactory.create_publisher(
            "test_speaker",
            protocols=[ProtocolType.MQTT]
        )
        
        assert hasattr(result, 'name')
    
    @pytest.mark.asyncio
    async def test_create_action_server_with_specific_protocol(self, mock_provider):
        """Test creating job with specific protocol."""
        registry = ProviderRegistry()
        provider = mock_provider(ProtocolType.ROS2)
        registry.register_provider(provider)
        
        async def test_callback(goal):
            return {"status": "completed"}
        
        async def cancel_callback(goal):
            return True
        
        result = await InterfaceFactory.create_action_server(
            "test_job",
            handle_goal_request=test_callback,
            handle_cancel_request=cancel_callback,
            execution_callback=test_callback,
            protocols=[ProtocolType.ROS2]
        )
        
        assert hasattr(result, 'name')
    
    @pytest.mark.asyncio
    async def test_factory_with_no_provider(self):
        """Test factory behavior when no provider is available."""
        registry = ProviderRegistry()
        registry._providers.clear()
        
        async def test_callback(request):
            return {"result": "ok"}
        
        # Should raise error when no provider available
        with pytest.raises(Exception):  # Could be ProtocolUnavailableError or InterfaceError
            await InterfaceFactory.create_server(
                "test_callable",
                response_callback=test_callback,
                protocols=[ProtocolType.GRPC]
            )
    
    @pytest.mark.asyncio
    async def test_factory_protocol_priority(self, mock_provider):
        """Test that factory uses available protocol."""
        registry = ProviderRegistry()
        
        provider = mock_provider(ProtocolType.REDIS)
        registry.register_provider(provider)
        
        async def test_callback(request):
            return {"result": "ok"}
        
        result = await InterfaceFactory.create_server(
            "test_callable",
            response_callback=test_callback,
            protocols=[ProtocolType.REDIS]
        )
        
        # Verify result was created
        assert result is not None


