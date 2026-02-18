"""
Unit tests for AbstractProtocolProvider and ProviderRegistry.
"""
import pytest
from unittest.mock import Mock, AsyncMock
from vyra_base.com.providers import AbstractProtocolProvider, ProviderRegistry
from vyra_base.com.core.types import ProtocolType


class TestAbstractProtocolProvider:
    """Test AbstractProtocolProvider interface."""
    
    def test_abstract_provider_cannot_be_instantiated(self):
        """Test that abstract provider cannot be instantiated directly."""
        with pytest.raises(TypeError):
            AbstractProtocolProvider(ProtocolType.ROS2)
    
    def test_custom_provider_implementation(self):
        """Test creating a custom provider implementation."""
        class MockProvider(AbstractProtocolProvider):
            def __init__(self, protocol: ProtocolType):
                super().__init__(protocol)
                self._available = True
            
            async def check_availability(self) -> bool:
                return self._available
            
            def is_available(self) -> bool:
                return self._available
            
            async def initialize(self):
                self._initialized = True
            
            async def shutdown(self):
                self._initialized = False
            
            async def create_server(self, name: str, **kwargs):
                return Mock()
            
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        provider = MockProvider(ProtocolType.REDIS)
        assert isinstance(provider, AbstractProtocolProvider)
        assert provider.protocol == ProtocolType.REDIS
        assert hasattr(provider, 'initialize')
        assert hasattr(provider, 'create_server')


class TestProviderRegistry:
    """Test ProviderRegistry singleton."""
    
    @pytest.fixture(autouse=True)
    def clear_registry(self):
        """Clear registry before each test."""
        registry = ProviderRegistry()
        registry._providers.clear()
        yield
        registry._providers.clear()
    
    def test_registry_is_singleton(self):
        """Test that registry is a singleton."""
        registry1 = ProviderRegistry()
        registry2 = ProviderRegistry()
        assert registry1 is registry2
    
    def test_register_provider(self):
        """Test registering a provider."""
        class MockProvider(AbstractProtocolProvider):
            def __init__(self):
                super().__init__(ProtocolType.REDIS)
                self._available = True
            
            async def check_availability(self) -> bool:
                return self._available
            
            def is_available(self) -> bool:
                return self._available
            
            async def initialize(self):
                self._initialized = True
            
            async def shutdown(self):
                self._initialized = False
            
            async def create_server(self, name: str, **kwargs):
                return Mock()
            
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        registry = ProviderRegistry()
        mock_provider = MockProvider()
        
        registry.register_provider(mock_provider)
        
        assert ProtocolType.REDIS in registry.list_registered()
    
    def test_get_provider(self):
        """Test getting a registered provider."""
        class MockProvider(AbstractProtocolProvider):
            def __init__(self):
                super().__init__(ProtocolType.REDIS)
                self._available = True
            
            async def check_availability(self) -> bool:
                return self._available
            
            def is_available(self) -> bool:
                return self._available
            
            async def initialize(self):
                self._initialized = True
            
            async def shutdown(self):
                self._initialized = False
            
            async def create_server(self, name: str, **kwargs):
                return Mock()
            
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        registry = ProviderRegistry()
        mock_provider = MockProvider()
        
        registry.register_provider(mock_provider)
        retrieved = registry.get_provider(ProtocolType.REDIS)
        
        assert retrieved is mock_provider
    
    def test_get_nonexistent_provider(self):
        """Test getting a provider that doesn't exist raises error."""
        registry = ProviderRegistry()
        
        with pytest.raises(Exception):  # ProviderNotFoundError
            registry.get_provider(ProtocolType.GRPC)
    
    def test_unregister_provider(self):
        """Test unregistering a provider."""
        class MockProvider(AbstractProtocolProvider):
            def __init__(self):
                super().__init__(ProtocolType.REDIS)
                self._available = True
            
            async def check_availability(self) -> bool:
                return self._available
            
            def is_available(self) -> bool:
                return self._available
            
            async def initialize(self):
                self._initialized = True
            
            async def shutdown(self):
                self._initialized = False
            
            async def create_server(self, name: str, **kwargs):
                return Mock()
            
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        registry = ProviderRegistry()
        mock_provider = MockProvider()
        
        registry.register_provider(mock_provider)
        assert ProtocolType.REDIS in registry.list_registered()
        
        # Test unregister functionality
        result = registry.unregister_provider(ProtocolType.REDIS)
        assert result is True
        assert ProtocolType.REDIS not in registry.list_registered()
    
    def test_get_all_providers(self):
        """Test getting all registered providers."""
        class MockProvider1(AbstractProtocolProvider):
            def __init__(self):
                super().__init__(ProtocolType.ROS2)
                self._available = True
            async def check_availability(self) -> bool:
                return self._available
            def is_available(self) -> bool:
                return self._available
            async def initialize(self):
                self._initialized = True
            async def shutdown(self):
                self._initialized = False
            async def create_server(self, name: str, **kwargs):
                return Mock()
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        class MockProvider2(AbstractProtocolProvider):
            def __init__(self):
                super().__init__(ProtocolType.REDIS)
                self._available = True
            async def check_availability(self) -> bool:
                return self._available
            def is_available(self) -> bool:
                return self._available
            async def initialize(self):
                self._initialized = True
            async def shutdown(self):
                self._initialized = False
            async def create_server(self, name: str, **kwargs):
                return Mock()
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        registry = ProviderRegistry()
        provider1 = MockProvider1()
        provider2 = MockProvider2()
        
        registry.register_provider(provider1)
        registry.register_provider(provider2)
        
        all_protocols = registry.list_registered()
        
        assert len(all_protocols) == 2
        assert ProtocolType.ROS2 in all_protocols
        assert ProtocolType.REDIS in all_protocols
    
    def test_register_replaces_existing_provider(self):
        """Test that registering replaces existing provider with force=True."""
        class MockProvider(AbstractProtocolProvider):
            def __init__(self, name: str):
                super().__init__(ProtocolType.REDIS)
                self._name = name
                self._available = True
            async def check_availability(self) -> bool:
                return self._available
            def is_available(self) -> bool:
                return self._available
            async def initialize(self):
                self._initialized = True
            async def shutdown(self):
                self._initialized = False
            async def create_server(self, name: str, **kwargs):
                return Mock()
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        registry = ProviderRegistry()
        provider1 = MockProvider("first")
        provider2 = MockProvider("second")
        
        registry.register_provider(provider1)
        registry.register_provider(provider2, force=True)
        
        retrieved = registry.get_provider(ProtocolType.REDIS)
        assert retrieved is provider2
        assert retrieved is not provider1
    
    def test_list_available_protocols(self):
        """Test listing all available protocols."""
        class MockProvider1(AbstractProtocolProvider):
            def __init__(self, protocol):
                super().__init__(protocol)
                self._available = True
            async def check_availability(self) -> bool:
                return self._available
            def is_available(self) -> bool:
                return self._available
            async def initialize(self):
                self._initialized = True
            async def shutdown(self):
                self._initialized = False
            async def create_server(self, name: str, **kwargs):
                return Mock()
            async def create_publisher(self, name: str, **kwargs):
                return Mock()
            async def create_action_server(self, name: str, **kwargs):
                return Mock()
            async def create_subscriber(self, name: str, **kwargs):
                return Mock()
            
            async def create_client(self, name: str, **kwargs):
                return Mock()
            
            async def create_action_client(self, name: str, **kwargs):
                return Mock()
        
        registry = ProviderRegistry()
        
        registry.register_provider(MockProvider1(ProtocolType.ROS2))
        registry.register_provider(MockProvider1(ProtocolType.REDIS))
        registry.register_provider(MockProvider1(ProtocolType.MQTT))
        
        protocols = registry.list_registered()
        
        assert len(protocols) == 3
        assert ProtocolType.ROS2 in protocols
        assert ProtocolType.REDIS in protocols
        assert ProtocolType.MQTT in protocols

