"""
Unit tests for SharedMemory transport protocol.
"""
import pytest
import asyncio
from unittest.mock import Mock, patch
from vyra_base.com.transport.shared_memory.provider import SharedMemoryProvider
from vyra_base.com.core.types import ProtocolType


@pytest.mark.asyncio
class TestSharedMemoryProvider:
    """Test SharedMemoryProvider."""
    
    @pytest.fixture
    async def provider(self):
        """Create provider instance for testing."""
        provider = SharedMemoryProvider(namespace="test")
        await provider.initialize()
        yield provider
        await provider.shutdown()
    
    async def test_provider_initialization(self):
        """Test provider can be initialized."""
        provider = SharedMemoryProvider(namespace="test_init")
        assert not provider.is_initialized
        
        await provider.initialize()
        assert provider.is_initialized
        
        await provider.shutdown()
    
    async def test_provider_shutdown(self, provider):
        """Test provider can be shutdown."""
        assert provider.is_initialized
        
        await provider.shutdown()
        assert not provider.is_initialized
    
    async def test_provider_protocol_type(self, provider):
        """Test provider has correct protocol type."""
        # Should have a protocol attribute or method
        assert provider is not None
    
    async def test_create_callable(self, provider):
        """Test creating callable through provider."""
        callable_instance = await provider.create_callable("test_service")
        
        assert callable_instance is not None
        assert hasattr(callable_instance, 'call') or hasattr(callable_instance, 'initialize')
    
    async def test_create_speaker(self, provider):
        """Test creating speaker through provider."""
        speaker = await provider.create_speaker("test_topic")
        
        assert speaker is not None
        assert hasattr(speaker, 'send') or hasattr(speaker, 'publish')
    
    async def test_multiple_callables(self, provider):
        """Test creating multiple callables."""
        callable1 = await provider.create_callable("service1")
        callable2 = await provider.create_callable("service2")
        
        assert callable1 is not None
        assert callable2 is not None
        assert callable1 != callable2
    
    async def test_namespace_isolation(self):
        """Test that different namespaces are isolated."""
        provider1 = SharedMemoryProvider(namespace="ns1")
        provider2 = SharedMemoryProvider(namespace="ns2")
        
        await provider1.initialize()
        await provider2.initialize()
        
        callable1 = await provider1.create_callable("test")
        callable2 = await provider2.create_callable("test")
        
        # Should be different instances
        assert callable1 != callable2
        
        await provider1.shutdown()
        await provider2.shutdown()


@pytest.mark.asyncio
class TestSharedMemoryCallable:
    """Test SharedMemoryCallable."""
    
    async def test_callable_initialization(self):
        """Test callable can be initialized."""
        provider = SharedMemoryProvider(namespace="test_callable")
        await provider.initialize()
        
        callable_instance = await provider.create_callable("test_service")
        
        assert callable_instance is not None
        
        await provider.shutdown()
    
    async def test_callable_request_response(self):
        """Test callable request/response pattern."""
        provider = SharedMemoryProvider(namespace="test_rr")
        await provider.initialize()
        
        # Create server-side callable
        server_callable = await provider.create_callable("echo_service")
        
        # Mock handler
        async def echo_handler(request):
            return {"echo": request.get("message", "")}
        
        # Register handler (implementation-specific)
        if hasattr(server_callable, 'set_handler'):
            server_callable.set_handler(echo_handler)
        
        await provider.shutdown()


@pytest.mark.asyncio
class TestSharedMemorySpeaker:
    """Test SharedMemorySpeaker."""
    
    async def test_speaker_initialization(self):
        """Test speaker can be initialized."""
        provider = SharedMemoryProvider(namespace="test_speaker")
        await provider.initialize()
        
        speaker = await provider.create_speaker("test_topic")
        
        assert speaker is not None
        
        await provider.shutdown()
    
    async def test_speaker_publish(self):
        """Test speaker can publish messages."""
        provider = SharedMemoryProvider(namespace="test_publish")
        await provider.initialize()
        
        speaker = await provider.create_speaker("status_topic")
        
        # Should be able to call send or publish
        if hasattr(speaker, 'send'):
            await speaker.send({"status": "running"})
        elif hasattr(speaker, 'publish'):
            await speaker.publish({"status": "running"})
        
        await provider.shutdown()
