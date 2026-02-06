"""
Unit tests for UDS (Unix Domain Socket) transport protocol.
"""
import pytest
import asyncio
import os
import tempfile
from vyra_base.com.transport.t_uds.provider import UDSProvider
from vyra_base.com.core.types import ProtocolType


@pytest.mark.asyncio
class TestUdsProvider:
    """Test UdsProvider."""
    
    @pytest.fixture
    async def socket_path(self):
        """Create temporary socket path."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "test.sock")
            yield path
    
    @pytest.fixture
    async def provider(self, socket_path):
        """Create provider instance for testing."""
        provider = UdsProvider(socket_path=socket_path)
        await provider.initialize()
        yield provider
        await provider.shutdown()
    
    async def test_provider_initialization(self, socket_path):
        """Test provider can be initialized."""
        provider = UdsProvider(socket_path=socket_path)
        assert not provider.is_initialized
        
        await provider.initialize()
        assert provider.is_initialized
        
        await provider.shutdown()
    
    async def test_provider_shutdown(self, provider):
        """Test provider can be shutdown."""
        assert provider.is_initialized
        
        await provider.shutdown()
        assert not provider.is_initialized
    
    async def test_create_callable(self, provider):
        """Test creating callable through provider."""
        callable_instance = await provider.create_callable("test_service")
        
        assert callable_instance is not None
    
    async def test_create_speaker(self, provider):
        """Test creating speaker through provider."""
        speaker = await provider.create_speaker("test_topic")
        
        assert speaker is not None
    
    async def test_multiple_services(self, provider):
        """Test creating multiple services."""
        service1 = await provider.create_callable("service1")
        service2 = await provider.create_callable("service2")
        
        assert service1 is not None
        assert service2 is not None


@pytest.mark.asyncio
class TestUdsCallable:
    """Test UdsCallable."""
    
    async def test_callable_creation(self, ):
        """Test callable can be created."""
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "test.sock")
            provider = UdsProvider(socket_path=socket_path)
            await provider.initialize()
            
            callable_instance = await provider.create_callable("test_service")
            
            assert callable_instance is not None
            
            await provider.shutdown()
    
    async def test_callable_lifecycle(self):
        """Test callable initialization and cleanup."""
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "lifecycle.sock")
            provider = UdsProvider(socket_path=socket_path)
            await provider.initialize()
            
            callable_instance = await provider.create_callable("lifecycle_service")
            
            # Should have initialization method
            if hasattr(callable_instance, 'initialize'):
                await callable_instance.initialize()
            
            # Should have cleanup method
            if hasattr(callable_instance, 'cleanup'):
                await callable_instance.cleanup()
            
            await provider.shutdown()


@pytest.mark.asyncio
class TestUdsSpeaker:
    """Test UdsSpeaker."""
    
    async def test_speaker_creation(self):
        """Test speaker can be created."""
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "speaker.sock")
            provider = UdsProvider(socket_path=socket_path)
            await provider.initialize()
            
            speaker = await provider.create_speaker("test_topic")
            
            assert speaker is not None
            
            await provider.shutdown()
    
    async def test_speaker_publish(self):
        """Test speaker can publish messages."""
        with tempfile.TemporaryDirectory() as tmpdir:
            socket_path = os.path.join(tmpdir, "publish.sock")
            provider = UdsProvider(socket_path=socket_path)
            await provider.initialize()
            
            speaker = await provider.create_speaker("data_topic")
            
            # Should be able to publish
            if hasattr(speaker, 'send'):
                await speaker.send({"data": "test"})
            elif hasattr(speaker, 'publish'):
                await speaker.publish({"data": "test"})
            
            await provider.shutdown()
