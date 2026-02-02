"""
Unit tests for Redis external protocol.
"""
import pytest
from unittest.mock import Mock, AsyncMock, patch
from vyra_base.com.external.redis.provider import RedisProvider
from vyra_base.com.core.types import ProtocolType


@pytest.mark.asyncio
class TestRedisProvider:
    """Test RedisProvider."""
    
    @pytest.fixture
    def mock_redis(self):
        """Create mock Redis client."""
        redis_mock = AsyncMock()
        redis_mock.ping = AsyncMock(return_value=True)
        redis_mock.close = AsyncMock()
        return redis_mock
    
    @pytest.fixture
    async def provider(self, mock_redis):
        """Create provider with mocked Redis."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(
                host="localhost",
                port=6379,
                db=0
            )
            await provider.initialize()
            yield provider
            await provider.shutdown()
    
    async def test_provider_initialization(self, mock_redis):
        """Test provider can be initialized."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(host="localhost", port=6379)
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
        speaker = await provider.create_speaker("test_channel")
        
        assert speaker is not None
    
    async def test_connection_params(self):
        """Test provider with different connection parameters."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            mock_redis = AsyncMock()
            mock_redis.ping = AsyncMock(return_value=True)
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(
                host="redis.example.com",
                port=6380,
                db=1,
                password="secret"
            )
            await provider.initialize()
            
            # Verify connection string was built correctly
            redis_module.from_url.assert_called_once()
            call_args = redis_module.from_url.call_args[0][0]
            assert "redis.example.com" in call_args
            assert "6380" in call_args
            
            await provider.shutdown()
    
    async def test_ssl_connection(self):
        """Test provider with SSL enabled."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            mock_redis = AsyncMock()
            mock_redis.ping = AsyncMock(return_value=True)
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(
                host="localhost",
                port=6379,
                ssl=True
            )
            await provider.initialize()
            
            # Verify SSL was configured
            call_args = redis_module.from_url.call_args[0][0]
            assert "rediss://" in call_args  # SSL connection string
            
            await provider.shutdown()


@pytest.mark.asyncio
class TestRedisCallable:
    """Test RedisCallable."""
    
    @pytest.fixture
    def mock_redis(self):
        """Create mock Redis client."""
        redis_mock = AsyncMock()
        redis_mock.ping = AsyncMock(return_value=True)
        redis_mock.close = AsyncMock()
        redis_mock.get = AsyncMock(return_value=None)
        redis_mock.set = AsyncMock(return_value=True)
        redis_mock.delete = AsyncMock(return_value=1)
        return redis_mock
    
    async def test_callable_creation(self, mock_redis):
        """Test callable can be created."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(host="localhost", port=6379)
            await provider.initialize()
            
            callable_instance = await provider.create_callable("test_service")
            
            assert callable_instance is not None
            
            await provider.shutdown()
    
    async def test_callable_call_method(self, mock_redis):
        """Test callable has call method."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(host="localhost", port=6379)
            await provider.initialize()
            
            callable_instance = await provider.create_callable("compute_service")
            
            # Should have call method
            assert hasattr(callable_instance, 'call')
            
            await provider.shutdown()


@pytest.mark.asyncio
class TestRedisSpeaker:
    """Test RedisSpeaker."""
    
    @pytest.fixture
    def mock_redis(self):
        """Create mock Redis client."""
        redis_mock = AsyncMock()
        redis_mock.ping = AsyncMock(return_value=True)
        redis_mock.close = AsyncMock()
        redis_mock.publish = AsyncMock(return_value=1)
        return redis_mock
    
    async def test_speaker_creation(self, mock_redis):
        """Test speaker can be created."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(host="localhost", port=6379)
            await provider.initialize()
            
            speaker = await provider.create_speaker("test_channel")
            
            assert speaker is not None
            
            await provider.shutdown()
    
    async def test_speaker_send_method(self, mock_redis):
        """Test speaker has send method."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(host="localhost", port=6379)
            await provider.initialize()
            
            speaker = await provider.create_speaker("status_channel")
            
            # Should have send method
            assert hasattr(speaker, 'send')
            
            await provider.shutdown()
    
    async def test_speaker_publish(self, mock_redis):
        """Test speaker can publish messages."""
        with patch('vyra_base.com.external.redis.provider.redis') as redis_module:
            redis_module.from_url = AsyncMock(return_value=mock_redis)
            
            provider = RedisProvider(host="localhost", port=6379)
            await provider.initialize()
            
            speaker = await provider.create_speaker("events_channel")
            
            # Publish message
            if hasattr(speaker, 'send'):
                await speaker.send({"event": "test"})
                mock_redis.publish.assert_called()
            
            await provider.shutdown()
