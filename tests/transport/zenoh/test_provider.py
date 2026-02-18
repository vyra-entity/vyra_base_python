"""
Unit tests for Zenoh provider.
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock

from vyra_base.com.transport.t_zenoh.provider import (
    ZenohProvider,
    ZENOH_AVAILABLE
)
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.exceptions import ProtocolUnavailableError, ProviderError


@pytest.mark.unit
@pytest.mark.skipif(not ZENOH_AVAILABLE, reason="Zenoh not available")
class TestZenohProvider:
    """Test Zenoh provider functionality."""
    
    @pytest.fixture
    def provider(self):
        """Create ZenohProvider instance."""
        return ZenohProvider(ProtocolType.ZENOH)
    
    def test_provider_creation(self, provider):
        """Test provider creation."""
        assert provider.protocol == ProtocolType.ZENOH
        assert not provider.is_initialized()
        assert provider._session is None
    
    @pytest.mark.asyncio
    async def test_check_availability(self, provider):
        """Test availability check."""
        available = await provider.check_availability()
        
        assert available == ZENOH_AVAILABLE
        assert provider.is_available() == ZENOH_AVAILABLE
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    async def test_initialize(self, mock_session_class, provider):
        """Test provider initialization."""
        # Mock session
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        config = {
            "mode": "client",
            "connect": ["tcp/localhost:7447"]
        }
        
        result = await provider.initialize(config)
        
        assert result is True
        assert provider.is_initialized()
        assert provider._session == mock_session
        mock_session.open.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_initialize_without_availability(self, provider):
        """Test initialization without availability."""
        provider._available = False
        
        with pytest.raises(ProtocolUnavailableError):
            await provider.initialize()
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    async def test_double_initialize(self, mock_session_class, provider):
        """Test double initialization."""
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        await provider.initialize()
        result = await provider.initialize()  # Second init
        
        assert result is True
        # open() should only be called once
        assert mock_session.open.call_count == 1
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    async def test_shutdown(self, mock_session_class, provider):
        """Test provider shutdown."""
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        await provider.initialize()
        await provider.shutdown()
        
        assert not provider.is_initialized()
        assert provider._session is None
        mock_session.close.assert_called_once()
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    @patch('vyra_base.com.transport.zenoh.provider.ZenohCallable')
    async def test_create_server(self, mock_callable_class, mock_session_class, provider):
        """Test creating a callable."""
        # Mock session
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        # Mock callable
        mock_callable = AsyncMock()
        mock_callable_class.return_value = mock_callable
        
        await provider.initialize()
        
        async def callback(req):
            return {"result": "ok"}
        
        callable = await provider.create_server("/test", callback)
        
        assert callable == mock_callable
        mock_callable.initialize.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_create_server_not_initialized(self, provider):
        """Test creating callable without initialization."""
        with pytest.raises(ProviderError, match="not initialized"):
            await provider.create_server("/test", Mock())
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSpeaker')
    async def test_create_publisher(self, mock_speaker_class, mock_session_class, provider):
        """Test creating a speaker."""
        # Mock session
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        # Mock speaker
        mock_speaker = AsyncMock()
        mock_speaker_class.return_value = mock_speaker
        
        await provider.initialize()
        
        speaker = await provider.create_publisher("/test")
        
        assert speaker == mock_speaker
        mock_speaker.initialize.assert_called_once()
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    @patch('vyra_base.com.transport.zenoh.provider.ZenohJob')
    async def test_create_action_server(self, mock_job_class, mock_session_class, provider):
        """Test creating a job."""
        # Mock session
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        # Mock job
        mock_job = AsyncMock()
        mock_job_class.return_value = mock_job
        
        await provider.initialize()
        
        async def callback(params):
            return {"result": "done"}
        
        job = await provider.create_action_server("/test_job", callback)
        
        assert job == mock_job
        mock_job.initialize.assert_called_once()
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.provider.ZenohSession')
    async def test_get_session(self, mock_session_class, provider):
        """Test getting session."""
        mock_session = AsyncMock()
        mock_session.is_open = True
        mock_session.session.info().zid.return_value = "test_id"
        mock_session_class.return_value = mock_session
        
        assert provider.get_session() is None
        
        await provider.initialize()
        
        assert provider.get_session() == mock_session


@pytest.mark.unit
class TestZenohProviderAvailability:
    """Test Zenoh availability checks."""
    
    @pytest.mark.skipif(ZENOH_AVAILABLE, reason="Test requires Zenoh to be unavailable")
    def test_zenoh_not_available(self):
        """Test behavior when Zenoh is not available."""
        assert not ZENOH_AVAILABLE
