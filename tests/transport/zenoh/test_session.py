"""
Unit tests for Zenoh session management.
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock

from vyra_base.com.transport.t_zenoh.session import (
    ZenohSession,
    SessionConfig,
    SessionMode,
    ZENOH_AVAILABLE
)


@pytest.mark.unit
@pytest.mark.skipif(not ZENOH_AVAILABLE, reason="Zenoh not available")
class TestZenohSession:
    """Test Zenoh session lifecycle."""
    
    @pytest.fixture
    def session_config(self):
        """Create test session configuration."""
        return SessionConfig(
            mode=SessionMode.PEER,
            connect=["tcp/localhost:7447"],
            timeout_ms=1000
        )
    
    def test_session_config_creation(self, session_config):
        """Test SessionConfig creation."""
        assert session_config.mode == SessionMode.PEER
        assert session_config.connect == ["tcp/localhost:7447"]
        assert session_config.timeout_ms == 1000
    
    def test_session_config_to_zenoh_config(self, session_config):
        """Test conversion to Zenoh config dict."""
        config = session_config.to_zenoh_config()
        
        assert config["mode"] == "peer"
        assert config["connect"]["endpoints"] == ["tcp/localhost:7447"]
        assert config["scouting"]["multicast"]["enabled"] is True
    
    @pytest.mark.asyncio
    async def test_session_creation(self, session_config):
        """Test ZenohSession creation."""
        session = ZenohSession(session_config)
        
        assert not session.is_open
        assert session.config == session_config
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.session.zenoh')
    async def test_session_open(self, mock_zenoh, session_config):
        """Test session opening."""
        # Mock Zenoh session
        mock_zenoh_session = Mock()
        mock_zenoh_session.info().zid.return_value = "test_session_id"
        mock_zenoh.open.return_value = mock_zenoh_session
        mock_zenoh.Config.from_json5.return_value = Mock()
        
        session = ZenohSession(session_config)
        result = await session.open()
        
        assert result is True
        assert session.is_open
        assert session._session == mock_zenoh_session
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.session.zenoh')
    async def test_session_close(self, mock_zenoh, session_config):
        """Test session closing."""
        # Mock Zenoh session
        mock_zenoh_session = Mock()
        mock_zenoh_session.info().zid.return_value = "test_session_id"
        mock_zenoh_session.close = Mock()
        mock_zenoh.open.return_value = mock_zenoh_session
        mock_zenoh.Config.from_json5.return_value = Mock()
        
        session = ZenohSession(session_config)
        await session.open()
        
        assert session.is_open
        
        await session.close()
        
        assert not session.is_open
        mock_zenoh_session.close.assert_called_once()
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.session.zenoh')
    async def test_session_double_open(self, mock_zenoh, session_config):
        """Test opening already open session."""
        mock_zenoh_session = Mock()
        mock_zenoh_session.info().zid.return_value = "test_session_id"
        mock_zenoh.open.return_value = mock_zenoh_session
        mock_zenoh.Config.from_json5.return_value = Mock()
        
        session = ZenohSession(session_config)
        await session.open()
        result = await session.open()  # Second open
        
        assert result is True
        assert session.is_open
    
    @pytest.mark.asyncio
    async def test_session_double_close(self, session_config):
        """Test closing already closed session."""
        session = ZenohSession(session_config)
        await session.close()  # Close without opening
        await session.close()  # Double close
        
        assert not session.is_open
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.session.zenoh')
    async def test_declare_publisher(self, mock_zenoh, session_config):
        """Test declaring a publisher."""
        mock_zenoh_session = Mock()
        mock_zenoh_session.info().zid.return_value = "test_session_id"
        mock_publisher = Mock()
        mock_zenoh_session.declare_publisher.return_value = mock_publisher
        mock_zenoh.open.return_value = mock_zenoh_session
        mock_zenoh.Config.from_json5.return_value = Mock()
        
        session = ZenohSession(session_config)
        await session.open()
        
        publisher = session.declare_publisher("/test/topic")
        
        assert publisher == mock_publisher
        mock_zenoh_session.declare_publisher.assert_called_once_with("/test/topic")
    
    @pytest.mark.asyncio
    @patch('vyra_base.com.transport.zenoh.session.zenoh')
    async def test_declare_subscriber(self, mock_zenoh, session_config):
        """Test declaring a subscriber."""
        mock_zenoh_session = Mock()
        mock_zenoh_session.info().zid.return_value = "test_session_id"
        mock_subscriber = Mock()
        mock_zenoh_session.declare_subscriber.return_value = mock_subscriber
        mock_zenoh.open.return_value = mock_zenoh_session
        mock_zenoh.Config.from_json5.return_value = Mock()
        
        session = ZenohSession(session_config)
        await session.open()
        
        callback = Mock()
        subscriber = session.declare_subscriber("/test/topic", callback)
        
        assert subscriber == mock_subscriber
        mock_zenoh_session.declare_subscriber.assert_called_once_with(
            "/test/topic", callback
        )
    
    @pytest.mark.asyncio
    async def test_declare_publisher_not_open(self, session_config):
        """Test declaring publisher on closed session."""
        session = ZenohSession(session_config)
        
        with pytest.raises(RuntimeError, match="Session not open"):
            session.declare_publisher("/test/topic")
    
    @pytest.mark.asyncio
    async def test_context_manager(self, session_config):
        """Test async context manager."""
        with patch('vyra_base.com.transport.zenoh.session.zenoh') as mock_zenoh:
            mock_zenoh_session = Mock()
            mock_zenoh_session.info().zid.return_value = "test_session_id"
            mock_zenoh_session.close = Mock()
            mock_zenoh.open.return_value = mock_zenoh_session
            mock_zenoh.Config.from_json5.return_value = Mock()
            
            async with ZenohSession(session_config) as session:
                assert session.is_open
            
            # Should be closed after context exit
            assert not session.is_open


@pytest.mark.unit
class TestSessionMode:
    """Test SessionMode enum."""
    
    def test_session_modes(self):
        """Test all session modes."""
        assert SessionMode.PEER.value == "peer"
        assert SessionMode.CLIENT.value == "client"
        assert SessionMode.ROUTER.value == "router"
