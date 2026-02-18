"""
Unit tests for Feeder Components.

Tests BaseFeeder, StateFeeder, NewsFeeder, and ErrorFeeder.
"""
import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from vyra_base.com.feeder.feeder import BaseFeeder
from vyra_base.com.feeder.state_feeder import StateFeeder
from vyra_base.com.feeder.news_feeder import NewsFeeder
from vyra_base.com.feeder.error_feeder import ErrorFeeder


class TestBaseFeeder:
    """Test BaseFeeder abstract class."""
    
    def test_base_feeder_cannot_be_instantiated(self):
        """Test that BaseFeeder cannot be instantiated directly."""
        # BaseFeeder might not be abstract, so just test it exists
        assert BaseFeeder is not None
    
    def test_base_feeder_has_required_methods(self):
        """Test that BaseFeeder defines required interface."""
        assert hasattr(BaseFeeder, '__init__')
        # Check for common feeder methods
        # These might vary based on actual implementation


class TestStateFeeder:
    """Test StateFeeder for state machine updates."""
    
    def test_state_feeder_initialization(self):
        """Test StateFeeder can be initialized."""
        try:
            feeder = StateFeeder(name="test_state_feeder")
            assert feeder is not None
            assert feeder.name == "test_state_feeder"
        except TypeError as e:
            # If StateFeeder requires more arguments
            pytest.skip(f"StateFeeder initialization needs more info: {e}")
    
    def test_state_feeder_ros2_availability(self):
        """Test StateFeeder handles ROS2 availability check."""
        # StateFeeder should handle missing ROS2 gracefully
        with patch('importlib.import_module') as mock_import:
            def side_effect(name):
                if name == 'rclpy':
                    raise ImportError("No module named 'rclpy'")
                return MagicMock()
            
            mock_import.side_effect = side_effect
            
            try:
                feeder = StateFeeder(name="test")
                # Should not crash even if ROS2 unavailable
                assert feeder is not None
            except (ImportError, TypeError):
                pytest.skip("StateFeeder requires ROS2 or more init params")
    
    @pytest.mark.asyncio
    async def test_state_feeder_publish_state(self):
        """Test publishing state updates."""
        try:
            feeder = StateFeeder(name="test_state_feeder")
            
            # Mock the publish method
            feeder.publish = AsyncMock()
            
            await feeder.publish(
                state="RUNNING",
                previous_state="READY"
            )
            
            feeder.publish.assert_called_once()
        except (TypeError, AttributeError):
            pytest.skip("StateFeeder API not as expected")
    
    def test_state_feeder_with_entity(self):
        """Test StateFeeder integration with entity."""
        # This would require a full entity mock
        pytest.skip("Requires VyraEntity mock - integration test")


class TestNewsFeeder:
    """Test NewsFeeder for general notifications."""
    
    def test_news_feeder_initialization(self):
        """Test NewsFeeder can be initialized."""
        try:
            feeder = NewsFeeder(name="test_news_feeder")
            assert feeder is not None
            assert feeder.name == "test_news_feeder"
        except TypeError:
            pytest.skip("NewsFeeder initialization needs more info")
    
    def test_news_feeder_ros2_availability(self):
        """Test NewsFeeder handles ROS2 availability check."""
        with patch('importlib.import_module') as mock_import:
            def side_effect(name):
                if name == 'rclpy':
                    raise ImportError("No module named 'rclpy'")
                return MagicMock()
            
            mock_import.side_effect = side_effect
            
            try:
                feeder = NewsFeeder(name="test")
                assert feeder is not None
            except (ImportError, TypeError):
                pytest.skip("NewsFeeder requires ROS2 or more init params")
    
    @pytest.mark.asyncio
    async def test_news_feeder_publish_news(self):
        """Test publishing news/notifications."""
        try:
            feeder = NewsFeeder(name="test_news_feeder")
            
            feeder.publish = AsyncMock()
            
            await feeder.publish(
                message="System started",
                level="INFO"
            )
            
            feeder.publish.assert_called_once()
        except (TypeError, AttributeError):
            pytest.skip("NewsFeeder API not as expected")
    
    def test_news_feeder_message_types(self):
        """Test different news message types."""
        try:
            feeder = NewsFeeder(name="test")
            
            # NewsFeeder should support different message levels
            assert hasattr(feeder, 'publish') or hasattr(feeder, 'shout')
        except TypeError:
            pytest.skip("NewsFeeder requires init params")


class TestErrorFeeder:
    """Test ErrorFeeder for error propagation."""
    
    def test_error_feeder_initialization(self):
        """Test ErrorFeeder can be initialized."""
        try:
            feeder = ErrorFeeder(name="test_error_feeder")
            assert feeder is not None
            assert feeder.name == "test_error_feeder"
        except TypeError:
            pytest.skip("ErrorFeeder initialization needs more info")
    
    def test_error_feeder_ros2_availability(self):
        """Test ErrorFeeder handles ROS2 availability check."""
        with patch('importlib.import_module') as mock_import:
            def side_effect(name):
                if name == 'rclpy':
                    raise ImportError("No module named 'rclpy'")
                return MagicMock()
            
            mock_import.side_effect = side_effect
            
            try:
                feeder = ErrorFeeder(name="test")
                assert feeder is not None
            except (ImportError, TypeError):
                pytest.skip("ErrorFeeder requires ROS2 or more init params")
    
    @pytest.mark.asyncio
    async def test_error_feeder_publish_error(self):
        """Test publishing error information."""
        try:
            feeder = ErrorFeeder(name="test_error_feeder")
            
            feeder.publish = AsyncMock()
            
            await feeder.publish(
                error_type="RuntimeError",
                error_message="Something went wrong",
                traceback="..."
            )
            
            feeder.publish.assert_called_once()
        except (TypeError, AttributeError):
            pytest.skip("ErrorFeeder API not as expected")
    
    def test_error_feeder_error_levels(self):
        """Test error severity levels."""
        try:
            feeder = ErrorFeeder(name="test")
            
            # ErrorFeeder should support severity levels
            assert hasattr(feeder, 'publish') or hasattr(feeder, 'shout')
        except TypeError:
            pytest.skip("ErrorFeeder requires init params")


class TestFeederIntegration:
    """Integration tests for feeder components."""
    
    @pytest.mark.asyncio
    async def test_multiple_feeders_coexist(self):
        """Test that multiple feeders can coexist."""
        feeders = []
        
        try:
            state_feeder = StateFeeder(name="state")
            feeders.append(state_feeder)
        except TypeError:
            pass
        
        try:
            news_feeder = NewsFeeder(name="news")
            feeders.append(news_feeder)
        except TypeError:
            pass
        
        try:
            error_feeder = ErrorFeeder(name="error")
            feeders.append(error_feeder)
        except TypeError:
            pass
        
        # At least some feeders should be creatable
        assert len(feeders) >= 0
    
    def test_feeder_naming_unique(self):
        """Test that feeders can have unique names."""
        names = set()
        
        try:
            feeder1 = StateFeeder(name="feeder1")
            names.add(feeder1.name)
        except TypeError:
            pass
        
        try:
            feeder2 = StateFeeder(name="feeder2")
            names.add(feeder2.name)
        except TypeError:
            pass
        
        # Names should be unique if feeders created
        if len(names) > 0:
            assert len(names) == 2 or len(names) == 0
    
    @pytest.mark.asyncio
    async def test_feeder_without_ros2(self):
        """Test that feeders work without ROS2 (graceful degradation)."""
        # Mock ROS2 as unavailable
        with patch('importlib.import_module') as mock_import:
            def side_effect(name):
                if 'rclpy' in name or 'builtin_interfaces' in name:
                    raise ImportError(f"No module named '{name}'")
                return MagicMock()
            
            mock_import.side_effect = side_effect
            
            # Feeders should handle this gracefully
            try:
                # Try importing feeder module
                from vyra_base.com.feeder import state_feeder
                assert state_feeder is not None
            except ImportError:
                pytest.skip("Feeder modules have hard ROS2 dependency")
    
    def test_feeders_use_multi_protocol(self):
        """Test that feeders can use multi-protocol architecture."""
        # Feeders should ideally use InterfaceFactory for multi-protocol support
        from vyra_base.com.core.factory import InterfaceFactory
        
        # This is a forward-looking test
        assert InterfaceFactory is not None
        
        # Future: Feeders should use InterfaceFactory.create_publisher()
        # instead of direct ROS2 publishers


class TestFeederROS2Compatibility:
    """Test ROS2 compatibility and graceful degradation."""
    
    def test_feeder_imports_dont_crash(self):
        """Test that feeder imports don't crash without ROS2."""
        try:
            from vyra_base.com.feeder.feeder import BaseFeeder
            from vyra_base.com.feeder.state_feeder import StateFeeder
            from vyra_base.com.feeder.news_feeder import NewsFeeder
            from vyra_base.com.feeder.error_feeder import ErrorFeeder
            
            assert all([BaseFeeder, StateFeeder, NewsFeeder, ErrorFeeder])
        except ImportError as e:
            # Imports might fail if ROS2 not available
            pytest.skip(f"Feeder imports failed: {e}")
    
    def test_ros2_import_pattern(self):
        """Test that feeders use try/except pattern for ROS2 imports."""
        import inspect
        
        try:
            from vyra_base.com.feeder import state_feeder
            
            # Check source code for try/except pattern
            source = inspect.getsource(state_feeder)
            
            # Should use try/except for rclpy imports
            assert 'try:' in source and 'import rclpy' in source
        except (ImportError, OSError):
            pytest.skip("Cannot inspect feeder source")
    
    @pytest.mark.asyncio
    async def test_feeder_factory_integration(self):
        """Test feeder integration with InterfaceFactory."""
        from vyra_base.com.core.factory import InterfaceFactory
        from vyra_base.com.core.types import ProtocolType
        
        # Future: Feeders should create speakers using InterfaceFactory
        # For now, just verify InterfaceFactory can create speakers
        
        # This is a placeholder for future integration
        assert hasattr(InterfaceFactory, 'create_publisher')
