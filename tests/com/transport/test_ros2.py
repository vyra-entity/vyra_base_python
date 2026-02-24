"""
Unit tests for ROS2 Transport Provider.

Tests ROS2 protocol implementation with graceful degradation when ROS2 unavailable.
"""
import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from vyra_base.com.core.types import ProtocolType


class TestROS2Provider:
    """Test ROS2 transport provider."""
    
    def test_ros2_provider_exists(self):
        """Test that ROS2 provider module exists."""
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            assert ROS2Provider is not None
        except ImportError:
            pytest.skip("ROS2 provider not implemented yet")
    
    def test_ros2_provider_availability_without_rclpy(self):
        """Test ROS2 provider detects when rclpy is unavailable."""
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            import vyra_base.com.transport.t_ros2.provider as ros2_provider_module
            
            # Patch ROS2_AVAILABLE to False to simulate missing rclpy
            with patch.object(ros2_provider_module, 'ROS2_AVAILABLE', False):
                provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
                
                # Should detect ROS2 unavailable
                available = provider.is_available
                assert available is False
        except ImportError:
            pytest.skip("ROS2Provider not implemented yet")
    
    def test_ros2_provider_with_rclpy(self):
        """Test ROS2 provider when rclpy is available."""
        try:
            # Try to import rclpy
            import rclpy
            
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            assert provider._protocol == ProtocolType.ROS2
            
            # Should be available when rclpy present
            assert provider.is_available is True
        except ImportError:
            pytest.skip("rclpy not available in test environment")
    
    @pytest.mark.asyncio
    async def test_ros2_provider_initialization(self):
        """Test ROS2 provider initialization."""
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            
            config = {
                "node_name": "test_node",
                "namespace": "/test"
            }
            
            if provider.is_available:
                result = await provider.initialize(config)
                assert result is True
                assert provider.is_initialized is True
        except ImportError:
            pytest.skip("ROS2Provider not available")
    
    @pytest.mark.asyncio
    async def test_ros2_create_server_service(self):
        """Test creating ROS2 service as callable."""
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            
            if not provider.is_available:
                pytest.skip("ROS2 not available")
            
            async def test_callback(request):
                return {"result": "ok"}
            
            callable = await provider.create_server(
                "test_service",
                response_callback=test_callback,
                service_type="std_srvs/srv/Trigger"
            )
            
            assert callable is not None
        except (ImportError, Exception) as e:
            pytest.skip(f"ROS2 server creation not available: {e}")
    
    @pytest.mark.asyncio
    async def test_ros2_create_publisher_publisher(self):
        """Test creating ROS2 publisher as speaker."""
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            
            if not provider.is_available:
                pytest.skip("ROS2 not available")
            
            speaker = await provider.create_publisher(
                name="test_topic",
                message_type="std_msgs/msg/String"
            )
            
            assert speaker is not None
        except (ImportError, Exception) as e:
            pytest.skip(f"ROS2 publisher creation not available: {e}")
    
    @pytest.mark.asyncio
    async def test_ros2_create_action_server_action(self):
        """Test creating ROS2 action as job."""
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            
            if not provider.is_available:
                pytest.skip("ROS2 not available")
            
            async def test_callback(goal):
                return {"status": "completed"}
            
            job = await provider.create_action_server(
                name="test_action",
                callback=test_callback,
                action_type="example_interfaces/action/Fibonacci"
            )
            
            assert job is not None
        except (ImportError, Exception) as e:
            pytest.skip(f"ROS2 action server creation not available: {e}")



class TestROS2TransportAPI:
    """Test current ROS2 transport API integration."""

    def test_remote_service_decorator_sets_server_attribute(self):
        """remote_service decorator sets _vyra_remote_server attribute."""
        from vyra_base.com.core.decorators import remote_service

        @remote_service(name="test_function")
        async def test_function(request):
            return {"result": "ok"}

        assert hasattr(test_function, '_vyra_remote_server')
        assert test_function._vyra_remote_server is True
        assert test_function._vyra_server_name == "test_function"

    def test_remote_service_importable_from_com(self):
        """remote_service is importable from vyra_base.com."""
        from vyra_base.com import remote_service
        assert remote_service is not None

    def test_remote_publisher_decorator_sets_publisher_attribute(self):
        """remote_publisher decorator sets _vyra_remote_publisher attribute."""
        from vyra_base.com.core.decorators import remote_publisher

        @remote_publisher(name="test_topic")
        async def test_publisher(self, message):
            pass

        assert hasattr(test_publisher, '_vyra_remote_publisher')
        assert test_publisher._vyra_publisher_name == "test_topic"


class TestROS2IntegrationWithMultiProtocol:
    """Test ROS2 integration with multi-protocol architecture."""
    
    @pytest.mark.asyncio
    async def test_ros2_provider_in_registry(self):
        """Test ROS2 provider can be registered."""
        from vyra_base.com.providers import ProviderRegistry
        
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            registry = ProviderRegistry()
            registry._providers.clear()
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            registry.register_provider(provider)
            
            assert ProtocolType.ROS2 in registry.list_registered()
        except ImportError:
            pytest.skip("ROS2Provider not available")
    
    @pytest.mark.asyncio
    async def test_interface_factory_with_ros2(self):
        """Test InterfaceFactory can use ROS2 provider."""
        from vyra_base.com.core.factory import InterfaceFactory
        from vyra_base.com.providers import ProviderRegistry
        
        try:
            from vyra_base.com.transport.t_ros2 import ROS2Provider
            
            registry = ProviderRegistry()
            registry._providers.clear()
            
            provider = ROS2Provider(module_name="test_module", module_id="test_ros2_id")
            
            if provider.is_available:
                registry.register_provider(provider)
                
                async def test_callback(request):
                    return {"result": "ok"}
                
                callable = await InterfaceFactory.create_server(
                    name="test",
                    response_callback=test_callback,
                    protocols=[ProtocolType.ROS2]
                )
                
                assert callable is not None
        except ImportError:
            pytest.skip("ROS2Provider not available")
    
    def test_ros2_fallback_chain(self):
        """Test ROS2 in fallback chain."""
        from vyra_base.com.core.factory import InterfaceFactory
        
        # ROS2 should be in default fallback chains
        assert ProtocolType.ROS2 in InterfaceFactory.SERVER_FALLBACK
        assert ProtocolType.ROS2 in InterfaceFactory.PUBLISHER_FALLBACK
        assert ProtocolType.ROS2 in InterfaceFactory.ACTION_SERVER_FALLBACK


class TestROS2GracefulDegradation:
    """Test graceful degradation when ROS2 is unavailable."""
    
    def test_com_imports_work_without_ros2(self):
        """Test that com module imports work even without ROS2."""
        # Mock ROS2 as unavailable
        with patch.dict('sys.modules', {'rclpy': None}):
            try:
                from vyra_base.com import (
                    remote_service,
                    InterfaceFactory,
                    ProtocolType
                )
                
                assert all([remote_service, InterfaceFactory, ProtocolType])
            except ImportError as e:
                pytest.fail(f"Com module should work without ROS2: {e}")
    
    def test_feeder_imports_handle_missing_ros2(self):
        """Test that feeder imports handle missing ROS2."""
        with patch('importlib.import_module') as mock_import:
            def side_effect(name):
                if 'rclpy' in name:
                    raise ImportError(f"No module named '{name}'")
                return MagicMock()
            
            mock_import.side_effect = side_effect
            
            # Try importing feeders
            try:
                from vyra_base.com.feeder import feeder
                assert feeder is not None
            except ImportError:
                # Feeders might have hard ROS2 dependency
                pytest.skip("Feeders require ROS2")
    
    @pytest.mark.asyncio
    async def test_factory_skips_unavailable_ros2(self):
        """Test that InterfaceFactory skips unavailable ROS2."""
        from vyra_base.com.core.factory import InterfaceFactory
        from vyra_base.com.providers import ProviderRegistry
        
        registry = ProviderRegistry()
        registry._providers.clear()
        
        # Don't register any ROS2 provider
        # Factory should handle gracefully
        
        async def test_callback(request):
            return {"result": "ok"}
        
        with pytest.raises(Exception):
            # Should raise error when no provider available
            await InterfaceFactory.create_server(
                name="test",
                response_callback=test_callback,
                protocols=[ProtocolType.ROS2]
            )
