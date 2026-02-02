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
            from vyra_base.com.transport.ros2 import ros2_provider
            assert ros2_provider is not None
        except ImportError:
            pytest.skip("ROS2 provider not implemented yet")
    
    def test_ros2_provider_availability_without_rclpy(self):
        """Test ROS2 provider detects when rclpy is unavailable."""
        with patch('importlib.import_module') as mock_import:
            mock_import.side_effect = ImportError("No module named 'rclpy'")
            
            try:
                from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
                provider = ROS2Provider()
                
                # Should detect ROS2 unavailable
                available = provider.is_available()
                assert available is False
            except ImportError:
                pytest.skip("ROS2Provider not implemented yet")
    
    def test_ros2_provider_with_rclpy(self):
        """Test ROS2 provider when rclpy is available."""
        try:
            # Try to import rclpy
            import rclpy
            
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            provider = ROS2Provider()
            assert provider.protocol == ProtocolType.ROS2
            
            # Should be available when rclpy present
            assert provider.is_available() is True
        except ImportError:
            pytest.skip("rclpy not available in test environment")
    
    @pytest.mark.asyncio
    async def test_ros2_provider_initialization(self):
        """Test ROS2 provider initialization."""
        try:
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            provider = ROS2Provider()
            
            config = {
                "node_name": "test_node",
                "namespace": "/test"
            }
            
            if provider.is_available():
                result = await provider.initialize(config)
                assert result is True
                assert provider.is_initialized() is True
        except ImportError:
            pytest.skip("ROS2Provider not available")
    
    @pytest.mark.asyncio
    async def test_ros2_create_callable_service(self):
        """Test creating ROS2 service as callable."""
        try:
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            provider = ROS2Provider()
            
            if not provider.is_available():
                pytest.skip("ROS2 not available")
            
            async def test_callback(request):
                return {"result": "ok"}
            
            callable = await provider.create_callable(
                name="test_service",
                callback=test_callback,
                service_type="std_srvs/srv/Trigger"
            )
            
            assert callable is not None
        except ImportError:
            pytest.skip("ROS2Provider not implemented")
    
    @pytest.mark.asyncio
    async def test_ros2_create_speaker_publisher(self):
        """Test creating ROS2 publisher as speaker."""
        try:
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            provider = ROS2Provider()
            
            if not provider.is_available():
                pytest.skip("ROS2 not available")
            
            speaker = await provider.create_speaker(
                name="test_topic",
                message_type="std_msgs/msg/String"
            )
            
            assert speaker is not None
        except ImportError:
            pytest.skip("ROS2Provider not implemented")
    
    @pytest.mark.asyncio
    async def test_ros2_create_job_action(self):
        """Test creating ROS2 action as job."""
        try:
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            provider = ROS2Provider()
            
            if not provider.is_available():
                pytest.skip("ROS2 not available")
            
            async def test_callback(goal):
                return {"status": "completed"}
            
            job = await provider.create_job(
                name="test_action",
                callback=test_callback,
                action_type="example_interfaces/action/Fibonacci"
            )
            
            assert job is not None
        except ImportError:
            pytest.skip("ROS2Provider not implemented")


class TestROS2Datalayer:
    """Test ROS2 datalayer (legacy) integration."""
    
    def test_datalayer_imports(self):
        """Test that datalayer imports work."""
        try:
            from vyra_base.com.datalayer.interface_factory import DataSpace
            from vyra_base.com.datalayer.callable import VyraCallable
            from vyra_base.com.datalayer.speaker import VyraSpeaker
            from vyra_base.com.datalayer.job import VyraJob
            
            assert all([DataSpace, VyraCallable, VyraSpeaker, VyraJob])
        except ImportError as e:
            pytest.skip(f"Datalayer imports failed: {e}")
    
    def test_dataspace_singleton(self):
        """Test DataSpace is a singleton-like registry."""
        try:
            from vyra_base.com.datalayer.interface_factory import DataSpace
            
            # DataSpace uses class methods (singleton pattern)
            assert hasattr(DataSpace, 'add_callable')
            assert hasattr(DataSpace, 'add_speaker')
            assert hasattr(DataSpace, 'add_job')
            assert hasattr(DataSpace, 'get_callable')
            assert hasattr(DataSpace, 'get_speaker')
            assert hasattr(DataSpace, 'get_job')
        except ImportError:
            pytest.skip("DataSpace not available")
    
    def test_vyra_callable_structure(self):
        """Test VyraCallable dataclass structure."""
        try:
            from vyra_base.com.datalayer.callable import VyraCallable
            
            callable = VyraCallable(
                name="test_callable",
                type=None,
                description="Test callable"
            )
            
            assert callable.name == "test_callable"
            assert callable.description == "Test callable"
        except ImportError:
            pytest.skip("VyraCallable not available")
    
    def test_vyra_speaker_structure(self):
        """Test VyraSpeaker dataclass structure."""
        try:
            from vyra_base.com.datalayer.speaker import VyraSpeaker
            
            speaker = VyraSpeaker(
                name="test_speaker",
                type=None,
                description="Test speaker"
            )
            
            assert speaker.name == "test_speaker"
            assert speaker.description == "Test speaker"
        except ImportError:
            pytest.skip("VyraSpeaker not available")
    
    def test_vyra_job_structure(self):
        """Test VyraJob dataclass structure."""
        try:
            from vyra_base.com.datalayer.job import VyraJob
            
            job = VyraJob(
                name="test_job",
                type=None,
                description="Test job"
            )
            
            assert job.name == "test_job"
            assert job.description == "Test job"
        except ImportError:
            pytest.skip("VyraJob not available")


class TestROS2BackwardCompatibility:
    """Test backward compatibility with old ROS2 datalayer API."""
    
    def test_remote_callable_decorator_datalayer(self):
        """Test remote_callable decorator from datalayer."""
        try:
            from vyra_base.com.core.decorators import remote_callable
            
            @remote_callable
            async def test_function(request):
                return {"result": "ok"}
            
            # Should have marker attribute
            assert hasattr(test_function, '_remote_callable')
        except ImportError:
            pytest.skip("Datalayer remote_callable not available")
    
    def test_remote_callable_decorator_new_api(self):
        """Test remote_callable decorator from new com API."""
        from vyra_base.com import remote_callable
        
        @remote_callable
        async def test_function(request):
            return {"result": "ok"}
        
        # Should have marker attribute
        assert hasattr(test_function, '_remote_callable')
    
    def test_decorators_are_same(self):
        """Test that old and new decorators are compatible."""
        try:
             from vyra_base.com.core.decorators import remote_callable as old_decorator
            from vyra_base.com import remote_callable as new_decorator
            
            # Both should exist
            assert old_decorator is not None
            assert new_decorator is not None
            
            # Both should work the same way
            @old_decorator
            async def old_func(req):
                return {}
            
            @new_decorator
            async def new_func(req):
                return {}
            
            assert hasattr(old_func, '_remote_callable')
            assert hasattr(new_func, '_remote_callable')
        except ImportError:
            pytest.skip("Cannot compare decorators")


class TestROS2IntegrationWithMultiProtocol:
    """Test ROS2 integration with multi-protocol architecture."""
    
    @pytest.mark.asyncio
    async def test_ros2_provider_in_registry(self):
        """Test ROS2 provider can be registered."""
        from vyra_base.com.providers import ProviderRegistry
        
        try:
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            registry = ProviderRegistry()
            registry._providers.clear()
            
            provider = ROS2Provider()
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
            from vyra_base.com.transport.ros2.ros2_provider import ROS2Provider
            
            registry = ProviderRegistry()
            registry._providers.clear()
            
            provider = ROS2Provider()
            
            if provider.is_available():
                registry.register_provider(provider)
                
                async def test_callback(request):
                    return {"result": "ok"}
                
                callable = await InterfaceFactory.create_callable(
                    name="test",
                    callback=test_callback,
                    protocols=[ProtocolType.ROS2]
                )
                
                assert callable is not None
        except ImportError:
            pytest.skip("ROS2Provider not available")
    
    def test_ros2_fallback_chain(self):
        """Test ROS2 in fallback chain."""
        from vyra_base.com.core.factory import InterfaceFactory
        
        # ROS2 should be in default fallback chains
        assert ProtocolType.ROS2 in InterfaceFactory.CALLABLE_FALLBACK
        assert ProtocolType.ROS2 in InterfaceFactory.SPEAKER_FALLBACK
        assert ProtocolType.ROS2 in InterfaceFactory.JOB_FALLBACK


class TestROS2GracefulDegradation:
    """Test graceful degradation when ROS2 is unavailable."""
    
    def test_com_imports_work_without_ros2(self):
        """Test that com module imports work even without ROS2."""
        # Mock ROS2 as unavailable
        with patch.dict('sys.modules', {'rclpy': None}):
            try:
                from vyra_base.com import (
                    remote_callable,
                    InterfaceFactory,
                    ProtocolType
                )
                
                assert all([remote_callable, InterfaceFactory, ProtocolType])
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
            await InterfaceFactory.create_callable(
                name="test",
                callback=test_callback,
                protocols=[ProtocolType.ROS2]
            )
