"""
Unit Tests for TopicBuilder Dynamic Interface Loading

Tests new dynamic interface loading features in TopicBuilder.
"""
import pytest
import json
import tempfile
import shutil
from pathlib import Path
from unittest.mock import patch, MagicMock

from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType


class TestTopicBuilderDynamic:
    """Test suite for TopicBuilder dynamic interface loading."""
    
    @pytest.fixture
    def mock_interface_paths(self):
        """Create mock interface directory structure."""
        temp_dir = Path(tempfile.mkdtemp())
        
        interface_dir = temp_dir / "test_interfaces"
        config_dir = interface_dir / "config"
        proto_dir = interface_dir / "proto"
        
        config_dir.mkdir(parents=True)
        proto_dir.mkdir(parents=True)
        
        # Create mock JSON metadata
        metadata = {
            "functionname": "get_modules",
            "type": "callable",
            "filetype": ["GetModules.srv", "GetModules.proto"],
            "displaystyle": {"visible": True}
        }
        
        (config_dir / "test_meta.json").write_text(json.dumps([metadata]))
        
        # Create mock protobuf file
        (proto_dir / "GetModules_pb2.py").write_text(
            "# Mock protobuf\nclass Request:\n    pass\n"
        )
        
        yield interface_dir
        
        shutil.rmtree(temp_dir)
    
    def test_init_with_interface_loading_enabled(self, mock_interface_paths):
        """Test initialization with interface loading enabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)],
            enable_interface_loading=True
        )
        
        assert builder._interface_loader is not None
        assert builder.module_name == "v2_modulemanager"
        assert builder.module_id == "abc123"
    
    def test_init_with_interface_loading_disabled(self):
        """Test initialization with interface loading disabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        assert builder._interface_loader is None
    
    def test_init_backward_compatibility(self):
        """Test that old initialization style still works."""
        # Old style: just module_name and module_id
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        # Should work and have interface loader enabled by default
        assert builder.module_name == "v2_modulemanager"
        assert builder.module_id == "abc123"
        assert builder._interface_loader is not None
    
    def test_build_topic_name(self):
        """Test pure topic naming without interface loading."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        topic = builder.build_topic_name("get_modules")
        
        assert topic == "v2_modulemanager_abc123/get_modules"
    
    def test_build_topic_name_with_subaction(self):
        """Test topic naming with subaction."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        topic = builder.build_topic_name("set_config", subaction="theme")
        
        assert topic == "v2_modulemanager_abc123/set_config/theme"
    
    def test_build_method_backward_compatibility(self):
        """Test that old build() method still works as before."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        topic = builder.build("get_modules")
        
        assert topic == "v2_modulemanager_abc123/get_modules"
    
    def test_load_interface_type_loader_disabled(self):
        """Test loading interface when loader is disabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        interface = builder.load_interface_type("get_modules", protocol="ros2")
        
        assert interface is None  # Should return None gracefully
    
    @patch('vyra_base.com.core.interface_loader.InterfaceLoader.get_interface_for_function')
    def test_load_interface_type_ros2(self, mock_get_interface, mock_interface_paths):
        """Test loading ROS2 interface type."""
        mock_type = MagicMock()
        mock_get_interface.return_value = mock_type
        
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        interface = builder.load_interface_type("get_modules", protocol="ros2")
        
        assert interface == mock_type
        mock_get_interface.assert_called_once_with("get_modules", "ros2")
    
    @patch('vyra_base.com.core.interface_loader.InterfaceLoader.get_interface_for_function')
    def test_load_interface_type_protobuf(self, mock_get_interface, mock_interface_paths):
        """Test loading protobuf interface."""
        mock_module = MagicMock()
        mock_get_interface.return_value = mock_module
        
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        interface = builder.load_interface_type("get_modules", protocol="zenoh")
        
        assert interface == mock_module
        mock_get_interface.assert_called_once_with("get_modules", "zenoh")
    
    @patch('vyra_base.com.core.interface_loader.InterfaceLoader.get_interface_for_function')
    def test_build_with_interface(self, mock_get_interface, mock_interface_paths):
        """Test building topic name and loading interface together."""
        mock_type = MagicMock()
        mock_get_interface.return_value = mock_type
        
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        topic, interface = builder.build_with_interface(
            "get_modules",
            interface_type=InterfaceType.CALLABLE,
            protocol="ros2"
        )
        
        assert topic == "v2_modulemanager_abc123/get_modules"
        assert interface == mock_type
    
    @patch('vyra_base.com.core.interface_loader.InterfaceLoader.get_interface_for_function')
    def test_build_with_interface_with_subaction(self, mock_get_interface, mock_interface_paths):
        """Test build_with_interface with subaction."""
        mock_type = MagicMock()
        mock_get_interface.return_value = mock_type
        
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        topic, interface = builder.build_with_interface(
            "set_config",
            subaction="theme",
            protocol="ros2"
        )
        
        assert topic == "v2_modulemanager_abc123/set_config/theme"
        assert interface == mock_type
    
    def test_build_with_interface_loader_disabled(self):
        """Test build_with_interface when loader is disabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        topic, interface = builder.build_with_interface("get_modules")
        
        assert topic == "v2_modulemanager_abc123/get_modules"
        assert interface is None  # No interface loaded
    
    def test_get_interface_loader(self, mock_interface_paths):
        """Test getting the interface loader instance."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        loader = builder.get_interface_loader()
        
        assert loader is not None
        assert hasattr(loader, 'load_ros2_interface')
        assert hasattr(loader, 'load_protobuf_interface')
    
    def test_get_interface_loader_disabled(self):
        """Test getting loader when disabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        loader = builder.get_interface_loader()
        
        assert loader is None
    
    @patch('vyra_base.com.core.interface_loader.InterfaceLoader.load_interface_metadata')
    def test_reload_interface_metadata(self, mock_load_metadata, mock_interface_paths):
        """Test reloading interface metadata."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        builder.reload_interface_metadata()
        
        # Should call load_interface_metadata with reload=True
        mock_load_metadata.assert_called_once_with(reload=True)
    
    def test_reload_interface_metadata_disabled(self):
        """Test reloading metadata when loader is disabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        # Should not raise exception, just log warning
        builder.reload_interface_metadata()
    
    @patch('vyra_base.com.core.interface_loader.InterfaceLoader.get_cache_stats')
    def test_get_loaded_interfaces_stats(self, mock_get_stats, mock_interface_paths):
        """Test getting interface loading statistics."""
        mock_stats = {
            'ros2_interfaces': 3,
            'protobuf_interfaces': 2,
            'metadata_entries': 10
        }
        mock_get_stats.return_value = mock_stats
        
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            interface_paths=[str(mock_interface_paths)]
        )
        
        stats = builder.get_loaded_interfaces_stats()
        
        assert stats == mock_stats
    
    def test_get_loaded_interfaces_stats_disabled(self):
        """Test getting stats when loader is disabled."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        stats = builder.get_loaded_interfaces_stats()
        
        assert stats == {}  # Empty dict when disabled
    
    def test_slim_mode_compatibility(self):
        """Test that TopicBuilder works in slim mode (no ROS2)."""
        # TopicBuilder should work for topic naming even without ROS2
        builder = TopicBuilder(
            "v2_modulemanager",
            "abc123",
            enable_interface_loading=False
        )
        
        # Topic naming should work
        topic = builder.build_topic_name("get_modules")
        assert topic == "v2_modulemanager_abc123/get_modules"
        
        # Interface loading returns None gracefully
        interface = builder.load_interface_type("get_modules")
        assert interface is None
