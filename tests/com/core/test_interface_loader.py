"""
Unit Tests for InterfaceLoader

Tests dynamic loading of ROS2 and Protocol Buffer interfaces.
"""
import pytest
import json
import tempfile
import shutil
from pathlib import Path
from unittest.mock import patch, MagicMock

from vyra_base.com.core.interface_loader import InterfaceLoader


class TestInterfaceLoader:
    """Test suite for InterfaceLoader."""
    
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
        metadata = [
            {
                "functionname": "get_interface_list",
                "type": "callable",
                "filetype": ["VBASEGetInterfaceList.srv", "VBASEGetInterfaceList.proto"],
                "displaystyle": {"visible": True}
            },
            {
                "functionname": "state_feed",
                "type": "speaker",
                "filetype": ["VBASEStateFeed.msg", "VBASEStateFeed.proto"],
                "displaystyle": {"visible": True}
            },
            {
                "functionname": "hidden_function",
                "type": "callable",
                "filetype": ["Hidden.srv"],
                "displaystyle": {"visible": False}
            }
        ]
        
        (config_dir / "test_meta.json").write_text(json.dumps(metadata))
        
        # Create mock protobuf _pb2.py file
        (proto_dir / "VBASEGetInterfaceList_pb2.py").write_text(
            "# Mock protobuf module\nclass Request:\n    pass\n"
        )
        
        yield interface_dir
        
        shutil.rmtree(temp_dir)
    
    def test_init_with_default_paths(self):
        """Test initialization with default paths from registry."""
        loader = InterfaceLoader()
        
        assert loader.interface_paths is not None
        assert len(loader.interface_paths) >= 1
    
    def test_init_with_custom_paths(self, mock_interface_paths):
        """Test initialization with custom interface paths."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        assert len(loader.interface_paths) == 1
        assert mock_interface_paths in loader.interface_paths
    
    def test_load_interface_metadata(self, mock_interface_paths):
        """Test loading metadata from JSON files."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        metadata = loader.load_interface_metadata()
        
        # Should load visible functions only
        assert "get_interface_list" in metadata
        assert "state_feed" in metadata
        assert "hidden_function" not in metadata  # Not visible
        
        # Check metadata content
        meta = metadata["get_interface_list"]
        assert meta["type"] == "callable"
        assert "VBASEGetInterfaceList.srv" in meta["filetype"]
    
    def test_load_interface_metadata_caching(self, mock_interface_paths):
        """Test that metadata is cached."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        # Load first time
        metadata1 = loader.load_interface_metadata()
        
        # Load second time (should use cache)
        metadata2 = loader.load_interface_metadata()
        
        assert metadata1 is metadata2  # Same object (cached)
    
    def test_load_interface_metadata_reload(self, mock_interface_paths):
        """Test forcing reload of metadata."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        # Load first time
        metadata1 = loader.load_interface_metadata()
        
        # Force reload
        metadata2 = loader.load_interface_metadata(reload=True)
        
        # Should have same content but different objects
        assert metadata1 is not metadata2
        assert metadata1 == metadata2
    
    @patch('vyra_base.com.core.interface_loader._ROS2_AVAILABLE', False)
    def test_load_ros2_interface_unavailable(self):
        """Test ROS2 interface loading when ROS2 not available."""
        loader = InterfaceLoader(auto_update_paths=False)
        
        interface = loader.load_ros2_interface("std_msgs/msg/String")
        
        assert interface is None  # Should return None gracefully
    
    @patch('vyra_base.com.core.interface_loader._ROS2_AVAILABLE', True)
    @patch('vyra_base.com.core.interface_loader.get_message')
    def test_load_ros2_message(self, mock_get_message):
        """Test loading ROS2 message interface."""
        # Mock the ROS2 message type
        mock_msg_type = MagicMock()
        mock_get_message.return_value = mock_msg_type
        
        loader = InterfaceLoader(auto_update_paths=False)
        
        interface = loader.load_ros2_interface("std_msgs/msg/String")
        
        assert interface == mock_msg_type
        mock_get_message.assert_called_once_with("std_msgs/msg/String")
    
    @patch('vyra_base.com.core.interface_loader._ROS2_AVAILABLE', True)
    @patch('vyra_base.com.core.interface_loader.get_service')
    def test_load_ros2_service(self, mock_get_service):
        """Test loading ROS2 service interface."""
        mock_srv_type = MagicMock()
        mock_get_service.return_value = mock_srv_type
        
        loader = InterfaceLoader(auto_update_paths=False)
        
        interface = loader.load_ros2_interface("vyra_module_template_interfaces/srv/VBASEGetInterfaceList")
        
        assert interface == mock_srv_type
        mock_get_service.assert_called_once()
    
    @patch('vyra_base.com.core.interface_loader._ROS2_AVAILABLE', True)
    @patch('vyra_base.com.core.interface_loader.get_action')
    def test_load_ros2_action(self, mock_get_action):
        """Test loading ROS2 action interface."""
        mock_action_type = MagicMock()
        mock_get_action.return_value = mock_action_type
        
        loader = InterfaceLoader(auto_update_paths=False)
        
        interface = loader.load_ros2_interface("action_msgs/action/MyAction")
        
        assert interface == mock_action_type
        mock_get_action.assert_called_once()
    
    def test_load_ros2_interface_invalid_format(self):
        """Test loading with invalid interface path format."""
        loader = InterfaceLoader(auto_update_paths=False)
        
        interface = loader.load_ros2_interface("invalid_format")
        
        assert interface is None
    
    def test_load_ros2_interface_caching(self):
        """Test that loaded ROS2 interfaces are cached."""
        with patch('vyra_base.com.core.interface_loader._ROS2_AVAILABLE', True):
            with patch('vyra_base.com.core.interface_loader.get_message') as mock_get:
                mock_type = MagicMock()
                mock_get.return_value = mock_type
                
                loader = InterfaceLoader(auto_update_paths=False)
                
                # Load twice
                interface1 = loader.load_ros2_interface("std_msgs/msg/String")
                interface2 = loader.load_ros2_interface("std_msgs/msg/String")
                
                assert interface1 == interface2
                # Should only call get_message once (second is cached)
                assert mock_get.call_count == 1
    
    def test_load_protobuf_interface(self, mock_interface_paths):
        """Test loading protobuf interface from _pb2.py file."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        # This should load the mock _pb2 file
        module = loader.load_protobuf_interface("VBASEGetInterfaceList")
        
        # Should successfully load the module
        assert module is not None
        # Module should be in sys.modules
        assert "VBASEGetInterfaceList_pb2" in sys.modules
    
    def test_load_protobuf_interface_not_found(self, mock_interface_paths):
        """Test loading non-existent protobuf interface."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        module = loader.load_protobuf_interface("NonExistent")
        
        assert module is None
    
    def test_get_interface_for_function_ros2(self, mock_interface_paths):
        """Test getting interface by function name for ROS2."""
        with patch('vyra_base.com.core.interface_loader._ROS2_AVAILABLE', True):
            with patch('vyra_base.com.core.interface_loader.get_service') as mock_get:
                mock_type = MagicMock()
                mock_get.return_value = mock_type
                
                loader = InterfaceLoader(
                    interface_paths=[mock_interface_paths],
                    auto_update_paths=False
                )
                
                interface = loader.get_interface_for_function(
                    "get_interface_list",
                    protocol="ros2"
                )
                
                # Should load ROS2 service interface
                assert interface == mock_type
    
    def test_get_interface_for_function_protobuf(self, mock_interface_paths):
        """Test getting interface by function name for protobuf protocols."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        interface = loader.get_interface_for_function(
            "get_interface_list",
            protocol="zenoh"
        )
        
        # Should load protobuf module
        assert interface is not None
    
    def test_get_interface_for_function_not_found(self, mock_interface_paths):
        """Test getting interface for non-existent function."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        interface = loader.get_interface_for_function(
            "non_existent_function",
            protocol="ros2"
        )
        
        assert interface is None
    
    def test_clear_cache(self, mock_interface_paths):
        """Test clearing all caches."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        # Load some data to populate cache
        loader.load_interface_metadata()
        
        # Check cache has data
        stats_before = loader.get_cache_stats()
        assert stats_before['metadata_entries'] > 0
        
        # Clear cache
        loader.clear_cache()
        
        # Check cache is empty
        stats_after = loader.get_cache_stats()
        assert stats_after['metadata_entries'] == 0
    
    def test_get_cache_stats(self, mock_interface_paths):
        """Test getting cache statistics."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        stats = loader.get_cache_stats()
        
        assert 'ros2_interfaces' in stats
        assert 'protobuf_interfaces' in stats
        assert 'metadata_entries' in stats
    
    def test_repr(self, mock_interface_paths):
        """Test string representation."""
        loader = InterfaceLoader(
            interface_paths=[mock_interface_paths],
            auto_update_paths=False
        )
        
        repr_str = repr(loader)
        
        assert "InterfaceLoader" in repr_str
        assert "paths=" in repr_str


# Import sys for sys.modules test
import sys
