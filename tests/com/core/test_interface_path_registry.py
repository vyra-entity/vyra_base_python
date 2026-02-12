"""
Unit Tests for InterfacePathRegistry

Tests the singleton registry for interface base paths.
"""
import pytest
from pathlib import Path
import tempfile
import shutil

from vyra_base.com.core.interface_path_registry import (
    InterfacePathRegistry,
    get_interface_registry
)


class TestInterfacePathRegistry:
    """Test suite for InterfacePathRegistry."""
    
    @pytest.fixture(autouse=True)
    def reset_singleton(self):
        """Reset singleton between tests."""
        # Store original instance
        original = InterfacePathRegistry._instance
        
        # Reset for test
        InterfacePathRegistry._instance = None
        
        yield
        
        # Restore original after test
        InterfacePathRegistry._instance = original
    
    @pytest.fixture
    def temp_interface_dirs(self):
        """Create temporary interface directories for testing."""
        temp_dir = Path(tempfile.mkdtemp())
        
        # Create mock interface directory structures
        interface1 = temp_dir / "interface1"
        interface2 = temp_dir / "interface2"
        
        (interface1 / "config").mkdir(parents=True)
        (interface1 / "proto").mkdir(parents=True)
        (interface1 / "callable").mkdir(parents=True)
        
        (interface2 / "config").mkdir(parents=True)
        (interface2 / "proto").mkdir(parents=True)
        
        # Create some dummy files
        (interface1 / "config" / "test_meta.json").write_text('{"test": "data"}')
        (interface2 / "proto" / "Test_pb2.py").write_text('# Generated protobuf')
        
        yield interface1, interface2
        
        # Cleanup
        shutil.rmtree(temp_dir)
    
    def test_singleton_pattern(self):
        """Test that InterfacePathRegistry follows singleton pattern."""
        registry1 = InterfacePathRegistry.get_instance()
        registry2 = InterfacePathRegistry.get_instance()
        
        assert registry1 is registry2, "Should return same instance"
    
    def test_get_instance_convenience_function(self):
        """Test convenience function returns singleton."""
        registry1 = get_interface_registry()
        registry2 = InterfacePathRegistry.get_instance()
        
        assert registry1 is registry2
    
    def test_default_path_initialization(self):
        """Test that registry initializes with default vyra_base path."""
        registry = InterfacePathRegistry.get_instance()
        paths = registry.get_interface_paths()
        
        assert len(paths) >= 1, "Should have at least default path"
        
        # Default path should end with 'interfaces'
        default_path = paths[0]
        assert default_path.name == "interfaces"
        assert default_path.exists()
    
    def test_set_interface_paths_valid(self, temp_interface_dirs):
        """Test setting valid interface paths."""
        interface1, interface2 = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1), str(interface2)])
        
        paths = registry.get_interface_paths()
        
        assert len(paths) == 2
        assert interface1 in paths
        assert interface2 in paths
    
    def test_set_interface_paths_replaces_existing(self, temp_interface_dirs):
        """Test that set_interface_paths replaces existing paths."""
        interface1, interface2 = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        
        # Set first path
        registry.set_interface_paths([str(interface1)])
        assert len(registry.get_interface_paths()) == 1
        
        # Replace with second path
        registry.set_interface_paths([str(interface2)])
        paths = registry.get_interface_paths()
        
        assert len(paths) == 1
        assert interface2 in paths
        assert interface1 not in paths
    
    def test_set_interface_paths_invalid_path(self):
        """Test that invalid paths are skipped with warning."""
        registry = InterfacePathRegistry.get_instance()
        
        # Try to set non-existent path
        non_existent = "/tmp/nonexistent_interface_path_12345"
        
        with pytest.raises(ValueError, match="No valid interface paths"):
            registry.set_interface_paths([non_existent])
    
    def test_set_interface_paths_file_not_directory(self, temp_interface_dirs):
        """Test that files are rejected, only directories accepted."""
        interface1, _ = temp_interface_dirs
        file_path = interface1 / "config" / "test_meta.json"
        
        registry = InterfacePathRegistry.get_instance()
        
        with pytest.raises(ValueError, match="No valid interface paths"):
            registry.set_interface_paths([str(file_path)])
    
    def test_add_interface_path(self, temp_interface_dirs):
        """Test adding a path without replacing existing."""
        interface1, interface2 = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1)])
        
        # Add second path
        registry.add_interface_path(str(interface2))
        
        paths = registry.get_interface_paths()
        assert len(paths) == 2
        assert interface1 in paths
        assert interface2 in paths
    
    def test_add_interface_path_duplicate(self, temp_interface_dirs):
        """Test that adding duplicate path is ignored."""
        interface1, _ = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1)])
        
        # Try to add same path again
        registry.add_interface_path(str(interface1))
        
        paths = registry.get_interface_paths()
        assert len(paths) == 1
    
    def test_get_config_paths(self, temp_interface_dirs):
        """Test get_config_paths returns config subdirectories."""
        interface1, interface2 = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1), str(interface2)])
        
        config_paths = registry.get_config_paths()
        
        assert len(config_paths) == 2
        assert interface1 / "config" in config_paths
        assert interface2 / "config" in config_paths
    
    def test_get_proto_paths(self, temp_interface_dirs):
        """Test get_proto_paths returns proto subdirectories."""
        interface1, interface2 = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1), str(interface2)])
        
        proto_paths = registry.get_proto_paths()
        
        assert len(proto_paths) == 2
        assert interface1 / "proto" in proto_paths
        assert interface2 / "proto" in proto_paths
    
    def test_clear_interface_paths(self, temp_interface_dirs):
        """Test clearing paths resets to default."""
        interface1, _ = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1)])
        
        # Clear and reset to default
        registry.clear_interface_paths()
        
        paths = registry.get_interface_paths()
        
        # Should have default path again
        assert len(paths) >= 1
        assert paths[0].name == "interfaces"
    
    def test_thread_safety(self, temp_interface_dirs):
        """Test that registry is thread-safe."""
        import threading
        
        interface1, interface2 = temp_interface_dirs
        registry = InterfacePathRegistry.get_instance()
        
        results = []
        
        def access_registry():
            registry.set_interface_paths([str(interface1)])
            paths = registry.get_interface_paths()
            results.append(len(paths))
        
        # Create multiple threads
        threads = [threading.Thread(target=access_registry) for _ in range(10)]
        
        # Start all threads
        for t in threads:
            t.start()
        
        # Wait for completion
        for t in threads:
            t.join()
        
        # All threads should succeed
        assert len(results) == 10
    
    def test_repr(self, temp_interface_dirs):
        """Test string representation."""
        interface1, _ = temp_interface_dirs
        
        registry = InterfacePathRegistry.get_instance()
        registry.set_interface_paths([str(interface1)])
        
        repr_str = repr(registry)
        
        assert "InterfacePathRegistry" in repr_str
        assert "paths=" in repr_str
        assert "locations=" in repr_str
