"""
pytest configuration for vyra_base_python test suite
"""

import pytest
import asyncio
import sys
from pathlib import Path

# Add src directory to Python path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))


@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(autouse=True)
def reset_singleton_state(request):
    """Reset any singleton state between tests"""
    # Skip for tests that don't need DataSpace (e.g., standalone helper tests)
    if 'no_dataspace_reset' in request.keywords:
        yield
        return
    
    # Reset DataSpace between tests if it exists
    try:
        from vyra_base.com.transport.registry import interface_registry
        interface_registry.clear_all()
    except ImportError:
        pass
    
    yield
    
    # Cleanup after test
    try:
        from vyra_base.com.transport.registry import interface_registry
        interface_registry.clear_all()
    except ImportError:
        pass


@pytest.fixture
def mock_redis_client():
    """Fixture providing a mock Redis client"""
    from unittest.mock import Mock, AsyncMock
    
    mock_client = Mock()
    mock_client.ping = AsyncMock(return_value=True)
    mock_client.set = AsyncMock(return_value=True)
    mock_client.get = AsyncMock(return_value=b'{"test": "data"}')
    mock_client.delete = AsyncMock(return_value=1)
    mock_client.exists = AsyncMock(return_value=True)
    mock_client.keys = AsyncMock(return_value=[b'key1', b'key2'])
    mock_client.hset = AsyncMock(return_value=1)
    mock_client.hget = AsyncMock(return_value=b'hash_value')
    mock_client.publish = AsyncMock(return_value=1)
    mock_client.close = AsyncMock()
    
    return mock_client


@pytest.fixture
def mock_ros2_node():
    """Fixture providing a mock ROS2 node"""
    from unittest.mock import Mock
    
    mock_node = Mock()
    mock_node.get_name.return_value = "test_node"
    mock_node.get_namespace.return_value = "/"
    mock_node.create_publisher.return_value = Mock()
    mock_node.create_subscription.return_value = Mock()
    mock_node.create_service.return_value = Mock()
    mock_node.create_client.return_value = Mock()
    
    return mock_node


@pytest.fixture
def sample_state_entry():
    """Fixture providing a sample StateEntry for tests"""
    from datetime import datetime
    try:
        from vyra_base.defaults.entries import StateEntry
        return StateEntry(
            _type=None,
            previous="initial",
            trigger="start",
            current="running",
            module_id="test_module_123",
            module_name="test_module",
            timestamp=datetime.now()
        )
    except ImportError:
        # Return mock if module doesn't exist
        from unittest.mock import Mock
        mock_entry = Mock()
        mock_entry.previous = "initial"
        mock_entry.trigger = "start"
        mock_entry.current = "running"
        mock_entry.module_id = "test_module_123"
        mock_entry.module_name = "test_module"
        mock_entry.timestamp = datetime.now()
        return mock_entry


@pytest.fixture
def sample_module_entry():
    """Fixture providing a sample ModuleEntry for tests"""
    try:
        from vyra_base.defaults.entries import ModuleEntry
        return ModuleEntry(
            uuid="12345678-1234-1234-1234-123456789abc",
            name="test_module",
            template="vyra_module_template",
            description="Test module for testing",
            version="1.0.0"
        )
    except ImportError:
        # Return mock if module doesn't exist
        from unittest.mock import Mock
        mock_entry = Mock()
        mock_entry.name = "test_module"
        mock_entry.uuid = "12345678-1234-1234-1234-123456789abc"
        mock_entry.version = "1.0.0"
        mock_entry.description = "Test module for testing"
        mock_entry.template = "vyra_module_template"
        return mock_entry


@pytest.fixture
def temp_config_file(tmp_path):
    """Fixture providing a temporary configuration file"""
    config_data = {
        "redis": {
            "host": "localhost",
            "port": 6379,
            "database": 0,
            "ssl": False
        },
        "logging": {
            "level": "INFO",
            "file": "/tmp/test.log"
        },
        "module": {
            "name": "test_module",
            "version": "1.0.0"
        }
    }
    
    import json
    config_file = tmp_path / "test_config.json"
    config_file.write_text(json.dumps(config_data, indent=2))
    
    return config_file


# Test markers
pytest_plugins = []

# Custom markers
def pytest_configure(config):
    """Configure custom pytest markers"""
    config.addinivalue_line(
        "markers", "unit: mark test as a unit test"
    )
    config.addinivalue_line(
        "markers", "integration: mark test as an integration test"
    )
    config.addinivalue_line(
        "markers", "asyncio: mark test as async test"
    )
    config.addinivalue_line(
        "markers", "redis: mark test as requiring Redis"
    )
    config.addinivalue_line(
        "markers", "ros2: mark test as requiring ROS2"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )


def pytest_collection_modifyitems(config, items):
    """Modify collected test items"""
    # Add asyncio marker to async tests
    for item in items:
        if asyncio.iscoroutinefunction(item.function):
            item.add_marker(pytest.mark.asyncio)
        
        # Mark Redis tests
        if "redis" in str(item.fspath).lower() or "redis" in item.name.lower():
            item.add_marker(pytest.mark.redis)
        
        # Mark ROS2 tests
        if "ros2" in str(item.fspath).lower() or "communication" in str(item.fspath).lower():
            item.add_marker(pytest.mark.ros2)
        
        # Mark integration tests
        if "integration" in str(item.fspath).lower() or "test_integration" in item.name:
            item.add_marker(pytest.mark.integration)
        else:
            item.add_marker(pytest.mark.unit)


# Test timeout configuration - removed problematic hook