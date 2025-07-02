"""
Global test configuration and fixtures for the vyra_base test suite.
"""
import sys
import pytest
from unittest.mock import MagicMock, patch

# Create comprehensive ROS2 mocks
class AutoMock(MagicMock):
    """A MagicMock that automatically creates child mocks for any attribute access."""
    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)
        return AutoMock()

def create_mock_module():
    """Create a mock module that can handle arbitrary attribute access."""
    return AutoMock()

# Mock ROS2 dependencies immediately upon import
# This needs to happen before any vyra_base modules are imported
_ros2_mock_modules = [
    'rclpy',
    'rclpy.action',
    'rclpy.node',
    'rclpy.qos',
    'rclpy.executors',
    'rclpy.callback_groups',
    'rclpy.duration',
    'rclpy.time', 
    'rclpy.task',
    'rclpy.client',
    'rclpy.service',
    'rclpy.publisher',
    'rclpy.subscription',
    'rclpy.timer',
    'rclpy.logging',
    'rclpy.parameter',
    'rclpy.clock',
    'builtin_interfaces',
    'builtin_interfaces.msg',
    'unique_identifier_msgs',
    'unique_identifier_msgs.msg',
    'std_msgs',
    'std_msgs.msg',
    'geometry_msgs',
    'geometry_msgs.msg',
    'sensor_msgs',
    'sensor_msgs.msg',
    'action_msgs',
    'action_msgs.msg',
]

# Apply mocks immediately
for module_name in _ros2_mock_modules:
    sys.modules[module_name] = create_mock_module()

# Also mock the main rclpy module with specific behavior
rclpy_mock = create_mock_module()
sys.modules['rclpy'] = rclpy_mock


@pytest.fixture(scope="session", autouse=True)
def setup_test_environment():
    """Setup test environment variables and configurations."""
    # Add any global test setup here
    yield
    # Add any global test cleanup here
