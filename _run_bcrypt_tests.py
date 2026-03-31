"""Runner script to test bcrypt integration without ROS2 plugin interference."""
import sys
# Remove ROS2 paths to avoid launch_testing plugin
sys.path = [p for p in sys.path if '/opt/ros/' not in p]

import pytest
sys.exit(pytest.main([
    '/home/holgder/VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/tests/unit/test_internal_usermanager_bcrypt.py',
    '-v', '--tb=short', '--no-header', '-c', '/dev/null'
]))
