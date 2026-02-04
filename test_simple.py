"""
Simple test to check if basic imports work
"""
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

def test_basic_imports():
    """Test that we can import the main modules"""
    try:
        import vyra_base
        print("✅ vyra_base imported successfully")
    except ImportError as e:
        print(f"❌ vyra_base import failed: {e}")
    
    try:
        from vyra_base.defaults import exceptions
        print("✅ vyra_base.defaults.exceptions imported successfully")
    except ImportError as e:
        print(f"❌ vyra_base.defaults.exceptions import failed: {e}")
    
    try:
        from vyra_base.defaults import constants
        print("✅ vyra_base.defaults.constants imported successfully")
    except ImportError as e:
        print(f"❌ vyra_base.defaults.constants import failed: {e}")
    
    try:
        from vyra_base.defaults import info
        print("✅ vyra_base.defaults.info imported successfully")
    except ImportError as e:
        print(f"❌ vyra_base.defaults.info import failed: {e}")
    
    try:
        from vyra_base.storage import redis_client
        print("✅ vyra_base.com.transport.redis.communication imported successfully")
    except ImportError as e:
        print(f"❌ vyra_base.com.transport.redis.communication import failed: {e}")

if __name__ == "__main__":
    test_basic_imports()