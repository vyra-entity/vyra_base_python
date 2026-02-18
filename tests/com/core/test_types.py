"""
Unit tests for ProtocolType enum and core type definitions.
"""
import pytest
from vyra_base.com.core.types import ProtocolType, InterfaceType


class TestProtocolType:
    """Test protocol type enum."""
    
    def test_protocol_types_exist(self):
        """Test all protocol types are defined."""
        assert hasattr(ProtocolType, 'ROS2')
        assert hasattr(ProtocolType, 'SHARED_MEMORY')
        assert hasattr(ProtocolType, 'UDS')
        assert hasattr(ProtocolType, 'REDIS')
        assert hasattr(ProtocolType, 'GRPC')
        assert hasattr(ProtocolType, 'MQTT')
        assert hasattr(ProtocolType, 'REST')
        assert hasattr(ProtocolType, 'WEBSOCKET')
        assert hasattr(ProtocolType, 'MODBUS')
        assert hasattr(ProtocolType, 'OPCUA')
    
    def test_protocol_values(self):
        """Test protocol types have expected string values."""
        assert ProtocolType.ROS2 == "ros2"
        assert ProtocolType.SHARED_MEMORY == "sharedmemory"
        assert ProtocolType.UDS == "uds"
        assert ProtocolType.REDIS == "redis"
        assert ProtocolType.GRPC == "grpc"
        assert ProtocolType.MQTT == "mqtt"
        assert ProtocolType.REST == "rest"
        assert ProtocolType.WEBSOCKET == "websocket"
        assert ProtocolType.MODBUS == "modbus"
        assert ProtocolType.OPCUA == "opcua"
    
    def test_protocol_comparison(self):
        """Test protocol types can be compared."""
        assert ProtocolType.ROS2 == ProtocolType.ROS2
        assert ProtocolType.ROS2 != ProtocolType.REDIS
        assert ProtocolType.GRPC == "grpc"
    
    def test_protocol_in_list(self):
        """Test protocol types can be used in lists."""
        protocols = [ProtocolType.ROS2, ProtocolType.REDIS]
        assert ProtocolType.ROS2 in protocols
        assert ProtocolType.MQTT not in protocols


class TestInterfaceType:
    """Test interface type enum."""
    
    def test_interface_types_exist(self):
        """Test all interface types are defined."""
        assert hasattr(InterfaceType, 'SERVER')
        assert hasattr(InterfaceType, 'PUBLISHER')
        assert hasattr(InterfaceType, 'ACTION_SERVER')
    
    def test_interface_values(self):
        """Test interface types have expected string values."""
        assert InterfaceType.SERVER == "server"
        assert InterfaceType.PUBLISHER == "publisher"
        assert InterfaceType.ACTION_SERVER == "actionServer"
    
    def test_interface_comparison(self):
        """Test interface types can be compared."""
        assert InterfaceType.SERVER == InterfaceType.SERVER
        assert InterfaceType.SERVER != InterfaceType.PUBLISHER
        assert InterfaceType.ACTION_SERVER == "actionServer"
