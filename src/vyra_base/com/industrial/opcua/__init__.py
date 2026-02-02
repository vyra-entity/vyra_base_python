"""
OPC UA Protocol Support

OPC Unified Architecture for SCADA/MES northbound communication.
"""

from vyra_base.com.industrial.opcua.provider import OpcuaProvider
from vyra_base.com.industrial.opcua.callable import OpcuaCallable

__all__ = ["OpcuaProvider", "OpcuaCallable"]
