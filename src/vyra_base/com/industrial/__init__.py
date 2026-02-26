"""
Industrial Communication Protocols

Northbound protocols for SCADA/MES integration (Modbus, OPC UA).
These are intended for communication with higher-level systems, not field devices.
"""

# Modbus (optional dependency)
try:
    from vyra_base.com.industrial.modbus import (
        ModbusBaseClient,
        ModbusTCPClient,
        ModbusRTUClient
    )
    MODBUS_AVAILABLE = True
except ImportError:
    ModbusProvider = None
    MODBUS_AVAILABLE = False

# OPC UA (optional dependency)
try:
    from vyra_base.com.industrial.opcua import (
        OpcuaClient,
        OpcuaServer,
        OpcuaSubscriptionHandler,
        OpcuaConnectionHandler,
        OpcuaNodeHandler
    )
    OPCUA_AVAILABLE = True
except ImportError:
    OpcuaProvider = None
    OPCUA_AVAILABLE = False

__all__ = [
    "ModbusBaseClient",
    "ModbusTCPClient",
    "ModbusRTUClient",
    "OpcuaClient",
    "OpcuaServer",
    "OpcuaSubscriptionHandler",
    "OpcuaConnectionHandler",
    "OpcuaNodeHandler",
    "MODBUS_AVAILABLE",
    "OPCUA_AVAILABLE",
]
