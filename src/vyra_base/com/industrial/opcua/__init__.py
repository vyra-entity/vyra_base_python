"""
OPC UA Industrial Protocol Module

OPC Unified Architecture for SCADA/MES northbound communication.

Components:
    - OpcuaClient: Client for connecting to OPC UA servers
    - OpcuaServer: Server for hosting OPC UA endpoints
    - Handlers: Connection, subscription, and node handlers

Features:
    - Client/Server architecture
    - Node read/write operations
    - Method calls
    - Subscriptions with data change notifications
    - Security (authentication, encryption)

Usage:
    >>> # Client
    >>> from vyra_base.com.industrial.opcua import OpcuaClient
    >>> 
    >>> client = OpcuaClient(endpoint="opc.tcp://192.168.1.10:4840")
    >>> await client.connect()
    >>> value = await client.read_node("ns=2;i=1001")
    >>> await client.close()
    >>> 
    >>> # Server
    >>> from vyra_base.com.industrial.opcua import OpcuaServer
    >>> 
    >>> async with OpcuaServer(endpoint="opc.tcp://0.0.0.0:4840") as server:
    ...     idx = await server.register_namespace("http://example.com")
    ...     node = await server.add_variable(idx, "Temperature", 23.5)
"""
import logging

logger = logging.getLogger(__name__)

# Try importing client
try:
    from vyra_base.com.industrial.opcua.opcua_client import (
        OpcuaClient,
        OPCUA_AVAILABLE,
    )
    _client_available = OPCUA_AVAILABLE
except ImportError as e:
    OpcuaClient = None
    OPCUA_AVAILABLE = False
    _client_available = False
    logger.debug(f"⚠️  OPC UA client unavailable: {e}")

# Try importing server
try:
    from vyra_base.com.industrial.opcua.opcua_server import (
        OpcuaServer,
        OPCUA_SERVER_AVAILABLE,
    )
    _server_available = OPCUA_SERVER_AVAILABLE
except ImportError as e:
    OpcuaServer = None
    OPCUA_SERVER_AVAILABLE = False
    _server_available = False
    logger.debug(f"⚠️  OPC UA server unavailable: {e}")

# Try importing handlers
try:
    from vyra_base.com.industrial.opcua.handlers import (
        OpcuaSubscriptionHandler,
        OpcuaConnectionHandler,
        OpcuaNodeHandler,
        OpcuaSecurityMode,
        OpcuaSecurityPolicy,
        OpcuaNodeInfo,
    )
    _handlers_available = True
except ImportError as e:
    OpcuaSubscriptionHandler = None
    OpcuaConnectionHandler = None
    OpcuaNodeHandler = None
    OpcuaSecurityMode = None
    OpcuaSecurityPolicy = None
    OpcuaNodeInfo = None
    _handlers_available = False
    logger.debug(f"⚠️  OPC UA handlers unavailable: {e}")

if _client_available and _server_available:
    logger.info("✅ OPC UA fully available (client + server + handlers)")
elif _client_available:
    logger.info("✅ OPC UA client available")
elif _server_available:
    logger.info("✅ OPC UA server available")
else:
    logger.debug("❌ OPC UA unavailable")

__all__ = [
    # Availability flags
    "OPCUA_AVAILABLE",
    "OPCUA_SERVER_AVAILABLE",
    # Client/Server
    "OpcuaClient",
    "OpcuaServer",
    # Handlers
    "OpcuaSubscriptionHandler",
    "OpcuaConnectionHandler",
    "OpcuaNodeHandler",
    "OpcuaSecurityMode",
    "OpcuaSecurityPolicy",
    "OpcuaNodeInfo"
]
