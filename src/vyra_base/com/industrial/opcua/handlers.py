"""
OPC UA Handlers

Common handler classes for OPC UA client and server operations.
Provides connection management, node operations, and subscription handling.
"""
from __future__ import annotations

import logging
from typing import Any, Optional, Callable, Dict
from dataclasses import dataclass
from enum import Enum

from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if asyncua is available
try:
    from asyncua import ua
    from asyncua.common.node import Node
    from asyncua.common.subscription import Subscription, SubHandler
    ASYNCUA_AVAILABLE = True
except ImportError:
    ASYNCUA_AVAILABLE = False


class OpcuaSecurityMode(Enum):
    """OPC UA security modes."""
    NONE = "None"
    SIGN = "Sign"
    SIGN_AND_ENCRYPT = "SignAndEncrypt"


class OpcuaSecurityPolicy(Enum):
    """OPC UA security policies."""
    NONE = "None"
    BASIC128RSA15 = "Basic128Rsa15"
    BASIC256 = "Basic256"
    BASIC256SHA256 = "Basic256Sha256"


@dataclass
class OpcuaNodeInfo:
    """OPC UA node information."""
    node_id: str
    namespace_idx: int
    name: str
    value: Any
    data_type: str
    writable: bool = False


class OpcuaSubscriptionHandler(SubHandler):
    """
    Handler for OPC UA subscriptions.
    
    Receives data change notifications and forwards them to callbacks.
    """
    
    def __init__(self, callback: Optional[Callable] = None):
        """
        Initialize subscription handler.
        
        Args:
            callback: Callback function(node_id, value)
        """
        if ASYNCUA_AVAILABLE:
            super().__init__()
        self.callback = callback
        self._node_map: Dict[int, str] = {}
    
    def register_node(self, node: Any, node_id: str):
        """Register a node for tracking."""
        if hasattr(node, 'nodeid'):
            self._node_map[node.nodeid.Identifier] = node_id
    
    def datachange_notification(self, node: Any, val: Any, data: Any):
        """
        Handle data change notification.
        
        Args:
            node: Node that changed
            val: New value
            data: Additional data
        """
        try:
            node_id = self._node_map.get(
                node.nodeid.Identifier,
                str(node.nodeid)
            )
            
            logger.debug(f"📡 Data change: {node_id} = {val}")
            
            if self.callback:
                self.callback(node_id, val)
                
        except Exception as e:
            logger.error(f"❌ Error in datachange notification: {e}")


class OpcuaConnectionHandler:
    """
    Handler for OPC UA connection management.
    
    Provides connection lifecycle management and automatic reconnection.
    """
    
    def __init__(
        self,
        endpoint: str,
        on_connect: Optional[Callable] = None,
        on_disconnect: Optional[Callable] = None,
        reconnect_interval: float = 5.0,
    ):
        """
        Initialize connection handler.
        
        Args:
            endpoint: OPC UA endpoint
            on_connect: Callback on successful connection
            on_disconnect: Callback on disconnection
            reconnect_interval: Seconds between reconnection attempts
        """
        self.endpoint = endpoint
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.reconnect_interval = reconnect_interval
        self._connected = False
        self._reconnecting = False
    
    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected
    
    async def handle_connect(self):
        """Handle successful connection."""
        self._connected = True
        self._reconnecting = False
        logger.info(f"✅ OPC UA connected: {self.endpoint}")
        
        if self.on_connect:
            try:
                await self.on_connect()
            except Exception as e:
                logger.error(f"❌ Error in on_connect callback: {e}")
    
    async def handle_disconnect(self):
        """Handle disconnection."""
        self._connected = False
        logger.warning(f"⚠️  OPC UA disconnected: {self.endpoint}")
        
        if self.on_disconnect:
            try:
                await self.on_disconnect()
            except Exception as e:
                logger.error(f"❌ Error in on_disconnect callback: {e}")


class OpcuaNodeHandler:
    """
    Handler for OPC UA node operations.
    
    Provides high-level node management and value conversion.
    """
    
    def __init__(self):
        """Initialize node handler."""
        self._nodes: Dict[str, Node] = {}
    
    def register_node(self, node_id: str, node: Node):
        """
        Register a node for tracking.
        
        Args:
            node_id: Node identifier
            node: Node object
        """
        self._nodes[node_id] = node
        logger.debug(f"📝 Registered node: {node_id}")
    
    def get_node(self, node_id: str) -> Optional[Node]:
        """
        Get a registered node.
        
        Args:
            node_id: Node identifier
            
        Returns:
            Node object or None
        """
        return self._nodes.get(node_id)
    
    def has_node(self, node_id: str) -> bool:
        """
        Check if node is registered.
        
        Args:
            node_id: Node identifier
            
        Returns:
            True if node exists
        """
        return node_id in self._nodes
    
    @staticmethod
    def parse_node_id(node_id: str) -> tuple[int, str]:
        """
        Parse node ID string.
        
        Args:
            node_id: Node ID (e.g., "ns=2;i=1001" or "ns=2;s=Temperature")
            
        Returns:
            (namespace_index, identifier)
        """
        try:
            parts = node_id.split(';')
            ns_part = parts[0]
            id_part = parts[1]
            
            namespace_idx = int(ns_part.split('=')[1])
            identifier = id_part.split('=')[1]
            
            return namespace_idx, identifier
            
        except Exception as e:
            logger.error(f"❌ Failed to parse node ID '{node_id}': {e}")
            raise ValueError(f"Invalid node ID format: {node_id}")
    
    @staticmethod
    def format_node_id(namespace_idx: int, identifier: str, is_numeric: bool = False) -> str:
        """
        Format node ID string.
        
        Args:
            namespace_idx: Namespace index
            identifier: Node identifier
            is_numeric: True for numeric ID (i=), False for string (s=)
            
        Returns:
            Formatted node ID (e.g., "ns=2;i=1001")
        """
        prefix = "i" if is_numeric else "s"
        return f"ns={namespace_idx};{prefix}={identifier}"
    
    @staticmethod
    def convert_value_to_variant(value: Any, data_type: Optional[str] = None) -> Any:
        """
        Convert Python value to OPC UA variant.
        
        Args:
            value: Python value
            data_type: Optional OPC UA data type
            
        Returns:
            OPC UA variant
        """
        if not ASYNCUA_AVAILABLE:
            return value
        
        # Auto-detect type if not specified
        if data_type is None:
            if isinstance(value, bool):
                data_type = "Boolean"
            elif isinstance(value, int):
                data_type = "Int32"
            elif isinstance(value, float):
                data_type = "Double"
            elif isinstance(value, str):
                data_type = "String"
        
        # Map to OPC UA types
        type_map = {
            "Boolean": ua.VariantType.Boolean,
            "Int16": ua.VariantType.Int16,
            "Int32": ua.VariantType.Int32,
            "Int64": ua.VariantType.Int64,
            "UInt16": ua.VariantType.UInt16,
            "UInt32": ua.VariantType.UInt32,
            "UInt64": ua.VariantType.UInt64,
            "Float": ua.VariantType.Float,
            "Double": ua.VariantType.Double,
            "String": ua.VariantType.String,
            "ByteString": ua.VariantType.ByteString,
        }

        if not data_type:
            logger.debug("Data type not specified, returning value as string variant")
            return ua.Variant(str(value), type_map.get("String"))
        
        variant_type = type_map.get(data_type)
        if variant_type:
            return ua.Variant(value, variant_type)
        
        return value
