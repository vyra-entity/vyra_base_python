"""
OPC UA Client for vyra_base

High-level wrapper for OPC UA communication with SCADA/MES systems.

Example:
    >>> client = OpcuaClient(endpoint="opc.tcp://192.168.1.100:4840")
    >>> await client.connect()
    >>> value = await client.read_node("ns=2;i=1001")
    >>> await client.write_node("ns=2;i=1002", 42.5)
    >>> await client.close()
"""
from __future__ import annotations

import logging
from typing import Any, Optional, Callable, List
from pathlib import Path

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if asyncua is available
try:
    from asyncua import Client, ua
    from asyncua.common.subscription import Subscription
    OPCUA_AVAILABLE = True
except ImportError:
    OPCUA_AVAILABLE = False


class OpcuaClient:
    """
    High-level OPC UA Client wrapper.
    
    Features:
    - Read/Write node values
    - Method calls
    - Subscriptions with callbacks
    - Security (authentication, encryption)
    
    Args:
        endpoint: OPC UA server endpoint (opc.tcp://...)
        username: Optional username for authentication
        password: Optional password for authentication
        certificate: Optional client certificate path
        private_key: Optional private key path
        security_mode: Security mode (None, Sign, SignAndEncrypt)
        security_policy: Security policy URL
        timeout: Connection timeout in seconds
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        endpoint: str,
        username: Optional[str] = None,
        password: Optional[str] = None,
        certificate: Optional[str] = None,
        private_key: Optional[str] = None,
        security_mode: str = "None",
        security_policy: Optional[str] = None,
        timeout: float = 10.0,
    ):
        """Initialize OPC UA client."""
        if not OPCUA_AVAILABLE:
            raise ImportError("asyncua not installed. Install with: pip install asyncua")
        
        self.endpoint = endpoint
        self.username = username
        self.password = password
        self.certificate = certificate
        self.private_key = private_key
        self.security_mode = security_mode
        self.security_policy = security_policy
        self.timeout = timeout
        
        self._client: Optional[Client] = None
        self._connected = False
        self._subscriptions: List[Subscription] = []
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish connection to OPC UA server."""
        try:
            Logger.info(f"🔌 Connecting to OPC UA server: {self.endpoint}")
            
            # Create client
            self._client = Client(url=self.endpoint, timeout=self.timeout)
            
            if not self._client:
                raise ConnectionError("OPC UA client initialization failed")

            # Set security
            if self.certificate and self.private_key:
                await self._client.set_security(
                    getattr(ua.SecurityPolicyType, self.security_policy or "Basic256Sha256"),
                    certificate=self.certificate,
                    private_key=self.private_key,
                    server_certificate=None,  # Auto-discovery
                    mode=getattr(ua.MessageSecurityMode, self.security_mode)
                )
            
            # Set authentication
            if self.username and self.password:
                self._client.set_user(self.username)
                self._client.set_password(self.password)
            
            # Connect
            await self._client.connect()
            
            self._connected = True
            Logger.info(f"✅ Connected to OPC UA server: {self.endpoint}")
            
        except Exception as e:
            Logger.error(f"❌ Failed to connect to OPC UA server: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close connection to OPC UA server."""
        # Unsubscribe all
        for sub in self._subscriptions:
            try:
                await sub.delete()
            except:
                pass
        self._subscriptions.clear()
        
        if self._client:
            await self._client.disconnect()
            self._client = None
        
        self._connected = False
        Logger.debug(f"OPC UA connection closed: {self.endpoint}")
    
    async def _ensure_connected(self) -> Client:
        """Ensure client is connected."""
        if not self._connected or self._client is None:
            await self.connect()
        
        if self._client is None:
            raise RuntimeError("OPC UA connection failed")
        
        return self._client
    
    @ErrorTraceback.w_check_error_exist
    async def read_node(self, node_id: str) -> Any:
        """
        Read value from OPC UA node.
        
        Args:
            node_id: Node ID (e.g., "ns=2;i=1001" or "ns=2;s=Temperature")
            
        Returns:
            Node value
        """
        client = await self._ensure_connected()
        
        try:
            node = client.get_node(node_id)
            value = await node.read_value()
            Logger.debug(f"📖 OPC UA read: {node_id} = {value}")
            return value
        except Exception as e:
            Logger.error(f"❌ OPC UA read failed: {node_id}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def write_node(self, node_id: str, value: Any) -> None:
        """
        Write value to OPC UA node.
        
        Args:
            node_id: Node ID
            value: Value to write
        """
        client = await self._ensure_connected()
        
        try:
            node = client.get_node(node_id)
            await node.write_value(value)
            Logger.debug(f"✏️ OPC UA write: {node_id} = {value}")
        except Exception as e:
            Logger.error(f"❌ OPC UA write failed: {node_id}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def call_method(
        self,
        object_id: str,
        method_id: str,
        *args
    ) -> Any:
        """
        Call OPC UA method.
        
        Args:
            object_id: Object node ID
            method_id: Method node ID
            *args: Method arguments
            
        Returns:
            Method result
        """
        client = await self._ensure_connected()
        
        try:
            parent = client.get_node(object_id)
            result = await parent.call_method(method_id, *args)
            Logger.debug(f"📞 OPC UA method call: {object_id}.{method_id} -> {result}")
            return result
        except Exception as e:
            Logger.error(f"❌ OPC UA method call failed: {method_id}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def subscribe(
        self,
        node_ids: List[str],
        callback: Callable,
        interval: int = 1000
    ) -> None:
        """
        Subscribe to OPC UA node changes.
        
        Args:
            node_ids: List of node IDs to monitor
            callback: Callback function(node_id, value)
            interval: Publishing interval in milliseconds
        """
        client = await self._ensure_connected()
        
        try:
            # Create subscription
            handler = _SubscriptionHandler(callback)
            sub = await client.create_subscription(interval, handler)
            
            # Monitor nodes
            nodes = [client.get_node(node_id) for node_id in node_ids]
            await sub.subscribe_data_change(nodes)
            
            self._subscriptions.append(sub)
            Logger.info(f"📥 OPC UA subscribed to {len(node_ids)} nodes")
            
        except Exception as e:
            Logger.error(f"❌ OPC UA subscription failed: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def health_check(self) -> bool:
        """
        Check if OPC UA server is reachable.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected and self._client is not None


class _SubscriptionHandler:
    """Internal subscription handler."""
    
    def __init__(self, callback: Callable):
        self.callback = callback
    
    async def datachange_notification(self, node, val, data):
        """Handle data change notification."""
        try:
            node_id = node.nodeid.to_string()
            await self.callback(node_id, val)
        except Exception as e:
            logger.error(f"Error in OPC UA subscription callback: {e}")
