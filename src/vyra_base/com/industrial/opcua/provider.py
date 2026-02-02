"""
OPC UA Protocol Provider

Northbound OPC UA communication for SCADA/MES integration.
Uses asyncua for protocol implementation.
"""

import logging
from typing import Optional, Dict, Any, List
from vyra_base.com.providers import AbstractProtocolProvider
from vyra_base.com.core.types import ProtocolType, VyraCallable, VyraSpeaker, VyraJob
from vyra_base.com.core.exceptions import ProviderError, InterfaceError

logger = logging.getLogger(__name__)

# Optional import
try:
    from asyncua import Client, ua
    from asyncua.common.subscription import Subscription
    OPCUA_AVAILABLE = True
except ImportError:
    Client = None
    ua = None
    Subscription = None
    OPCUA_AVAILABLE = False


class OpcuaProvider(AbstractProtocolProvider):
    """
    OPC UA Protocol Provider
    
    Northbound communication for SCADA/MES systems.
    Supports reading/writing nodes, method calls, and subscriptions.
    
    Args:
        endpoint: OPC UA server endpoint (e.g., "opc.tcp://localhost:4840")
        username: Optional username for authentication
        password: Optional password for authentication
        certificate: Optional client certificate path
        private_key: Optional private key path
        security_mode: Security mode ("None", "Sign", "SignAndEncrypt")
        security_policy: Security policy URL
        timeout: Connection timeout in seconds (default: 10.0)
    
    Example:
        # Anonymous connection
        provider = OpcuaProvider(
            endpoint="opc.tcp://192.168.1.100:4840"
        )
        
        # Authenticated connection
        provider = OpcuaProvider(
            endpoint="opc.tcp://192.168.1.100:4840",
            username="admin",
            password="secret"
        )
        
        # Secure connection
        provider = OpcuaProvider(
            endpoint="opc.tcp://192.168.1.100:4840",
            certificate="/path/to/cert.pem",
            private_key="/path/to/key.pem",
            security_mode="SignAndEncrypt"
        )
    """
    
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
        super().__init__(ProtocolType.OPCUA)
        
        if not OPCUA_AVAILABLE:
            raise ProviderError(
                "OPC UA not available. Install with: pip install asyncua"
            )
        
        self._endpoint = endpoint
        self._username = username
        self._password = password
        self._certificate = certificate
        self._private_key = private_key
        self._security_mode = security_mode
        self._security_policy = security_policy
        self._timeout = timeout
        self._client: Optional[Client] = None
        self._subscription: Optional[Subscription] = None
    
    async def initialize(self) -> None:
        """Initialize OPC UA client connection"""
        if self._initialized:
            return
        
        try:
            # Create client
            self._client = Client(url=self._endpoint, timeout=self._timeout)
            
            # Set security
            if self._security_mode != "None":
                if self._certificate and self._private_key:
                    await self._client.set_security(
                        self._security_policy or ua.SecurityPolicyType.Basic256Sha256,
                        certificate=self._certificate,
                        private_key=self._private_key,
                        mode=getattr(ua.MessageSecurityMode, self._security_mode)
                    )
                else:
                    logger.warning(
                        f"Security mode {self._security_mode} requires certificate/key"
                    )
            
            # Set authentication
            if self._username and self._password:
                self._client.set_user(self._username)
                self._client.set_password(self._password)
            
            # Connect
            logger.info(f"Connecting to OPC UA server: {self._endpoint}")
            await self._client.connect()
            
            # Get server info
            server_info = await self._client.get_server_node().read_browse_name()
            logger.info(f"Connected to OPC UA server: {server_info}")
            
            self._initialized = True
            logger.info("✅ OPC UA provider initialized")
        
        except Exception as e:
            raise ProviderError(f"OPC UA initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Disconnect from OPC UA server"""
        if self._subscription:
            await self._subscription.delete()
            self._subscription = None
        
        if self._client:
            await self._client.disconnect()
            self._client = None
            logger.info("OPC UA connection closed")
        
        self._initialized = False
    
    async def is_available(self) -> bool:
        """Check if OPC UA connection is active"""
        if not self._initialized or not self._client:
            return False
        
        try:
            # Check if we can read the server node
            await self._client.get_server_node().read_browse_name()
            return True
        except Exception:
            return False
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create OPC UA callable interface
        
        Args:
            name: Interface name (not used for OPC UA)
            callback: Server callback (not supported for OPC UA)
            **kwargs: Additional arguments
        
        Returns:
            OpcuaCallable instance
        
        Note:
            OPC UA is client-only (no server callback support in this provider)
        """
        if not self._initialized:
            raise InterfaceError("Provider not initialized")
        
        if callback:
            logger.warning("OPC UA provider does not support server callbacks")
        
        from vyra_base.com.industrial.opcua.callable import OpcuaCallable
        
        callable_obj = OpcuaCallable(
            client=self._client,
            name=name
        )
        await callable_obj.initialize()
        
        return callable_obj
    
    async def create_speaker(
        self,
        name: str,
        callback: Optional[callable] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create OPC UA speaker for subscriptions
        
        Args:
            name: Node ID or path to subscribe to
            callback: Callback for subscription data changes
            **kwargs: Additional subscription parameters
        
        Returns:
            OpcuaSpeaker instance
        
        Note:
            Speaker is used for subscriptions (monitoring node changes)
        """
        if not self._initialized:
            raise InterfaceError("Provider not initialized")
        
        from vyra_base.com.industrial.opcua.speaker import OpcuaSpeaker
        
        # Create subscription if not exists
        if not self._subscription:
            self._subscription = await self._client.create_subscription(
                period=kwargs.get("period", 500),  # 500ms default
                handler=None
            )
        
        speaker_obj = OpcuaSpeaker(
            client=self._client,
            subscription=self._subscription,
            node_id=name,
            callback=callback
        )
        await speaker_obj.initialize()
        
        return speaker_obj
    
    async def create_job(
        self,
        name: str,
        execute_callback: Optional[callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create OPC UA job (not supported)
        
        OPC UA is request/response or subscription-based, no long-running jobs.
        """
        raise NotImplementedError(
            "OPC UA does not support job interfaces"
        )
    
    # Convenience methods for direct node access
    
    async def read_node(self, node_id: str) -> Any:
        """Read node value"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        node = self._client.get_node(node_id)
        return await node.read_value()
    
    async def write_node(self, node_id: str, value: Any) -> None:
        """Write node value"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        node = self._client.get_node(node_id)
        await node.write_value(value)
    
    async def browse_node(self, node_id: str) -> List[Dict[str, Any]]:
        """Browse node children"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        node = self._client.get_node(node_id)
        children = await node.get_children()
        
        results = []
        for child in children:
            browse_name = await child.read_browse_name()
            node_class = await child.read_node_class()
            results.append({
                "node_id": str(child.nodeid),
                "browse_name": browse_name.Name,
                "node_class": str(node_class)
            })
        
        return results
    
    async def call_method(
        self,
        object_id: str,
        method_id: str,
        *args
    ) -> Any:
        """Call OPC UA method"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        obj_node = self._client.get_node(object_id)
        method_node = self._client.get_node(method_id)
        
        return await obj_node.call_method(method_node, *args)
