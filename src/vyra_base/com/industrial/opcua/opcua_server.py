"""
OPC UA Server for vyra_base

High-level wrapper for hosting OPC UA server with namespace management.

Example:
    >>> server = OpcuaServer(endpoint="opc.tcp://0.0.0.0:4840")
    >>> await server.start()
    >>> 
    >>> # Add namespace and variables
    >>> idx = await server.register_namespace("http://example.com")
    >>> node = await server.add_variable(idx, "Temperature", 23.5)
    >>> 
    >>> await server.stop()
"""
from __future__ import annotations

import logging
from typing import Any, Optional, Callable, List, Dict
from pathlib import Path

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if asyncua is available
try:
    from asyncua import Server, ua
    from asyncua.common.node import Node
    OPCUA_SERVER_AVAILABLE = True
except ImportError:
    OPCUA_SERVER_AVAILABLE = False


class OpcuaServer:
    """
    High-level OPC UA Server wrapper.
    
    Features:
    - Host OPC UA server
    - Namespace management
    - Node creation (variables, objects, methods)
    - Security (authentication, encryption)
    - Event generation
    
    Args:
        endpoint: Server endpoint (default: opc.tcp://0.0.0.0:4840)
        name: Server name (default: "VYRA OPC UA Server")
        certificate: Optional server certificate path
        private_key: Optional private key path
        security_mode: Security mode (None, Sign, SignAndEncrypt)
        security_policy: Security policy URL
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        endpoint: str = "opc.tcp://0.0.0.0:4840",
        name: str = "VYRA OPC UA Server",
        certificate: Optional[str] = None,
        private_key: Optional[str] = None,
        security_mode: str = "None",
        security_policy: Optional[str] = None,
    ):
        """Initialize OPC UA server."""
        if not OPCUA_SERVER_AVAILABLE:
            raise ImportError("asyncua not installed. Install with: pip install asyncua")
        
        self.endpoint = endpoint
        self.name = name
        self.certificate = certificate
        self.private_key = private_key
        self.security_mode = security_mode
        self.security_policy = security_policy
        
        self._server: Optional[Server] = None
        self._running = False
        self._namespaces: Dict[str, int] = {}
        self._nodes: Dict[str, Node] = {}
    
    @ErrorTraceback.w_check_error_exist
    async def start(self) -> None:
        """Start OPC UA server."""
        try:
            Logger.info(f"🚀 Starting OPC UA server: {self.endpoint}")
            
            # Create server
            self._server = Server()

            if not self._server:
                raise RuntimeError("OPC UA server initialization failed")
            
            await self._server.init()
            
            # Set endpoint
            self._server.set_endpoint(self.endpoint)
            
            # Set server name
            self._server.set_server_name(self.name)
            
            # Set security if configured
            if self.certificate and self.private_key:
                await self._server.set_security_policy([
                    getattr(ua.SecurityPolicyType, self.security_policy or "Basic256Sha256")
                ])
                await self._server.load_certificate(self.certificate)
                await self._server.load_private_key(self.private_key)
            
            # Start server
            await self._server.start()
            self._running = True
            
            Logger.info(f"✅ OPC UA server started: {self.endpoint}")
            
        except Exception as e:
            Logger.error(f"❌ Failed to start OPC UA server: {e}")
            self._running = False
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def stop(self) -> None:
        """Stop OPC UA server."""
        if self._server and self._running:
            Logger.info(f"⏹️  Stopping OPC UA server: {self.endpoint}")
            await self._server.stop()
            self._server = None
            self._running = False
            self._namespaces.clear()
            self._nodes.clear()
            Logger.info("✅ OPC UA server stopped")
    
    @property
    def is_running(self) -> bool:
        """Check if server is running."""
        return self._running
    
    def _require_running(self):
        """Raise error if server is not running."""
        if not self._running or not self._server:
            raise RuntimeError("OPC UA server is not running")
    
    @ErrorTraceback.w_check_error_exist
    async def register_namespace(self, uri: str) -> int:
        """
        Register a new namespace.
        
        Args:
            uri: Namespace URI (e.g., "http://example.com")
            
        Returns:
            Namespace index
        """
        self._require_running()
        
        if uri in self._namespaces:
            Logger.debug(f"Namespace already registered: {uri}")
            return self._namespaces[uri]
        
        if not self._server:
            raise RuntimeError("OPC UA server is not initialized")

        try:
            idx = await self._server.register_namespace(uri)
            self._namespaces[uri] = idx
            Logger.info(f"✅ Registered namespace: {uri} (idx={idx})")
            return idx
            
        except Exception as e:
            Logger.error(f"❌ Failed to register namespace: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def add_variable(
        self,
        namespace_idx: int,
        name: str,
        value: Any,
        node_id: Optional[str] = None,
        writable: bool = True,
    ) -> Node:
        """
        Add a variable node.
        
        Args:
            namespace_idx: Namespace index
            name: Variable name
            value: Initial value
            node_id: Optional custom node ID
            writable: Whether variable is writable
            
        Returns:
            Node object
        """
        self._require_running()
        
        if not self._server:
            raise RuntimeError("OPC UA server is not initialized")
        
        try:
            # Get objects node
            objects = self._server.nodes.objects
            
            # Create node ID
            if node_id:
                full_node_id = f"ns={namespace_idx};s={node_id}"
            else:
                full_node_id = f"ns={namespace_idx};s={name}"
            
            # Check if node already exists
            if full_node_id in self._nodes:
                Logger.debug(f"Variable already exists: {full_node_id}")
                return self._nodes[full_node_id]
            
            # Add variable
            node = await objects.add_variable(
                nodeid=ua.NodeId(node_id or name, namespace_idx),
                bname=name,
                val=value
            )
            
            # Set writable
            if writable:
                await node.set_writable()
            
            self._nodes[full_node_id] = node
            Logger.info(f"✅ Added variable: {full_node_id} = {value}")
            return node
            
        except Exception as e:
            Logger.error(f"❌ Failed to add variable: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def add_object(
        self,
        namespace_idx: int,
        name: str,
        node_id: Optional[str] = None,
    ) -> Node:
        """
        Add an object node.
        
        Args:
            namespace_idx: Namespace index
            name: Object name
            node_id: Optional custom node ID
            
        Returns:
            Node object
        """
        self._require_running()
        
        if not self._server:
            raise RuntimeError("OPC UA server is not initialized")
        
        try:
            # Get objects node
            objects = self._server.nodes.objects
            
            # Create node ID
            if node_id:
                full_node_id = f"ns={namespace_idx};s={node_id}"
            else:
                full_node_id = f"ns={namespace_idx};s={name}"
            
            # Check if node already exists
            if full_node_id in self._nodes:
                Logger.debug(f"Object already exists: {full_node_id}")
                return self._nodes[full_node_id]
            
            # Add object
            node = await objects.add_object(
                nodeid=ua.NodeId(node_id or name, namespace_idx),
                bname=name
            )
            
            self._nodes[full_node_id] = node
            Logger.info(f"✅ Added object: {full_node_id}")
            return node
            
        except Exception as e:
            Logger.error(f"❌ Failed to add object: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def set_variable_value(self, node_id: str, value: Any) -> None:
        """
        Set value of a variable node.
        
        Args:
            node_id: Node ID (e.g., "ns=2;s=Temperature")
            value: New value
        """
        self._require_running()
        
        if not self._server:
            raise RuntimeError("OPC UA server is not initialized")
        
        try:
            if node_id not in self._nodes:
                # Try to get node by ID
                node = self._server.get_node(node_id)
            else:
                node = self._nodes[node_id]
            
            await node.write_value(value)
            Logger.debug(f"✍️  Set {node_id} = {value}")
            
        except Exception as e:
            Logger.error(f"❌ Failed to set variable value: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def get_variable_value(self, node_id: str) -> Any:
        """
        Get value of a variable node.
        
        Args:
            node_id: Node ID (e.g., "ns=2;s=Temperature")
            
        Returns:
            Node value
        """
        self._require_running()

        if not self._server:
            raise RuntimeError("OPC UA server is not initialized")
        
        try:
            if node_id not in self._nodes:
                # Try to get node by ID
                node = self._server.get_node(node_id)
            else:
                node = self._nodes[node_id]
            
            value = await node.read_value()
            Logger.debug(f"📖 Read {node_id} = {value}")
            return value
            
        except Exception as e:
            Logger.error(f"❌ Failed to get variable value: {e}")
            raise
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.start()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.stop()
