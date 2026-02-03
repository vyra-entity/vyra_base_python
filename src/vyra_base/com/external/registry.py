"""
External Protocol Registry

Central registry for managing and monitoring external protocol connections.
Provides service discovery, health monitoring, and connection pooling.

Example:
    >>> registry = ExternalRegistry()
    >>> 
    >>> # Register gRPC connection
    >>> await registry.register("grpc_scada", "grpc", "192.168.1.10:50051")
    >>> 
    >>> # Get connection
    >>> conn = await registry.get_connection("grpc_scada")
    >>> 
    >>> # Health check
    >>> health = await registry.health_check("grpc_scada")
"""
import asyncio
import logging
from datetime import datetime
from typing import Any, Dict, Optional, List
from dataclasses import dataclass, field
from enum import Enum

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)


class ProtocolStatus(Enum):
    """Protocol connection status."""
    UNKNOWN = "unknown"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    ERROR = "error"


class ProtocolType(Enum):
    """External protocol types."""
    GRPC = "grpc"
    MQTT = "mqtt"
    REST = "rest"
    WEBSOCKET = "websocket"
    OPCUA = "opcua"
    MODBUS = "modbus"


@dataclass
class ProtocolConnection:
    """Protocol connection information."""
    name: str
    protocol_type: str
    endpoint: str
    status: ProtocolStatus = ProtocolStatus.UNKNOWN
    client: Optional[Any] = None
    created_at: datetime = field(default_factory=datetime.now)
    last_connected: Optional[datetime] = None
    last_error: Optional[str] = None
    error_count: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def update_status(self, status: ProtocolStatus, error: Optional[str] = None):
        """Update connection status."""
        self.status = status
        if status == ProtocolStatus.CONNECTED:
            self.last_connected = datetime.now()
            self.error_count = 0
            self.last_error = None
        elif status == ProtocolStatus.ERROR:
            self.error_count += 1
            self.last_error = error


class ExternalRegistry:
    """
    Registry for managing external protocol connections.
    
    Features:
    - Connection registration and tracking
    - Health monitoring
    - Automatic reconnection
    - Service discovery
    - Connection pooling
    
    Example:
        >>> registry = ExternalRegistry()
        >>> 
        >>> # Register connections
        >>> await registry.register("grpc_mes", "grpc", "192.168.1.10:50051")
        >>> await registry.register("mqtt_broker", "mqtt", "192.168.1.20:1883")
        >>> 
        >>> # Get connection
        >>> conn = await registry.get_connection("grpc_mes")
        >>> 
        >>> # List all connections
        >>> connections = registry.list_connections()
        >>> 
        >>> # Health check
        >>> health = await registry.health_check_all()
    """
    
    def __init__(self):
        """Initialize external registry."""
        self._connections: Dict[str, ProtocolConnection] = {}
        self._health_check_interval = 30.0  # seconds
        self._health_check_task: Optional[asyncio.Task] = None
        self._running = False
    
    @ErrorTraceback.w_check_error_exist
    async def start(self):
        """Start registry and health monitoring."""
        if self._running:
            Logger.warning("Registry already running")
            return
        
        self._running = True
        Logger.info("🚀 External protocol registry started")
        
        # Start health check task
        self._health_check_task = asyncio.create_task(self._health_check_loop())
    
    @ErrorTraceback.w_check_error_exist
    async def stop(self):
        """Stop registry and close all connections."""
        if not self._running:
            return
        
        self._running = False
        Logger.info("⏹️  Stopping external protocol registry")
        
        # Cancel health check task
        if self._health_check_task:
            self._health_check_task.cancel()
            try:
                await self._health_check_task
            except asyncio.CancelledError:
                pass
        
        # Close all connections
        for name in list(self._connections.keys()):
            await self.unregister(name)
        
        Logger.info("✅ External protocol registry stopped")
    
    @ErrorTraceback.w_check_error_exist
    async def register(
        self,
        name: str,
        protocol_type: str,
        endpoint: str,
        client: Optional[Any] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> ProtocolConnection:
        """
        Register a new protocol connection.
        
        Args:
            name: Unique connection name
            protocol_type: Protocol type (grpc, mqtt, rest, etc.)
            endpoint: Connection endpoint (host:port, URL, etc.)
            client: Optional client instance
            metadata: Optional metadata dictionary
            
        Returns:
            ProtocolConnection object
        """
        if name in self._connections:
            Logger.warning(f"Connection '{name}' already registered, updating")
            await self.unregister(name)
        
        connection = ProtocolConnection(
            name=name,
            protocol_type=protocol_type,
            endpoint=endpoint,
            client=client,
            metadata=metadata or {}
        )
        
        self._connections[name] = connection
        Logger.info(f"✅ Registered {protocol_type} connection: {name} -> {endpoint}")
        
        return connection
    
    @ErrorTraceback.w_check_error_exist
    async def unregister(self, name: str) -> bool:
        """
        Unregister a protocol connection.
        
        Args:
            name: Connection name
            
        Returns:
            True if unregistered, False if not found
        """
        if name not in self._connections:
            Logger.warning(f"Connection '{name}' not found")
            return False
        
        connection = self._connections[name]
        
        # Close client if it has a close method
        if connection.client:
            try:
                if hasattr(connection.client, 'close'):
                    if asyncio.iscoroutinefunction(connection.client.close):
                        await connection.client.close()
                    else:
                        connection.client.close()
            except Exception as e:
                Logger.error(f"Error closing client: {e}")
        
        del self._connections[name]
        Logger.info(f"✅ Unregistered connection: {name}")
        return True
    
    def get_connection(self, name: str) -> Optional[ProtocolConnection]:
        """
        Get a registered connection.
        
        Args:
            name: Connection name
            
        Returns:
            ProtocolConnection or None
        """
        return self._connections.get(name)
    
    def get_client(self, name: str) -> Optional[Any]:
        """
        Get client instance for a connection.
        
        Args:
            name: Connection name
            
        Returns:
            Client instance or None
        """
        connection = self.get_connection(name)
        return connection.client if connection else None
    
    def list_connections(
        self,
        protocol_type: Optional[str] = None,
        status: Optional[ProtocolStatus] = None,
    ) -> List[ProtocolConnection]:
        """
        List registered connections.
        
        Args:
            protocol_type: Filter by protocol type
            status: Filter by status
            
        Returns:
            List of ProtocolConnection objects
        """
        connections = list(self._connections.values())
        
        if protocol_type:
            connections = [c for c in connections if c.protocol_type == protocol_type]
        
        if status:
            connections = [c for c in connections if c.status == status]
        
        return connections
    
    def has_connection(self, name: str) -> bool:
        """
        Check if connection exists.
        
        Args:
            name: Connection name
            
        Returns:
            True if exists
        """
        return name in self._connections
    
    @ErrorTraceback.w_check_error_exist
    async def health_check(self, name: str) -> Dict[str, Any]:
        """
        Perform health check on a connection.
        
        Args:
            name: Connection name
            
        Returns:
            Health status dictionary
        """
        connection = self.get_connection(name)
        if not connection:
            return {
                "name": name,
                "status": "not_found",
                "healthy": False
            }
        
        # Basic health check
        healthy = connection.status == ProtocolStatus.CONNECTED
        
        # Try client-specific health check if available
        if connection.client and hasattr(connection.client, 'is_connected'):
            try:
                healthy = connection.client.is_connected
            except Exception as e:
                Logger.error(f"Health check error for '{name}': {e}")
                healthy = False
                connection.update_status(ProtocolStatus.ERROR, str(e))
        
        return {
            "name": name,
            "protocol_type": connection.protocol_type,
            "endpoint": connection.endpoint,
            "status": connection.status.value,
            "healthy": healthy,
            "last_connected": connection.last_connected.isoformat() if connection.last_connected else None,
            "error_count": connection.error_count,
            "last_error": connection.last_error,
        }
    
    @ErrorTraceback.w_check_error_exist
    async def health_check_all(self) -> List[Dict[str, Any]]:
        """
        Perform health check on all connections.
        
        Returns:
            List of health status dictionaries
        """
        results = []
        for name in self._connections.keys():
            result = await self.health_check(name)
            results.append(result)
        
        return results
    
    async def _health_check_loop(self):
        """Background task for periodic health checks."""
        while self._running:
            try:
                await asyncio.sleep(self._health_check_interval)
                
                if not self._running:
                    break
                
                # Perform health checks
                results = await self.health_check_all()
                
                # Log unhealthy connections
                for result in results:
                    if not result["healthy"]:
                        Logger.warning(
                            f"⚠️  Unhealthy connection: {result['name']} "
                            f"({result['protocol_type']}) - {result['last_error']}"
                        )
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                Logger.error(f"Error in health check loop: {e}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get registry statistics.
        
        Returns:
            Statistics dictionary
        """
        connections = list(self._connections.values())
        
        by_protocol = {}
        by_status = {}
        
        for conn in connections:
            # Count by protocol
            by_protocol[conn.protocol_type] = by_protocol.get(conn.protocol_type, 0) + 1
            
            # Count by status
            by_status[conn.status.value] = by_status.get(conn.status.value, 0) + 1
        
        return {
            "total_connections": len(connections),
            "by_protocol": by_protocol,
            "by_status": by_status,
            "running": self._running,
        }
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.start()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.stop()


# Global registry instance
_global_registry: Optional[ExternalRegistry] = None


def get_global_registry() -> ExternalRegistry:
    """
    Get the global external protocol registry instance.
    
    Returns:
        ExternalRegistry singleton instance
    """
    global _global_registry
    if _global_registry is None:
        _global_registry = ExternalRegistry()
    return _global_registry
