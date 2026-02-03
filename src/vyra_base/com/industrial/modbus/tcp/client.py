"""
Modbus TCP Client

Modbus over TCP/IP for communication with PLCs and industrial devices.

Example:
    >>> client = ModbusTCPClient(host="192.168.1.10", port=502)
    >>> await client.connect()
    >>> values = await client.read_holding_registers(address=0, count=10)
    >>> await client.write_register(address=100, value=42)
    >>> await client.close()
"""
from __future__ import annotations

import logging
from typing import Any, Optional

from vyra_base.com.industrial.modbus.base import ModbusBaseClient
from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if pymodbus TCP is available
try:
    from pymodbus.client import AsyncModbusTcpClient
    MODBUS_TCP_AVAILABLE = True
except ImportError:
    MODBUS_TCP_AVAILABLE = False


class ModbusTCPClient(ModbusBaseClient):
    """
    Modbus TCP Client for communication over Ethernet.
    
    Features:
    - Modbus over TCP/IP (port 502)
    - Async communication
    - Automatic reconnection
    - Full Modbus function code support
    
    Args:
        host: Modbus server hostname/IP address
        port: Modbus server port (default: 502)
        unit_id: Modbus unit ID (default: 1)
        timeout: Connection timeout in seconds (default: 10.0)
    
    Example:
        >>> client = ModbusTCPClient(host="192.168.1.100")
        >>> await client.connect()
        >>> 
        >>> # Read holding registers
        >>> values = await client.read_holding_registers(address=0, count=10)
        >>> print(f"Values: {values}")
        >>> 
        >>> # Write single register
        >>> await client.write_register(address=100, value=42)
        >>> 
        >>> await client.close()
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        timeout: float = 10.0,
    ):
        """
        Initialize Modbus TCP client.
        
        Args:
            host: Modbus server hostname/IP
            port: TCP port (default: 502)
            unit_id: Modbus unit ID (default: 1)
            timeout: Connection timeout in seconds
        """
        if not MODBUS_TCP_AVAILABLE:
            raise ImportError(
                "pymodbus not installed. Install with: pip install pymodbus"
            )
        
        super().__init__(unit_id=unit_id, timeout=timeout)
        
        self.host = host
        self.port = port
        self._client: Optional[AsyncModbusTcpClient] = None
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish TCP connection to Modbus server."""
        try:
            Logger.info(f"🔌 Connecting to Modbus TCP: {self.host}:{self.port}")
            
            self._client = AsyncModbusTcpClient(
                host=self.host,
                port=self.port,
                timeout=self.timeout
            )
            
            await self._client.connect()
            
            if not self._client.connected:
                raise ConnectionError("Modbus TCP connection failed")
            
            self._connected = True
            Logger.info(f"✅ Connected to Modbus TCP: {self.host}:{self.port}")
            
        except Exception as e:
            Logger.error(f"❌ Failed to connect to Modbus TCP: {e}")
            self._connected = False
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close TCP connection to Modbus server."""
        if self._client:
            Logger.info(f"🔌 Closing Modbus TCP connection: {self.host}:{self.port}")
            self._client.close()
            self._client = None
            self._connected = False
            Logger.info("✅ Modbus TCP connection closed")
    
    def _get_client(self) -> Optional[AsyncModbusTcpClient]:
        """Get underlying pymodbus TCP client."""
        if not self._client:
            raise ConnectionError("Modbus TCP client not initialized")
        return self._client
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
