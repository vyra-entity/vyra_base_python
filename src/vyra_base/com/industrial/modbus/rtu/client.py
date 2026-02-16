"""
Modbus RTU Client

Modbus over Serial (RS232/RS485) for communication with PLCs and field devices.

Example:
    >>> client = ModbusRTUClient(port="/dev/ttyUSB0", baudrate=9600)
    >>> await client.connect()
    >>> values = await client.read_holding_registers(address=0, count=10)
    >>> await client.write_register(address=100, value=42)
    >>> await client.close()
"""
from __future__ import annotations

import logging
from typing import Any, Optional

from vyra_base.com.industrial.modbus.base import ModbusBaseClient
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if pymodbus RTU is available
try:
    from pymodbus.client import AsyncModbusSerialClient
    MODBUS_RTU_AVAILABLE = True
except ImportError:
    MODBUS_RTU_AVAILABLE = False


class ModbusRTUClient(ModbusBaseClient):
    """
    Modbus RTU Client for communication over serial port (RS232/RS485).
    
    Features:
    - Modbus RTU protocol
    - RS232/RS485 serial communication
    - Async communication
    - Configurable baudrate and parity
    - Full Modbus function code support
    
    Args:
        port: Serial port device (e.g., '/dev/ttyUSB0' or 'COM3')
        baudrate: Serial baudrate (default: 9600)
        bytesize: Number of data bits (default: 8)
        parity: Parity checking ('N'=None, 'E'=Even, 'O'=Odd) (default: 'N')
        stopbits: Number of stop bits (default: 1)
        unit_id: Modbus unit ID (default: 1)
        timeout: Communication timeout in seconds (default: 3.0)
    
    Example:
        >>> # RS485 connection
        >>> client = ModbusRTUClient(
        ...     port="/dev/ttyUSB0",
        ...     baudrate=19200,
        ...     parity='E',
        ...     unit_id=1
        ... )
        >>> await client.connect()
        >>> 
        >>> # Read holding registers
        >>> values = await client.read_holding_registers(address=0, count=10)
        >>> 
        >>> # Write single register
        >>> await client.write_register(address=100, value=42)
        >>> 
        >>> await client.close()
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        bytesize: int = 8,
        parity: str = 'N',
        stopbits: int = 1,
        unit_id: int = 1,
        timeout: float = 3.0,
    ):
        """
        Initialize Modbus RTU client.
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Serial baudrate (9600, 19200, 38400, etc.)
            bytesize: Data bits (7, 8)
            parity: Parity ('N', 'E', 'O')
            stopbits: Stop bits (1, 2)
            unit_id: Modbus unit ID
            timeout: Communication timeout
        """
        if not MODBUS_RTU_AVAILABLE:
            raise ImportError(
                "pymodbus not installed. Install with: pip install pymodbus"
            )
        
        super().__init__(unit_id=unit_id, timeout=timeout)
        
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self._client: Optional[AsyncModbusSerialClient] = None
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish serial connection to Modbus RTU device."""
        try:
            logger.info(
                f"🔌 Connecting to Modbus RTU: {self.port} "
                f"({self.baudrate} {self.bytesize}{self.parity}{self.stopbits})"
            )
            
            self._client = AsyncModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout
            )
            
            await self._client.connect()
            
            if not self._client.connected:
                raise ConnectionError("Modbus RTU connection failed")
            
            self._connected = True
            logger.info(f"✅ Connected to Modbus RTU: {self.port}")
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to Modbus RTU: {e}")
            self._connected = False
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close serial connection to Modbus RTU device."""
        if self._client:
            logger.info(f"🔌 Closing Modbus RTU connection: {self.port}")
            self._client.close()
            self._client = None
            self._connected = False
            logger.info("✅ Modbus RTU connection closed")
    
    def _get_client(self) -> AsyncModbusSerialClient:
        """Get underlying pymodbus serial client."""
        if not self._client:
            raise ConnectionError("Modbus RTU client not initialized")
        return self._client
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
