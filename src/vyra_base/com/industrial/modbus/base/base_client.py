"""
Modbus Base Client

Common functionality for Modbus TCP and RTU implementations.
Provides abstract base class and shared utilities.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, List, Optional

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if pymodbus is available
try:
    from pymodbus.exceptions import ModbusException
    PYMODBUS_AVAILABLE = True
except ImportError:
    PYMODBUS_AVAILABLE = False


class ModbusBaseClient(ABC):
    """
    Abstract base class for Modbus clients (TCP and RTU).
    
    Provides common interface and shared functionality for:
    - Connection management
    - Register read/write operations
    - Coil read/write operations
    - Error handling
    
    Subclasses must implement:
    - connect()
    - close()
    - _get_client() - returns the underlying pymodbus client
    """
    
    def __init__(
        self,
        unit_id: int = 1,
        timeout: float = 10.0,
    ):
        """
        Initialize Modbus base client.
        
        Args:
            unit_id: Modbus unit ID (default: 1)
            timeout: Operation timeout in seconds
        """
        if not PYMODBUS_AVAILABLE:
            raise ImportError(
                "pymodbus not installed. Install with: pip install pymodbus"
            )
        
        self.unit_id = unit_id
        self.timeout = timeout
        self._connected = False
    
    @abstractmethod
    async def connect(self) -> None:
        """Establish connection to Modbus server/device."""
        pass
    
    @abstractmethod
    async def close(self) -> None:
        """Close connection to Modbus server/device."""
        pass
    
    @abstractmethod
    def _get_client(self) -> Any:
        """Get underlying pymodbus client instance."""
        pass
    
    @property
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected
    
    def _require_connection(self):
        """Raise error if not connected."""
        if not self._connected:
            raise ConnectionError("Not connected to Modbus device")
    
    @ErrorTraceback.w_check_error_exist
    async def read_coils(
        self,
        address: int,
        count: int = 1,
    ) -> List[bool]:
        """
        Read coil status (0x01).
        
        Args:
            address: Start address
            count: Number of coils to read
            
        Returns:
            List of coil values (True/False)
        """
        self._require_connection()
        
        try:
            Logger.debug(f"📖 Reading {count} coils from address {address}")
            
            result = await self._get_client().read_coils(
                address=address,
                count=count,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to read coils: {result}")
            
            return result.bits[:count]
            
        except Exception as e:
            Logger.error(f"❌ Failed to read coils: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def read_discrete_inputs(
        self,
        address: int,
        count: int = 1,
    ) -> List[bool]:
        """
        Read discrete inputs (0x02).
        
        Args:
            address: Start address
            count: Number of inputs to read
            
        Returns:
            List of input values (True/False)
        """
        self._require_connection()
        
        try:
            Logger.debug(f"📖 Reading {count} discrete inputs from address {address}")
            
            result = await self._get_client().read_discrete_inputs(
                address=address,
                count=count,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to read discrete inputs: {result}")
            
            return result.bits[:count]
            
        except Exception as e:
            Logger.error(f"❌ Failed to read discrete inputs: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def read_holding_registers(
        self,
        address: int,
        count: int = 1,
    ) -> List[int]:
        """
        Read holding registers (0x03).
        
        Args:
            address: Start address
            count: Number of registers to read
            
        Returns:
            List of register values (16-bit integers)
        """
        self._require_connection()
        
        try:
            Logger.debug(f"📖 Reading {count} holding registers from address {address}")
            
            result = await self._get_client().read_holding_registers(
                address=address,
                count=count,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to read holding registers: {result}")
            
            return result.registers
            
        except Exception as e:
            Logger.error(f"❌ Failed to read holding registers: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def read_input_registers(
        self,
        address: int,
        count: int = 1,
    ) -> List[int]:
        """
        Read input registers (0x04).
        
        Args:
            address: Start address
            count: Number of registers to read
            
        Returns:
            List of register values (16-bit integers)
        """
        self._require_connection()
        
        try:
            Logger.debug(f"📖 Reading {count} input registers from address {address}")
            
            result = await self._get_client().read_input_registers(
                address=address,
                count=count,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to read input registers: {result}")
            
            return result.registers
            
        except Exception as e:
            Logger.error(f"❌ Failed to read input registers: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def write_coil(
        self,
        address: int,
        value: bool,
    ) -> None:
        """
        Write single coil (0x05).
        
        Args:
            address: Coil address
            value: Coil value (True/False)
        """
        self._require_connection()
        
        try:
            Logger.debug(f"✍️  Writing coil {address} = {value}")
            
            result = await self._get_client().write_coil(
                address=address,
                value=value,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to write coil: {result}")
            
            Logger.debug(f"✅ Coil {address} written successfully")
            
        except Exception as e:
            Logger.error(f"❌ Failed to write coil: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def write_register(
        self,
        address: int,
        value: int,
    ) -> None:
        """
        Write single holding register (0x06).
        
        Args:
            address: Register address
            value: Register value (16-bit integer)
        """
        self._require_connection()
        
        try:
            Logger.debug(f"✍️  Writing register {address} = {value}")
            
            result = await self._get_client().write_register(
                address=address,
                value=value,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to write register: {result}")
            
            Logger.debug(f"✅ Register {address} written successfully")
            
        except Exception as e:
            Logger.error(f"❌ Failed to write register: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def write_coils(
        self,
        address: int,
        values: List[bool],
    ) -> None:
        """
        Write multiple coils (0x0F).
        
        Args:
            address: Start address
            values: List of coil values
        """
        self._require_connection()
        
        try:
            Logger.debug(f"✍️  Writing {len(values)} coils from address {address}")
            
            result = await self._get_client().write_coils(
                address=address,
                values=values,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to write coils: {result}")
            
            Logger.debug(f"✅ {len(values)} coils written successfully")
            
        except Exception as e:
            Logger.error(f"❌ Failed to write coils: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def write_registers(
        self,
        address: int,
        values: List[int],
    ) -> None:
        """
        Write multiple holding registers (0x10).
        
        Args:
            address: Start address
            values: List of register values
        """
        self._require_connection()
        
        try:
            Logger.debug(f"✍️  Writing {len(values)} registers from address {address}")
            
            result = await self._get_client().write_registers(
                address=address,
                values=values,
                unit=self.unit_id
            )
            
            if result.isError():
                raise ModbusException(f"Failed to write registers: {result}")
            
            Logger.debug(f"✅ {len(values)} registers written successfully")
            
        except Exception as e:
            Logger.error(f"❌ Failed to write registers: {e}")
            raise
