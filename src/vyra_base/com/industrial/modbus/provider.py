"""
Modbus Protocol Provider

Northbound Modbus TCP/RTU communication for SCADA/MES integration.
Uses pymodbus for protocol implementation.
"""

import logging
from typing import Optional, Dict, Any, Literal
from vyra_base.com.providers import AbstractProtocolProvider
from vyra_base.com.core.types import ProtocolType, VyraCallable, VyraSpeaker, VyraJob
from vyra_base.com.core.exceptions import ProviderError, InterfaceError

logger = logging.getLogger(__name__)

# Optional import
try:
    from pymodbus.client import ModbusTcpClient, ModbusSerialClient
    from pymodbus.exceptions import ModbusException
    MODBUS_AVAILABLE = True
except ImportError:
    ModbusTcpClient = None
    ModbusSerialClient = None
    ModbusException = Exception
    MODBUS_AVAILABLE = False


class ModbusProvider(AbstractProtocolProvider):
    """
    Modbus TCP/RTU Protocol Provider
    
    Northbound communication for SCADA/MES systems.
    Supports reading/writing registers and coils.
    
    Args:
        mode: "tcp" or "rtu"
        host: Modbus server host (TCP mode)
        port: Modbus server port (TCP mode, default: 502)
        serial_port: Serial port path (RTU mode, e.g., "/dev/ttyUSB0")
        baudrate: Serial baudrate (RTU mode, default: 9600)
        parity: Serial parity (RTU mode, default: "N")
        stopbits: Serial stopbits (RTU mode, default: 1)
        bytesize: Serial bytesize (RTU mode, default: 8)
        timeout: Communication timeout in seconds (default: 3.0)
        unit_id: Modbus slave/unit ID (default: 1)
    
    Example:
        # TCP Mode
        provider = ModbusProvider(
            mode="tcp",
            host="192.168.1.100",
            port=502
        )
        
        # RTU Mode
        provider = ModbusProvider(
            mode="rtu",
            serial_port="/dev/ttyUSB0",
            baudrate=9600
        )
    """
    
    def __init__(
        self,
        mode: Literal["tcp", "rtu"] = "tcp",
        # TCP parameters
        host: str = "localhost",
        port: int = 502,
        # RTU parameters
        serial_port: Optional[str] = None,
        baudrate: int = 9600,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
        # Common parameters
        timeout: float = 3.0,
        unit_id: int = 1,
    ):
        super().__init__(ProtocolType.MODBUS)
        
        if not MODBUS_AVAILABLE:
            raise ProviderError(
                "Modbus not available. Install with: pip install pymodbus"
            )
        
        self._mode = mode
        self._host = host
        self._port = port
        self._serial_port = serial_port
        self._baudrate = baudrate
        self._parity = parity
        self._stopbits = stopbits
        self._bytesize = bytesize
        self._timeout = timeout
        self._unit_id = unit_id
        self._client: Optional[Any] = None
    
    async def initialize(self) -> None:
        """Initialize Modbus client connection"""
        if self._initialized:
            return
        
        try:
            if self._mode == "tcp":
                self._client = ModbusTcpClient(
                    host=self._host,
                    port=self._port,
                    timeout=self._timeout
                )
                logger.info(f"Connecting to Modbus TCP {self._host}:{self._port}")
            
            elif self._mode == "rtu":
                if not self._serial_port:
                    raise ProviderError("serial_port required for RTU mode")
                
                self._client = ModbusSerialClient(
                    port=self._serial_port,
                    baudrate=self._baudrate,
                    parity=self._parity,
                    stopbits=self._stopbits,
                    bytesize=self._bytesize,
                    timeout=self._timeout
                )
                logger.info(f"Connecting to Modbus RTU {self._serial_port}")
            
            else:
                raise ProviderError(f"Invalid mode: {self._mode}")
            
            # Connect
            if not self._client.connect():
                raise ProviderError("Failed to connect to Modbus server")
            
            self._initialized = True
            logger.info(f"✅ Modbus {self._mode.upper()} provider initialized")
        
        except Exception as e:
            raise ProviderError(f"Modbus initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Close Modbus connection"""
        if self._client:
            self._client.close()
            self._client = None
            logger.info("Modbus connection closed")
        
        self._initialized = False
    
    async def is_available(self) -> bool:
        """Check if Modbus connection is active"""
        if not self._initialized or not self._client:
            return False
        
        return self._client.connected
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create Modbus callable interface
        
        Args:
            name: Interface name (not used for Modbus)
            callback: Server callback (not supported for Modbus)
            **kwargs: Additional arguments
        
        Returns:
            ModbusCallable instance
        
        Note:
            Modbus is client-only (no server callback support)
        """
        if not self._initialized:
            raise InterfaceError("Provider not initialized")
        
        if callback:
            logger.warning("Modbus provider does not support server callbacks")
        
        from vyra_base.com.industrial.modbus.callable import ModbusCallable
        
        callable_obj = ModbusCallable(
            client=self._client,
            unit_id=self._unit_id,
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
        Create Modbus speaker (not supported)
        
        Modbus is request/response only, no pub/sub support.
        """
        raise NotImplementedError(
            "Modbus does not support speaker interfaces (no pub/sub)"
        )
    
    async def create_job(
        self,
        name: str,
        execute_callback: Optional[callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create Modbus job (not supported)
        
        Modbus is request/response only, no long-running jobs.
        """
        raise NotImplementedError(
            "Modbus does not support job interfaces"
        )
    
    # Convenience methods for direct register access
    
    def read_holding_registers(
        self,
        address: int,
        count: int = 1,
        unit_id: Optional[int] = None
    ):
        """Read holding registers (function code 3)"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        return self._client.read_holding_registers(
            address=address,
            count=count,
            slave=unit_id or self._unit_id
        )
    
    def write_register(
        self,
        address: int,
        value: int,
        unit_id: Optional[int] = None
    ):
        """Write single register (function code 6)"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        return self._client.write_register(
            address=address,
            value=value,
            slave=unit_id or self._unit_id
        )
    
    def read_coils(
        self,
        address: int,
        count: int = 1,
        unit_id: Optional[int] = None
    ):
        """Read coils (function code 1)"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        return self._client.read_coils(
            address=address,
            count=count,
            slave=unit_id or self._unit_id
        )
    
    def write_coil(
        self,
        address: int,
        value: bool,
        unit_id: Optional[int] = None
    ):
        """Write single coil (function code 5)"""
        if not self._client:
            raise ProviderError("Client not initialized")
        
        return self._client.write_coil(
            address=address,
            value=value,
            slave=unit_id or self._unit_id
        )
