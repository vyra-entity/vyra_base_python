"""
Modbus Industrial Protocol Module

Modbus protocol support for PLC and field device communication.
Split into base/tcp/rtu for clear separation of transport layers.

Architecture:
    - base/: Common Modbus functionality and base classes
    - tcp/: Modbus over TCP/IP (Ethernet)
    - rtu/: Modbus over Serial (RS232/RS485)
    - callable.py: VYRA Callable wrapper for Modbus operations
    - provider.py: VYRA Provider for Modbus integration

Supported Features:
    - Read/Write Coils (0x01, 0x05, 0x0F)
    - Read Discrete Inputs (0x02)
    - Read/Write Holding Registers (0x03, 0x06, 0x10)
    - Read Input Registers (0x04)

Usage:
    >>> # Modbus TCP
    >>> from vyra_base.com.industrial.modbus.tcp import ModbusTCPClient
    >>> 
    >>> async with ModbusTCPClient(host="192.168.1.10") as client:
    ...     values = await client.read_holding_registers(address=0, count=10)
    ...     await client.write_register(address=100, value=42)
    >>> 
    >>> # Modbus RTU
    >>> from vyra_base.com.industrial.modbus.rtu import ModbusRTUClient
    >>> 
    >>> async with ModbusRTUClient(port="/dev/ttyUSB0", baudrate=19200) as client:
    ...     values = await client.read_holding_registers(address=0, count=10)
"""
import logging

logger = logging.getLogger(__name__)

# Try importing base classes
try:
    from vyra_base.com.industrial.modbus.base import (
        ModbusBaseClient,
        MODBUS_BASE_AVAILABLE,
    )
    _base_available = MODBUS_BASE_AVAILABLE
except ImportError as e:
    ModbusBaseClient = None
    _base_available = False
    logger.debug(f"⚠️  Modbus base unavailable: {e}")

# Try importing TCP
try:
    from vyra_base.com.industrial.modbus.tcp import (
        ModbusTCPClient,
        MODBUS_TCP_AVAILABLE,
    )
    _tcp_available = MODBUS_TCP_AVAILABLE
except ImportError as e:
    ModbusTCPClient = None
    _tcp_available = False
    logger.debug(f"⚠️  Modbus TCP unavailable: {e}")

# Try importing RTU
try:
    from vyra_base.com.industrial.modbus.rtu import (
        ModbusRTUClient,
        MODBUS_RTU_AVAILABLE,
    )
    _rtu_available = MODBUS_RTU_AVAILABLE
except ImportError as e:
    ModbusRTUClient = None
    _rtu_available = False
    logger.debug(f"⚠️  Modbus RTU unavailable: {e}")

# Try importing callable (VYRA abstraction)
try:
    from vyra_base.com.industrial.modbus.callable import ModbusCallable
    _callable_available = True
except ImportError as e:
    ModbusCallable = None
    _callable_available = False
    logger.debug(f"⚠️  Modbus callable unavailable: {e}")

# Try importing provider
try:
    from vyra_base.com.industrial.modbus.provider import ModbusProvider
    _provider_available = True
except ImportError as e:
    ModbusProvider = None
    _provider_available = False
    logger.debug(f"⚠️  Modbus provider unavailable: {e}")

# Modbus is available if at least one transport (TCP or RTU) is available
MODBUS_AVAILABLE = _base_available and (_tcp_available or _rtu_available)

if MODBUS_AVAILABLE:
    if _tcp_available and _rtu_available:
        logger.info("✅ Modbus fully available (base + TCP + RTU)")
    elif _tcp_available:
        logger.info("✅ Modbus TCP available")
    elif _rtu_available:
        logger.info("✅ Modbus RTU available")
else:
    logger.debug("❌ Modbus unavailable")

# Backward compatibility: Export legacy ModbusClient as ModbusTCPClient
ModbusClient = ModbusTCPClient

__all__ = [
    # Availability flags
    "MODBUS_AVAILABLE",
    "MODBUS_TCP_AVAILABLE",
    "MODBUS_RTU_AVAILABLE",
    # Base classes
    "ModbusBaseClient",
    # Transport implementations
    "ModbusTCPClient",
    "ModbusRTUClient",
    # VYRA abstractions
    "ModbusCallable",
    "ModbusProvider",
    # Backward compatibility
    "ModbusClient",  # Alias for ModbusTCPClient
]
