"""
Modbus Base Module

Common functionality for Modbus implementations.
Provides abstract base class and shared utilities.

Components:
    - ModbusBaseClient: Abstract base for Modbus clients
    - PYMODBUS_AVAILABLE: Availability flag

Usage:
    >>> from vyra_base.com.industrial.modbus.base import ModbusBaseClient
    >>> 
    >>> class MyModbusClient(ModbusBaseClient):
    ...     async def connect(self):
    ...         # Implementation
    ...         pass
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.industrial.modbus.base.base_client import (
        ModbusBaseClient,
        PYMODBUS_AVAILABLE,
    )
    MODBUS_BASE_AVAILABLE = PYMODBUS_AVAILABLE
    logger.debug("✅ Modbus base available")
    
except ImportError as e:
    ModbusBaseClient = None
    PYMODBUS_AVAILABLE = False
    MODBUS_BASE_AVAILABLE = False
    logger.debug(f"⚠️  Modbus base unavailable: {e}")

__all__ = [
    "ModbusBaseClient",
    "PYMODBUS_AVAILABLE",
    "MODBUS_BASE_AVAILABLE",
]
