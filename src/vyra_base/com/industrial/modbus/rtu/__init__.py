"""
Modbus RTU Module

Modbus over Serial (RS232/RS485) for industrial communication.

Components:
    - ModbusRTUClient: Async Modbus RTU client
    - MODBUS_RTU_AVAILABLE: Availability flag

Usage:
    >>> from vyra_base.com.industrial.modbus.rtu import ModbusRTUClient
    >>> 
    >>> async with ModbusRTUClient(port="/dev/ttyUSB0", baudrate=19200) as client:
    ...     values = await client.read_holding_registers(0, 10)
    ...     await client.write_register(100, 42)
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.industrial.modbus.rtu.client import (
        ModbusRTUClient,
        MODBUS_RTU_AVAILABLE,
    )
    logger.debug("✅ Modbus RTU available")
    
except ImportError as e:
    ModbusRTUClient = None
    MODBUS_RTU_AVAILABLE = False
    logger.debug(f"⚠️  Modbus RTU unavailable: {e}")

__all__ = [
    "ModbusRTUClient",
    "MODBUS_RTU_AVAILABLE",
]
