"""
Modbus TCP Module

Modbus over TCP/IP for industrial communication.

Components:
    - ModbusTCPClient: Async Modbus TCP client
    - MODBUS_TCP_AVAILABLE: Availability flag

Usage:
    >>> from vyra_base.com.industrial.modbus.tcp import ModbusTCPClient
    >>> 
    >>> async with ModbusTCPClient(host="192.168.1.10") as client:
    ...     values = await client.read_holding_registers(0, 10)
    ...     await client.write_register(100, 42)
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.industrial.modbus.tcp.client import (
        ModbusTCPClient,
        MODBUS_TCP_AVAILABLE,
    )
    logger.debug("✅ Modbus TCP available")
    
except ImportError as e:
    ModbusTCPClient = None
    MODBUS_TCP_AVAILABLE = False
    logger.debug(f"⚠️  Modbus TCP unavailable: {e}")

__all__ = [
    "ModbusTCPClient",
    "MODBUS_TCP_AVAILABLE",
]
