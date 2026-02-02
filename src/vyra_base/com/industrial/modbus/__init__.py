"""
Modbus Protocol Support

Modbus TCP/RTU for SCADA/MES northbound communication.
"""

from vyra_base.com.industrial.modbus.provider import ModbusProvider
from vyra_base.com.industrial.modbus.callable import ModbusCallable

__all__ = ["ModbusProvider", "ModbusCallable"]
