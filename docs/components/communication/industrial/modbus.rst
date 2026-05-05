# vyra_base.com.industrial.modbus

Modbus Industrial Protocol Module
=================================

Modbus protocol support for PLC and field device communication.
Split into base/tcp/rtu for clear separation of transport layers.

**Architecture:**
- base/: Common Modbus functionality and base classes
- tcp/: Modbus over TCP/IP (Ethernet)
- rtu/: Modbus over Serial (RS232/RS485)
- callable.py: VYRA Callable wrapper for Modbus operations
- provider.py: VYRA Provider for Modbus integration

**Supported Features:**
- Read/Write Coils (0x01, 0x05, 0x0F)
- Read Discrete Inputs (0x02)
- Read/Write Holding Registers (0x03, 0x06, 0x10)
- Read Input Registers (0x04)

**Usage:**

.. code-block:: python

    # Modbus TCP
    from vyra_base.com.industrial.modbus.tcp import ModbusTCPClient
    
    async with ModbusTCPClient(host="192.168.1.10") as client:
        values = await client.read_holding_registers(address=0, count=10)
        await client.write_register(address=100, value=42)
    
    # Modbus RTU
    from vyra_base.com.industrial.modbus.rtu import ModbusRTUClient
    
    async with ModbusRTUClient(port="/dev/ttyUSB0", baudrate=19200) as client:
        values = await client.read_holding_registers(address=0, count=10)
