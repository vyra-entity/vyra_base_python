# Industrial Communication Protocols

Northbound protocols for SCADA/MES integration (Modbus, OPC UA).

## Overview

The Industrial Layer provides protocols for communicating with higher-level industrial systems:

| Protocol | Type        | Use Case              | Install                    |
|----------|-------------|-----------------------|----------------------------|
| Modbus   | TCP/RTU     | PLC, SCADA           | `pip install pymodbus`     |
| OPC UA   | Client/Sub  | MES, SCADA, Industry | `pip install asyncua`      |

**Important**: These are **northbound** protocols for integration with SCADA/MES systems, not for field device communication.

## Modbus Protocol

### Features
- **TCP Mode**: Modbus TCP over Ethernet
- **RTU Mode**: Modbus RTU over serial (RS-485)
- **Operations**: Read/write registers, coils, discrete inputs
- **Multi-slave**: Support for multiple unit IDs

### Usage

#### TCP Mode

```python
from vyra_base.com.industrial import ModbusProvider

# Connect to Modbus TCP server
provider = ModbusProvider(
    mode="tcp",
    host="192.168.1.100",
    port=502,
    unit_id=1
)
await provider.initialize()

# Create callable
modbus = await provider.create_callable("modbus_scada")

# Read holding registers
result = await modbus.call({
    "operation": "read_holding",
    "address": 100,
    "count": 10
})
print(result["data"])  # [1, 2, 3, ...]

# Write register
await modbus.call({
    "operation": "write_register",
    "address": 100,
    "value": 42
})

# Cleanup
await provider.shutdown()
```

#### RTU Mode

```python
# Connect to Modbus RTU device
provider = ModbusProvider(
    mode="rtu",
    serial_port="/dev/ttyUSB0",
    baudrate=9600,
    parity="N",
    stopbits=1,
    bytesize=8,
    unit_id=1
)
await provider.initialize()

modbus = await provider.create_callable("modbus_plc")

# Read coils
result = await modbus.call({
    "operation": "read_coils",
    "address": 0,
    "count": 16
})
print(result["data"])  # [True, False, True, ...]
```

### Supported Operations

**Read Operations**:
- `read_holding`: Read holding registers (FC 03)
- `read_input`: Read input registers (FC 04)
- `read_coils`: Read coils (FC 01)
- `read_discrete`: Read discrete inputs (FC 02)

**Write Operations**:
- `write_register`: Write single register (FC 06)
- `write_registers`: Write multiple registers (FC 16)
- `write_coil`: Write single coil (FC 05)
- `write_coils`: Write multiple coils (FC 15)

### Request Format

```python
{
    "operation": "read_holding",  # Operation type
    "address": 100,               # Register/coil address
    "count": 10,                  # Number of items (for reads)
    "value": 42,                  # Value (for single writes)
    "values": [1, 2, 3],         # Values (for multiple writes)
    "unit_id": 1                  # Optional unit ID override
}
```

### Response Format

```python
{
    "success": True,              # Operation success
    "data": [1, 2, 3, ...],      # Data (for reads)
    "error": None                 # Error message (if failed)
}
```

### Configuration

```python
ModbusProvider(
    mode="tcp",                   # "tcp" or "rtu"
    # TCP parameters
    host="localhost",             # Modbus server host
    port=502,                     # Modbus server port
    # RTU parameters
    serial_port="/dev/ttyUSB0",  # Serial port
    baudrate=9600,                # Baud rate
    parity="N",                   # Parity (N/E/O)
    stopbits=1,                   # Stop bits
    bytesize=8,                   # Byte size
    # Common
    timeout=3.0,                  # Timeout in seconds
    unit_id=1                     # Default unit/slave ID
)
```

## OPC UA Protocol

### Features
- **Client Mode**: Read/write nodes, call methods
- **Subscriptions**: Monitor node data changes
- **Security**: Username/password, certificates, encryption
- **Browsing**: Discover server address space

### Usage

#### Basic Connection

```python
from vyra_base.com.industrial import OpcuaProvider

# Anonymous connection
provider = OpcuaProvider(
    endpoint="opc.tcp://192.168.1.100:4840"
)
await provider.initialize()

# Create callable
opcua = await provider.create_callable("opcua_mes")

# Read node
result = await opcua.call({
    "operation": "read",
    "node_id": "ns=2;s=Temperature"
})
print(result["data"])  # 23.5

# Write node
await opcua.call({
    "operation": "write",
    "node_id": "ns=2;s=Setpoint",
    "value": 25.0
})

# Browse nodes
result = await opcua.call({
    "operation": "browse",
    "node_id": "ns=2;s=Sensors"
})
for child in result["data"]:
    print(f"{child['browse_name']}: {child['node_id']}")

# Cleanup
await provider.shutdown()
```

#### Authenticated Connection

```python
provider = OpcuaProvider(
    endpoint="opc.tcp://192.168.1.100:4840",
    username="operator",
    password="secret"
)
await provider.initialize()
```

#### Secure Connection

```python
provider = OpcuaProvider(
    endpoint="opc.tcp://192.168.1.100:4840",
    certificate="/path/to/client-cert.pem",
    private_key="/path/to/client-key.pem",
    security_mode="SignAndEncrypt",
    security_policy="Basic256Sha256"
)
await provider.initialize()
```

#### Method Calls

```python
result = await opcua.call({
    "operation": "call_method",
    "node_id": "ns=2;s=Controller",
    "method_id": "ns=2;s=StartProcess",
    "args": [100, "auto"]
})
print(result["data"])  # Method return value
```

#### Subscriptions

```python
def on_change(data):
    print(f"Temperature changed to {data['value']} at {data['timestamp']}")

# Create subscription speaker
speaker = await provider.create_speaker(
    "ns=2;s=Temperature",
    callback=on_change
)

# Speaker automatically monitors node changes
# Callback called on every data change
```

### Supported Operations

**Node Operations**:
- `read`: Read node value
- `write`: Write node value
- `read_attributes`: Read multiple node attributes
- `browse`: Browse node children

**Server Operations**:
- `get_root`: Get root node ID
- `get_objects`: Get objects node ID

**Method Operations**:
- `call_method`: Call OPC UA method

### Request Format

```python
{
    "operation": "read",          # Operation type
    "node_id": "ns=2;s=MyVar",   # Node identifier
    "value": 42,                  # Value (for write)
    "method_id": "ns=2;s=Method",# Method node (for call_method)
    "args": [1, 2, 3]            # Method arguments
}
```

### Response Format

```python
{
    "success": True,              # Operation success
    "data": 42,                   # Result data
    "error": None                 # Error message (if failed)
}
```

### Configuration

```python
OpcuaProvider(
    endpoint="opc.tcp://...",    # Server endpoint
    username=None,                # Auth username
    password=None,                # Auth password
    certificate=None,             # Client cert path
    private_key=None,             # Client key path
    security_mode="None",         # None/Sign/SignAndEncrypt
    security_policy=None,         # Security policy
    timeout=10.0                  # Connection timeout
)
```

## Integration with Factory

Use InterfaceFactory for automatic protocol fallback:

```python
from vyra_base.com import InterfaceFactory, ProtocolType

# Register Modbus provider
modbus_provider = ModbusProvider(mode="tcp", host="192.168.1.100")
await modbus_provider.initialize()

from vyra_base.com.providers import ProviderRegistry
registry = ProviderRegistry()
registry.register_provider(ProtocolType.MODBUS, modbus_provider)

# Create callable via factory
callable = await InterfaceFactory.create_callable(
    "scada_interface",
    protocols=[ProtocolType.MODBUS, ProtocolType.REST]
)
```

## Use Cases

### Modbus Use Cases

✅ **Good for**:
- Reading PLC registers
- Writing setpoints to controllers
- Polling sensor values
- SCADA integration

❌ **Not suitable for**:
- High-frequency data (>10 Hz)
- Large data transfers
- Real-time control loops
- Pub/sub patterns

### OPC UA Use Cases

✅ **Good for**:
- MES integration
- Historical data access
- Complex data structures
- Secure communication
- Method calls (remote procedures)
- Subscriptions (data change monitoring)

❌ **Not suitable for**:
- Simple register access (use Modbus)
- Low-level field devices
- Real-time deterministic control

## Performance Considerations

### Modbus

- **Latency**: 10-100ms per request
- **Throughput**: ~100 requests/second (TCP)
- **Optimization**: Use bulk reads (`read_registers` with count>1)

```python
# ✅ Good - Bulk read
result = await modbus.call({
    "operation": "read_holding",
    "address": 100,
    "count": 100  # Read 100 registers at once
})

# ❌ Avoid - Multiple single reads
for i in range(100):
    result = await modbus.call({
        "operation": "read_holding",
        "address": 100 + i,
        "count": 1
    })
```

### OPC UA

- **Latency**: 5-50ms per request
- **Throughput**: ~1000 requests/second
- **Optimization**: Use subscriptions for monitoring, not polling

```python
# ✅ Good - Subscription
speaker = await provider.create_speaker(
    "ns=2;s=Temperature",
    callback=on_change
)

# ❌ Avoid - Polling
while True:
    result = await opcua.call({
        "operation": "read",
        "node_id": "ns=2;s=Temperature"
    })
    await asyncio.sleep(0.1)
```

## Security Best Practices

### Modbus

- **Network Segmentation**: Isolate Modbus network
- **Firewall**: Restrict port 502 access
- **Authentication**: Use VPN for remote access
- **Monitoring**: Log all write operations

### OPC UA

- **Use Security**: Enable SignAndEncrypt mode
- **Certificates**: Use strong PKI certificates
- **Authentication**: Require username/password
- **Permissions**: Configure node access rights
- **Audit**: Enable server audit logging

```python
# ✅ Good - Secure OPC UA
provider = OpcuaProvider(
    endpoint="opc.tcp://server:4840",
    username="operator",
    password="strong_password",
    certificate="/path/to/cert.pem",
    private_key="/path/to/key.pem",
    security_mode="SignAndEncrypt"
)

# ❌ Avoid - Insecure production use
provider = OpcuaProvider(
    endpoint="opc.tcp://server:4840"
)  # Anonymous, no encryption
```

## Error Handling

```python
from vyra_base.com.core.exceptions import CallableError, ProviderError

try:
    # Modbus read
    result = await modbus.call({
        "operation": "read_holding",
        "address": 100,
        "count": 10
    })
    
    if not result["success"]:
        logger.error(f"Modbus error: {result['error']}")
    else:
        data = result["data"]

except CallableError as e:
    logger.error(f"Communication error: {e}")
except ProviderError as e:
    logger.error(f"Provider error: {e}")
```

## Testing

Mock providers for testing:

```python
class MockModbusProvider:
    """Mock Modbus for testing"""
    
    def __init__(self):
        self.registers = {i: 0 for i in range(1000)}
        self.coils = {i: False for i in range(1000)}
    
    async def initialize(self):
        pass
    
    async def create_callable(self, name, **kwargs):
        return MockModbusCallable(self.registers, self.coils)

@pytest.mark.asyncio
async def test_modbus_read(mock_modbus):
    mock_modbus.registers[100] = 42
    
    result = await callable.call({
        "operation": "read_holding",
        "address": 100,
        "count": 1
    })
    
    assert result["success"]
    assert result["data"][0] == 42
```

## Troubleshooting

### Modbus Issues

**Connection Refused**:
- Check host/port configuration
- Verify Modbus server is running
- Check firewall rules

**Timeout**:
- Increase timeout parameter
- Check network latency
- Verify unit ID is correct

**Invalid Register**:
- Check register address range
- Verify unit ID
- Check register type (holding vs input)

### OPC UA Issues

**BadCertificateInvalid**:
- Verify certificate path
- Check certificate expiration
- Trust server certificate

**BadUserAccessDenied**:
- Check username/password
- Verify user permissions
- Check node access rights

**BadTimeout**:
- Increase timeout parameter
- Check network connection
- Verify server is responding

## See Also

- [Core Components](../core/README.md) - Factory and decorators
- [Providers](../providers/README.md) - Provider pattern
- [Main README](../README.md) - Architecture overview
