# Example 04 — External Communication

This directory covers all currently available external protocols from
`src/vyra_base/com/external`.

## Coverage Map

| Folder | Description |
|--------|-------------|
| `01_tcp_udp/` | Async TCP/UDP echo and datagram examples |
| `02_grpc/` | gRPC client skeleton with optional runtime demo |
| `03_mqtt/` | MQTT client skeleton with optional runtime demo |
| `04_rest/` | REST client skeleton |
| `05_websocket/` | WebSocket client skeleton |
| `06_shared_memory/` | Shared memory write/read roundtrip |
| `07_modbus/` | Modbus TCP connection skeleton |
| `08_opcua/` | OPC UA client connection skeleton |

## Quick Start

```bash
cd vyra_base_python

# TCP/UDP demos
python examples/04_external_communication/01_tcp_udp/01_tcp_echo.py
python examples/04_external_communication/01_tcp_udp/02_udp_echo.py

# Optional dependency checks / skeletons
python examples/04_external_communication/02_grpc/01_grpc_client_skeleton.py
python examples/04_external_communication/03_mqtt/01_mqtt_client_skeleton.py
python examples/04_external_communication/04_rest/01_rest_client_skeleton.py
python examples/04_external_communication/05_websocket/01_websocket_client_skeleton.py
python examples/04_external_communication/06_shared_memory/01_shared_memory_roundtrip.py
python examples/04_external_communication/07_modbus/01_modbus_tcp_skeleton.py
python examples/04_external_communication/08_opcua/01_opcua_client_skeleton.py
```

## TCP Usage

### Server

```python
from vyra_base.com.external.tcp import AsyncTcpServer

server = AsyncTcpServer(host="0.0.0.0", port=9000)

@server.on_message
async def handle(client_id: str, data: bytes):
    await server.send(client_id, data)  # echo

await server.start()
```

### Client

```python
from vyra_base.com.external.tcp import AsyncTcpClient

async with AsyncTcpClient(host="10.0.0.1", port=9000) as client:
    await client.send_json({"cmd": "START"})
    reply = await client.receive_json()
```

## UDP Usage

### Server

```python
from vyra_base.com.external.udp import AsyncUdpServer

server = AsyncUdpServer(host="0.0.0.0", port=5000)

@server.on_datagram
async def handle(data: bytes, addr):
    await server.send_json({"ok": True}, addr[0], addr[1])

await server.start()
```

### Client

```python
from vyra_base.com.external.udp import AsyncUdpClient

async with AsyncUdpClient() as client:
    await client.send_json({"ping": True}, "10.0.0.1", 5000)
    obj, addr = await client.receive_json()
```

## Multicast

```python
# Server joins multicast group
server = AsyncUdpServer(host="0.0.0.0", port=5000, multicast_group="239.0.0.1")

# Client sends to multicast address
client = AsyncUdpClient(multicast_group="239.0.0.1")
await client.open(bind_port=5000)
await client.send_json({"event": "broadcast"}, "239.0.0.1", 5000)
```

## Features

| Feature | TCP | UDP |
|---------|-----|-----|
| Async context manager | ✅ | ✅ |
| JSON send/receive | ✅ | ✅ |
| Broadcast | ✅ server-side | ✅ subnet |
| Multicast | ❌ | ✅ |
| Auto-reconnect | ✅ client | — |
| Multi-client | ✅ | — |
| Callbacks | connected/disconnected/message | datagram |
