# Example 07 — Async TCP & UDP

Demonstrates the standalone async TCP and UDP client/server classes
from `vyra_base.com.external.tcp` and `vyra_base.com.external.udp`.

No external dependencies required — both use Python's `asyncio` stdlib.

## Files

| File | Description |
|------|-------------|
| `01_tcp_echo.py` | TCP echo server + JSON client demo |
| `02_udp_echo.py` | UDP datagram server + JSON client demo |

## Quick Start

```bash
cd vyra_base_python
python examples/07_tcp_udp/01_tcp_echo.py
python examples/07_tcp_udp/02_udp_echo.py
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
