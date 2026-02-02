# External Communication Protocols

Professional-grade external communication protocols for distributed systems.

## Overview

The External Layer provides 5 enterprise protocols for cross-network, cloud, and web communication:

| Protocol   | Type           | Use Case                    | Install                        |
|------------|----------------|----------------------------|--------------------------------|
| Redis      | Pub/Sub + KV   | Caching, messaging         | `pip install redis`            |
| gRPC       | RPC            | High-performance RPC       | `pip install grpcio`           |
| MQTT       | Pub/Sub        | IoT, constrained devices   | `pip install paho-mqtt`        |
| REST       | HTTP API       | Web services               | `pip install aiohttp`          |
| WebSocket  | Bidirectional  | Real-time web apps         | `pip install websockets`       |

## Redis Protocol

### Features
- **Pub/Sub**: Channel-based messaging
- **Streams**: Append-only log with consumer groups
- **Security**: TLS, ACL, username/password
- **Clustering**: Redis Cluster support
- **Patterns**: Wildcard subscriptions

### Usage

```python
from vyra_base.com.external import RedisProvider

# Initialize with TLS
provider = RedisProvider(
    host="redis.example.com",
    port=6379,
    username="admin",
    password="secret",
    ssl=True,
    ssl_ca_certs="/path/to/ca.pem"
)
await provider.initialize()

# Create speaker (publisher)
speaker = await provider.create_speaker("events")
await speaker.shout({"type": "update", "value": 42})

# Create speaker with subscriber callback
def on_message(msg):
    print(f"Received: {msg}")

speaker = await provider.create_speaker("events", callback=on_message)
```

### Configuration

```python
RedisProvider(
    host="localhost",           # Redis server
    port=6379,                  # Redis port
    db=0,                       # Database number
    username=None,              # ACL username
    password=None,              # Auth password
    ssl=False,                  # Enable TLS
    ssl_ca_certs=None,          # CA certificate
    ssl_certfile=None,          # Client cert
    ssl_keyfile=None,           # Client key
    decode_responses=True,      # Auto-decode strings
    max_connections=50          # Connection pool
)
```

## gRPC Protocol

### Features
- **Unix Sockets**: Local high-performance IPC
- **Streaming**: Unary, server-stream, client-stream, bidirectional
- **Protobuf**: Efficient binary serialization
- **TLS**: Optional encryption
- **Load Balancing**: Client-side load balancing

### Usage

```python
from vyra_base.com.external import GrpcProvider

# Server mode
provider = GrpcProvider(
    socket_path="/tmp/my_service.sock",
    is_server=True
)
await provider.initialize()

callable_server = await provider.create_callable(
    "calculate",
    callback=lambda req: {"result": req["x"] + req["y"]}
)

# Client mode
client_provider = GrpcProvider(
    socket_path="/tmp/my_service.sock",
    is_server=False
)
await client_provider.initialize()

callable_client = await client_provider.create_callable("calculate")
result = await callable_client.call({"x": 10, "y": 32})
print(result)  # {"result": 42}
```

### Configuration

```python
GrpcProvider(
    socket_path="/tmp/vyra.sock",  # Unix socket path
    is_server=True,                # Server or client mode
    max_workers=10,                # Server thread pool
    options=[                      # gRPC channel options
        ('grpc.max_send_message_length', 10 * 1024 * 1024),
        ('grpc.max_receive_message_length', 10 * 1024 * 1024)
    ]
)
```

## MQTT Protocol

### Features
- **QoS Levels**: 0 (fire-and-forget), 1 (at least once), 2 (exactly once)
- **Retained Messages**: Last message cached by broker
- **Last Will Testament**: Automatic disconnect notification
- **Wildcards**: `+` (single level), `#` (multi-level)
- **Clean Session**: Persistent or transient subscriptions

### Usage

```python
from vyra_base.com.external import MqttProvider

# Connect to broker
provider = MqttProvider(
    broker="mqtt.example.com",
    port=1883,
    client_id="my_device",
    username="user",
    password="pass",
    qos=1,
    clean_session=True
)
await provider.initialize()

# Publish with QoS
speaker = await provider.create_speaker("sensor/temperature")
await speaker.shout({"value": 23.5, "unit": "°C"})

# Subscribe with pattern
def on_sensor_data(msg):
    print(f"Sensor data: {msg}")

speaker = await provider.create_speaker(
    "sensor/+/data",  # Matches sensor/temp/data, sensor/hum/data
    callback=on_sensor_data
)
```

### Configuration

```python
MqttProvider(
    broker="localhost",         # MQTT broker host
    port=1883,                  # Broker port (8883 for TLS)
    client_id=None,             # Auto-generated if None
    username=None,              # Auth username
    password=None,              # Auth password
    qos=0,                      # Quality of Service (0/1/2)
    retain=False,               # Retain messages
    clean_session=True,         # Clean session flag
    keepalive=60,               # Keepalive interval
    will_topic=None,            # LWT topic
    will_payload=None,          # LWT message
    will_qos=0,                 # LWT QoS
    will_retain=False,          # LWT retain
    tls_enable=False,           # Enable TLS
    tls_ca_certs=None,          # CA certificate
    tls_certfile=None,          # Client cert
    tls_keyfile=None            # Client key
)
```

## REST Protocol

### Features
- **HTTP Methods**: GET, POST, PUT, DELETE, PATCH
- **JSON Payloads**: Automatic serialization/deserialization
- **CORS**: Cross-origin resource sharing
- **TLS**: HTTPS support
- **Middleware**: Request/response processing

### Usage

```python
from vyra_base.com.external import RestProvider

# Start server
provider = RestProvider(
    host="0.0.0.0",
    port=8443,
    ssl_cert="/path/to/cert.pem",
    ssl_key="/path/to/key.pem"
)
await provider.initialize()

# Create endpoint
async def handle_request(request):
    data = request["body"]
    return {"result": data["value"] * 2}

callable = await provider.create_callable(
    "/api/calculate",
    callback=handle_request
)

# Make client request
client_provider = RestProvider(is_server=False)
await client_provider.initialize()

client_callable = await client_provider.create_callable("/api/calculate")
result = await client_callable.call(
    {"value": 21},
    method="POST",
    base_url="https://api.example.com"
)
```

### Configuration

```python
RestProvider(
    host="0.0.0.0",            # Server bind address
    port=8443,                 # Server port
    is_server=True,            # Server or client mode
    ssl_cert=None,             # TLS certificate
    ssl_key=None,              # TLS key
    cors_enabled=True,         # CORS support
    cors_origins=["*"],        # Allowed origins
    timeout=30.0,              # Request timeout
    max_connections=100        # Connection pool
)
```

## WebSocket Protocol

### Features
- **Real-time**: Low-latency bidirectional communication
- **Connection Pooling**: Efficient connection reuse
- **Broadcast**: Send to all connected clients
- **Compression**: Per-message compression
- **Ping/Pong**: Automatic keepalive

### Usage

```python
from vyra_base.com.external import WebSocketProvider

# Server mode
provider = WebSocketProvider(
    host="0.0.0.0",
    port=8765,
    is_server=True
)
await provider.initialize()

# Broadcast messages
speaker = await provider.create_speaker("updates")
await speaker.shout({"event": "status", "value": "online"})

# Client mode
client_provider = WebSocketProvider(
    uri="ws://localhost:8765",
    is_server=False
)
await client_provider.initialize()

# Receive messages
def on_message(msg):
    print(f"Received: {msg}")

client_speaker = await client_provider.create_speaker(
    "updates",
    callback=on_message
)
```

### Configuration

```python
WebSocketProvider(
    host="0.0.0.0",            # Server bind (server mode)
    port=8765,                 # Server port (server mode)
    uri=None,                  # WebSocket URI (client mode)
    is_server=True,            # Server or client mode
    ssl_cert=None,             # TLS certificate
    ssl_key=None,              # TLS key
    compression=None,          # Compression type
    ping_interval=20,          # Keepalive interval
    ping_timeout=20,           # Keepalive timeout
    max_size=10*1024*1024,     # Max message size
    max_queue=32               # Message queue size
)
```

## Protocol Selection

### Automatic Fallback

```python
from vyra_base.com import InterfaceFactory, ProtocolType

# Tries Redis → MQTT → WebSocket
speaker = await InterfaceFactory.create_speaker(
    "events",
    protocols=[
        ProtocolType.REDIS,
        ProtocolType.MQTT,
        ProtocolType.WEBSOCKET
    ]
)
```

### Use Cases

**Redis**: Best for distributed systems, caching, pub/sub with patterns
- ✅ Distributed cache
- ✅ Message broker
- ✅ Session storage
- ❌ Real-time web

**gRPC**: Best for high-performance microservices
- ✅ Low latency RPC
- ✅ Streaming data
- ✅ Load balancing
- ❌ Browser support

**MQTT**: Best for IoT, embedded systems
- ✅ Low bandwidth
- ✅ Unreliable networks
- ✅ Constrained devices
- ❌ Large payloads

**REST**: Best for web APIs, microservices
- ✅ Stateless
- ✅ Cacheable
- ✅ Browser support
- ❌ Real-time updates

**WebSocket**: Best for real-time web applications
- ✅ Low latency
- ✅ Bidirectional
- ✅ Browser support
- ❌ Firewall issues

## Security Considerations

### TLS/SSL

All protocols support TLS:

```python
# Redis
RedisProvider(ssl=True, ssl_ca_certs="/path/to/ca.pem")

# MQTT
MqttProvider(tls_enable=True, tls_ca_certs="/path/to/ca.pem")

# REST
RestProvider(ssl_cert="/path/to/cert.pem", ssl_key="/path/to/key.pem")

# WebSocket
WebSocketProvider(ssl_cert="/path/to/cert.pem", ssl_key="/path/to/key.pem")
```

### Authentication

```python
# Username/Password
RedisProvider(username="admin", password="secret")
MqttProvider(username="device", password="token")

# Certificate-based
RestProvider(
    ssl_cert="/path/to/client-cert.pem",
    ssl_key="/path/to/client-key.pem"
)
```

### Network Segmentation

- **Redis**: Use separate databases per application
- **MQTT**: Use topic-based ACLs
- **REST**: Use API keys, OAuth2
- **gRPC**: Use Unix sockets for local, TLS for remote

## Performance Tuning

### Connection Pooling

```python
# Redis connection pool
RedisProvider(max_connections=100)

# REST connection pool
RestProvider(max_connections=50)
```

### Timeouts

```python
# Request timeouts
RestProvider(timeout=5.0)

# WebSocket keepalive
WebSocketProvider(ping_interval=20, ping_timeout=20)
```

### Message Size Limits

```python
# gRPC message size
GrpcProvider(options=[
    ('grpc.max_send_message_length', 100 * 1024 * 1024),
    ('grpc.max_receive_message_length', 100 * 1024 * 1024)
])

# WebSocket message size
WebSocketProvider(max_size=10 * 1024 * 1024)
```

## Troubleshooting

### Connection Refused
```
ConnectionRefusedError: [Errno 111] Connection refused
```
**Solution**: Check service is running, verify host/port, check firewall

### TLS Handshake Failed
```
ssl.SSLError: [SSL: CERTIFICATE_VERIFY_FAILED]
```
**Solution**: Verify certificate path, check CA trust chain, disable verification (dev only)

### Protocol Not Available
```
ModuleNotFoundError: No module named 'redis'
```
**Solution**: Install required package: `pip install redis`

### Message Too Large
```
ValueError: Message exceeds maximum size
```
**Solution**: Increase max_size, split message, use compression

## Testing

See [../../tests/com/external/](../../tests/com/external/) for unit tests.

```bash
# Install test dependencies
pip install -e ".[test]"

# Run external protocol tests
pytest tests/com/external/ -v

# Test specific protocol
pytest tests/com/external/test_redis_provider.py -v
```
