# VYRA Communication Architecture - Visual Overview

## System Architecture

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         VYRA MODULE (Container)                            │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  APPLICATION LAYER                                                   │  │
│  │  ┌────────────┐    ┌────────────┐  ┌────────────┐                    │  │
│  │  │ Component  │    │  Domain    │  │  Service   │                    │  │
│  │  │  Classes   │    │  Logic     │  │  Layer     │                    │  │
│  │  └──────┬─────┘    └──────┬─────┘  └──────┬─────┘                    │  │
│  │         │ @remote_callable│               │                          │  │
│  │         │ @monitored      │               │                          │  │
│  │         └─────────┬───────┴───────────────┘                          │  │
│  └───────────────────┼──────────────────────────────────────────────────┘  │
│                      │                                                     │
│  ┌───────────────────▼──────────────────────────────────────────────────┐  │
│  │  VYRA CORE LAYER                                                     │  │
│  │  ┌──────────────┐  ┌───────────────┐  ┌──────────────┐               │  │
│  │  │  DataSpace   │  │  Provider     │  │  Interface   │               │  │
│  │  │  Registry    │  │  Registry     │  │  Factory     │               │  │
│  │  └──────┬───────┘  └───────┬───────┘  └──────┬───────┘               │  │
│  │         │                  │                 │                       │  │
│  │         └──────────────────┼─────────────────┘                       │  │
│  └────────────────────────────┼─────────────────────────────────────────┘  │
│                               │                                            │
│  ┌────────────────────────────▼─────────────────────────────────────────┐  │
│  │  PROTOCOL ABSTRACTION (Multi-Provider)                               │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │  CAL: Communication Abstraction Layer                          │  │  │
│  │  │  ┌──────────┐  ┌─────────────────┐  ┌─────────────┐            │  │  │
│  │  │  │   ROS2   │  │ Shared Memory   │  │     UDS     │            │  │  │
│  │  │  │ Provider │  │    Provider     │  │  Provider   │            │  │  │
│  │  │  │(Optional)│  │  (POSIX IPC)    │  │  (Sockets)  │            │  │  │
│  │  │  └────┬─────┘  └────────┬────────┘  └──────┬──────┘            │  │  │
│  │  └───────┼─────────────────┼──────────────────┼───────────────────┘  │  │
│  │          │                 │                  │                      │  │
│  │  ┌───────▼─────────────────▼──────────────────▼───────────────────┐  │  │
│  │  │  EXTERNAL: Network Communication                               │  │  │
│  │  │  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────────┐          │  │  │
│  │  │  │Redis │  │ gRPC │  │ MQTT │  │ REST │  │WebSocket │          │  │  │
│  │  │  │ (TLS)│  │ (UDS)│  │ (QoS)│  │(HTTP)│  │   (WS)   │          │  │  │
│  │  │  └──────┘  └──────┘  └──────┘  └──────┘  └──────────┘          │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                                                                      │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │  INDUSTRIAL: SCADA/MES Layer                                   │  │  │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │  │  │
│  │  │  │ Modbus TCP  │  │   OPC UA    │  │  PROFINET   │             │  │  │
│  │  │  │  Provider   │  │  Provider   │  │  Provider   │             │  │  │
│  │  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │  │  │
│  │  │         │                │                │                    │  │  │
│  │  │  ┌──────▼────────────────▼────────────────▼─────────┐          │  │  │
│  │  │  │  Industrial Bus Base                             │          │  │  │
│  │  │  │  • Object Dictionary (Register Mapping)          │          │  │  │
│  │  │  │  • Cycle Manager (RT Polling)                    │          │  │  │
│  │  │  │  • Diagnostics (Bus Health Monitoring)           │          │  │  │
│  │  │  └──────────────────────────────────────────────────┘          │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                            │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  MONITORING LAYER (Prometheus Metrics)                               │  │
│  │  • vyra_com_calls_total                                              │  │
│  │  • vyra_com_call_duration_seconds                                    │  │
│  │  • vyra_com_active_connections                                       │  │
│  │  • vyra_com_errors_total                                             │  │
│  │  • vyra_com_message_size_bytes                                       │  │
│  │  └────────────────────┬──────────────────────────────────────────────┘  │
│                          │ :8000/metrics                                   │
└──────────────────────────┼─────────────────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────────────────────┐
│                    DOCKER SWARM INFRASTRUCTURE                             │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐  ┌──────────────┐      │
│  │ Prometheus  │◄─┤   Traefik    │  │    Redis    │  │   MQTT       │      │
│  │   Service   │  │    Proxy     │  │   Service   │  │   Broker     │      │
│  └──────┬──────┘  └──────────────┘  └─────────────┘  └──────────────┘      │
│         │                                                                  │
│         ▼                                                                  │
│  ┌─────────────┐                                                           │
│  │   Grafana   │                                                           │
│  │  Dashboard  │                                                           │
│  └─────────────┘                                                           │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Examples

### Example 1: ROS2 Service Call (Full Image)
```
Application (@remote_callable)
    │
    ▼
Core (Factory creates ROS2 Callable)
    │
    ▼
ROS2 Provider
    │
    ▼
ROS2 Transport (Service Server)
    │
    ▼
DDS Network
    │
    ▼
Remote Module (ROS2 Transport)
    │
    ▼
Callback Execution
    │
    ▼
[Prometheus: record latency, count]
```

### Example 2: Shared Memory IPC (Slim Image)
```
Application (@remote_callable)
    │
    ▼
Core (Factory creates SM Callable)
    │
    ▼
SharedMemory Provider
    │
    ▼
POSIX SHM Transport
    │  - Serialize (Binary Protocol)
    │  - Write to /dev/shm/vyra_module_xyz
    │  - Signal via semaphore
    │
    ▼
Filesystem Discovery (/tmp/vyra_sockets/)
    │  - Read PID file
    │  - Verify process alive
    │
    ▼
Remote Module (SM Transport)
    │
    ▼
Callback Execution
    │
    ▼
[Prometheus: record latency, count]
```

### Example 3: Industrial Modbus Read
```
Application (read_holding_register)
    │
    ▼
Industrial Bus Base
    │  - Consult Object Dictionary
    │  - Map "motor_speed" → Register 40001
    │
    ▼
Modbus Provider
    │
    ▼
Cycle Manager (RT Loop)
    │  - Poll every 100ms
    │  - Check bus health
    │
    ▼
Modbus TCP Client (pymodbus)
    │
    ▼
PLC/RTU (Factory Floor)
    │
    ▼
Diagnostics
    │  - Bus timeout? → ErrorFeeder
    │  - CRC error? → ErrorFeeder
    │
    ▼
[Prometheus: record bus latency, errors]
```

### Example 4: Multi-Protocol Broadcast (Feeder)
```
Application (entity.publish_news("System started"))
    │
    ▼
NewsFeeder (protocols=[ROS2, MQTT, Redis])
    │
    ├─► ROS2 Handler → /news topic
    │
    ├─► MQTT Handler → vyra/news/module_name
    │
    └─► Redis Handler → PUBLISH news_channel
    │
    ▼
[Prometheus: 3 publishes recorded]
```

## Protocol Selection Logic

### Automatic Fallback Chain:
```python
def select_protocol(config):
    """
    Auto-select best available protocol.
    """
    if config.get('preferred_protocol'):
        return config['preferred_protocol']
    
    # Fallback chain
    if ROS2_AVAILABLE and config.get('multi_node', True):
        return ProtocolType.ROS2
    
    if SHARED_MEMORY_AVAILABLE and config.get('high_performance', False):
        return ProtocolType.SHARED_MEMORY
    
    if UDS_AVAILABLE:
        return ProtocolType.UDS
    
    raise ProtocolUnavailableError("No transport protocol available")
```

### Configuration Examples:

**Full Image (ROS2 Primary):**
```yaml
# config/communication.yml
communication:
  primary: ros2
  fallback: [shared_memory, uds]
  monitoring:
    enabled: true
    prometheus_port: 8000
  protocols:
    ros2:
      enabled: true
      domain_id: 42
    shared_memory:
      enabled: true
      max_segment_size: 1048576  # 1MB
    redis:
      enabled: true
      host: redis
      tls: true
```

**Slim Image (SharedMemory Primary):**
```yaml
communication:
  primary: shared_memory
  fallback: [uds]
  monitoring:
    enabled: true
    prometheus_port: 8000
  protocols:
    ros2:
      enabled: false  # Not installed
    shared_memory:
      enabled: true
      discovery_path: /tmp/vyra_sockets
```

## Deployment Scenarios

### Scenario 1: Full Stack (Development)
- **Image**: vyra_base:full (2.5 GB)
- **Protocols**: ROS2 + SharedMemory + Redis + MQTT + Modbus + OPC UA
- **Use Case**: Testing all integrations

### Scenario 2: Slim Edge (Production)
- **Image**: vyra_base:slim (800 MB)
- **Protocols**: SharedMemory + gRPC (UDS) + Redis
- **Use Case**: Edge devices, low latency, no ROS2 overhead

### Scenario 3: Industrial Gateway
- **Image**: vyra_base:industrial (1.2 GB)
- **Protocols**: ROS2 + Modbus TCP + OPC UA
- **Use Case**: Factory floor integration, SCADA/MES bridge

### Scenario 4: Cloud Connector
- **Image**: vyra_base:cloud (1.0 GB)
- **Protocols**: MQTT + REST + WebSocket
- **Use Case**: IoT cloud integration, remote monitoring

## Performance Characteristics

| Protocol | Latency | Throughput | CPU Usage | Reliability |
|----------|---------|------------|-----------|-------------|
| **ROS2** | 1-5 ms | High | Medium | Very High (DDS QoS) |
| **Shared Memory** | 50-500 µs | Very High | Low | High (no network) |
| **UDS** | 100-1000 µs | High | Low | Very High (local) |
| **Redis** | 1-10 ms | Medium | Medium | High (persistence) |
| **gRPC** | 500 µs-2 ms | Very High | Medium | Very High (HTTP/2) |
| **MQTT** | 5-50 ms | Medium | Low | Medium (QoS 0-2) |
| **Modbus TCP** | 10-100 ms | Low | Low | High (industrial) |
| **OPC UA** | 5-50 ms | Medium | Medium | Very High (standards) |

## Security Considerations

### Transport Security:
- **ROS2**: SROS2 with DDS Security
- **Shared Memory**: Filesystem permissions + PID validation
- **UDS**: Filesystem permissions
- **Redis**: TLS + Client certificates
- **gRPC**: mTLS over UDS
- **MQTT**: TLS + Username/Password
- **Modbus**: VPN/Firewall (no built-in security)
- **OPC UA**: Certificate-based authentication

### Access Control:
- **Application Layer**: Role-based via AccessLevel enum
- **Transport Layer**: Protocol-specific security
- **Monitoring**: Separate Prometheus scraping credentials

---

**END OF ARCHITECTURE DOCUMENTATION**
