# VYRA Monitoring Integration

## Overview

VYRA uses a comprehensive monitoring stack for observability across all communication protocols and container resources.

## Components

### 1. Prometheus (Existing Docker Service)
**Location**: Docker Swarm Stack Service  
**Port**: `9090`  
**Purpose**: Time-series metrics collection

**Configuration** (`prometheus/prometheus.yml`):
```yaml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  # VYRA Module metrics
  - job_name: 'vyra_modules'
    static_configs:
      - targets: 
          - 'v2_modulemanager:8000'
          - 'v2_dashboard:8000'
    metrics_path: '/metrics'
    
  # Container metrics via cAdvisor
  - job_name: 'cadvisor'
    static_configs:
      - targets: ['cadvisor:8080']
    
  # Prometheus self-monitoring
  - job_name: 'prometheus'
    static_configs:
      - targets: ['localhost:9090']
```

### 2. cAdvisor (Container Advisor)
**Image**: `gcr.io/cadvisor/cadvisor:latest`  
**Port**: `8080`  
**Purpose**: Container resource metrics

**Docker Compose** (`docker-compose.monitoring.yml`):
```yaml
services:
  cadvisor:
    image: gcr.io/cadvisor/cadvisor:latest
    container_name: vyra_cadvisor
    ports:
      - "8080:8080"
    volumes:
      - /:/rootfs:ro
      - /var/run:/var/run:ro
      - /sys:/sys:ro
      - /var/lib/docker/:/var/lib/docker:ro
      - /dev/disk/:/dev/disk:ro
    privileged: true
    devices:
      - /dev/kmsg
    networks:
      - vyra_network
    deploy:
      mode: global  # Run on all nodes
```

**Metrics Provided**:
- `container_cpu_usage_seconds_total` - CPU usage
- `container_memory_usage_bytes` - Memory usage
- `container_network_receive_bytes_total` - Network RX
- `container_network_transmit_bytes_total` - Network TX
- `container_fs_usage_bytes` - Filesystem usage

### 3. Grafana (Existing Docker Service)
**Port**: `3000`  
**Purpose**: Visualization and dashboards

**Datasources**:
- Prometheus (primary)
- Loki (logs, if configured)

---

## VYRA Communication Metrics

### Standard Metrics (All Protocols)

#### Call Metrics
```promql
# Total calls
vyra_com_calls_total{protocol="ros2", interface_type="callable", interface_name="motor_start", status="success"}

# Call duration histogram
vyra_com_call_duration_seconds_bucket{protocol="shared_memory", interface_type="callable", interface_name="get_position", le="0.005"}

# Call rate (calls per second)
rate(vyra_com_calls_total[1m])

# Error rate
rate(vyra_com_calls_total{status="error"}[5m])
```

#### Connection Metrics
```promql
# Active connections per protocol
vyra_com_active_connections{protocol="ros2"}

# Protocol availability
vyra_com_protocol_info
```

#### Error Metrics
```promql
# Total errors by type
vyra_com_errors_total{protocol="ros2", interface_type="callable", error_type="TimeoutError"}

# Error rate
rate(vyra_com_errors_total[5m])
```

#### Message Size Metrics
```promql
# Message size distribution
vyra_com_message_size_bytes_bucket{protocol="mqtt", interface_type="speaker", direction="send", le="1024"}

# Average message size
rate(vyra_com_message_size_bytes_sum[5m]) / rate(vyra_com_message_size_bytes_count[5m])
```

---

### ROS2/DDS Specific Metrics

#### Topic Throughput
```promql
# Messages per second on topic
rate(vyra_ros2_topic_messages_total{topic_name="/news", direction="send"}[1m])

# Total messages
vyra_ros2_topic_messages_total
```

#### QoS Violations
```promql
# Message drops due to QoS
vyra_ros2_message_drops_total{topic_name="/state", qos_policy="RELIABLE"}

# QoS violations by type
vyra_ros2_qos_violations_total{topic_name="/sensor_data", violation_type="deadline"}

# Violation rate
rate(vyra_ros2_qos_violations_total[5m])
```

#### ROS2 Discovery
```promql
# Active nodes in domain
vyra_ros2_discovered_nodes

# Active topics in domain
vyra_ros2_discovered_topics
```

#### Service Call Latency
```promql
# Service call duration
histogram_quantile(0.95, vyra_ros2_service_call_duration_seconds_bucket{service_name="initialize"})

# Average service latency
rate(vyra_ros2_service_call_duration_seconds_sum[5m]) / rate(vyra_ros2_service_call_duration_seconds_count[5m])
```

#### DDS Quality Metrics
```promql
# Writer liveliness lost
vyra_dds_writer_liveliness_lost{topic_name="/control"}

# Reader samples lost
vyra_dds_reader_samples_lost_total{topic_name="/sensor_data"}

# Deadline missed events
vyra_dds_deadline_missed_total{topic_name="/realtime_data", entity_type="publisher"}
```

---

## Grafana Dashboards

### Dashboard 1: VYRA Communication Overview

**Panels**:

1. **Protocol Health** (Gauge)
```promql
up{job="vyra_modules"}
```

2. **Call Rate by Protocol** (Graph)
```promql
sum(rate(vyra_com_calls_total[5m])) by (protocol)
```

3. **Latency Heatmap** (Heatmap)
```promql
vyra_com_call_duration_seconds_bucket
```

4. **Error Rate** (Graph)
```promql
sum(rate(vyra_com_errors_total[5m])) by (protocol, error_type)
```

5. **Active Connections** (Gauge)
```promql
vyra_com_active_connections
```

6. **Message Size Distribution** (Histogram)
```promql
vyra_com_message_size_bytes_bucket
```

### Dashboard 2: ROS2/DDS Monitoring

**Panels**:

1. **Topic Throughput** (Graph)
```promql
rate(vyra_ros2_topic_messages_total[1m])
```

2. **Message Drops** (Counter)
```promql
vyra_ros2_message_drops_total
```

3. **QoS Violations** (Graph)
```promql
rate(vyra_ros2_qos_violations_total[5m])
```

4. **Discovery Status** (Stat)
```promql
vyra_ros2_discovered_nodes
vyra_ros2_discovered_topics
```

5. **Service Latency P95** (Gauge)
```promql
histogram_quantile(0.95, vyra_ros2_service_call_duration_seconds_bucket)
```

6. **DDS Health** (Alert List)
- Writer liveliness lost
- Reader samples lost
- Deadline missed

### Dashboard 3: Container Resources (cAdvisor)

**Panels**:

1. **CPU Usage** (Graph)
```promql
rate(container_cpu_usage_seconds_total{name=~"vos2_ws_v2_.*"}[5m]) * 100
```

2. **Memory Usage** (Graph)
```promql
container_memory_usage_bytes{name=~"vos2_ws_v2_.*"} / 1024 / 1024
```

3. **Network I/O** (Graph)
```promql
rate(container_network_receive_bytes_total{name=~"vos2_ws_v2_.*"}[5m])
rate(container_network_transmit_bytes_total{name=~"vos2_ws_v2_.*"}[5m])
```

4. **Filesystem Usage** (Gauge)
```promql
container_fs_usage_bytes{name=~"vos2_ws_v2_.*"}
```

5. **Container Restarts** (Counter)
```promql
changes(container_start_time_seconds{name=~"vos2_ws_v2_.*"}[1h])
```

---

## Module Integration

### Python Code (Automatic)

Metrics are automatically collected via decorators:

```python
from vyra_base.com.core import remote_callable
from vyra_base.com.monitoring import monitored_callable

class MyComponent:
    @remote_callable
    @monitored_callable(protocol=ProtocolType.ROS2, track_message_size=True)
    async def my_service(self, request, response):
        # Your code here
        return response  # Metrics collected automatically
```

### Manual Metrics (Advanced)

```python
from vyra_base.com.monitoring import metrics

# Record custom call
metrics.record_call(
    protocol=ProtocolType.SHARED_MEMORY,
    interface_type=InterfaceType.CALLABLE,
    interface_name="custom_function",
    duration=0.005,
    success=True
)

# Record ROS2 topic message
metrics.record_ros2_topic_message(topic_name="/sensor_data", direction="send")

# Record QoS violation
metrics.record_ros2_qos_violation(topic_name="/control", violation_type="deadline")

# Update discovery metrics
metrics.update_ros2_discovery(nodes_count=5, topics_count=12)

# Record DDS event
metrics.record_dds_deadline_missed(topic_name="/realtime", entity_type="publisher")
```

### Expose Metrics Endpoint

Modules automatically expose metrics at `:8000/metrics`:

```python
# In module main.py
from prometheus_client import start_http_server

# Start metrics server
start_http_server(8000)
logger.info("📊 Metrics server started on :8000/metrics")
```

---

## Alerting Rules

Create `prometheus/alerts.yml`:

```yaml
groups:
  - name: vyra_communication
    interval: 30s
    rules:
      - alert: HighErrorRate
        expr: rate(vyra_com_errors_total[5m]) > 10
        for: 2m
        labels:
          severity: warning
        annotations:
          summary: "High error rate on {{ $labels.protocol }}"
          description: "{{ $labels.protocol }} has {{ $value }} errors/sec"
      
      - alert: HighLatency
        expr: histogram_quantile(0.95, vyra_com_call_duration_seconds_bucket) > 1.0
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "High latency on {{ $labels.interface_name }}"
          description: "P95 latency is {{ $value }}s"
      
      - alert: ROS2MessageDrops
        expr: rate(vyra_ros2_message_drops_total[5m]) > 1
        for: 1m
        labels:
          severity: critical
        annotations:
          summary: "ROS2 dropping messages on {{ $labels.topic_name }}"
          description: "{{ $value }} messages/sec dropped"
      
      - alert: DDSQoSViolation
        expr: rate(vyra_ros2_qos_violations_total[5m]) > 0.1
        for: 2m
        labels:
          severity: warning
        annotations:
          summary: "QoS violations on {{ $labels.topic_name }}"
          description: "{{ $labels.violation_type }} violations detected"
      
      - alert: ContainerMemoryHigh
        expr: (container_memory_usage_bytes / container_spec_memory_limit_bytes) > 0.9
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "Container {{ $labels.name }} memory usage high"
          description: "Memory usage at {{ $value | humanizePercentage }}"
```

---

## Performance Benchmarking

### Latency Comparison Query
```promql
histogram_quantile(0.95, 
  sum(vyra_com_call_duration_seconds_bucket) by (protocol, le)
)
```

### Throughput by Protocol
```promql
sum(rate(vyra_com_calls_total{status="success"}[5m])) by (protocol)
```

### Resource Efficiency
```promql
# CPU per successful call
rate(container_cpu_usage_seconds_total[5m]) / 
rate(vyra_com_calls_total{status="success"}[5m])
```

---

## Troubleshooting

### No Metrics Appearing

1. **Check prometheus_client installed:**
```bash
pip list | grep prometheus-client
```

2. **Verify metrics endpoint:**
```bash
curl http://localhost:8000/metrics
```

3. **Check Prometheus targets:**
- Open `http://localhost:9090/targets`
- Verify module scrape jobs are UP

### High Cardinality Warning

If too many unique label combinations:
- Limit `interface_name` to essential services
- Aggregate by `interface_type` instead
- Use recording rules for common queries

### cAdvisor Not Starting

Ensure privileged mode:
```yaml
privileged: true
devices:
  - /dev/kmsg
```

---

## Best Practices

1. **Use Decorators**: Prefer `@monitored_callable` over manual metrics
2. **Limit Labels**: Keep cardinality under 10k combinations
3. **Dashboard Refresh**: Set to 30s-1m for production
4. **Alert Thresholds**: Tune based on actual baseline
5. **Recording Rules**: Pre-compute expensive queries
6. **Retention**: Keep 15d for detailed metrics, 90d for aggregates

---

**Related Documentation**:
- Prometheus: https://prometheus.io/docs/
- cAdvisor: https://github.com/google/cadvisor
- Grafana: https://grafana.com/docs/
