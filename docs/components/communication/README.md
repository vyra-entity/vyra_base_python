# Communication Layer Documentation

This directory contains documentation for VYRA's communication layer.

## Components

### Core (`core_overview.rst`)
Topic builders and interface factories for communication setup.

### Handler (`handler_overview.rst`)  
All communication handlers: ROS2, Zenoh, Redis, UDS, Database, Logging.

### Transport (`transport_overview.rst`)
Transport protocols: Redis, ROS2, UDS, Zenoh.

### External (`external_overview.rst`)
External protocols: gRPC, MQTT, REST, WebSocket, Shared Memory, TCP, UDP.

### Industrial (`industrial_overview.rst`)
Industrial protocols: Modbus (TCP/RTU), OPC-UA.

### Feeder (`feeder_overview.rst`)
Canonical source: `src/vyra_base/com/feeder/README.md`

---

**Note**: This directory consolidates previous scattered German README files
and splits .rst files into unified `*_overview.rst` per component for easier maintenance.
