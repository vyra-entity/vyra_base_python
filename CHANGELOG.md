# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]


## [0.1.8+build.35] - 2026-02-06

### Build

Simple rebuild


## [0.1.8+build.34] - 2026-02-06

### Build

Update auto generated .proto file script to add the header and update namings (add VBASE...)


## [0.1.8+build.33] - 2026-02-06

### Build

 - Add zenoh as transport option with protobuf support 
 - Refactor transport module structure to support multiple implementations (e.g., ROS2, Redis, UDS, Zenoh) under a unified interface 
 - Update documentation and examples to reflect new transport options and structure 
 - Add tests for new Zenoh transport implementation and converters - Ensure backward compatibility for existing transports while introducing new ones 
 - Update README files and documentation to guide users on how to use the new transport options and converters


## [0.1.8+build.32] - 2026-02-05

### Build

Updating interfaces to not depend on ros2 but on vyra interfaces. Therefor we can depict not only ros2 as interface but all transport protocols defined in com/transport


## [0.1.8+build.31] - 2026-02-05

### Build

Bugfixing: redis->callable->self.redis_client



## [0.1.0] - 2025-06-11

First draft

<!-- ### Added
Added \_\_init__ to all folders -->

