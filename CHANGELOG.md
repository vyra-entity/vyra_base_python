# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]


## [0.1.8+build.62] - 2026-02-18

### Build

update generate_interfaces


## [0.1.8+build.61] - 2026-02-18

### Build

bugfix generate_interfaces.py


## [0.1.8+build.60] - 2026-02-18

### Build

Bugfix extract_interfaces


## [0.1.8+build.59] - 2026-02-18

### Build

update .proto interfaces available in msg srv and action folder


## [0.1.8+build.58] - 2026-02-18

### Build

massive refactoring, cleaning, test and docs update


## [0.1.8+build.54] - 2026-02-18

### Build

Adding blueprint and update decorators
Change interface options from speaker, callable, job to publisher, server and actionServer


## [0.1.8+build.51] - 2026-02-16

### Build

Update logger ,docs, and transport structure to use publisher, service and action in the future instead ob speaker, callable and job


## [0.1.8+build.50] - 2026-02-16

### Build

Remove static dead Logger and adding python base logger


## [0.1.8+build.48] - 2026-02-12

### Build

Update vyra transport layer to work with dynamic loading interface types


## [0.1.8+build.45] - 2026-02-11

### Build

make logger autark and update testings


## [0.1.8+build.42] - 2026-02-09

### Build

Update FunctionConfigEntry ros2type->interfacetype to be not ros2 dependent but type agnostic


## [0.1.8+build.41] - 2026-02-09

### Build

Bugfixing: Wrong types in feeder.py


## [0.1.8+build.40] - 2026-02-09

### Build

Update types metadata structure:[bugfix]


## [0.1.8+build.39] - 2026-02-09

### Build

Update transport structure in com->transport and bugfixing in development process


## [0.1.8+build.38] - 2026-02-09

### Build

update speaker, callable, job structure to all transport provider


## [0.1.8+build.37] - 2026-02-09

### Build

Bugfixing topic_builder


## [0.1.8+build.36] - 2026-02-09

### Build

adding topic_builder to generate a generic naming convention for all transport protocols


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

