# VYRA Security Framework Examples

This directory contains example implementations of the VYRA security framework.

## Examples

### 1. Secure Server Node (`secure_server_node.py`)

Demonstrates how to create a module that provides security services:
- Implements `RequestAccess` service for authentication
- Manages security sessions
- Validates incoming messages with SafetyMetadata
- Supports Security Levels 1-4 (HMAC)

**Run:**
```bash
python3 secure_server_node.py
```

### 2. Secure Client Node (`secure_client_node.py`)

Demonstrates how to create a client that authenticates and communicates securely:
- Requests access with `RequestAccess` service
- Creates security context from response
- Uses `SecurePublisher` for secure message publishing
- Automatic SafetyMetadata generation

**Run:**
```bash
python3 secure_client_node.py
```

## Running the Examples

1. Start the secure server in one terminal:
   ```bash
   cd /home/holgder/VYRA/vyra_base_python
   source /opt/ros/jazzy/setup.bash  # or your ROS2 distribution
   python3 examples/security/secure_server_node.py
   ```

2. Start the secure client in another terminal:
   ```bash
   cd /home/holgder/VYRA/vyra_base_python
   source /opt/ros/jazzy/setup.bash
   python3 examples/security/secure_client_node.py
   ```

3. Observe the authentication process and secure communication in the logs.

## Security Levels Demonstrated

- **Level 4 (HMAC)**: Both examples use HMAC-SHA256 for message authentication
- The server validates incoming messages with HMAC signatures
- The client automatically signs messages with the provided HMAC key

## Prerequisites

- ROS2 installed and sourced
- vyra_base_python interfaces built
- Python dependencies: cryptography, rclpy

## Notes

- The examples use simplified message types for demonstration
- In production, use proper message definitions with `SafetyMetadata` fields
- Certificate-based security (Level 5) requires additional CA setup
