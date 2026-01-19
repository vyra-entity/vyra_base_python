# SafetyMetadata.msg
# Security metadata for ROS2 communication in VYRA framework
# Contains information about the security level and authentication data

# Security level (1-5):
# 1 = NONE: No security checks
# 2 = BASIC_AUTH: Module ID verification
# 3 = EXTENDED_AUTH: Module ID + password authentication
# 4 = HMAC: HMAC-SHA256 signature validation
# 5 = DIGITAL_SIGNATURE: Certificate-based digital signature
uint8 security_level

# Name of the sending module (e.g., "v2_modulemanager")
string sender_name

# Unique identifier of the sending module
string sender_id

# Timestamp when the message was created
builtin_interfaces/Time timestamp

# Random number used once (nonce) for replay attack prevention
uint64 nonce

# Security payload (e.g., HMAC signature or digital signature)
# Format depends on security_level:
# - Level 1: empty
# - Level 2: empty
# - Level 3: empty
# - Level 4: HMAC-SHA256 signature (hex string)
# - Level 5: Digital signature (base64 encoded)
string security_payload

# Algorithm identifier for the security mechanism used
# Examples: "HMAC-SHA256", "RSA-SHA256", "ECDSA-P256"
string algorithm_id
