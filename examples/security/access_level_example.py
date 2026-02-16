"""
Complete Access Level Example
==============================

This example demonstrates how to use access levels with @security_required
decorator in a VYRA module.

Directory Structure:
    module/
    ├── src/
    │   ├── module_name/
    │   │   ├── _base_.py
    │   │   └── application/
    │   │       └── application.py
    │   └── module_name_interfaces/
    │       └── config/
    │           └── module_meta.json
"""

# =============================================================================
# 1. Interface Configuration (src/module_name_interfaces/config/module_meta.json)
# =============================================================================

INTERFACE_CONFIG = """
[
    {
        "tags": ["ros2_srv"],
        "type": "callable",
        "functionname": "get_status",
        "displayname": "Get Status",
        "description": "Public service - anyone can call",
        "filetype": "GetStatus.srv",
        "access_level": 1,
        "params": [],
        "returns": [
            {
                "name": "status",
                "datatype": "string"
            }
        ]
    },
    {
        "tags": ["ros2_srv"],
        "type": "callable",
        "functionname": "set_config",
        "displayname": "Set Configuration",
        "description": "Configure system - requires password",
        "filetype": "SetConfig.srv",
        "access_level": 3,
        "params": [
            {
                "name": "key",
                "datatype": "string"
            },
            {
                "name": "value",
                "datatype": "string"
            }
        ],
        "returns": [
            {
                "name": "success",
                "datatype": "bool"
            }
        ]
    },
    {
        "tags": ["ros2_srv"],
        "type": "callable",
        "functionname": "start_operation",
        "displayname": "Start Operation",
        "description": "Start critical operation - requires HMAC",
        "filetype": "StartOperation.srv",
        "access_level": 4,
        "params": [
            {
                "name": "operation_id",
                "datatype": "string"
            }
        ],
        "returns": [
            {
                "name": "success",
                "datatype": "bool"
            }
        ]
    }
]
"""

# =============================================================================
# 2. Module Base Configuration (src/module_name/_base_.py)
# =============================================================================
import logging

from vyra_base.core.entity import VyraEntity
from vyra_base.security import SecurityLevel
from vyra_base.defaults.entries import (
    StateEntry, NewsEntry, ErrorEntry, ModuleEntry
)

async def build_base():
    """Build module base with security enabled."""
    
    # Configure security
    module_config = {
        'security': {
            'enabled': True,
            'max_level': SecurityLevel.HMAC,  # Support up to Level 4
            'session_duration': 3600,
            'module_passwords': {
                # Module IDs that can authenticate with password
                'dashboard-module-uuid': 'dashboard_password',
                'operator-module-uuid': 'operator_password',
                'admin-module-uuid': 'admin_password'
            }
        },
        # Other module config...
    }
    
    # Create entries
    state_entry = StateEntry(...)
    news_entry = NewsEntry(...)
    error_entry = ErrorEntry(...)
    module_entry = ModuleEntry(name="example_module", ...)
    
    # Create entity with security enabled
    entity = VyraEntity(
        state_entry=state_entry,
        news_entry=news_entry,
        error_entry=error_entry,
        module_entry=module_entry,
        module_config=module_config
    )
    
    # Security manager is now available
    if entity.security_manager:
        print(f"✅ Security enabled: Max Level {entity.security_manager.max_security_level}")
    
    return entity


# =============================================================================
# 3. Application with Access Control (src/module_name/application/application.py)
# =============================================================================

from vyra_base.state import OperationalStateMachine
from vyra_base.com import remote_service
from vyra_base.security import security_required, SecurityLevel

logger = logging.getLogger(__name__)

class Application(OperationalStateMachine):
    """
    Application with different access levels for services.
    """
    
    def __init__(self, entity: VyraEntity, *args, **kwargs):
        super().__init__(entity.state_machine.fsm)
        self.entity = entity
        
        # Log security status
        if entity.security_manager:
            logger.info(f"Security enabled: Max level {entity.security_manager.max_security_level}")
        else:
            logger.warning("Security disabled - all services are public")
    
    # -------------------------------------------------------------------------
    # Level 1: Public Service (No authentication required)
    # -------------------------------------------------------------------------
    
    @remote_service()
    @security_required(security_level=SecurityLevel.NONE)
    def get_status(self, request=None, response=None):
        """
        Public service - anyone can call without authentication.
        Matches access_level: 1 in module_meta.json
        """
        logger.info("get_status called")
        
        if response is None:
            return response

        response.status = "running"
        response.success = True
        return response
    
    # -------------------------------------------------------------------------
    # Level 3: Password Required
    # -------------------------------------------------------------------------
    
    @remote_service()
    @security_required(security_level=SecurityLevel.EXTENDED_AUTH)
    def set_config(self, request=None, response=None):
        """
        Configuration service - requires Level 3 (password) or higher.
        Matches access_level: 3 in module_meta.json
        
        Decorator validates:
        - SafetyMetadata exists in request
        - Session token is valid
        - Module granted level >= 3
        
        If validation fails, response.success = False automatically.
        """
        logger.info(f"set_config called: {request.key} = {request.value}")
        
        # If we get here, access is granted
        # Store configuration
        self.entity.config[request.key] = request.value
        
        response.success = True
        response.message = f"Configuration {request.key} updated"
        return response
    
    # -------------------------------------------------------------------------
    # Level 4: HMAC Signature Required
    # -------------------------------------------------------------------------
    
    @remote_service()
    @security_required(security_level=SecurityLevel.HMAC)
    def start_operation(self, request=None, response=None):
        """
        Critical operation - requires Level 4 (HMAC) or higher.
        Matches access_level: 4 in module_meta.json
        
        This ensures:
        - Message integrity via HMAC-SHA256
        - Valid session with Level 4 access
        - Calling module has been granted HMAC level
        """
        logger.info(f"start_operation called: {request.operation_id}")
        
        # If we get here, HMAC validation passed
        # Execute operation
        result = self.entity.start_operation(request.operation_id)
        
        response.success = result
        response.message = "Operation started" if result else "Operation failed"
        return response


# =============================================================================
# 4. Client Example - Requesting Access and Calling Services
# =============================================================================

import asyncio
import uuid
from rclpy.node import Node
from vyra_base.security import (
    create_security_context,
    SecureServiceClient,
    SecurityLevel
)
from vyra_base_interfaces.srv import VBASERequestAccess

class ClientModule(Node):
    """Example client module requesting different access levels."""
    
    def __init__(self):
        super().__init__('client_module')
        self.module_id = str(uuid.uuid4())
        self.security_context = None
    
    async def authenticate(self, requested_level: int, password: str = ""):
        """
        Authenticate with server module.
        
        :param requested_level: Security level to request (1-5)
        :param password: Password (required for Level 3+)
        """
        # Create RequestAccess client
        access_client = self.create_client(
            VBASERequestAccess,
            '/example_module/request_access'
        )
        
        while not access_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for RequestAccess service...')
        
        # Build request
        request = VBASERequestAccess.Request()
        request.module_name = self.get_name()
        request.module_id = self.module_id
        request.requested_role = "client"
        request.requested_sl = requested_level
        request.password = password
        
        # Call service
        future = access_client.call_async(request)
        response = await future
        
        if response is None:
            self.get_logger().error("❌ No response from RequestAccess service")
            return False
        
        if response.success:
            self.get_logger().info(
                f"✅ Access granted: Level {response.granted_sl}"
            )
            
            # Create security context
            self.security_context = create_security_context(
                module_name=request.module_name,
                module_id=request.module_id,
                security_level=response.granted_sl,
                session_token=response.session_token,
                hmac_key=response.hmac_key
            )
            
            return True
        else:
            self.get_logger().error(
                f"❌ Access denied: {response.message}"
            )
            return False
    
    async def call_public_service(self):
        """Call Level 1 service (no auth required)."""
        # For Level 1, no authentication needed
        # But we still use SecureServiceClient for consistency
        
        client = SecureServiceClient(
            self,
            GetStatus,
            '/example_module/get_status',
            self.security_context  # Can be None for Level 1
        )
        
        request = GetStatus.Request()
        response = await client.call_async(request)
        
        print(f"Status: {response.status}")
    
    async def call_config_service(self):
        """Call Level 3 service (password required)."""
        if not self.security_context:
            print("❌ Not authenticated")
            return
        
        if self.security_context.security_level < SecurityLevel.EXTENDED_AUTH:
            print("❌ Insufficient access level for set_config")
            return
        
        client = SecureServiceClient(
            self,
            SetConfig,
            '/example_module/set_config',
            self.security_context
        )
        
        request = SetConfig.Request()
        request.key = "max_speed"
        request.value = "100"
        
        response = await client.call_async(request)
        
        if response.success:
            print(f"✅ Config updated: {response.message}")
        else:
            print(f"❌ Failed: {response.message}")
    
    async def call_operation_service(self):
        """Call Level 4 service (HMAC required)."""
        if not self.security_context:
            print("❌ Not authenticated")
            return
        
        if self.security_context.security_level < SecurityLevel.HMAC:
            print("❌ Insufficient access level for start_operation")
            return
        
        client = SecureServiceClient(
            self,
            StartOperation,
            '/example_module/start_operation',
            self.security_context
        )
        
        request = StartOperation.Request()
        request.operation_id = "op_12345"
        
        response = await client.call_async(request)
        
        if response.success:
            print(f"✅ Operation started: {response.message}")
        else:
            print(f"❌ Failed: {response.message}")


# =============================================================================
# 5. Usage Example
# =============================================================================

async def main():
    """Complete usage example."""
    import rclpy
    rclpy.init()
    
    client = ClientModule()
    
    # Scenario 1: Call public service (no auth)
    print("\n=== Scenario 1: Public Service (Level 1) ===")
    await client.call_public_service()
    
    # Scenario 2: Authenticate with Level 3, call config service
    print("\n=== Scenario 2: Config Service (Level 3) ===")
    success = await client.authenticate(
        requested_level=3,
        password="operator_password"
    )
    if success:
        await client.call_config_service()
    
    # Scenario 3: Try to call Level 4 service with Level 3 access (DENIED)
    print("\n=== Scenario 3: Try Level 4 with Level 3 access ===")
    await client.call_operation_service()  # Will fail
    
    # Scenario 4: Re-authenticate with Level 4, call operation service
    print("\n=== Scenario 4: Operation Service (Level 4) ===")
    success = await client.authenticate(
        requested_level=4,
        password="admin_password"
    )
    if success:
        await client.call_operation_service()
    
    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())


# =============================================================================
# Expected Output
# =============================================================================

"""
=== Scenario 1: Public Service (Level 1) ===
Status: running

=== Scenario 2: Config Service (Level 3) ===
✅ Access granted: Level 3
✅ Config updated: Configuration max_speed updated

=== Scenario 3: Try Level 4 with Level 3 access ===
❌ Insufficient access level for start_operation

=== Scenario 4: Operation Service (Level 4) ===
✅ Access granted: Level 4
✅ Operation started: Operation started
"""
