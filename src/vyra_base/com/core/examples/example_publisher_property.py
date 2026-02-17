"""
Example: Publisher with Property Setter Pattern

Demonstrates using @remote_publisher decorator to create auto-publishing properties,
similar to the pattern shown in the infrastructure sketch (Untitled-2).

This pattern allows you to write:
    component.status = {"state": "running"}
    
Instead of:
    await component.publish_status({"state": "running"})

Run this example:
    python -m vyra_base.com.core.examples.example_publisher_property
"""

import asyncio
import logging
from typing import Any, Optional

from vyra_base.com.core.decorators import remote_publisher, bind_decorated_callbacks
from vyra_base.com.core.blueprints import PublisherBlueprint
from vyra_base.com.core.callback_registry import CallbackRegistry
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.core.types import ProtocolType

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class RobotController:
    """Example robot controller with publisher interfaces."""
    
    def __init__(self):
        self._status = None
        self._position = None
        self._temperature = None
        
        # Publishers will be created during interface initialization
        self._publisher_status = None
        self._publisher_position = None
        self._publisher_temperature = None
    
    # ========================================================================
    # METHOD 1: Standard Decorator Approach (Explicit Publishing)
    # ========================================================================
    
    @remote_publisher(
        name="robot/status",
        protocols=[ProtocolType.REDIS],
        namespace="robot"
    )
    async def publish_status(self, message: dict):
        """Publish status updates."""
        pass  # Implementation replaced by decorator
    
    @remote_publisher(
        name="robot/position",
        protocols=[ProtocolType.REDIS],
        namespace="robot"
    )
    async def publish_position(self, message: dict):
        """Publish position updates."""
        pass  # Implementation replaced by decorator
    
    # ========================================================================
    # METHOD 2: Property Setter Pattern (Auto-Publishing)
    # ========================================================================
    
    @property
    def status(self) -> Optional[dict]:
        """Get current status."""
        return self._status
    
    @status.setter
    def status(self, value: dict):
        """
        Set status and automatically publish.
        
        This provides a cleaner API:
            robot.status = {"state": "running", "battery": 85}
        
        Instead of:
            await robot.publish_status({"state": "running", "battery": 85})
        """
        self._status = value
        logger.info(f"📤 Status changed, publishing: {value}")
        
        # Trigger async publish in background
        if self._publisher_status:
            asyncio.create_task(self._publisher_status.shout(value))
        else:
            logger.warning("⚠️  Status publisher not initialized, cannot publish")
    
    @property
    def position(self) -> Optional[dict]:
        """Get current position."""
        return self._position
    
    @position.setter
    def position(self, value: dict):
        """Set position and automatically publish."""
        self._position = value
        logger.info(f"📤 Position changed, publishing: {value}")
        
        if self._publisher_position:
            asyncio.create_task(self._publisher_position.shout(value))
        else:
            logger.warning("⚠️  Position publisher not initialized")
    
    @property
    def temperature(self) -> Optional[float]:
        """Get current temperature."""
        return self._temperature
    
    @temperature.setter
    def temperature(self, value: float):
        """Set temperature and automatically publish."""
        self._temperature = value
        logger.info(f"📤 Temperature changed, publishing: {value}°C")
        
        # For temperature, we might want to publish via the method-based publisher
        if hasattr(self, 'publish_temperature'):
            asyncio.create_task(self.publish_temperature({"value": value, "unit": "celsius"}))
    
    # ========================================================================
    # Helper: Initialize Publishers
    # ========================================================================
    
    async def initialize_publishers(self):
        """
        Create publisher interfaces from blueprints.
        
        In a real VYRA module, this would be called by entity.set_interfaces()
        """
        logger.info("🏗️  Initializing publishers...")
        
        # Get blueprints and create publishers
        status_bp = CallbackRegistry.get_blueprint("robot/status", namespace="robot")
        position_bp = CallbackRegistry.get_blueprint("robot/position", namespace="robot")
        
        if status_bp:
            try:
                self._publisher_status = await InterfaceFactory.create_from_blueprint(status_bp)
                logger.info(f"✅ Status publisher created")
            except Exception as e:
                logger.error(f"❌ Failed to create status publisher: {e}")
        
        if position_bp:
            try:
                self._publisher_position = await InterfaceFactory.create_from_blueprint(position_bp)
                logger.info(f"✅ Position publisher created")
            except Exception as e:
                logger.error(f"❌ Failed to create position publisher: {e}")
        
        logger.info("✅ Publisher initialization complete")
    
    # ========================================================================
    # Example Methods Using Publishers
    # ========================================================================
    
    async def start_mission(self):
        """Example: Start a mission and publish updates."""
        logger.info("🚀 Starting mission...")
        
        # Method 1: Explicit publishing
        await self.publish_status({"state": "starting", "mission_id": "M001"})
        await asyncio.sleep(0.5)
        
        # Method 2: Property setter (auto-publishing)
        self.status = {"state": "running", "mission_id": "M001", "progress": 0}
        await asyncio.sleep(0.5)
        
        # Simulate mission progress
        for progress in [25, 50, 75, 100]:
            self.status = {
                "state": "running",
                "mission_id": "M001",
                "progress": progress
            }
            self.position = {"x": progress, "y": progress / 2, "z": 10}
            await asyncio.sleep(0.5)
        
        self.status = {"state": "completed", "mission_id": "M001", "progress": 100}
        logger.info("✅ Mission complete!")


async def demonstrate_property_pattern():
    """Demonstrate the property setter publishing pattern."""
    logger.info("\n" + "="*60)
    logger.info("🏡 PROPERTY SETTER PATTERN")
    logger.info("="*60)
    
    # Create controller
    robot = RobotController()
    logger.info("✅ Robot controller created")
    
    # Bind callbacks (blueprints already registered by decorators)
    logger.info("\n📋 Binding callbacks...")
    bind_decorated_callbacks(robot, namespace="robot")
    
    # Initialize publishers
    logger.info("\n🔌 Initializing publishers...")
    try:
        await robot.initialize_publishers()
    except Exception as e:
        logger.warning(f"⚠️  Publisher initialization failed (Redis not running?): {e}")
        logger.info("Continuing with demo (publishing will be logged but not sent)")
    
    # Run mission with auto-publishing properties
    logger.info("\n🎬 Running mission with auto-publishing properties...")
    await robot.start_mission()
    
    logger.info("\n✨ Benefits of Property Setter Pattern:")
    logger.info("  ✓ Cleaner API: robot.status = {...} instead of await publish_status(...)")
    logger.info("  ✓ Automatic publishing on state changes")
    logger.info("  ✓ No need to remember to call publish")
    logger.info("  ✓ Still works with async event loop (asyncio.create_task)")
    
    logger.info("\n⚠️  Considerations:")
    logger.info("  - Property setters are sync, but use background tasks for async publish")
    logger.info("  - Error handling needs care (publishing happens in background)")
    logger.info("  - Race conditions possible if setting properties rapidly")


async def demonstrate_comparison():
    """Compare different publishing approaches."""
    logger.info("\n" + "="*60)
    logger.info("⚖️  COMPARISON: Publishing Approaches")
    logger.info("="*60)
    
    robot = RobotController()
    
    await robot.initialize_publishers()
    
    logger.info("\n1️⃣  Explicit Method Call:")
    logger.info("   await robot.publish_status({'state': 'idle'})")
    try:
        await robot.publish_status({"state": "idle"})
    except Exception as e:
        logger.info(f"   (Would publish if Redis running: {e})")
    
    logger.info("\n2️⃣  Property Setter:")
    logger.info("   robot.status = {'state': 'active'}  # Auto-publishes!")
    robot.status = {"state": "active"}
    
    logger.info("\n3️⃣  Direct Publisher Access:")
    logger.info("   await robot._publisher_status.shout({'state': 'busy'})")
    if robot._publisher_status:
        try:
            await robot._publisher_status.shout({"state": "busy"})
        except Exception as e:
            logger.info(f"   (Would publish if Redis running: {e})")
    
    logger.info("\n📊 Choose based on your use case:")
    logger.info("  - Property setter: Best UX for frequently changing state")
    logger.info("  - Explicit method: Clear intent, easier error handling")
    logger.info("  - Direct access: Low-level control, rare usage")


async def main():
    """Run all demonstrations."""
    logger.info("╔" + "="*58 + "╗")
    logger.info("║     PUBLISHER WITH PROPERTY SETTER PATTERN DEMO         ║")
    logger.info("╚" + "="*58 + "╝")
    
    # Clear registry
    CallbackRegistry.clear()
    
    await demonstrate_property_pattern()
    await asyncio.sleep(0.5)
    
    await demonstrate_comparison()
    
    logger.info("\n" + "="*60)
    logger.info("📚 Summary")
    logger.info("="*60)
    logger.info("""
    Property Setter Pattern for Publishers:
    
    class Component:
        @property
        def value(self):
            return self._value
        
        @value.setter
        def value(self, new_value):
            self._value = new_value
            asyncio.create_task(self._publisher.shout(new_value))
    
    Usage:  component.value = 42  # Automatically publishes!
    
    This pattern is particularly useful for:
    - State machines (status updates)
    - Real-time sensor data
    - Configuration changes
    - Any frequently changing values
    """)


if __name__ == "__main__":
    asyncio.run(main())
