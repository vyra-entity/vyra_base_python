"""
Example: Basic Service with New Decorator System

Demonstrates the two-phase initialization pattern:
1. Decorators create blueprints during class definition
2. Callbacks bind during component initialization

Run this example:
    python -m vyra_base.com.core.examples.example_basic_service
"""

import asyncio
import logging

from vyra_base.com.core.decorators import remote_service, bind_decorated_callbacks
from vyra_base.com.core.blueprints import ServiceBlueprint
from vyra_base.com.core.callback_registry import CallbackRegistry
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.core.types import ProtocolType

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class CalculatorComponent:
    """Example component with service interfaces using new decorators."""
    
    def __init__(self):
        self.calculation_count = 0
    
    @remote_service(
        name="add",
        protocols=[ProtocolType.REDIS],  # Prefer Redis for simplicity
        namespace="calculator"
    )
    async def add_numbers(self, request, response=None):
        """Add two numbers - service callback."""
        self.calculation_count += 1
        result = request.get("x", 0) + request.get("y", 0)
        logger.info(f"🧮 Adding {request.get('x')} + {request.get('y')} = {result}")
        return {"result": result, "count": self.calculation_count}
    
    @remote_service(
        name="multiply",
        protocols=[ProtocolType.REDIS],
        namespace="calculator"
    )
    async def multiply_numbers(self, request, response=None):
        """Multiply two numbers - service callback."""
        self.calculation_count += 1
        result = request.get("x", 1) * request.get("y", 1)
        logger.info(f"🧮 Multiplying {request.get('x')} × {request.get('y')} = {result}")
        return {"result": result, "count": self.calculation_count}
    
    @remote_service(
        name="divide",
        protocols=[ProtocolType.REDIS],
        namespace="calculator"
    )
    async def divide_numbers(self, request, response=None):
        """Divide two numbers - service callback."""
        self.calculation_count += 1
        x = request.get("x", 0)
        y = request.get("y", 1)
        
        if y == 0:
            logger.error("❌ Division by zero!")
            return {"error": "Division by zero", "count": self.calculation_count}
        
        result = x / y
        logger.info(f"🧮 Dividing {x} ÷ {y} = {result}")
        return {"result": result, "count": self.calculation_count}


async def demonstrate_old_way():
    """Show how it used to work (BEFORE refactoring)."""
    logger.info("\n" + "="*60)
    logger.info("⏪ OLD WAY (Before Refactoring)")
    logger.info("="*60)
    logger.info("""
    Old approach:
    1. Create service directly with callback
    2. No separation between definition and implementation
    3. Tight coupling between interface and callback
    
    Code (OLD):
        component = CalculatorComponent()
        service = await InterfaceFactory.create_server(
            name="add",
            response_callback=component.add_numbers,  # Must exist NOW
            protocols=[ProtocolType.REDIS]
        )
    
    Problems:
    - Cannot define interface before component exists
    - Hard to test without full component
    - Metadata scattered across codebase
    """)


async def demonstrate_new_way():
    """Show the new two-phase initialization."""
    logger.info("\n" + "="*60)
    logger.info("✨ NEW WAY (After Refactoring)")
    logger.info("="*60)
    
    # PHASE 1: Blueprint Creation (happens during decoration)
    logger.info("\n📋 PHASE 1: Blueprint Registration")
    logger.info("-" * 60)
    logger.info("Blueprints created automatically by decorators during class definition.")
    
    # Check what's registered
    CallbackRegistry.debug_print(namespace="calculator")
    
    # PHASE 2: Component Creation & Callback Binding
    logger.info("\n🔗 PHASE 2: Create Component & Bind Callbacks")
    logger.info("-" * 60)
    
    component = CalculatorComponent()
    logger.info("Created component instance")
    
    # Bind decorated methods to their blueprints
    binding_results = bind_decorated_callbacks(component, namespace="calculator")
    logger.info(f"Binding results: {binding_results}")
    
    # Verify bindings
    logger.info("\n✅ After Binding:")
    CallbackRegistry.debug_print(namespace="calculator")
    
    # PHASE 3: Create Interfaces from Bound Blueprints
    logger.info("\n🏗️  PHASE 3: Create Interfaces from Blueprints")
    logger.info("-" * 60)
    
    # Get bound blueprints and create interfaces
    add_bp = CallbackRegistry.get_blueprint("add", namespace="calculator")
    if add_bp and add_bp.is_bound():
        logger.info(f"Creating service from blueprint: {add_bp.name}")
        try:
            service = await InterfaceFactory.create_from_blueprint(add_bp)
            logger.info(f"✅ Service created: {service}")
        except Exception as e:
            logger.error(f"❌ Failed to create service: {e}")
            logger.info("Note: This example requires Redis to be running")
    
    logger.info("\n🎯 Benefits of New Approach:")
    logger.info("  ✓ Blueprints can be registered before component exists")
    logger.info("  ✓ Clear separation: Configuration (blueprint) vs Implementation (callback)")
    logger.info("  ✓ Late binding enables dynamic interface registration")
    logger.info("  ✓ Easy testing - mock callbacks without full component")
    logger.info("  ✓ Metadata centralized in blueprints")


async def demonstrate_late_binding():
    """Show late binding capability."""
    logger.info("\n" + "="*60)
    logger.info("🕒 LATE BINDING DEMONSTRATION")
    logger.info("="*60)
    
    # Create blueprint manually (simulating JSON metadata loading)
    logger.info("\n1️⃣  Create blueprint from metadata (no callback yet)")
    blueprint = ServiceBlueprint(
        name="square",
        protocols=[ProtocolType.REDIS],
        metadata={"description": "Square a number"}
    )
    CallbackRegistry.register_blueprint(blueprint, namespace="calculator")
    logger.info(f"✅ Registered blueprint: {blueprint.name}, bound={blueprint.is_bound()}")
    
    # Try to create interface (will be pending)
    logger.info("\n2️⃣  Try to create interface without callback")
    try:
        service = await InterfaceFactory.create_from_blueprint(blueprint)
        if service is None:
            logger.info("✅ Interface registered as pending (no callback yet)")
    except Exception as e:
        logger.info(f"Interface creation pending: {e}")
    
    # Later: bind callback
    logger.info("\n3️⃣  Later: Component loads and callback binds")
    
    async def square_callback(request, response=None):
        """Square callback added later."""
        x = request.get("x", 0)
        result = x * x
        logger.info(f"🧮 Squaring {x}² = {result}")
        return {"result": result}
    
    # Bind callback to existing blueprint
    success = CallbackRegistry.bind_callback("square", square_callback, namespace="calculator")
    logger.info(f"✅ Callback bound: {success}")
    
    # Now create interface
    logger.info("\n4️⃣  Create interface with bound callback")
    blueprint = CallbackRegistry.get_blueprint("square", namespace="calculator")
    if blueprint and blueprint.is_bound():
        try:
            service = await InterfaceFactory.create_from_blueprint(blueprint)
            logger.info(f"✅ Service created successfully: {service}")
        except Exception as e:
            logger.error(f"❌ Failed: {e}")
            logger.info("Note: This example requires Redis to be running")


async def main():
    """Run all demonstrations."""
    logger.info("╔" + "="*58 + "╗")
    logger.info("║  VYRA DECORATOR SYSTEM - TWO-PHASE INITIALIZATION DEMO  ║")
    logger.info("╚" + "="*58 + "╝")
    
    # Clear registry for clean demo
    CallbackRegistry.clear()
    
    await demonstrate_old_way()
    await asyncio.sleep(0.5)
    
    await demonstrate_new_way()
    await asyncio.sleep(0.5)
    
    await demonstrate_late_binding()
    
    logger.info("\n" + "="*60)
    logger.info("📚 Summary")
    logger.info("="*60)
    logger.info("""
    The new two-phase decorator system enables:
    
    Phase 1 (Decoration):   Blueprints created → Registered in CallbackRegistry
    Phase 2 (Initialization): Callbacks bound → Interfaces created
    
    This decoupling allows:
    - Metadata-driven interface definitions (from JSON)
    - Late binding of implementations
    - Better testability
    - Dynamic reconfiguration
    
    See also:
    - example_publisher.py - Publisher with property setter pattern
    - example_action.py - Long-running actions
    - example_late_binding.py - Advanced late binding scenarios
    """)


if __name__ == "__main__":
    asyncio.run(main())
