"""
Callback Registry - Late Binding Management

Manages the lifecycle of handler blueprints and their callback bindings.
Provides thread-safe global registry with per-module namespacing.

Two-Phase Lifecycle:
1. Phase 1 (Entity Init): Register blueprints from JSON metadata
2. Phase 2 (Component Init): Bind callbacks from decorated methods

Example:
    >>> # Phase 1: During entity creation
    >>> blueprint = ServiceBlueprint(name="calculate", protocols=[ProtocolType.ROS2])
    >>> CallbackRegistry.register_blueprint(blueprint)
    
    >>> # Phase 2: During component initialization
    >>> def my_calculate(request, response=None):
    ...     return {"result": request["x"] + request["y"]}
    >>> CallbackRegistry.bind_callback("calculate", my_calculate)
    
    >>> # Later: Retrieve bound blueprint
    >>> blueprint = CallbackRegistry.get_blueprint("calculate")
    >>> blueprint.is_bound()
    True
"""

from __future__ import annotations

import logging
import threading
from typing import Dict, List, Optional, Set, Callable

from vyra_base.com.core.blueprints import (
    AnyBlueprint,
    HandlerBlueprint,
    InterfaceType
)

logger = logging.getLogger(__name__)


class CallbackRegistry:
    """
    Thread-safe global registry for handler blueprints.
    
    Features:
    - Per-module namespacing via path prefixes
    - Late binding support (register blueprint, bind callback later)
    - Thread-safe operations
    - Debugging tools (list unbound, statistics)
    
    The registry uses a singleton pattern via class methods.
    """
    
    _lock = threading.RLock()
    _blueprints: Dict[str, AnyBlueprint] = {}
    _initialized = False
    
    @classmethod
    def initialize(cls) -> None:
        """Initialize the registry (idempotent)"""
        with cls._lock:
            if not cls._initialized:
                cls._blueprints = {}
                cls._initialized = True
                logger.debug("📋 CallbackRegistry initialized")
    
    @classmethod
    def clear(cls) -> None:
        """Clear all blueprints (primarily for testing)"""
        with cls._lock:
            count = len(cls._blueprints)
            cls._blueprints.clear()
            logger.debug(f"🗑️  CallbackRegistry cleared ({count} blueprints removed)")
    
    @classmethod
    def register_blueprint(
        cls,
        blueprint: AnyBlueprint,
        namespace: Optional[str] = None,
        override: bool = False
    ) -> str:
        """
        Register a blueprint in the registry.
        
        Args:
            blueprint: Blueprint to register
            namespace: Optional module namespace (e.g., "v2_modulemanager")
            override: If True, replace existing blueprint with same name
            
        Returns:
            Fully qualified name (with namespace if provided)
            
        Raises:
            ValueError: If blueprint with same name already exists and override=False
            
        Example:
            >>> blueprint = ServiceBlueprint(name="initialize", ...)
            >>> full_name = CallbackRegistry.register_blueprint(
            ...     blueprint, namespace="v2_modulemanager"
            ... )
            >>> print(full_name)
            'v2_modulemanager/initialize'
        """
        with cls._lock:
            cls.initialize()
            
            # Build fully qualified name
            if namespace:
                full_name = f"{namespace.strip('/')}/{blueprint.name}"
            else:
                full_name = blueprint.name
            
            # Check for duplicates
            if full_name in cls._blueprints and not override:
                raise ValueError(
                    f"Blueprint '{full_name}' already registered. "
                    f"Use override=True to replace."
                )
            
            # Register
            cls._blueprints[full_name] = blueprint
            
            if override and full_name in cls._blueprints:
                logger.warning(f"⚠️  Overriding blueprint '{full_name}'")
            else:
                logger.debug(f"📝 Registered blueprint '{full_name}' (type: {blueprint.interface_type.value})")
            
            return full_name
    
    @classmethod
    def get_blueprint(
        cls,
        name: str,
        namespace: Optional[str] = None
    ) -> Optional[AnyBlueprint]:
        """
        Retrieve a blueprint by name.
        
        Args:
            name: Blueprint name (can be short name or fully qualified)
            namespace: Optional namespace to prepend
            
        Returns:
            Blueprint if found, None otherwise
            
        Example:
            >>> # With explicit namespace
            >>> bp = CallbackRegistry.get_blueprint("initialize", namespace="v2_modulemanager")
            
            >>> # With fully qualified name
            >>> bp = CallbackRegistry.get_blueprint("v2_modulemanager/initialize")
        """
        with cls._lock:
            cls.initialize()
            
            # Try with namespace first
            if namespace:
                full_name = f"{namespace.strip('/')}/{name}"
                if full_name in cls._blueprints:
                    return cls._blueprints[full_name]
            
            # Try exact name
            if name in cls._blueprints:
                return cls._blueprints[name]
            
            return None
    
    @classmethod
    def bind_callback(
        cls,
        name: str,
        callback: Callable,
        callback_type: str = 'default',
        namespace: Optional[str] = None
    ) -> bool:
        """
        Bind a callback to a registered blueprint.
        
        For ActionBlueprint, callback_type specifies which lifecycle callback
        to bind ('on_goal', 'on_cancel', 'execute'). For other blueprints,
        use 'default'.
        
        Args:
            name: Blueprint name to bind to
            callback: Implementation function
            callback_type: For ActionBlueprint: 'on_goal', 'on_cancel', 'execute'
                          For others: 'default' (ignored)
            namespace: Optional namespace
            
        Returns:
            True if binding succeeded, False if blueprint not found
            
        Raises:
            RuntimeError: If blueprint already has a bound callback
            ValueError: If callback signature is invalid or callback_type invalid
            
        Example:
            >>> # Service binding
            >>> async def my_service(request, response=None):
            ...     return {"status": "ok"}
            >>> CallbackRegistry.bind_callback("my_service", my_service)
            True
            
            >>> # ActionServer binding (multi-callback)
            >>> async def execute_action(goal_handle):
            ...     return {"result": "done"}
            >>> CallbackRegistry.bind_callback(
            ...     "process", execute_action, callback_type='execute'
            ... )
            True
        """
        with cls._lock:
            cls.initialize()
            
            blueprint = cls.get_blueprint(name, namespace)
            if blueprint is None:
                logger.error(f"❌ Cannot bind callback: Blueprint '{name}' not found")
                return False
            
            try:
                # ActionBlueprint has multi-callback support
                from vyra_base.com.core.blueprints import ActionBlueprint
                
                if isinstance(blueprint, ActionBlueprint) and callback_type != 'default':
                    blueprint.bind_callback(callback, callback_type)
                    full_name = cls._find_full_name(blueprint)
                    logger.info(f"✅ Bound '{callback_type}' callback to '{full_name}'")
                else:
                    blueprint.bind_callback(callback)
                    full_name = cls._find_full_name(blueprint)
                    logger.info(f"✅ Bound callback to '{full_name}'")
                
                return True
            except (RuntimeError, ValueError) as e:
                logger.error(f"❌ Failed to bind callback to '{name}': {e}")
                raise
    
    @classmethod
    def unbind_callback(
        cls,
        name: str,
        namespace: Optional[str] = None
    ) -> bool:
        """
        Remove callback binding from a blueprint.
        
        Args:
            name: Blueprint name
            namespace: Optional namespace
            
        Returns:
            True if unbinding succeeded, False if blueprint not found
        """
        with cls._lock:
            cls.initialize()
            
            blueprint = cls.get_blueprint(name, namespace)
            if blueprint is None:
                return False
            
            old_callback = blueprint.unbind_callback()
            if old_callback:
                logger.debug(f"🔓 Unbound callback from '{name}'")
            
            return old_callback is not None
    
    @classmethod
    def list_all(
        cls,
        namespace: Optional[str] = None,
        interface_type: Optional[InterfaceType] = None
    ) -> List[str]:
        """
        List all registered blueprint names.
        
        Args:
            namespace: Filter by namespace prefix
            interface_type: Filter by interface type
            
        Returns:
            List of blueprint names
        """
        with cls._lock:
            cls.initialize()
            
            names = list(cls._blueprints.keys())
            
            # Filter by namespace
            if namespace:
                prefix = f"{namespace.strip('/')}/"
                names = [n for n in names if n.startswith(prefix)]
            
            # Filter by type
            if interface_type:
                names = [
                    n for n in names
                    if cls._blueprints[n].interface_type == interface_type
                ]
            
            return sorted(names)
    
    @classmethod
    def list_unbound(cls, namespace: Optional[str] = None) -> List[str]:
        """
        List blueprints that don't have callbacks bound yet.
        
        Useful for debugging initialization issues.
        
        Args:
            namespace: Filter by namespace
            
        Returns:
            List of unbound blueprint names
            
        Example:
            >>> unbound = CallbackRegistry.list_unbound()
            >>> if unbound:
            ...     logger.warning(f"Unbound interfaces: {unbound}")
        """
        with cls._lock:
            cls.initialize()
            
            all_names = cls.list_all(namespace)
            unbound = [
                name for name in all_names
                if not cls._blueprints[name].is_bound()
            ]
            return unbound
    
    @classmethod
    def list_bound(cls, namespace: Optional[str] = None) -> List[str]:
        """
        List blueprints that have callbacks bound.
        
        Args:
            namespace: Filter by namespace
            
        Returns:
            List of bound blueprint names
        """
        with cls._lock:
            cls.initialize()
            
            all_names = cls.list_all(namespace)
            bound = [
                name for name in all_names
                if cls._blueprints[name].is_bound()
            ]
            return bound
    
    @classmethod
    def get_statistics(cls, namespace: Optional[str] = None) -> Dict[str, int]:
        """
        Get registry statistics.
        
        Args:
            namespace: Filter by namespace
            
        Returns:
            Dictionary with counts by type and binding status
            
        Example:
            >>> stats = CallbackRegistry.get_statistics()
            >>> print(stats)
            {'total': 15, 'bound': 12, 'unbound': 3, 'services': 8, ...}
        """
        with cls._lock:
            cls.initialize()
            
            all_names = cls.list_all(namespace)
            blueprints = [cls._blueprints[name] for name in all_names]
            
            stats = {
                'total': len(blueprints),
                'bound': sum(1 for bp in blueprints if bp.is_bound()),
                'unbound': sum(1 for bp in blueprints if not bp.is_bound()),
            }
            
            # Count by interface type
            for itype in InterfaceType:
                count = sum(1 for bp in blueprints if bp.interface_type == itype)
                stats[itype.value + 's'] = count
            
            return stats
    
    @classmethod
    def remove_blueprint(
        cls,
        name: str,
        namespace: Optional[str] = None
    ) -> bool:
        """
        Remove a blueprint from the registry.
        
        Args:
            name: Blueprint name
            namespace: Optional namespace
            
        Returns:
            True if removed, False if not found
        """
        with cls._lock:
            cls.initialize()
            
            # Build possible names
            names_to_try = []
            if namespace:
                names_to_try.append(f"{namespace.strip('/')}/{name}")
            names_to_try.append(name)
            
            for full_name in names_to_try:
                if full_name in cls._blueprints:
                    del cls._blueprints[full_name]
                    logger.debug(f"🗑️  Removed blueprint '{full_name}'")
                    return True
            
            return False
    
    @classmethod
    def exists(
        cls,
        name: str,
        namespace: Optional[str] = None
    ) -> bool:
        """
        Check if a blueprint exists in the registry.
        
        Args:
            name: Blueprint name
            namespace: Optional namespace
            
        Returns:
            True if blueprint exists
        """
        with cls._lock:
            return cls.get_blueprint(name, namespace) is not None
    
    @classmethod
    def _find_full_name(cls, blueprint: AnyBlueprint) -> str:
        """Find the full registered name for a blueprint instance"""
        for name, bp in cls._blueprints.items():
            if bp is blueprint:
                return name
        return blueprint.name  # Fallback to blueprint's own name
    
    @classmethod
    def debug_print(cls, namespace: Optional[str] = None) -> None:
        """
        Print registry contents for debugging.
        
        For ActionBlueprints, shows multi-callback status (on_goal/on_cancel/execute).
        
        Args:
            namespace: Filter by namespace
        """
        with cls._lock:
            cls.initialize()
            
            from vyra_base.com.core.blueprints import ActionBlueprint
            
            print("\n" + "="*60)
            print("📋 CALLBACK REGISTRY DEBUG")
            print("="*60)
            
            stats = cls.get_statistics(namespace)
            print(f"\n📊 Statistics:")
            for key, value in stats.items():
                print(f"   {key:12s}: {value}")
            
            print(f"\n📝 Blueprints:")
            all_names = cls.list_all(namespace)
            
            if not all_names:
                print("   (none)")
            else:
                for name in all_names:
                    bp = cls._blueprints[name]
                    bound_icon = "✓" if bp.is_bound() else "✗"
                    protocols = ",".join(p.value for p in bp.protocols[:2])
                    
                    # For ActionBlueprint, show multi-callback status
                    if isinstance(bp, ActionBlueprint):
                        callbacks_status = {
                            k: ("✓" if v else "✗")
                            for k, v in bp._callbacks.items()
                        }
                        callback_detail = f"({callbacks_status['on_goal']}/goal {callbacks_status['on_cancel']}/cancel {callbacks_status['execute']}/exec)"
                        print(f"   [{bound_icon}] {name:40s} {bp.interface_type.value:12s} [{protocols}] {callback_detail}")
                    else:
                        print(f"   [{bound_icon}] {name:40s} {bp.interface_type.value:12s} [{protocols}]")
            
            unbound = cls.list_unbound(namespace)
            if unbound:
                print(f"\n⚠️  Unbound blueprints ({len(unbound)}):")
                for name in unbound:
                    print(f"   - {name}")
            
            print("="*60 + "\n")
