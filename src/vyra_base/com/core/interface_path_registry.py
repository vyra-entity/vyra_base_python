"""
Interface Path Registry for Dynamic Interface Loading

Singleton registry that stores interface base paths for module-wide access.
Enables dynamic discovery of ROS2 interfaces (.srv/.msg/.action) and 
Protocol Buffer definitions (*_pb2.py) from registered paths.

Thread-safe singleton pattern ensures consistent path configuration across
all components (TopicBuilder, InterfaceLoader, Protocol Providers).
"""
from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


class InterfacePathRegistry:
    """
    Thread-safe singleton registry for interface base paths.
    
    Stores paths to interface directories containing:
    - /config/*.json - Interface metadata
    - /server/*.srv - ROS2 service definitions
    - /publisher/*.msg - ROS2 message definitions  
    - /actionServer/*.action - ROS2 action definitions
    - /proto/*.proto - Protocol Buffer definitions
    - /proto/*_pb2.py - Generated Python protobuf modules
    
    Default path points to vyra_base/interfaces (embedded package).
    Modules can register additional paths via set_interface_paths().
    
    Examples:
        >>> registry = InterfacePathRegistry.get_instance()
        >>> registry.set_interface_paths([
        ...     "/workspace/install/v2_modulemanager_interfaces/share/v2_modulemanager_interfaces",
        ...     "/workspace/install/vyra_module_template_interfaces/share/vyra_module_template_interfaces"
        ... ])
        >>> paths = registry.get_interface_paths()
    """
    
    _instance: Optional[InterfacePathRegistry] = None
    _lock = threading.Lock()
    
    def __init__(self):
        """Private constructor - use get_instance() instead."""
        if InterfacePathRegistry._instance is not None:
            raise RuntimeError(
                "InterfacePathRegistry is a singleton. Use get_instance() instead."
            )
        
        self._interface_paths: list[Path] = []
        self._paths_lock = threading.Lock()
        
        # Set default path to vyra_base/interfaces
        self._set_default_path()
        
        logger.debug("InterfacePathRegistry initialized with default paths")
    
    @classmethod
    def get_instance(cls) -> InterfacePathRegistry:
        """
        Get singleton instance (thread-safe).
        
        Returns:
            InterfacePathRegistry singleton instance
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = InterfacePathRegistry()
        return cls._instance
    
    def _set_default_path(self) -> None:
        """Set default path to vyra_base/interfaces (relative to this file)."""
        # This file is in: vyra_base/com/core/interface_path_registry.py
        # Default interfaces are in: vyra_base/interfaces/
        vyra_base_dir = Path(__file__).parent.parent.parent.resolve()
        default_interface_path = vyra_base_dir / "interfaces"
        
        if default_interface_path.exists():
            self._interface_paths = [default_interface_path]
            logger.info(f"✓ Default interface path: {default_interface_path}")
        else:
            logger.warning(
                f"⚠️ Default interface path not found: {default_interface_path}"
            )
            self._interface_paths = []
    
    def set_interface_paths(self, paths: list[str | Path]) -> None:
        """
        Set interface base paths (replaces existing paths).
        
        Args:
            paths: List of interface directory paths
            
        Raises:
            ValueError: If any path doesn't exist or isn't readable
            
        Examples:
            >>> registry.set_interface_paths([
            ...     "/workspace/install/v2_modulemanager_interfaces/share/v2_modulemanager_interfaces",
            ...     "/workspace/install/vyra_module_template_interfaces/share/vyra_module_template_interfaces"
            ... ])
        """
        with self._paths_lock:
            validated_paths: list[Path] = []
            
            for path_str in paths:
                path = Path(path_str).resolve()
                
                if not path.exists():
                    logger.warning(
                        f"⚠️ Interface path does not exist: {path}"
                    )
                    continue
                
                if not path.is_dir():
                    logger.warning(
                        f"⚠️ Interface path is not a directory: {path}"
                    )
                    continue
                
                validated_paths.append(path)
                logger.debug(f"✓ Validated interface path: {path}")
            
            if not validated_paths:
                raise ValueError(
                    "No valid interface paths provided. At least one valid path required."
                )
            
            self._interface_paths = validated_paths
            logger.info(
                f"✓ Interface paths updated: {len(validated_paths)} path(s) registered"
            )
            for idx, path in enumerate(validated_paths, 1):
                logger.info(f"  [{idx}] {path}")
    
    def add_interface_path(self, path: str | Path) -> None:
        """
        Add an interface path to existing paths (without replacing).
        
        Args:
            path: Interface directory path to add
            
        Raises:
            ValueError: If path doesn't exist or isn't readable
        """
        with self._paths_lock:
            path_obj = Path(path).resolve()
            
            if not path_obj.exists():
                raise ValueError(f"Interface path does not exist: {path_obj}")
            
            if not path_obj.is_dir():
                raise ValueError(f"Interface path is not a directory: {path_obj}")
            
            if path_obj in self._interface_paths:
                logger.debug(f"Interface path already registered: {path_obj}")
                return
            
            self._interface_paths.append(path_obj)
            logger.info(f"✓ Added interface path: {path_obj}")
    
    def get_interface_paths(self) -> list[Path]:
        """
        Get list of registered interface paths.
        
        Returns:
            Copy of interface paths list (thread-safe)
        """
        with self._paths_lock:
            return self._interface_paths.copy()
    
    def clear_interface_paths(self) -> None:
        """Clear all interface paths and reset to default."""
        with self._paths_lock:
            self._set_default_path()
            logger.info("Interface paths reset to default")
    
    def get_config_paths(self) -> list[Path]:
        """
        Get paths to config directories (/config subdirs of interface paths).
        
        Returns:
            List of existing config directory paths
        """
        with self._paths_lock:
            config_paths = []
            for interface_path in self._interface_paths:
                config_path = interface_path / "config"
                if config_path.exists() and config_path.is_dir():
                    config_paths.append(config_path)
            return config_paths
    
    def get_proto_paths(self) -> list[Path]:
        """
        Get paths to proto directories (/proto subdirs of interface paths).
        
        Returns:
            List of existing proto directory paths
        """
        with self._paths_lock:
            proto_paths = []
            for interface_path in self._interface_paths:
                proto_path = interface_path / "proto"
                if proto_path.exists() and proto_path.is_dir():
                    proto_paths.append(proto_path)
            return proto_paths
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        with self._paths_lock:
            return (
                f"InterfacePathRegistry("
                f"paths={len(self._interface_paths)}, "
                f"locations={[str(p) for p in self._interface_paths]}"
                f")"
            )


# Convenience module-level function
def get_interface_registry() -> InterfacePathRegistry:
    """
    Convenience function to get singleton instance.
    
    Returns:
        InterfacePathRegistry singleton
    """
    return InterfacePathRegistry.get_instance()
