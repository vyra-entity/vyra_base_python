"""
Dynamic Interface Loader for ROS2 and Protocol Buffer Interfaces

Loads interface type classes at runtime from string names, supporting:
- ROS2 Interfaces: .srv (services), .msg (messages), .action (actions)
- Protocol Buffers: *_pb2.py generated modules

Eliminates need for compile-time imports, enabling true dynamic interface
discovery and cross-module communication without hardcoded dependencies.
"""
from __future__ import annotations

import importlib
import importlib.util
import json
import logging
import sys
from pathlib import Path
from typing import Any, Optional, Union

from vyra_base.com.core.interface_path_registry import InterfacePathRegistry

logger = logging.getLogger(__name__)

# Check ROS2 availability
try:
    _ROS2_AVAILABLE = importlib.util.find_spec("rclpy") is not None
except (ValueError, AttributeError):
    # find_spec raises ValueError if module is in sys.modules but __spec__ is None
    # (e.g. when ROS2 is sourced but running in a venv)
    _ROS2_AVAILABLE = "rclpy" in __import__("sys").modules

if _ROS2_AVAILABLE:
    try:
        from rosidl_runtime_py.utilities import get_message, get_service, get_action
    except ImportError:
        logger.warning(
            "⚠️ rosidl_runtime_py not available. ROS2 dynamic interface loading disabled."
        )
        _ROS2_AVAILABLE = False
        get_message = None  # type: ignore[assignment]
        get_service = None  # type: ignore[assignment]
        get_action = None  # type: ignore[assignment]
else:
    get_message = None  # type: ignore[assignment]
    get_service = None  # type: ignore[assignment]
    get_action = None  # type: ignore[assignment]


class InterfaceLoader:
    """
    Dynamic loader for ROS2 and Protocol Buffer interfaces.
    
    Provides runtime loading of interface type classes from string names,
    eliminating compile-time import dependencies.
    
    Features:
        - ROS2 interface loading via rosidl_runtime_py
        - Protocol Buffer module loading (*_pb2.py)
        - Interface metadata loading from JSON configs
        - Caching for performance
        - Graceful fallback when ROS2 unavailable
    
    Examples:
        >>> loader = InterfaceLoader()
        
        # Load ROS2 service interface
        >>> srv_type = loader.load_ros2_interface("your_module_interfaces/srv/VBASEGetInterfaceList")
        >>> print(srv_type)  # <class 'your_module_interfaces.srv.VBASEGetInterfaceList'>
        
        # Load Protocol Buffer interface
        >>> pb_type = loader.load_protobuf_interface("VBASEGetInterfaceList")
        >>> print(pb_type)  # <module 'VBASEGetInterfaceList_pb2'>
        
        # Load by function name from metadata
        >>> interface = loader.get_interface_for_function("get_interface_list", protocol="ros2")
    """
    
    def __init__(
        self,
        interface_paths: Optional[list[Path]] = None,
        auto_update_paths: bool = True
    ):
        """
        Initialize interface loader.
        
        Args:
            interface_paths: List of base interface paths. If None, uses registry default.
            auto_update_paths: If True, automatically updates environment paths for discovery
        """
        # Get interface paths from registry if not provided
        if interface_paths is None:
            registry = InterfacePathRegistry.get_instance()
            self.interface_paths = registry.get_interface_paths()
        else:
            self.interface_paths = [Path(p).resolve() for p in interface_paths]
        
        # Caches for loaded interfaces
        self._ros2_interface_cache: dict[str, type] = {}
        self._protobuf_interface_cache: dict[str, Any] = {}
        self._metadata_cache: Optional[dict[str, dict]] = None
        
        # Update environment if requested
        if auto_update_paths:
            self._update_environment_paths()
        
        logger.debug(
            f"InterfaceLoader initialized with {len(self.interface_paths)} interface path(s)"
        )
    
    def _update_environment_paths(self) -> None:
        """Update AMENT_PREFIX_PATH and sys.path from interface paths."""
        from vyra_base.helper.ros2_env_helper import (
            update_ament_prefix_path,
            update_python_path
        )
        
        for interface_path in self.interface_paths:
            # Interface path is typically: .../install/<package>/share/<package>
            # We need: .../install for AMENT_PREFIX_PATH
            # We need: .../install/<package>/lib/python3.x/site-packages for sys.path
            
            # Try to find install directory (2-3 levels up)
            install_dir = interface_path.parent.parent.parent
            if install_dir.name == "install":
                update_ament_prefix_path(install_dir)
            
            # Try to find Python site-packages
            package_lib = interface_path.parent.parent / "lib"
            if package_lib.exists():
                for python_dir in package_lib.glob("python*"):
                    site_packages = python_dir / "site-packages"
                    if site_packages.exists():
                        update_python_path(site_packages)
    
    def load_ros2_interface(self, interface_path: str) -> Optional[type]:
        """
        Load ROS2 interface type from string path.
        
        Uses rosidl_runtime_py.utilities to dynamically load interface classes
        without compile-time imports.
        
        Args:
            interface_path: Interface path in format "package/type/Name"
                Examples:
                - "std_msgs/msg/String"
                - "your_module_interfaces/srv/VBASEGetInterfaceList"
                - "action_msgs/action/MyAction"
        
        Returns:
            Interface type class, or None if unavailable
        
        Examples:
            >>> loader.load_ros2_interface("std_msgs/msg/String")
            <class 'std_msgs.msg._string.String'>
        """
        # Check cache first
        if interface_path in self._ros2_interface_cache:
            logger.debug(f"Cache hit for ROS2 interface: {interface_path}")
            return self._ros2_interface_cache[interface_path]
        
        if not _ROS2_AVAILABLE:
            logger.debug(
                f"ROS2 not available, cannot load interface: {interface_path}"
            )
            return None
        
        try:
            # Parse interface path
            parts = interface_path.split('/')
            if len(parts) != 3:
                logger.error(
                    f"❌ Invalid interface path format: {interface_path}. "
                    f"Expected 'package/type/Name'"
                )
                return None
            
            package_name, interface_type, interface_name = parts
            
            # Load interface based on type
            interface_class = None
            
            if not _ROS2_AVAILABLE:
                logger.error("ROS2 not installed")
                return None

            if interface_type == "msg" and get_message is not None:
                interface_class = get_message(interface_path)
            elif interface_type == "srv" and get_service is not None:
                interface_class = get_service(interface_path)
            elif interface_type == "action" and get_action is not None:
                interface_class = get_action(interface_path)
            else:
                logger.error(
                    f"❌ Unknown interface type '{interface_type}' in: {interface_path}"
                )
                return None
            
            # Cache successful load
            self._ros2_interface_cache[interface_path] = interface_class
            logger.info(f"✓ Loaded ROS2 interface: {interface_path}")
            
            return interface_class
        
        except Exception as e:
            logger.error(
                f"❌ Failed to load ROS2 interface '{interface_path}': {e}",
                exc_info=True
            )
            return None
    
    def load_protobuf_interface(self, interface_name: str) -> Optional[Any]:
        """
        Load Protocol Buffer interface module (*_pb2.py).
        
        Searches proto/ directories in registered interface paths for
        generated Python protobuf modules.
        
        Args:
            interface_name: Base name of interface (without _pb2 suffix)
                Examples:
                - "VBASEGetInterfaceList" → VBASEGetInterfaceList_pb2.py
                - "VBASEStateFeed" → VBASEStateFeed_pb2.py
        
        Returns:
            Loaded protobuf module, or None if not found
        
        Examples:
            >>> loader.load_protobuf_interface("VBASEGetInterfaceList")
            <module 'VBASEGetInterfaceList_pb2'>
        """
        # Check cache first
        if interface_name in self._protobuf_interface_cache:
            logger.debug(f"Cache hit for protobuf interface: {interface_name}")
            return self._protobuf_interface_cache[interface_name]
        
        # Build module name
        built_proto_name = f"{interface_name}_pb2"
        
        # Try direct import first (if module already in sys.path)
        try:
            module = importlib.import_module(built_proto_name)
            self._protobuf_interface_cache[interface_name] = module
            logger.info(f"✓ Loaded protobuf interface via import: {built_proto_name}")
            return module
        except ImportError:
            logger.warning(
                f"⚠️ Protobuf module '{built_proto_name}' not found via import, "
                f"searching interface paths..."
            )
            pass
        
        # Search in interface paths
        proto_paths: list[Path] = []
        for interface_path in self.interface_paths:
            proto_subpaths = [
                interface_path / "msg" / "_gen",
                interface_path / "srv" / "_gen",
                interface_path / "action" / "_gen"
            ]
            for proto_subpath in proto_subpaths:
                if proto_subpath.is_dir():
                    proto_paths.append(proto_subpath)
        
        for proto_path in proto_paths:
            module_file = proto_path / f"{built_proto_name}.py"
            
            if not module_file.exists():
                continue
            
            try:
                # Load module from file path
                spec = importlib.util.spec_from_file_location(
                    built_proto_name,
                    module_file
                )
                
                if spec is None or spec.loader is None:
                    continue
                
                module = importlib.util.module_from_spec(spec)
                sys.modules[built_proto_name] = module
                spec.loader.exec_module(module)
                
                # Cache successful load
                self._protobuf_interface_cache[interface_name] = module
                logger.info(
                    f"✓ Loaded protobuf interface from file: {module_file}"
                )
                
                return module
            
            except Exception as e:
                logger.error(
                    f"❌ Failed to load protobuf module from {module_file}: {e}",
                    exc_info=True
                )
                continue
        
        logger.warning(
            f"⚠️ Protobuf interface not found: {interface_name} "
            f"(searched in {len(proto_paths)} proto path(s))"
        )
        return None
    
    def load_interface_metadata(self, reload: bool = False) -> dict[str, dict]:
        """
        Load interface metadata from JSON config files.
        
        Scans config/ directories in registered interface paths and
        loads all *.json files containing interface metadata.
        
        Args:
            reload: Force reload even if cached
        
        Returns:
            Dict mapping function_name → metadata dict
        
        Examples:
            >>> metadata = loader.load_interface_metadata()
            >>> print(metadata.keys())
            dict_keys(['get_interface_list', 'health_check', 'initialize', ...])
            
            >>> meta = metadata['get_interface_list']
            >>> print(meta['type'])  # 'callable'
            >>> print(meta['filetype'])  # ['VBASEGetInterfaceList.srv', 'VBASEGetInterfaceList.proto']
        """
        # Return cache if available and not forcing reload
        if self._metadata_cache is not None and not reload:
            return self._metadata_cache
        
        metadata: dict[str, dict] = {}
        
        # Derive config paths from self.interface_paths
        config_paths: list[Path] = []
        for interface_path in self.interface_paths:
            config_path = interface_path / "config"
            if config_path.is_dir():
                config_paths.append(config_path)
        
        for config_path in config_paths:
            # Find all JSON files
            json_files = list(config_path.glob("*.json"))
            
            for json_file in json_files:
                try:
                    with open(json_file, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    # Handle both single object and array of objects
                    entries = data if isinstance(data, list) else [data]
                    
                    for entry in entries:
                        # Skip if not visible or missing required fields
                        if not entry.get('displaystyle', {}).get('visible', True):
                            continue
                        
                        function_name = entry.get('functionname')
                        if not function_name:
                            logger.warning(
                                f"⚠️ Metadata entry missing 'functionname' in {json_file}"
                            )
                            continue
                        
                        # Store metadata by function name
                        if function_name in metadata:
                            logger.debug(
                                f"Overwriting metadata for '{function_name}' "
                                f"from {json_file}"
                            )
                        
                        metadata[function_name] = entry
                
                except json.JSONDecodeError as e:
                    logger.error(
                        f"❌ Invalid JSON in {json_file}: {e}"
                    )
                except Exception as e:
                    logger.error(
                        f"❌ Failed to load metadata from {json_file}: {e}",
                        exc_info=True
                    )
        
        # Cache loaded metadata
        self._metadata_cache = metadata
        
        logger.info(
            f"✓ Loaded metadata for {len(metadata)} interface(s) "
            f"from {len(config_paths)} config path(s)"
        )
        
        return metadata
    
    def get_interface_for_function(
        self,
        function_name: str,
        protocol: str = "ros2"
    ) -> Optional[Union[type, Any]]:
        """
        Load interface type for a function by name.
        
        Looks up function in metadata, extracts appropriate interface path
        based on protocol, and loads the interface.
        
        Args:
            function_name: Name of function (from functionname in metadata)
            protocol: Protocol to use: "ros2", "zenoh", "redis", "uds"
                (ros2 uses .srv/.msg/.action, others use .proto)
        
        Returns:
            Interface type/module, or None if not found
        
        Examples:
            >>> # Load ROS2 service interface
            >>> srv = loader.get_interface_for_function("get_interface_list", protocol="ros2")
            
            >>> # Load protobuf for Zenoh
            >>> pb = loader.get_interface_for_function("get_interface_list", protocol="zenoh")
        """
        # Load metadata
        metadata = self.load_interface_metadata()
        
        if function_name not in metadata:
            logger.warning(
                f"⚠️ No metadata found for function: {function_name}"
            )
            return None
        
        meta = metadata[function_name]
        interface_type = meta.get('type', 'callable')
        filetypes = meta.get('filetype', [])
        
        if not filetypes:
            logger.error(
                f"❌ No filetypes specified in metadata for: {function_name}"
            )
            return None
        
        # Determine which file type to use based on protocol
        if protocol.lower() == "ros2":
            # Look for ROS2 interface (.srv, .msg, .action)
            for filetype in filetypes:
                if filetype.endswith('.srv'):
                    # Extract interface path from filename
                    interface_name = filetype[:-4]  # Remove .srv
                    # Need to determine package name - check tags or use vyra_module_template_interfaces
                    package_name = self._infer_package_name(meta, interface_name)
                    interface_path = f"{package_name}/srv/{interface_name}"
                    return self.load_ros2_interface(interface_path)
                
                elif filetype.endswith('.msg'):
                    interface_name = filetype[:-4]  # Remove .msg
                    package_name = self._infer_package_name(meta, interface_name)
                    interface_path = f"{package_name}/msg/{interface_name}"
                    return self.load_ros2_interface(interface_path)
                
                elif filetype.endswith('.action'):
                    interface_name = filetype[:-7]  # Remove .action
                    package_name = self._infer_package_name(meta, interface_name)
                    interface_path = f"{package_name}/action/{interface_name}"
                    return self.load_ros2_interface(interface_path)
        
        else:
            # Look for protobuf interface (.proto → _pb2.py)
            for filetype in filetypes:
                if filetype.endswith('.proto'):
                    interface_name = filetype[:-6]  # Remove .proto
                    return self.load_protobuf_interface(interface_name)
        
        logger.warning(
            f"⚠️ No suitable interface found for function '{function_name}' "
            f"with protocol '{protocol}'"
        )
        return None
    
    def _infer_package_name(self, metadata: dict, interface_name: str) -> str:
        """
        Infer ROS2 package name from metadata or default to vyra_module_template_interfaces.
        
        Args:
            metadata: Interface metadata dict
            interface_name: Name of interface
        
        Returns:
            Package name
        """
        # Check if package info in metadata (future extension)
        if 'package' in metadata:
            return metadata['package']
        
        # Try to infer from interface paths first (most reliable)
        for interface_path in self.interface_paths:
            # Path typically: .../share/<package_name>
            if "share" in interface_path.parts:
                share_idx = interface_path.parts.index("share")
                if share_idx + 1 < len(interface_path.parts):
                    return interface_path.parts[share_idx + 1]
        
        # Fallback to default
        return "vyra_module_template_interfaces"
    
    def clear_cache(self) -> None:
        """Clear all cached interfaces and metadata."""
        self._ros2_interface_cache.clear()
        self._protobuf_interface_cache.clear()
        self._metadata_cache = None
        logger.info("✓ Interface cache cleared")
    
    def get_cache_stats(self) -> dict[str, int]:
        """
        Get cache statistics.
        
        Returns:
            Dict with cache sizes
        """
        return {
            'ros2_interfaces': len(self._ros2_interface_cache),
            'protobuf_interfaces': len(self._protobuf_interface_cache),
            'metadata_entries': len(self._metadata_cache) if self._metadata_cache else 0
        }
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        stats = self.get_cache_stats()
        return (
            f"InterfaceLoader("
            f"paths={len(self.interface_paths)}, "
            f"ros2_cached={stats['ros2_interfaces']}, "
            f"pb_cached={stats['protobuf_interfaces']}, "
            f"metadata={stats['metadata_entries']}"
            f")"
        )
