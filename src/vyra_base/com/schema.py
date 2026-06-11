"""
Schema Resolver

The SchemaResolver resolves wire-format
schema objects (ROS2 type classes, protobuf modules) from filesystem paths
at runtime — no compile-time imports required.

Design:
- ``SchemaType`` enum distinguishes ROS2 from Protobuf schemas.
- ``SchemaResolver`` is the base class; both ``Ros2Resolver`` and
  ``ProtoResolver`` derive from it and override ``_do_load``.
- The unified entry-point is ``load_interface(name, interface_type)``,
  replacing the previous ``load_ros2_interface`` / ``load_protobuf_interface``
  split.
- ``get_interface_for_function(function_name, protocol)`` provides the same
  high-level lookup that TopicBuilder and transport providers use.
- ``load_interface_metadata`` has been **removed** — that responsibility
  now lives exclusively in ``ManifestResolver``.

Thread-safety: each resolver instance owns its own cache protected by a
reentrant lock.
"""

from __future__ import annotations

import asyncio
import importlib
import importlib.util
import logging
import pkgutil
import sys
import threading
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# ROS2 availability probe
# ---------------------------------------------------------------------------
try:
    _ROS2_AVAILABLE = importlib.util.find_spec("rclpy") is not None
except (ValueError, AttributeError):
    _ROS2_AVAILABLE = "rclpy" in sys.modules

if _ROS2_AVAILABLE:
    try:
        from rosidl_runtime_py.utilities import get_message, get_service, get_action
    except ImportError:
        logger.warning(
            "rosidl_runtime_py not available — ROS2 dynamic interface loading disabled."
        )
        _ROS2_AVAILABLE = False
        get_message = get_service = get_action = None  # type: ignore[assignment]
else:
    get_message = get_service = get_action = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Enum
# ---------------------------------------------------------------------------

class SchemaType(Enum):
    """Identifies the wire-format schema family for an interface."""
    ROS2 = "ros2"
    PROTOBUF = "protobuf"


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class SchemaResolver:
    """
    Base resolver for wire-format schema objects.

    Subclasses implement ``_do_load`` for their specific schema family.
    The base class provides:
    - Path management (reads from ``ManifestResolver`` if none given)
    - A shared cache per instance
    - ``get_interface_for_function`` that delegates to metadata-guided lookup
    - ``load_interface(name, interface_type)`` as the unified entry-point

    Args:
        interface_paths:    Override the paths used for schema discovery.
                            If omitted the ManifestResolver singleton is queried.
        auto_update_paths:  When True, update ``AMENT_PREFIX_PATH`` / ``sys.path``
                            on construction to assist ROS2 discovery.
    """

    def __init__(
        self,
        interface_paths: Optional[List[Path]] = None,
        auto_update_paths: bool = True,
    ) -> None:
        if interface_paths is None:
            from vyra_base.com.manifest import ManifestResolver
            self.interface_paths: List[Path] = (
                ManifestResolver.get_instance().get_manifest_paths()
            )
        else:
            self.interface_paths = [Path(p).resolve() for p in interface_paths]

        self._cache: Dict[str, Any] = {}
        self._lock = threading.RLock()

        # asyncio event for path/cache changes (optional integration)
        self._change_event: Optional[asyncio.Event] = None

        if auto_update_paths:
            self._update_environment_paths()

        logger.debug(
            "%s initialised with %d path(s).",
            self.__class__.__name__,
            len(self.interface_paths),
        )

    # ---- asyncio event ----------------------------------------------------

    def set_event_loop(self) -> None:
        self._change_event = asyncio.Event()

    @property
    def change_event(self) -> Optional[asyncio.Event]:
        return self._change_event

    def _emit_change(self) -> None:
        if self._change_event is not None:
            try:
                self._change_event.set()
            except RuntimeError:
                pass

    # ---- environment helpers ----------------------------------------------

    def _update_environment_paths(self) -> None:
        """Add discovered AMENT / site-packages paths for ROS2 resolution."""
        try:
            from vyra_base.helper.ros2_env_helper import (
                update_ament_prefix_path,
                update_python_path,
            )
        except ImportError:
            return

        for ipath in self.interface_paths:
            install_dir = ipath.parent.parent.parent
            if install_dir.name == "install":
                update_ament_prefix_path(install_dir)

            package_lib = ipath.parent.parent / "lib"
            if package_lib.exists():
                for python_dir in package_lib.glob("python*"):
                    site_packages = python_dir / "site-packages"
                    if site_packages.exists():
                        update_python_path(site_packages)

    # ---- unified load entry-point ----------------------------------------

    def load_interface(
        self, name: str, interface_type: SchemaType
    ) -> Optional[Any]:
        """
        Load a schema object by name and type.

        Delegates to ``_do_load`` in the concrete subclass.

        Args:
            name:           Schema identifier.
                            - For ROS2: ``"package/type/Name"``
                              e.g. ``"std_msgs/msg/String"``
                            - For Protobuf: base name without ``_pb2`` suffix
                              e.g. ``"VBASEGetInterfaceList"``
            interface_type: ``SchemaType.ROS2`` or ``SchemaType.PROTOBUF``.

        Returns:
            The loaded type/module, or None on failure.
        """
        cache_key = f"{interface_type.value}::{name}"
        with self._lock:
            if cache_key in self._cache:
                logger.debug("Cache hit: %s", cache_key)
                return self._cache[cache_key]

        result = self._do_load(name, interface_type)

        if result is not None:
            with self._lock:
                self._cache[cache_key] = result

        return result

    def _do_load(self, name: str, interface_type: SchemaType) -> Optional[Any]:
        """
        Concrete loading logic — override in subclasses.

        The default implementation dispatches to the internal helpers based
        on ``interface_type``, making ``SchemaResolver`` usable directly as
        a combined resolver.
        """
        if interface_type == SchemaType.ROS2:
            return self._load_ros2(name)
        if interface_type == SchemaType.PROTOBUF:
            return self._load_protobuf(name)
        logger.error("Unknown SchemaType: %s", interface_type)
        return None

    # ---- high-level function lookup --------------------------------------

    def get_interface_for_function(
        self,
        function_name: str,
        protocol: str = "ros2",
    ) -> Optional[Any]:
        """
        Look up the schema for a function name using metadata from ManifestResolver.

        Args:
            function_name: The ``functionname`` from the metadata.
            protocol:      ``"ros2"`` → ROS2 type,
                           any other value → Protobuf module.

        Returns:
            Schema object, or None if not resolvable.
        """
        from vyra_base.com.manifest import ManifestResolver
        metadata = ManifestResolver.get_instance().load_interface_metadata()

        if function_name not in metadata:
            logger.warning(
                "get_interface_for_function: no metadata for '%s'.",
                function_name,
            )
            return None

        meta = metadata[function_name]
        filetypes: List[str] = meta.get("filetype", [])

        if not filetypes:
            logger.error(
                "get_interface_for_function: no filetype entries for '%s'.",
                function_name,
            )
            return None

        if protocol.lower() == "ros2":
            for ft in filetypes:
                for ext, subtype in ((".srv", "srv"), (".msg", "msg"), (".action", "action")):
                    if ft.endswith(ext):
                        iname = ft[: -len(ext)]
                        pkg = self._infer_ros2_package_name(meta, iname)
                        return self.load_interface(
                            f"{pkg}/{subtype}/{iname}", SchemaType.ROS2
                        )
        else:
            for ft in filetypes:
                if ft.endswith(".proto"):
                    iname = ft[:-6]
                    return self.load_interface(iname, SchemaType.PROTOBUF)

        logger.warning(
            "get_interface_for_function: no matching filetype for "
            "'%s' with protocol '%s'.",
            function_name,
            protocol,
        )
        return None

    # ---- cache helpers ---------------------------------------------------

    def clear_cache(self) -> None:
        with self._lock:
            self._cache.clear()
        logger.info("%s cache cleared.", self.__class__.__name__)
        self._emit_change()

    def get_cache_stats(self) -> Dict[str, int]:
        with self._lock:
            ros2 = sum(1 for k in self._cache if k.startswith("ros2::"))
            proto = sum(1 for k in self._cache if k.startswith("protobuf::"))
            return {"ros2": ros2, "protobuf": proto, "total": len(self._cache)}

    def __repr__(self) -> str:
        stats = self.get_cache_stats()
        return (
            f"{self.__class__.__name__}("
            f"paths={len(self.interface_paths)}, "
            f"cached_ros2={stats['ros2']}, "
            f"cached_proto={stats['protobuf']})"
        )

    # ---- internal ROS2 loader --------------------------------------------

    def _load_ros2(self, interface_path: str) -> Optional[type]:
        if not _ROS2_AVAILABLE:
            logger.debug("ROS2 not available — cannot load: %s", interface_path)
            return None

        parts = interface_path.split("/")
        if len(parts) != 3:
            logger.error(
                "Invalid ROS2 path format '%s' — expected 'pkg/type/Name'.",
                interface_path,
            )
            return None

        _, itype, _ = parts
        try:
            if itype == "msg" and get_message is not None:
                result = get_message(interface_path)
            elif itype == "srv" and get_service is not None:
                result = get_service(interface_path)
            elif itype == "action" and get_action is not None:
                result = get_action(interface_path)
            else:
                logger.error(
                    "Unknown ROS2 interface type '%s' in: %s", itype, interface_path
                )
                return None
            logger.info("Loaded ROS2 schema: %s", interface_path)
            return result
        except Exception as exc:
            logger.error(
                "Failed to load ROS2 schema '%s': %s", interface_path, exc,
                exc_info=True,
            )
            return None

    # ---- internal Protobuf loader ----------------------------------------

    def _load_protobuf(self, interface_name: str) -> Optional[Any]:
        module_name = f"{interface_name}_pb2"

        # 1) Try direct import (already on sys.path or in sys.modules)
        try:
            module = importlib.import_module(module_name)
            logger.info("Loaded protobuf schema via import: %s", module_name)
            return module
        except ImportError:
            pass
        except TypeError as exc:
            if "duplicate symbol" in str(exc):
                existing = sys.modules.get(module_name)
                if existing is not None:
                    logger.debug(
                        "Protobuf '%s' already in pool; reusing sys.modules entry.",
                        module_name,
                    )
                    return existing
                return None
            raise

        # 2) Search _gen directories inside registered paths
        gen_dirs: List[Path] = []
        for base in self.interface_paths:
            for sub in ("msg/_gen", "srv/_gen", "action/_gen"):
                candidate = base / sub
                if candidate.is_dir():
                    gen_dirs.append(candidate)

        for gen_dir in gen_dirs:
            module_file = gen_dir / f"{module_name}.py"
            if not module_file.exists():
                continue

            try:
                spec = importlib.util.spec_from_file_location(module_name, module_file)
                if spec is None or spec.loader is None:
                    continue
                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module
                spec.loader.exec_module(module)  # type: ignore[union-attr]
                logger.info("Loaded protobuf schema from file: %s", module_file)
                return module
            except TypeError as exc:
                if "duplicate symbol" in str(exc):
                    sys.modules.pop(module_name, None)
                    logger.debug(
                        "Protobuf '%s' duplicate symbol in %s — skipping.",
                        module_name,
                        module_file,
                    )
                    continue
                sys.modules.pop(module_name, None)
                logger.error(
                    "Failed to load protobuf from %s: %s", module_file, exc,
                    exc_info=True,
                )
            except Exception as exc:
                sys.modules.pop(module_name, None)
                logger.error(
                    "Failed to load protobuf from %s: %s", module_file, exc,
                    exc_info=True,
                )
                continue

        logger.warning(
            "Protobuf schema not found: '%s' (searched %d dir(s)).",
            interface_name,
            len(gen_dirs),
        )
        return None

    # ---- package name inference ------------------------------------------

    def _infer_ros2_package_name(self, metadata: dict, interface_name: str) -> str:
        """Infer the ROS2 package name from metadata or filesystem heuristics."""
        _STANDARD = frozenset({
            "builtin_interfaces", "action_msgs", "std_msgs", "std_srvs",
            "geometry_msgs", "sensor_msgs", "nav_msgs", "rcl_interfaces",
            "rosgraph_msgs", "unique_identifier_msgs", "lifecycle_msgs",
            "composition_interfaces", "diagnostic_msgs", "shape_msgs",
            "stereo_msgs", "trajectory_msgs", "visualization_msgs",
            "statistics_msgs", "test_msgs", "example_interfaces",
            "type_description_interfaces", "service_msgs",
            "logging_demo", "pendulum_msgs",
        })

        if "package" in metadata:
            return metadata["package"]

        for ipath in self.interface_paths:
            if "share" in ipath.parts:
                idx = ipath.parts.index("share")
                if idx + 1 < len(ipath.parts):
                    return ipath.parts[idx + 1]
            if ipath.name.endswith("_interfaces"):
                return ipath.name

        try:
            for _, modname, ispkg in pkgutil.iter_modules():
                if (
                    modname.endswith("_interfaces")
                    and ispkg
                    and modname not in _STANDARD
                ):
                    try:
                        mod = importlib.import_module(modname)
                        if any(hasattr(mod, a) for a in ("action", "srv", "msg")):
                            return modname
                    except Exception:
                        continue
        except Exception:
            pass

        import os
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
            if not prefix:
                continue
            share_dir = Path(prefix) / "share"
            if share_dir.is_dir():
                for pkg_dir in share_dir.iterdir():
                    if (
                        pkg_dir.name.endswith("_interfaces")
                        and pkg_dir.is_dir()
                        and pkg_dir.name not in _STANDARD
                    ):
                        return pkg_dir.name

        logger.warning(
            "_infer_package_name: could not infer package, "
            "falling back to 'vyra_module_template_interfaces'."
        )
        return "vyra_module_template_interfaces"


# ---------------------------------------------------------------------------
# Specialised subclasses
# ---------------------------------------------------------------------------

class Ros2Resolver(SchemaResolver):
    """
    Resolver specialised for ROS2 interfaces (.srv/.msg/.action).

    ``load_interface`` always uses ``SchemaType.ROS2`` regardless of the
    ``interface_type`` argument so callers can pass either value safely.
    """

    def _do_load(self, name: str, interface_type: SchemaType) -> Optional[Any]:
        return self._load_ros2(name)


class ProtoResolver(SchemaResolver):
    """
    Resolver specialised for Protocol Buffer interfaces (*_pb2.py).

    ``load_interface`` always uses ``SchemaType.PROTOBUF`` regardless of the
    ``interface_type`` argument.
    """

    def _do_load(self, name: str, interface_type: SchemaType) -> Optional[Any]:
        return self._load_protobuf(name)
