"""
Manifest Resolver

The ManifestResolver owns:
- Registration of filesystem paths that contain *.meta.json definitions
- Loading and caching of interface metadata from those config paths
- JSON Schema validation of metadata files against the bundled schema
- Change notification so the EndpointOrchestrator reacts to new paths

All paths are deduplicated on entry (add_manifest_paths is the single
addition point).  Validation uses self.manifest_schema_path which defaults
to vyra_base/assets/schemas/interface_config.json and can be overridden
per-instance (e.g. for custom schemas in tests or derived modules).
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


class ManifestResolver:
    """
    Thread-safe singleton resolver for interface manifest paths and metadata.

    Responsibilities:
    - Maintain a deduplicated list of base directories that contain a
      ``config/`` subdirectory with ``*.meta.json`` files.
    - Load and cache all interface metadata entries, keyed by functionname.
    - Validate raw JSON files against the bundled interface_config.json schema.
    - Emit an asyncio.Event change notification when paths change so that
      the EndpointOrchestrator can react asynchronously.

    Usage::

        resolver = ManifestResolver.get_instance()
        resolver.add_manifest_paths(["/workspace/src/my_module_interfaces"])
        metadata = resolver.load_interface_metadata()
    """

    _instance: Optional[ManifestResolver] = None
    _singleton_lock = threading.Lock()

    def __init__(self) -> None:
        if ManifestResolver._instance is not None:
            raise RuntimeError(
                "ManifestResolver is a singleton — use get_instance()."
            )

        self._paths: List[Path] = []
        self._paths_lock = threading.Lock()
        self._metadata_cache: Optional[Dict[str, Dict[str, Any]]] = None

        # JSON Schema path — may be overridden per instance before first use
        self.manifest_schema_path: Path = (
            Path(__file__).parent.parent  # vyra_base/
            / "assets"
            / "schemas"
            / "interface_config.json"
        )

        # asyncio event — created lazily when an event loop is available
        self._change_event: Optional[asyncio.Event] = None

        # Set the built-in vyra_base/interfaces path as default
        self._set_default_path()

        logger.debug("ManifestResolver initialised.")

    # -----------------------------------------------------------------------
    # Singleton
    # -----------------------------------------------------------------------

    @classmethod
    def get_instance(cls) -> ManifestResolver:
        """Return the process-wide singleton (thread-safe)."""
        if cls._instance is None:
            with cls._singleton_lock:
                if cls._instance is None:
                    cls._instance = ManifestResolver()
        return cls._instance

    # -----------------------------------------------------------------------
    # asyncio change event
    # -----------------------------------------------------------------------

    def set_event_loop(self) -> None:
        """
        Initialise the asyncio change event bound to the running loop.
        Call once from within the event loop before starting the orchestrator.
        """
        self._change_event = asyncio.Event()

    @property
    def change_event(self) -> Optional[asyncio.Event]:
        return self._change_event

    def _emit_change_event(self) -> None:
        """Signal a state change to the orchestrator."""
        if self._change_event is not None:
            try:
                self._change_event.set()
            except RuntimeError:
                pass  # event loop not yet running

    # -----------------------------------------------------------------------
    # Default path
    # -----------------------------------------------------------------------

    def _set_default_path(self) -> None:
        """Register the built-in vyra_base/interfaces directory."""
        # This file: vyra_base/com/manifest.py → parent.parent = vyra_base/
        vyra_base_dir = Path(__file__).parent.parent.resolve()
        default_path = vyra_base_dir / "interfaces"
        if default_path.exists():
            self._paths = [default_path]
            logger.info("ManifestResolver: default path → %s", default_path)
        else:
            logger.warning(
                "ManifestResolver: default interface path not found: %s",
                default_path,
            )
            self._paths = []

    # -----------------------------------------------------------------------
    # Path management (single public mutation point)
    # -----------------------------------------------------------------------

    def add_manifest_paths(self, paths: List[str | Path]) -> None:
        """
        Add one or more manifest base directories.

        Paths are resolved, validated, and silently deduplicated — adding a
        path that is already registered is a no-op (not an error).

        Args:
            paths: Iterable of directory paths that contain a ``config/``
                   subdirectory with ``*.meta.json`` files.

        Raises:
            ValueError: If *no* valid path could be added from the list.
        """
        with self._paths_lock:
            added: List[Path] = []
            for raw in paths:
                path = Path(raw).resolve()

                if not path.exists():
                    logger.warning(
                        "ManifestResolver.add_manifest_paths: "
                        "path does not exist: %s",
                        path,
                    )
                    continue
                if not path.is_dir():
                    logger.warning(
                        "ManifestResolver.add_manifest_paths: "
                        "path is not a directory: %s",
                        path,
                    )
                    continue
                if path in self._paths:
                    logger.debug(
                        "ManifestResolver.add_manifest_paths: "
                        "already registered, skipping: %s",
                        path,
                    )
                    continue

                self._paths.append(path)
                added.append(path)

            if not added and not self._paths:
                raise ValueError(
                    "ManifestResolver: no valid paths provided and no prior "
                    "paths registered."
                )

            if added:
                # Invalidate metadata cache so next call reloads from all paths
                self._metadata_cache = None
                for idx, p in enumerate(added, 1):
                    logger.info(
                        "ManifestResolver: added path [%d] %s", idx, p
                    )
                self._emit_change_event()

    # -----------------------------------------------------------------------
    # Path accessors
    # -----------------------------------------------------------------------

    def get_manifest_paths(self) -> List[Path]:
        """Return a snapshot of the registered base paths."""
        with self._paths_lock:
            return list(self._paths)

    def get_config_paths(self) -> List[Path]:
        """Return existing ``*/config`` subdirectories of registered paths."""
        with self._paths_lock:
            result: List[Path] = []
            for base in self._paths:
                cp = base / "config"
                if cp.is_dir():
                    result.append(cp)
            return result

    # -----------------------------------------------------------------------
    # Metadata loading
    # -----------------------------------------------------------------------

    def load_interface_metadata(self, reload: bool = False) -> Dict[str, Dict[str, Any]]:
        """
        Load all ``*.meta.json`` entries from registered config paths.

        Results are keyed by ``functionname``.  A cache is maintained and
        invalidated whenever paths change or ``reload=True`` is requested.

        Args:
            reload: Force re-scan even if cache is warm.

        Returns:
            Mapping of ``functionname`` → raw metadata dict.
        """
        if self._metadata_cache is not None and not reload:
            return self._metadata_cache

        metadata: Dict[str, Dict[str, Any]] = {}
        config_paths = self.get_config_paths()

        for config_path in config_paths:
            for json_file in config_path.glob("*.meta.json"):
                try:
                    with open(json_file, encoding="utf-8") as fh:
                        data = json.load(fh)
                except json.JSONDecodeError as exc:
                    logger.error(
                        "ManifestResolver: invalid JSON in %s: %s", json_file, exc
                    )
                    continue
                except OSError as exc:
                    logger.error(
                        "ManifestResolver: cannot read %s: %s", json_file, exc
                    )
                    continue

                entries = data if isinstance(data, list) else [data]
                for entry in entries:
                    fn = entry.get("functionname")
                    if not fn:
                        logger.warning(
                            "ManifestResolver: entry in %s missing "
                            "'functionname' — skipped.",
                            json_file,
                        )
                        continue
                    if fn in metadata:
                        logger.debug(
                            "ManifestResolver: overwriting metadata for '%s' "
                            "(from %s).",
                            fn,
                            json_file,
                        )
                    metadata[fn] = entry

        self._metadata_cache = metadata
        logger.info(
            "ManifestResolver: loaded %d interface definition(s) from "
            "%d config path(s).",
            len(metadata),
            len(config_paths),
        )
        return metadata

    # -----------------------------------------------------------------------
    # Validation
    # -----------------------------------------------------------------------

    def validate_metadata(
        self,
        meta_files: List[Path],
        schema_path: Optional[Path] = None,
    ) -> Tuple[List[Path], List[Tuple[Path, str]]]:
        """
        Validate a list of ``*.meta.json`` files against the JSON Schema.

        Uses ``self.manifest_schema_path`` by default; override with the
        optional ``schema_path`` argument for one-off checks.

        Args:
            meta_files:  Paths to ``*.meta.json`` files to validate.
            schema_path: Override schema path (default: ``self.manifest_schema_path``).

        Returns:
            ``(valid_files, errors)`` where ``errors`` is a list of
            ``(path, error_message)`` pairs for files that failed validation.
        """
        effective_schema = schema_path or self.manifest_schema_path

        try:
            import jsonschema  # optional dependency
        except ImportError:
            logger.warning(
                "ManifestResolver.validate_metadata: 'jsonschema' not "
                "installed — validation skipped."
            )
            return list(meta_files), []

        # Load schema
        try:
            with open(effective_schema, encoding="utf-8") as fh:
                schema = json.load(fh)
        except (OSError, json.JSONDecodeError) as exc:
            logger.error(
                "ManifestResolver.validate_metadata: cannot load schema "
                "%s: %s",
                effective_schema,
                exc,
            )
            return list(meta_files), []

        valid: List[Path] = []
        errors: List[Tuple[Path, str]] = []

        for meta_file in meta_files:
            try:
                with open(meta_file, encoding="utf-8") as fh:
                    data = json.load(fh)
            except (OSError, json.JSONDecodeError) as exc:
                errors.append((meta_file, str(exc)))
                logger.warning(
                    "ManifestResolver.validate_metadata: cannot read %s: %s",
                    meta_file,
                    exc,
                )
                continue

            entries = data if isinstance(data, list) else [data]
            file_valid = True
            for entry in entries:
                try:
                    jsonschema.validate(entry, schema)
                except jsonschema.ValidationError as exc:
                    errors.append((meta_file, exc.message))
                    logger.warning(
                        "ManifestResolver.validate_metadata: %s → %s",
                        meta_file.name,
                        exc.message,
                    )
                    file_valid = False
                    break

            if file_valid:
                valid.append(meta_file)

        logger.info(
            "ManifestResolver.validate_metadata: %d valid, %d invalid "
            "(schema: %s).",
            len(valid),
            len(errors),
            effective_schema.name,
        )
        return valid, errors

    # -----------------------------------------------------------------------
    # Misc
    # -----------------------------------------------------------------------

    def clear(self) -> None:
        """Reset to default path and invalidate metadata cache."""
        with self._paths_lock:
            self._metadata_cache = None
            self._set_default_path()
            logger.info("ManifestResolver: reset to default paths.")
            self._emit_change_event()

    def __repr__(self) -> str:
        with self._paths_lock:
            return (
                f"ManifestResolver("
                f"paths={len(self._paths)}, "
                f"cached={'yes' if self._metadata_cache is not None else 'no'}"
                f")"
            )


def get_manifest_resolver() -> ManifestResolver:
    """Convenience accessor for the process-wide ManifestResolver singleton."""
    return ManifestResolver.get_instance()
