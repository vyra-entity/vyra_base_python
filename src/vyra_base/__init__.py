import os
import shutil
from pathlib import Path
from typing import Any

import vyra_base
from vyra_base.com.converter import interface


def extract_interfaces(target_path: str | Path):
    """
    Copy VYRA interface config files and build files from the pip-installed library.

    Only copies JSON/YAML/XML metadata files from the config/ directory and the
    ROS2 build scaffolding files (package.xml, CMakeLists.template.txt).

    Interface files (.msg, .srv, .action, .proto) are NOT copied — they are
    generated on demand by InterfaceGenerator (interfaces/tools/generate_interfaces.py)
    from the config metadata.

    :param target_path: Path to the target interface package directory.
    :type target_path: str or pathlib.Path

    :return: None
    :rtype: None
    """
    package_path: Path = Path(vyra_base.__file__).parent / 'interfaces'
    source_path = Path(package_path)

    if isinstance(target_path, str):
        target_path = Path(target_path)

    print(f"Extracting VYRA interface configs from {source_path} to {target_path}")

    # Copy ROS2 build scaffolding files
    build_files = ['package.xml', 'CMakeLists.template.txt']
    for build_file in build_files:
        source_file = source_path / build_file
        if source_file.exists():
            shutil.copy2(source_file, target_path / build_file)
            print(f"Copied {build_file} to {target_path}")

    # Copy config metadata files (JSON / YAML / XML)
    config_path: Path = source_path / 'config'
    target_config: Path = target_path / 'config'

    if not target_config.exists():
        target_config.mkdir(parents=True, exist_ok=True)

    for ftype in ['json', 'yaml', 'xml']:
        for file in config_path.glob(f'*.{ftype}'):
            shutil.copy2(file, target_config)
            print(f"Copied {file.name} to {target_config}")
    
    # Copy all base interface files that are not described in config files
    # These will be used as complex types in other interface files
    interface_subs = ['msg', 'srv', 'action']
    for i_type in interface_subs:
        iface_path: Path = source_path / i_type
        t_subs_path: Path = target_path / i_type
        for file in iface_path.glob(f'*.{i_type}'):
            shutil.copy2(file, t_subs_path)
            print(f"Copied {file.name} to {t_subs_path}")
        
        for p_file in iface_path.glob(f'*.proto'):
            shutil.copy2(p_file, t_subs_path)
            print(f"Copied {p_file.name} to {t_subs_path}")

    print(f"VYRA interface config extraction to {target_path} successful.")


def get_reserved_list() -> dict[str, Any]:
    """
    Get the list of reserved usernames.

    :return: List of reserved usernames.
    :rtype: list[str]
    """

    vyra_base_path = Path(vyra_base.__file__).parent / "interfaces" / "config"
 
    reserved_file = vyra_base_path / "RESERVED.list"
    if not reserved_file.exists():
        raise FileNotFoundError(f"Reserved list file not found: {reserved_file}")
    
    reserved = {}
    with open(reserved_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line or '|' not in line:
                continue
            
            parts = [p.strip() for p in line.split('|')]
            if len(parts) >= 2:
                interface_name = parts[0]
                function_name = parts[1] if parts[1] != '-' else None
                config_file = parts[2] if len(parts) > 2 else 'unknown'
                reserved[interface_name] = {
                    'function_name': function_name,
                    'config_file': config_file
                }
    
    return reserved


def get_vyra_base_config_files() -> set[str]:
    """
    Return the set of config file names shipped with vyra_base.

    Used by setup_interfaces.py to distinguish module-specific config files
    (which must be validated against the RESERVED list) from files that were
    copied from vyra_base itself.

    :return: Set of file names (basename only) present in the vyra_base interfaces/config/ directory.
    :rtype: set[str]
    """
    config_path: Path = Path(vyra_base.__file__).parent / "interfaces" / "config"
    return {f.name for f in config_path.iterdir() if f.is_file()}


def validate_config_schema(
    config_files: list,
    schema_path: "Path | None" = None,
) -> "tuple[list[Path], list[tuple[Path, str]]]":
    """
    Validate interface config JSON files against the VYRA ``interface_config.json`` schema.

    Each file in *config_files* is loaded as JSON and validated against the
    bundled ``assets/schemas/interface_config.json`` schema (or a custom
    *schema_path*).  Files that are not valid JSON or fail schema validation
    are collected in the *invalid_files* return value so the caller can log
    warnings and exclude them from interface generation.

    Requires the ``jsonschema`` package (automatically installed as a
    ``vyra_base`` dependency).  If ``jsonschema`` is unexpectedly missing the
    function logs a warning and returns all files as valid so the build pipeline
    continues without interruption.

    Example usage in ``setup_interfaces.py``::

        import vyra_base
        valid, invalid = vyra_base.validate_config_schema(
            list(config_path.glob("*_meta.json"))
        )
        for bad_file, reason in invalid:
            logging.warning("Schema violation in %s: %s", bad_file.name, reason)

    :param config_files:  Iterable of :class:`pathlib.Path` objects (or path
                          strings) pointing to ``*_meta.json`` files to validate.
    :type config_files:   list[Path | str]

    :param schema_path:   Path to a custom JSON Schema file.  When ``None`` the
                          bundled schema at
                          ``vyra_base/assets/schemas/interface_config.json``
                          is used automatically.
    :type schema_path:    pathlib.Path | None

    :return: A two-element tuple ``(valid_files, invalid_files)`` where
             *valid_files* is the subset of *config_files* that passed
             validation and *invalid_files* is a list of
             ``(Path, reason: str)`` pairs for every file that failed.
    :rtype: tuple[list[Path], list[tuple[Path, str]]]
    """
    import json
    import logging as _logging
    import copy

    _log = _logging.getLogger(__name__)

    try:
        import jsonschema  # type: ignore[import-untyped]
    except ImportError:
        _log.warning(
            "validate_config_schema: 'jsonschema' package not installed – "
            "skipping JSON schema validation. "
            "Install with: pip install jsonschema"
        )
        return [Path(f) for f in config_files], []

    # Resolve schema path
    if schema_path is None:
        schema_path = (
            Path(vyra_base.__file__).parent / "assets" / "schemas" / "interface_config.json"
        )

    schema_path = Path(schema_path)
    if not schema_path.exists():
        _log.warning(
            "validate_config_schema: Schema file not found at %s – "
            "skipping JSON schema validation.",
            schema_path,
        )
        return [Path(f) for f in config_files], []

    with open(schema_path, "r", encoding="utf-8") as _fh:
        schema = json.load(_fh)

    # ── Inject datatype enum from type_definitions.json (single source of truth) ──
    _type_defs_path = schema_path.parent / "type_definitions.json"
    if _type_defs_path.exists():
        try:
            with open(_type_defs_path, "r", encoding="utf-8") as _fh:
                _type_defs = json.load(_fh)

            # Collect all base type names (exclude comment keys starting with '_')
            _base_types: list[str] = [
                k for k in _type_defs.get("ros2_map", {})
                if not k.startswith("_") and not k.endswith("[]")
            ]
            _base_types += [
                t for t in _type_defs.get("valid_ros2_native_types", {}).get("types", [])
                if t not in _base_types
            ]
            # Build the full enum: base types + their array variants
            _valid_types = sorted(
                set(_base_types) | {t + "[]" for t in _base_types}
            )

            # Deep-copy schema before mutating so the function is side-effect-free
            schema = copy.deepcopy(schema)
            _field_def = (
                schema
                .get("$defs", {})
                .get("FieldDefinition", {})
                .get("properties", {})
                .get("datatype", {})
            )
            if _field_def is not None and "anyOf" in _field_def:
                # Inject enum into the first anyOf branch (primitive types branch)
                _field_def["anyOf"][0]["enum"] = _valid_types
                _field_def["anyOf"][0].pop("$comment", None)
        except Exception as _tdexc:
            _log.warning(
                "validate_config_schema: Could not load type_definitions.json "
                "for enum injection: %s – datatype field will not be enum-validated.",
                _tdexc,
            )
    else:
        _log.warning(
            "validate_config_schema: type_definitions.json not found at %s – "
            "datatype field will not be enum-validated.",
            _type_defs_path,
        )

    valid: list[Path] = []
    invalid: list[tuple[Path, str]] = []

    for raw_path in config_files:
        config_file = Path(raw_path)
        # ── Parse JSON ────────────────────────────────────────────────────────
        try:
            with open(config_file, "r", encoding="utf-8") as _fh:
                data = json.load(_fh)
        except (json.JSONDecodeError, OSError) as exc:
            invalid.append((config_file, f"JSON parse error: {exc}"))
            continue

        # ── Validate against schema ───────────────────────────────────────────
        try:
            jsonschema.validate(instance=data, schema=schema)
            valid.append(config_file)
        except jsonschema.ValidationError as exc:
            # Build a concise location string from the JSON path
            path_str = (
                " -> ".join(str(p) for p in exc.absolute_path)
                if exc.absolute_path
                else "root"
            )
            invalid.append((config_file, f"[{path_str}] {exc.message}"))
        except jsonschema.SchemaError as exc:
            # The bundled schema itself is broken – do not block the pipeline
            _log.error(
                "validate_config_schema: The schema at %s is invalid: %s",
                schema_path,
                exc.message,
            )
            # Return all remaining files as valid so the build is not blocked
            valid.extend(
                Path(f)
                for f in config_files
                if Path(f) not in {p for p, _ in invalid} and Path(f) not in valid
            )
            return valid, invalid

    return valid, invalid