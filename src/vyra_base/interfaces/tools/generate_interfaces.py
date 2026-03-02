#!/usr/bin/env python3
"""
VYRA Interface Generator
Generates ROS2 interface files (.msg, .srv, .action) from JSON metadata.

Usage:
    python generate_interfaces.py                  # Generate all interfaces
    python generate_interfaces.py --validate       # Validate only, no generation
    python generate_interfaces.py --cleanup        # Mark deprecated files
"""

import argparse
import json
import logging
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s: %(message)s"
)
logger = logging.getLogger(__name__)

def _load_type_definitions() -> dict:
    """
    Load the VYRA type definitions from ``assets/schemas/type_definitions.json``.

    This file is the *single source of truth* for all type mappings used across
    the codebase.  The function resolves the path relative to this file so it
    works both in the source tree and after ``pip install``.

    Returns a fallback hardcoded dict when the JSON file cannot be found or
    parsed, ensuring the generator remains functional in minimal environments.
    """
    _schema_dir = Path(__file__).parent.parent.parent / "assets" / "schemas"
    _path = _schema_dir / "type_definitions.json"
    if _path.exists():
        try:
            with open(_path, "r", encoding="utf-8") as _fh:
                return json.load(_fh)
        except Exception as _exc:  # pragma: no cover
            logger.warning(
                "Could not load type_definitions.json from %s: %s "
                "– falling back to hardcoded defaults.",
                _path,
                _exc,
            )
    else:
        logger.warning(
            "type_definitions.json not found at %s "
            "– falling back to hardcoded defaults.",
            _path,
        )
    # ── Fallback ─────────────────────────────────────────────────────────────
    return {
        "ros2_map": {
            "bool": "bool", "boolean": "bool", "int": "int32",
            "int8": "int8", "int16": "int16", "int32": "int32", "int64": "int64",
            "uint": "uint32", "uint8": "uint8", "uint16": "uint16",
            "uint32": "uint32", "uint64": "uint64",
            "float": "float32", "float32": "float32", "float64": "float64",
            "double": "float64", "str": "string", "string": "string",
            "datetime": "builtin_interfaces/Time", "time": "builtin_interfaces/Time",
            "duration": "builtin_interfaces/Duration",
            "string[]": "string", "int[]": "int32", "int32[]": "int32",
            "float[]": "float32", "bool[]": "bool",
        },
        "proto_map": {
            "bool": "bool", "boolean": "bool",
            "int8": "int32", "uint8": "uint32", "int16": "int32", "uint16": "uint32",
            "int32": "int32", "int64": "int64", "uint32": "uint32", "uint64": "uint64",
            "float": "float", "float32": "float", "float64": "double",
            "double": "double", "str": "string", "string": "string",
            "datetime": "int64", "time": "int64", "duration": "int64",
        },
        "valid_ros2_native_types": {
            "types": [
                "bool", "byte", "char",
                "int8", "uint8", "int16", "uint16", "int32", "uint32",
                "int64", "uint64", "float32", "float64", "string",
            ]
        },
    }


_TYPE_DEFS: Dict[str, Any] = _load_type_definitions()

# Type mapping: JSON/Python type name → ROS2 IDL type.
# Source of truth: assets/schemas/type_definitions.json (ros2_map).
# Keys beginning with '_' are metadata comments and are excluded.
TYPE_MAP: Dict[str, str] = {
    k: v
    for k, v in _TYPE_DEFS.get("ros2_map", {}).items()
    if not k.startswith("_")
}

# Proto type map: JSON/Python type name → Protobuf scalar type.
# Source of truth: assets/schemas/type_definitions.json (proto_map).
_PROTO_MAP: Dict[str, str] = {
    k: v
    for k, v in _TYPE_DEFS.get("proto_map", {}).items()
    if not k.startswith("_")
}

# Valid ROS2 IDL primitive types accepted verbatim (no mapping needed).
# Source of truth: assets/schemas/type_definitions.json (valid_ros2_native_types.types).
VALID_ROS2_TYPES: Set[str] = set(
    _TYPE_DEFS.get("valid_ros2_native_types", {}).get("types", []) or []
)


def to_ros2_class_name(functionname: str) -> str:
    """
    Convert a function name (snake_case or camelCase) to a valid ROS2 interface class name.
    
    ROS2 interface names must match '^[A-Z][A-Za-z0-9]*$' (no underscores).
    
    Examples:
        set_parameter     → SetParameter
        get_interface_list → GetInterfaceList
        updateParamEvent  → UpdateParamEvent
        initialize        → Initialize
    """
    # Split on underscores and capitalise each part
    parts = functionname.split("_")
    # For each part, capitalise first letter and keep the rest as-is
    camel = "".join(part[0].upper() + part[1:] for part in parts if part)
    return f"VBASE{camel}"


def filetype_to_interface_name(filetype_entry: str) -> str:
    """
    Extract and validate the interface class name from a filetype entry.

    The filetype entry is expected to be a filename like 'VBASESetParam.proto'.
    The stem (without extension) is used as the interface name after verifying
    it is CamelCase ('^[A-Z][A-Za-z0-9]*$'). If it contains underscores or
    starts with a lowercase letter it is converted to CamelCase automatically.

    Examples:
        'VBASESetParam.proto'       → 'VBASESetParam'   (already valid)
        'vbase_set_param.proto'     → 'VbaseSetParam'   (converted + warning)
        'VBASEGetInterfaceList.proto' → 'VBASEGetInterfaceList'
    """
    import re

    stem = Path(filetype_entry).stem  # strip any extension

    # Check for valid CamelCase: starts with uppercase, only alphanumeric
    camel_pattern = re.compile(r'^[A-Z][A-Za-z0-9]*$')
    if camel_pattern.match(stem):
        return stem

    # Convert: split on underscores / non-alphanumeric, capitalise each segment
    parts = re.split(r'[_\-\s]+', stem)
    converted = "".join(part[0].upper() + part[1:] for part in parts if part)
    logger.warning(
        f"filetype '{stem}' is not valid CamelCase – auto-converted to '{converted}'"
    )
    return converted


class InterfaceGenerator:
    """Generates ROS2 interfaces from JSON metadata."""

    def __init__(self, interfaces_path: Path):
        self.interfaces_path = interfaces_path
        self.config_path = interfaces_path / "config"
        self.msg_path = interfaces_path / "msg"
        self.srv_path = interfaces_path / "srv"
        self.action_path = interfaces_path / "action"

        self.autogenerated_message = "Generated by Vyra Copy Script"
        # .proto files are stored alongside their ROS2 counterparts (msg/srv/action)
        
        # Track referenced files
        self.referenced_files: Set[str] = set()
        self.generated_messages: Set[str] = set()
        
    def ensure_directories(self):
        """Create output directories if they don't exist."""
        for path in [self.config_path, self.msg_path, self.srv_path, self.action_path]:
            # Guard against stale non-directory entries (e.g. empty placeholder
            # files or broken symlinks that cause FileExistsError with exist_ok=True)
            if path.exists() and not path.is_dir():
                path.unlink()
            path.mkdir(parents=True, exist_ok=True)
        
    def map_type(self, datatype: str) -> Tuple[str, bool]:
        """
        Map Python/JSON type to ROS2 type.
        
        Args:
            datatype: Input type string
            
        Returns:
            Tuple of (ros2_type, is_array)
        """
        is_array = datatype.endswith("[]")
        base_type = datatype.rstrip("[]")
        
        # Check if it's in the type map
        if datatype in TYPE_MAP:
            return TYPE_MAP[datatype], is_array
        if base_type in TYPE_MAP:
            return TYPE_MAP[base_type], is_array
            
        # Check if it's already a valid ROS2 type
        if base_type in VALID_ROS2_TYPES:
            return base_type, is_array
            
        # Check if it's a ROS2 message type (contains /)
        if "/" in base_type:
            return base_type, is_array
            
        # Unknown type
        raise ValueError(f"Unknown type: {datatype}")
        
    def validate_type(self, datatype: str) -> bool:
        """Validate that a type can be mapped to ROS2."""
        try:
            self.map_type(datatype)
            return True
        except ValueError:
            return False
            
    def generate_field(self, field: Dict[str, Any], is_return: bool = False) -> str:
        """Generate a single field definition."""
        name = field.get("name") or field.get("displayname", "").lower().replace(" ", "_")
        datatype = field.get("datatype", "string")
        
        if not name:
            raise ValueError(f"Field missing name: {field}")
            
        # Map type
        try:
            ros2_type, is_array = self.map_type(datatype)
        except ValueError as e:
            logger.error(f"Type validation failed for field '{name}': {e}")
            raise
            
        # Build field string
        if is_array:
            field_str = f"{ros2_type}[] {name}"
        else:
            field_str = f"{ros2_type} {name}"
            
        # Add comment with description
        description = field.get("description", "")
        if description:
            # Escape description for comment
            description = description.replace("\n", " ").strip()
            field_str += f"  # {description}"
            
        return field_str
        
    def generate_msg(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .msg file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        # For message (formerly message), only 'returns' are used as message fields
        returns = metadata.get("returns", [])
        
        if not returns:
            logger.warning(f"Message '{functionname}' has no returns fields")
            return None
            
        lines = [
            f"# {self.autogenerated_message}",
            f"# {metadata.get('displayname', functionname)}",
            f"# {metadata.get('description', '')}",
            f"# Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
        ]
        
        # Generate fields
        for field in returns:
            try:
                lines.append(self.generate_field(field))
            except Exception as e:
                logger.error(f"Failed to generate field for {functionname}: {e}")
                return None
                
        return "\n".join(lines) + "\n"
        
    def generate_proto_msg(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .proto message file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        returns = metadata.get("returns", [])
        if not returns:
            logger.warning(f"Message '{functionname}' has no returns fields")
            return None
            
        message_name = to_ros2_class_name(functionname)
        
        lines = [
            'syntax = "proto3";',
            "",
            f"// {self.autogenerated_message}",
            f"// {metadata.get('displayname', functionname)}",
            f"// {metadata.get('description', '')}",
            "// Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
            f"message {message_name} {{",
        ]
        
        # Generate fields with proto3 field numbers
        for idx, field in enumerate(returns, start=1):
            name = field.get("name") or field.get("displayname", "").lower().replace(" ", "_")
            datatype = field.get("datatype", "string")
            description = field.get("description", "")
            
            # Map to proto types
            proto_type = self._map_to_proto_type(datatype)
            is_repeated = datatype.endswith("[]")
            
            if description:
                lines.append(f"  // {description}")
                
            if is_repeated:
                lines.append(f"  repeated {proto_type} {name} = {idx};")
            else:
                lines.append(f"  {proto_type} {name} = {idx};")
                
        lines.append("}")
        lines.append("")
        
        return "\n".join(lines)
        
    def generate_proto_srv(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .proto service file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        params = metadata.get("params", [])
        returns = metadata.get("returns", [])
        
        base_name = to_ros2_class_name(functionname)
        request_name = f"{base_name}Request"
        response_name = f"{base_name}Response"
        
        lines = [
            'syntax = "proto3";',
            "",
            f"// {self.autogenerated_message}",
            f"// {metadata.get('displayname', functionname)}",
            f"// {metadata.get('description', '')}",
            "// Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
            f"// Request message",
            f"message {request_name} {{",
        ]
        
        # Generate request fields
        if params:
            for idx, param in enumerate(params, start=1):
                name = param.get("name") or param.get("displayname", "").lower().replace(" ", "_")
                datatype = param.get("datatype", "string")
                description = param.get("description", "")
                
                proto_type = self._map_to_proto_type(datatype)
                is_repeated = datatype.endswith("[]")
                
                if description:
                    lines.append(f"  // {description}")
                    
                if is_repeated:
                    lines.append(f"  repeated {proto_type} {name} = {idx};")
                else:
                    lines.append(f"  {proto_type} {name} = {idx};")
        else:
            lines.append("  // No request parameters")
            
        lines.extend([
            "}",
            "",
            f"// Response message",
            f"message {response_name} {{",
        ])
        
        # Generate response fields
        if returns:
            for idx, ret in enumerate(returns, start=1):
                name = ret.get("name") or ret.get("displayname", "").lower().replace(" ", "_")
                datatype = ret.get("datatype", "string")
                description = ret.get("description", "")
                
                proto_type = self._map_to_proto_type(datatype)
                is_repeated = datatype.endswith("[]")
                
                if description:
                    lines.append(f"  // {description}")
                    
                if is_repeated:
                    lines.append(f"  repeated {proto_type} {name} = {idx};")
                else:
                    lines.append(f"  {proto_type} {name} = {idx};")
        else:
            lines.append("  // No response fields")
            
        lines.append("}")
        lines.append("")
        
        return "\n".join(lines)
        
    def generate_combined_proto(self, metadata_entries: List[Dict[str, Any]], proto_filename: str) -> Optional[str]:
        """
        Generate a combined .proto file containing message pairs for multiple services.

        When several service/action/message entries in a metadata file share the
        same ``filetype`` proto filename (e.g. ``VBASEVolatile.proto``), they are
        merged into one proto file.  Each entry contributes one Request + one
        Response message whose names are derived from their ``functionname``.

        Only "service" entries are supported here.  Message and action entries
        that share a proto name are handled individually (edge case; emit a
        warning).

        Args:
            metadata_entries: List of metadata dicts that all reference the same proto file.
            proto_filename: The shared .proto filename (e.g. ``VBASEVolatile.proto``).

        Returns:
            Combined proto file content as a string, or None on failure.
        """
        if not metadata_entries:
            return None

        lines = [
            'syntax = "proto3";',
            "",
            f"// {self.autogenerated_message}",
            f"// Combined proto: {proto_filename}",
            "// Contains message pairs for multiple related services.",
            "// Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
        ]

        for metadata in metadata_entries:
            functionname = metadata.get("functionname")
            if not functionname:
                continue

            interface_type = metadata.get("type", "service")
            if interface_type != "service":
                logger.warning(
                    f"generate_combined_proto: non-service entry '{functionname}' "
                    f"in shared proto '{proto_filename}' – skipping"
                )
                continue

            base_name = to_ros2_class_name(functionname)
            request_name = f"{base_name}Request"
            response_name = f"{base_name}Response"
            params = metadata.get("params", [])
            returns = metadata.get("returns", [])

            lines.append(f"// {metadata.get('displayname', functionname)}: {metadata.get('description', '')}")
            lines.append(f"message {request_name} {{")
            if params:
                for idx, param in enumerate(params, start=1):
                    name = param.get("name") or param.get("displayname", "").lower().replace(" ", "_")
                    datatype = param.get("datatype", "string")
                    description = param.get("description", "")
                    proto_type = self._map_to_proto_type(datatype)
                    is_repeated = datatype.endswith("[]")
                    if description:
                        lines.append(f"  // {description}")
                    if is_repeated:
                        lines.append(f"  repeated {proto_type} {name} = {idx};")
                    else:
                        lines.append(f"  {proto_type} {name} = {idx};")
            else:
                lines.append("  // No request parameters")
            lines.extend(["}", "", f"message {response_name} {{"])
            if returns:
                for idx, ret in enumerate(returns, start=1):
                    name = ret.get("name") or ret.get("displayname", "").lower().replace(" ", "_")
                    datatype = ret.get("datatype", "string")
                    description = ret.get("description", "")
                    proto_type = self._map_to_proto_type(datatype)
                    is_repeated = datatype.endswith("[]")
                    if description:
                        lines.append(f"  // {description}")
                    if is_repeated:
                        lines.append(f"  repeated {proto_type} {name} = {idx};")
                    else:
                        lines.append(f"  {proto_type} {name} = {idx};")
            else:
                lines.append("  // No response fields")
            lines.extend(["}", ""])

        return "\n".join(lines)

    def generate_proto_action(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .proto action file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        params = metadata.get("params", [])
        returns = metadata.get("returns", [])
        feedback = metadata.get("feedback", [])
        
        base_name = to_ros2_class_name(functionname)
        goal_name = f"{base_name}Goal"
        result_name = f"{base_name}Result"
        feedback_name = f"{base_name}Feedback"
        
        lines = [
            'syntax = "proto3";',
            "",
            f"// {self.autogenerated_message}",
            f"// {metadata.get('displayname', functionname)}",
            f"// {metadata.get('description', '')}",
            "// Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
            f"// Goal message",
            f"message {goal_name} {{",
        ]
        
        # Generate goal fields
        if params:
            for idx, param in enumerate(params, start=1):
                name = param.get("name") or param.get("displayname", "").lower().replace(" ", "_")
                datatype = param.get("datatype", "string")
                description = param.get("description", "")
                
                proto_type = self._map_to_proto_type(datatype)
                is_repeated = datatype.endswith("[]")
                
                if description:
                    lines.append(f"  // {description}")
                    
                if is_repeated:
                    lines.append(f"  repeated {proto_type} {name} = {idx};")
                else:
                    lines.append(f"  {proto_type} {name} = {idx};")
        else:
            lines.append("  // No goal parameters")
            
        lines.extend([
            "}",
            "",
            f"// Result message",
            f"message {result_name} {{",
        ])
        
        # Generate result fields
        if returns:
            for idx, ret in enumerate(returns, start=1):
                name = ret.get("name") or ret.get("displayname", "").lower().replace(" ", "_")
                datatype = ret.get("datatype", "string")
                description = ret.get("description", "")
                
                proto_type = self._map_to_proto_type(datatype)
                is_repeated = datatype.endswith("[]")
                
                if description:
                    lines.append(f"  // {description}")
                    
                if is_repeated:
                    lines.append(f"  repeated {proto_type} {name} = {idx};")
                else:
                    lines.append(f"  {proto_type} {name} = {idx};")
        else:
            lines.append("  // No result fields")
            
        lines.extend([
            "}",
            "",
            f"// Feedback message",
            f"message {feedback_name} {{",
        ])
        
        # Generate feedback fields
        if feedback:
            for idx, fb in enumerate(feedback, start=1):
                name = fb.get("name") or fb.get("displayname", "").lower().replace(" ", "_")
                datatype = fb.get("datatype", "string")
                description = fb.get("description", "")
                
                proto_type = self._map_to_proto_type(datatype)
                is_repeated = datatype.endswith("[]")
                
                if description:
                    lines.append(f"  // {description}")
                    
                if is_repeated:
                    lines.append(f"  repeated {proto_type} {name} = {idx};")
                else:
                    lines.append(f"  {proto_type} {name} = {idx};")
        else:
            lines.append("  // No feedback fields")
            
        lines.append("}")
        lines.append("")
        
        return "\n".join(lines)
        
    def _map_to_proto_type(self, datatype: str) -> str:
        """
        Map a JSON/Python type name to its Protobuf scalar type.

        Uses the ``proto_map`` from ``assets/schemas/type_definitions.json``
        (loaded at module level as :data:`_PROTO_MAP`) so the mapping stays in
        sync with the rest of the type system without duplicating it here.
        """
        base_type = datatype.rstrip("[]")

        if base_type in _PROTO_MAP:
            return _PROTO_MAP[base_type]

        # Complex types like builtin_interfaces/Time → int64 (Unix ns)
        if "/" in base_type:
            return "int64"  # Simplified for cross-transport compatibility

        return "string"
        
    def generate_srv(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .srv file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        params = metadata.get("params", [])
        returns = metadata.get("returns", [])
        
        lines = [
            f"# {self.autogenerated_message}",
            f"# {metadata.get('displayname', functionname)}",
            f"# {metadata.get('description', '')}",
            f"# Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
            "# Request",
        ]
        
        # Generate request fields
        if params:
            for param in params:
                try:
                    lines.append(self.generate_field(param))
                except Exception as e:
                    logger.error(f"Failed to generate param for {functionname}: {e}")
                    return None
        else:
            lines.append("# No request parameters")
            
        # Separator
        lines.extend(["", "---", "", "# Response"])
        
        # Generate response fields
        if returns:
            for ret in returns:
                try:
                    lines.append(self.generate_field(ret, is_return=True))
                except Exception as e:
                    logger.error(f"Failed to generate return for {functionname}: {e}")
                    return None
        else:
            lines.append("# No response fields")
            
        return "\n".join(lines) + "\n"
        
    def generate_action(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .action file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        # Actions have goal (params), result (returns), and feedback
        params = metadata.get("params", [])
        returns = metadata.get("returns", [])
        feedback = metadata.get("feedback", [])
        
        lines = [
            f"# {self.autogenerated_message}",
            f"# {metadata.get('displayname', functionname)}",
            f"# {metadata.get('description', '')}",
            f"# Generated from JSON metadata - DO NOT EDIT MANUALLY",
            "",
            "# Goal",
        ]
        
        # Generate goal fields
        if params:
            for param in params:
                try:
                    lines.append(self.generate_field(param))
                except Exception as e:
                    logger.error(f"Failed to generate goal param for {functionname}: {e}")
                    return None
        else:
            lines.append("# No goal parameters")
            
        # Separator
        lines.extend(["", "---", "", "# Result"])
        
        # Generate result fields
        if returns:
            for ret in returns:
                try:
                    lines.append(self.generate_field(ret, is_return=True))
                except Exception as e:
                    logger.error(f"Failed to generate result for {functionname}: {e}")
                    return None
        else:
            lines.append("# No result fields")
            
        # Separator
        lines.extend(["", "---", "", "# Feedback"])
        
        # Generate feedback fields
        if feedback:
            for fb in feedback:
                try:
                    lines.append(self.generate_field(fb))
                except Exception as e:
                    logger.error(f"Failed to generate feedback for {functionname}: {e}")
                    return None
        else:
            lines.append("# No feedback fields")
            
        return "\n".join(lines) + "\n"
        
    def process_metadata_file(self, json_path: Path, dry_run: bool = False) -> Tuple[int, int]:
        """
        Process a single JSON metadata file.
        
        Returns:
            Tuple of (success_count, error_count)
        """
        logger.info(f"Processing {json_path.name}...")
        
        try:
            with open(json_path) as f:
                metadata_list = json.load(f)
        except Exception as e:
            logger.error(f"Failed to load {json_path}: {e}")
            return 0, 1
            
        if not isinstance(metadata_list, list):
            logger.error(f"{json_path} does not contain a list")
            return 0, 1
            
        success_count = 0
        error_count = 0

        # Pre-scan: identify proto filenames that are shared by more than one entry.
        # Shared protos will be generated as combined files after the main loop.
        _proto_ref_count: Dict[str, int] = {}
        for _m in metadata_list:
            for _ft in _m.get("filetype", []):
                if _ft.endswith(".proto"):
                    _proto_ref_count[_ft] = _proto_ref_count.get(_ft, 0) + 1

        _shared_protos: Set[str] = {k for k, v in _proto_ref_count.items() if v > 1}
        # Collect entries for combined proto generation: proto_filename -> [metadata, ...]
        _combined_groups: Dict[str, List[Dict[str, Any]]] = {}

        for metadata in metadata_list:
            interface_type = metadata.get("type")
            functionname = metadata.get("functionname")
            
            if not interface_type or not functionname:
                logger.warning(f"Skipping incomplete metadata: {metadata}")
                error_count += 1
                continue
                
            # Track referenced proto files
            filetypes = metadata.get("filetype", [])
            for ft in filetypes:
                self.referenced_files.add(ft)

            # Derive the interface base name from the filetype list.
            # The filetype entry (e.g. 'VBASESetParam.proto') carries the canonical
            # name. functionname is only the callback identifier used in module code.
            proto_filetypes = [ft for ft in filetypes if ft.endswith(".proto")]
            if proto_filetypes:
                interface_name = filetype_to_interface_name(proto_filetypes[0])
            elif filetypes:
                interface_name = filetype_to_interface_name(filetypes[0])
            else:
                # Fallback: derive from functionname as before
                logger.warning(
                    f"No filetype entry for '{functionname}' – deriving name from functionname"
                )
                interface_name = to_ros2_class_name(functionname)

            # Determine if this entry's proto is shared (combined output later)
            is_shared_proto = bool(proto_filetypes and proto_filetypes[0] in _shared_protos)
            if is_shared_proto:
                # Collect into combined group; ROS2/.srv/.msg/.action still generated below
                shared_key = proto_filetypes[0]
                _combined_groups.setdefault(shared_key, []).append(metadata)
                # For shared proto entries, derive the ROS2 interface filename from
                # the functionname (each function gets its own .srv file), not from
                # the shared filetype (which would cause all to overwrite one file).
                interface_name = to_ros2_class_name(functionname)

            # Generate interface based on type
            try:
                # Check if proto files should be generated (only for non-shared protos here)
                generate_proto = bool(proto_filetypes) and not is_shared_proto

                if interface_type == "message":
                    content = self.generate_msg(metadata)
                    if content:
                        filename = f"{interface_name}.msg"
                        output_path = self.msg_path / filename

                        if not dry_run:
                            with open(output_path, "w") as f:
                                f.write(content)
                            logger.info(f"  ✓ Generated {filename}")
                        else:
                            logger.info(f"  [DRY RUN] Would generate {filename}")

                        self.generated_messages.add(filename)
                        success_count += 1

                    # Generate .proto if requested (non-shared only)
                    if generate_proto:
                        proto_content = self.generate_proto_msg(metadata)
                        if proto_content:
                            proto_filename = proto_filetypes[0]  # use exact name from filetype
                            proto_output_path = self.msg_path / proto_filename

                            if not dry_run:
                                with open(proto_output_path, "w") as f:
                                    f.write(proto_content)
                                logger.info(f"  ✓ Generated {proto_filename}")
                            else:
                                logger.info(f"  [DRY RUN] Would generate {proto_filename}")

                            self.generated_messages.add(proto_filename)

                elif interface_type == "service":
                    content = self.generate_srv(metadata)
                    if content:
                        filename = f"{interface_name}.srv"
                        output_path = self.srv_path / filename

                        if not dry_run:
                            with open(output_path, "w") as f:
                                f.write(content)
                            logger.info(f"  ✓ Generated {filename}")
                        else:
                            logger.info(f"  [DRY RUN] Would generate {filename}")

                        self.generated_messages.add(filename)
                        success_count += 1

                    # Generate .proto if requested (non-shared only)
                    if generate_proto:
                        proto_content = self.generate_proto_srv(metadata)
                        if proto_content:
                            proto_filename = proto_filetypes[0]  # use exact name from filetype
                            proto_output_path = self.srv_path / proto_filename

                            if not dry_run:
                                with open(proto_output_path, "w") as f:
                                    f.write(proto_content)
                                logger.info(f"  ✓ Generated {proto_filename}")
                            else:
                                logger.info(f"  [DRY RUN] Would generate {proto_filename}")

                            self.generated_messages.add(proto_filename)

                elif interface_type == "action":
                    content = self.generate_action(metadata)
                    if content:
                        filename = f"{interface_name}.action"
                        output_path = self.action_path / filename

                        if not dry_run:
                            with open(output_path, "w") as f:
                                f.write(content)
                            logger.info(f"  ✓ Generated {filename}")
                        else:
                            logger.info(f"  [DRY RUN] Would generate {filename}")

                        self.generated_messages.add(filename)
                        success_count += 1

                    # Generate .proto if requested (non-shared only)
                    if generate_proto:
                        proto_content = self.generate_proto_action(metadata)
                        if proto_content:
                            proto_filename = proto_filetypes[0]  # use exact name from filetype
                            proto_output_path = self.action_path / proto_filename

                            if not dry_run:
                                with open(proto_output_path, "w") as f:
                                    f.write(proto_content)
                                logger.info(f"  ✓ Generated {proto_filename}")
                            else:
                                logger.info(f"  [DRY RUN] Would generate {proto_filename}")

                            self.generated_messages.add(proto_filename)
                else:
                    logger.warning(f"Unknown interface type '{interface_type}' for {functionname}")
                    error_count += 1
                    
            except Exception as e:
                logger.error(f"Failed to generate interface for {functionname}: {e}")
                error_count += 1

        # Generate combined proto files for shared proto groups
        for proto_fname, entries in _combined_groups.items():
            try:
                combined_content = self.generate_combined_proto(entries, proto_fname)
                if combined_content:
                    # Determine output path: assume service type (srv_path) for now
                    # All entries in a combined group should have the same interface_type
                    entry_types = {e.get("type") for e in entries}
                    if entry_types == {"service"}:
                        combined_out = self.srv_path / proto_fname
                    elif entry_types == {"message"}:
                        combined_out = self.msg_path / proto_fname
                    else:
                        combined_out = self.srv_path / proto_fname
                        logger.warning(
                            f"Mixed types in combined proto group '{proto_fname}': {entry_types}. "
                            f"Writing to srv_path."
                        )

                    if not dry_run:
                        with open(combined_out, "w") as f:
                            f.write(combined_content)
                        logger.info(f"  ✓ Generated combined {proto_fname} ({len(entries)} entries)")
                    else:
                        logger.info(f"  [DRY RUN] Would generate combined {proto_fname}")

                    self.generated_messages.add(proto_fname)
            except Exception as e:
                logger.error(f"Failed to generate combined proto '{proto_fname}': {e}")
                error_count += 1

        return success_count, error_count
        
    def mark_deprecated_files(self, dry_run: bool = False):
        """Mark proto files that are not referenced in any metadata as DEPRECATED."""
        # .proto files live alongside their ROS2 counterparts in msg/, srv/, action/
        search_paths = [self.msg_path, self.srv_path, self.action_path]
        proto_files = []
        for p in search_paths:
            if p.exists():
                proto_files.extend(p.glob("*.proto"))
        deprecated_count = 0
        
        for proto_file in proto_files:
            # Skip already deprecated files
            if ".DEPRECATED" in proto_file.name:
                continue
                
            if proto_file.name not in self.referenced_files:
                new_name = proto_file.parent / f"{proto_file.name}.DEPRECATED"
                
                if not dry_run:
                    proto_file.rename(new_name)
                    logger.info(f"  ✓ Marked as deprecated: {proto_file.name}")
                else:
                    logger.info(f"  [DRY RUN] Would deprecate: {proto_file.name}")
                    
                deprecated_count += 1
                
        if deprecated_count == 0:
            logger.info("No files to deprecate")
        else:
            logger.info(f"Deprecated {deprecated_count} unreferenced proto files")
            
    def run(self, validate_only: bool = False, cleanup: bool = False):
        """Run the interface generator."""
        if not validate_only:
            self.ensure_directories()
            
        # Find all JSON metadata files
        json_files = list(self.config_path.glob("*.json"))
        
        if not json_files:
            logger.error(f"No JSON files found in {self.config_path}")
            return 1
            
        logger.info(f"Found {len(json_files)} metadata files")
        
        total_success = 0
        total_errors = 0
        
        for json_file in json_files:
            success, errors = self.process_metadata_file(json_file, dry_run=validate_only)
            total_success += success
            total_errors += errors
            
        # Summary
        logger.info("\n" + "=" * 60)
        if validate_only:
            logger.info("VALIDATION COMPLETE")
        else:
            logger.info("GENERATION COMPLETE")
        logger.info(f"Successfully processed: {total_success}")
        logger.info(f"Errors: {total_errors}")
        logger.info(f"Referenced proto files: {len(self.referenced_files)}")
        logger.info("=" * 60)
        
        # Cleanup deprecated files if requested
        if cleanup:
            logger.info("\nMarking deprecated files...")
            self.mark_deprecated_files(dry_run=validate_only)
            
        return 0 if total_errors == 0 else 1


def main():
    parser = argparse.ArgumentParser(
        description="Generate ROS2 interfaces from VYRA JSON metadata"
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Validate metadata without generating files"
    )
    parser.add_argument(
        "--cleanup",
        action="store_true",
        help="Mark unreferenced proto files as DEPRECATED"
    )
    parser.add_argument(
        "--interfaces-path",
        type=Path,
        default=Path(__file__).parent.parent,
        help="Path to interfaces directory"
    )
    
    args = parser.parse_args()
    
    generator = InterfaceGenerator(args.interfaces_path)
    return generator.run(validate_only=args.validate, cleanup=args.cleanup)


if __name__ == "__main__":
    exit(main())
