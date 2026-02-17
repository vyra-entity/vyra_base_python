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

# Type mapping: Python/JSON types → ROS2 types
TYPE_MAP = {
    # Basic types
    "bool": "bool",
    "boolean": "bool",
    "int": "int32",
    "int8": "int8",
    "int16": "int16",
    "int32": "int32",
    "int64": "int64",
    "uint": "uint32",
    "uint8": "uint8",
    "uint16": "uint16",
    "uint32": "uint32",
    "uint64": "uint64",
    "float": "float32",
    "float32": "float32",
    "float64": "float64",
    "double": "float64",
    "str": "string",
    "string": "string",
    "datetime": "builtin_interfaces/Time",
    "time": "builtin_interfaces/Time",
    "duration": "builtin_interfaces/Duration",
    # Array indicators (will be processed separately)
    "string[]": "string",
    "int[]": "int32",
    "int32[]": "int32",
    "float[]": "float32",
    "bool[]": "bool",
}

# Valid ROS2 base types
VALID_ROS2_TYPES = {
    "bool", "byte", "char",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "float32", "float64",
    "string",
}


class InterfaceGenerator:
    """Generates ROS2 interfaces from JSON metadata."""

    def __init__(self, interfaces_path: Path):
        self.interfaces_path = interfaces_path
        self.config_path = interfaces_path / "config"
        self.msg_path = interfaces_path / "msg"
        self.srv_path = interfaces_path / "srv"
        self.action_path = interfaces_path / "action"
        self.proto_path = interfaces_path / "proto"
        
        # Track referenced files
        self.referenced_files: Set[str] = set()
        self.generated_messages: Set[str] = set()
        
    def ensure_directories(self):
        """Create output directories if they don't exist."""
        self.msg_path.mkdir(exist_ok=True)
        self.srv_path.mkdir(exist_ok=True)
        self.action_path.mkdir(exist_ok=True)
        
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
            
        # For publisher (formerly publisher), only 'returns' are used as message fields
        returns = metadata.get("returns", [])
        
        if not returns:
            logger.warning(f"Publisher '{functionname}' has no returns fields")
            return None
            
        lines = [
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
            logger.warning(f"Publisher '{functionname}' has no returns fields")
            return None
            
        message_name = f"VBASE{functionname[0].upper() + functionname[1:]}"
        
        lines = [
            'syntax = "proto3";',
            "",
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
        
        base_name = f"VBASE{functionname[0].upper() + functionname[1:]}"
        request_name = f"{base_name}Request"
        response_name = f"{base_name}Response"
        
        lines = [
            'syntax = "proto3";',
            "",
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
        
    def generate_proto_action(self, metadata: Dict[str, Any]) -> Optional[str]:
        """Generate .proto action file content from metadata."""
        functionname = metadata.get("functionname")
        if not functionname:
            logger.warning("Metadata missing functionname")
            return None
            
        params = metadata.get("params", [])
        returns = metadata.get("returns", [])
        feedback = metadata.get("feedback", [])
        
        base_name = f"VBASE{functionname[0].upper() + functionname[1:]}"
        goal_name = f"{base_name}Goal"
        result_name = f"{base_name}Result"
        feedback_name = f"{base_name}Feedback"
        
        lines = [
            'syntax = "proto3";',
            "",
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
        """Map Python/ROS2 type to Protobuf type."""
        # Remove array suffix for mapping
        base_type = datatype.rstrip("[]")
        
        proto_map = {
            "bool": "bool",
            "boolean": "bool",
            "int8": "int32",
            "uint8": "uint32",
            "int16": "int32",
            "uint16": "uint32",
            "int32": "int32",
            "uint32": "uint32",
            "int64": "int64",
            "uint64": "uint64",
            "float": "float",
            "float32": "float",
            "float64": "double",
            "double": "double",
            "str": "string",
            "string": "string",
            "datetime": "int64",  # Unix timestamp in nanoseconds
            "time": "int64",
            "duration": "int64",
        }
        
        if base_type in proto_map:
            return proto_map[base_type]
        
        # For complex types like builtin_interfaces/Time, use int64 as fallback
        if "/" in base_type:
            return "int64"  # Simplified for cross-transport compatibility
            
        # Default to string
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
                
            # Generate interface based on type
            try:
                # Check if proto files should be generated
                generate_proto = any(ft.endswith(".proto") for ft in filetypes)
                
                if interface_type == "publisher":
                    content = self.generate_msg(metadata)
                    if content:
                        # Generate VBASE<Name>.msg
                        filename = f"VBASE{functionname[0].upper() + functionname[1:]}.msg"
                        output_path = self.msg_path / filename
                        
                        if not dry_run:
                            with open(output_path, "w") as f:
                                f.write(content)
                            logger.info(f"  ✓ Generated {filename}")
                        else:
                            logger.info(f"  [DRY RUN] Would generate {filename}")
                            
                        self.generated_messages.add(filename)
                        success_count += 1
                    
                    # Generate .proto if requested
                    if generate_proto:
                        proto_content = self.generate_proto_msg(metadata)
                        if proto_content:
                            proto_filename = f"VBASE{functionname[0].upper() + functionname[1:]}.proto"
                            proto_output_path = self.msg_path / proto_filename
                            
                            if not dry_run:
                                with open(proto_output_path, "w") as f:
                                    f.write(proto_content)
                                logger.info(f"  ✓ Generated {proto_filename}")
                            else:
                                logger.info(f"  [DRY RUN] Would generate {proto_filename}")
                                
                            self.generated_messages.add(proto_filename)
                        
                elif interface_type == "server":
                    content = self.generate_srv(metadata)
                    if content:
                        filename = f"VBASE{functionname[0].upper() + functionname[1:]}.srv"
                        output_path = self.srv_path / filename
                        
                        if not dry_run:
                            with open(output_path, "w") as f:
                                f.write(content)
                            logger.info(f"  ✓ Generated {filename}")
                        else:
                            logger.info(f"  [DRY RUN] Would generate {filename}")
                            
                        self.generated_messages.add(filename)
                        success_count += 1
                    
                    # Generate .proto if requested
                    if generate_proto:
                        proto_content = self.generate_proto_srv(metadata)
                        if proto_content:
                            proto_filename = f"VBASE{functionname[0].upper() + functionname[1:]}.proto"
                            proto_output_path = self.srv_path / proto_filename
                            
                            if not dry_run:
                                with open(proto_output_path, "w") as f:
                                    f.write(proto_content)
                                logger.info(f"  ✓ Generated {proto_filename}")
                            else:
                                logger.info(f"  [DRY RUN] Would generate {proto_filename}")
                                
                            self.generated_messages.add(proto_filename)
                        
                elif interface_type == "actionServer":
                    content = self.generate_action(metadata)
                    if content:
                        filename = f"VBASE{functionname[0].upper() + functionname[1:]}.action"
                        output_path = self.action_path / filename
                        
                        if not dry_run:
                            with open(output_path, "w") as f:
                                f.write(content)
                            logger.info(f"  ✓ Generated {filename}")
                        else:
                            logger.info(f"  [DRY RUN] Would generate {filename}")
                            
                        self.generated_messages.add(filename)
                        success_count += 1
                    
                    # Generate .proto if requested
                    if generate_proto:
                        proto_content = self.generate_proto_action(metadata)
                        if proto_content:
                            proto_filename = f"VBASE{functionname[0].upper() + functionname[1:]}.proto"
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
                
        return success_count, error_count
        
    def mark_deprecated_files(self, dry_run: bool = False):
        """Mark proto files that are not referenced in any metadata as DEPRECATED."""
        if not self.proto_path.exists():
            logger.warning(f"Proto path {self.proto_path} does not exist")
            return
            
        proto_files = list(self.proto_path.glob("*.proto"))
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
