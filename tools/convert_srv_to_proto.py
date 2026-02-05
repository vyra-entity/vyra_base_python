#!/usr/bin/env python3
"""
Convert ROS2 .srv files to Protocol Buffers .proto files

This script converts all .srv files in vyra_base/interfaces/srv/
to equivalent .proto files in vyra_base/interfaces/redis/
"""

import re
from pathlib import Path

def parse_srv_file(srv_path: Path) -> dict:
    """Parse a ROS2 .srv file and extract request/response fields."""
    content = srv_path.read_text()
    
    # Split by ---
    parts = content.split('---')
    
    request_fields = []
    response_fields = []
    
    # Parse request (part before ---)
    if len(parts) > 0:
        for line in parts[0].strip().split('\n'):
            line = line.split('#')[0].strip()  # Remove comments
            if not line or line.startswith('#'):
                continue
            
            # Parse: type field_name [default_value]
            match = re.match(r'(\S+)\s+(\S+)', line)
            if match:
                field_type = match.group(1)
                field_name = match.group(2)
                request_fields.append((field_type, field_name))
    
    # Parse response (part after ---)
    if len(parts) > 1:
        for line in parts[1].strip().split('\n'):
            line = line.split('#')[0].strip()  # Remove comments
            if not line or line.startswith('#'):
                continue
            
            match = re.match(r'(\S+)\s+(\S+)', line)
            if match:
                field_type = match.group(1)
                field_name = match.group(2)
                response_fields.append((field_type, field_name))
    
    return {
        'request': request_fields,
        'response': response_fields
    }


def ros_type_to_proto(ros_type: str) -> str:
    """Convert ROS2 type to Protocol Buffers type."""
    type_map = {
        'bool': 'bool',
        'int8': 'int32',
        'uint8': 'uint32',
        'int16': 'int32',
        'uint16': 'uint32',
        'int32': 'int32',
        'uint32': 'uint32',
        'int64': 'int64',
        'uint64': 'uint64',
        'float32': 'float',
        'float64': 'double',
        'string': 'string',
        'time': 'int64',  # Unix timestamp
        'duration': 'int64',
    }
    
    # Handle arrays
    if '[]' in ros_type:
        base_type = ros_type.replace('[]', '').strip()
        proto_type = type_map.get(base_type, 'string')
        return f'repeated {proto_type}'
    
    return type_map.get(ros_type, 'string')


def generate_proto_file(srv_name: str, fields: dict, package_name: str = "vyra_base") -> str:
    """Generate Protocol Buffers .proto content from parsed fields."""
    
    proto_content = f'''syntax = "proto3";

package {package_name};

// Auto-generated from {srv_name}.srv
// Request message for {srv_name} service
message {srv_name}Request {{
'''
    
    # Add request fields
    field_num = 1
    for field_type, field_name in fields['request']:
        proto_type = ros_type_to_proto(field_type)
        proto_content += f'    {proto_type} {field_name} = {field_num};\n'
        field_num += 1
    
    if not fields['request']:
        proto_content += '    // Empty request\n'
    
    proto_content += f'''}}

// Response message for {srv_name} service
message {srv_name}Response {{
'''
    
    # Add response fields
    field_num = 1
    for field_type, field_name in fields['response']:
        proto_type = ros_type_to_proto(field_type)
        proto_content += f'    {proto_type} {field_name} = {field_num};\n'
        field_num += 1
    
    if not fields['response']:
        proto_content += '    // Empty response\n'
    
    proto_content += f'''}}

// Service definition (for Redis VyraCallable)
service {srv_name}Service {{
    rpc {srv_name} ({srv_name}Request) returns ({srv_name}Response);
}}
'''
    
    return proto_content


def main():
    # Paths
    base_dir = Path(__file__).parent.parent
    srv_dir = base_dir / "src" / "vyra_base" / "interfaces" / "srv"
    redis_dir = base_dir / "src" / "vyra_base" / "interfaces" / "redis"
    
    # Create redis directory
    redis_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Converting .srv files from {srv_dir} to {redis_dir}")
    
    # Process all .srv files
    for srv_file in srv_dir.glob("*.srv"):
        srv_name = srv_file.stem.replace("VBASE", "")  # Remove VBASE prefix
        
        print(f"Converting {srv_file.name} → {srv_name}.proto")
        
        # Parse .srv file
        fields = parse_srv_file(srv_file)
        
        # Generate .proto file
        proto_content = generate_proto_file(srv_name, fields)
        
        # Write .proto file
        proto_path = redis_dir / f"{srv_name}.proto"
        proto_path.write_text(proto_content)
        
        print(f"  ✅ Created {proto_path.name}")
    
    print(f"\n✅ Conversion complete! Created {len(list(redis_dir.glob('*.proto')))} .proto files")


if __name__ == "__main__":
    main()
