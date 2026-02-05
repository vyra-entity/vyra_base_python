import os
import shutil
from pathlib import Path
from typing import Any

import vyra_base


def extract_interfaces(target_path: str | Path):
    """
    Extract VYRA interface files from the pip-installed library into a module interface package.

    Extracts speaker (.msg), callable (.srv + .proto), and job (.action) interfaces
    using VYRA-specific terminology independent of ROS2 naming conventions.

    :param target_path: Path to the target package where interfaces will be extracted.
    :type target_path: str or pathlib.Path

    :return: None
    :rtype: None

    This function copies interface files from the installed vyra_base library:
    - .msg files → target/msg/ (from speaker/ directory)
    - .srv files → target/srv/ (from callable/ directory)
    - .action files → target/action/ (from job/ directory)
    - .proto files → target/proto/ (from proto/ directory, for Redis/gRPC)
    
    The source uses VYRA terminology (speaker/callable/job/proto) but outputs
    to ROS2-compatible directories (msg/srv/action) plus proto/ for gRPC.
    """
    import importlib.resources
    package_path: Path = Path(vyra_base.__file__).parent / 'interfaces'

    source_path = Path(package_path)

    if isinstance(target_path, str):
        target_path = Path(target_path)
    
    # Copy interface files from source to target
    print(f"Extracting VYRA interfaces from {source_path} to {target_path}")
    
    # Copy build files first
    build_files = ['package.xml', 'CMakeLists.template.txt']
    for build_file in build_files:
        source_file = source_path / build_file
        if source_file.exists():
            shutil.copy2(source_file, target_path / build_file)
            print(f"Copied {build_file} to {target_path}")
    
    # Map VYRA interface types to file extensions and target directories
    # source_dir → (file_ext, target_dir)
    interface_mapping = {
        'speaker': ('msg', 'msg'),      # Speaker → .msg files for ROS2
        'callable': ('srv', 'srv'),     # Callable → .srv files for ROS2
        'job': ('action', 'action'),    # Job → .action files for ROS2
        'proto': ('proto', 'proto')     # Proto → .proto files for Redis/gRPC
    }
    
    for source_type, (file_ext, target_type) in interface_mapping.items():
        source_dir: Path = source_path / source_type
        target_dir: Path = target_path / target_type
        
        if source_dir.exists():
            target_dir.mkdir(exist_ok=True)
            
            for file in source_dir.rglob(f'*.{file_ext}'):
                shutil.copy2(file, target_dir / file.name)
                print(f"Copied {file.name} ({source_type}) to {target_dir}")

    config_path: Path = source_path / 'config'
    target_config: Path = target_path / 'config'
    
    if not target_config.exists():
        target_config.mkdir(parents=True, exist_ok=True)
    
    for type in ['json', 'yaml', 'xml']:
        for file in config_path.glob(f'*.{type}'):
            shutil.copy2(file, target_config)
            print(f"Copied {file.name} to {target_config}")

    print(f"VYRA interface extraction to {target_path} successful.")


# Alias for backward compatibility
def extract_ros_interfaces(target_path: str | Path):
    """
    Deprecated: Use extract_interfaces() instead.
    
    This function is kept for backward compatibility.
    """
    print("⚠️  extract_ros_interfaces() is deprecated. Use extract_interfaces() instead.")
    return extract_interfaces(target_path)

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