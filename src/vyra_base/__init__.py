import os
import shutil
from pathlib import Path
from typing import Any

import vyra_base


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