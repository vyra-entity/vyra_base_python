import os
import shutil
from pathlib import Path

import vyra_base


def extract_ros_interfaces(target_path: str | Path):
    """
    Extract ROS2 interface files from the pip-installed library into a ROS2 package.

    :param target_package_path: Path to the target ROS2 package where interfaces will be extracted.
    :type target_package_path: str or pathlib.Path

    :return: None
    :rtype: None

    This function copies interface files (msg, srv, action, json) from the installed
    vyra_base library to the specified ROS2 package directory.
    """
    # Finde die installierte Library

    import importlib.resources
    package_path: Path = Path(vyra_base.__file__).parent / 'interfaces'

    source_path = Path(package_path)

    if isinstance(target_path, str):
        target_path = Path(target_path)
    
    # Copy interface files from source to target
    print(f"Extracting ROS2 interfaces from {source_path} to {target_path}")
    
    # Copy build files first
    build_files = ['package.xml', 'CMakeLists.template.txt']
    for build_file in build_files:
        source_file = source_path / build_file
        if source_file.exists():
            shutil.copy2(source_file, target_path / build_file)
            print(f"Copied {build_file} to {target_path}")
    
    for interface_type in ['msg', 'srv', 'action']:
        source_dir: Path = source_path / interface_type
        target_dir: Path = target_path / interface_type
        
        if source_dir.exists():
            target_dir.mkdir(exist_ok=True)
            for file in source_dir.rglob(f'*.{interface_type}'):
                shutil.copy2(file, target_dir / file.name)
                print(f"Copied {file.name} to {target_dir}")

    config_path: Path = source_path / 'config'
    target_config: Path = target_path / 'config'
    
    if not target_config.exists():
        target_config.mkdir(parents=True, exist_ok=True)
    
    for type in ['json', 'yaml', 'xml']:
        for file in config_path.glob(f'*.{type}'):
            shutil.copy2(file, target_config)
            print(f"Copied {file.name} to {target_config}")

    print(f"ROS2 interfaces extracted to {target_path} successfully.")