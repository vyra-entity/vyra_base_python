import os
import shutil
from pathlib import Path
import vos_base


def extract_ros_interfaces(target_package_path):
    """
    Extrahiert ROS2 Interface-Dateien aus der pip-installierten Library
    in ein ROS2 Package
    """
    # Finde die installierte Library

    import importlib.resources
    package_path: Path = Path(vos_base.__file__).parent / 'interfaces'

    source_path = Path(package_path)
    target_path = Path(target_package_path)
    
    # Copy interface files from source to target
    print(f"Extracting ROS2 interfaces from {source_path} to {target_path}")
    for interface_type in ['msg', 'srv', 'action', 'config']:
        source_dir: Path = source_path / interface_type
        target_dir: Path = target_path / interface_type
        
        if source_dir.exists():
            target_dir.mkdir(exist_ok=True)
            for file in source_dir.glob(f'*.{interface_type}'):
                shutil.copy2(file, target_dir / file.name)
                print(f"Copied {file.name} to {target_dir}")

    print(f"ROS2 interfaces extracted to {target_path} successfully.")