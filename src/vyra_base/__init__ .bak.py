import os
import shutil
from pathlib import Path
import vyra_base


def extract_ros_interfaces(target_package_path):
    """
    Extrahiert ROS2 Interface-Dateien aus der pip-installierten Library
    in ein ROS2 Package
    """
    # Finde die installierte Library

    import importlib.resources
    package_path = Path(vyra_base.__file__).parent / 'interfaces'

    source_path = Path(package_path)
    target_path = Path(target_package_path)
    
    # Copy interface files from source to target
    for interface_type in ['msg', 'srv', 'action']:
        source_dir = source_path / interface_type
        target_dir = target_path / interface_type
        
        if source_dir.exists():
            target_dir.mkdir(exist_ok=True)
            for file in source_dir.glob(f'*.{interface_type}'):
                shutil.copy2(file, target_dir / file.name)
                print(f"Copied {file.name} to {target_dir}")

    # Copy types.py to __init__.py
    init_file: Path = target_path / target_path.name / '__init__.py'
    if not init_file.exists():
        init_file.parent.mkdir(parents=True, exist_ok=True)
        init_file.touch()
        with open(source_path / 'config' / 'types.py', 'r') as source_file:
            type_content = source_file.read()
            type_content = type_content.replace('vos_base', target_path.name)
            with open(init_file, 'a') as target_file:
                target_file.write(type_content)
                print(f"Created {init_file} with type imports.")