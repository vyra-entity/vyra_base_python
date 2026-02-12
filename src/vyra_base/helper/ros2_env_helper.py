"""
ROS2 Environment Helper for Dynamic Interface Loading

Provides utilities to update environment variables and Python paths
to make dynamically built ROS2 interface packages discoverable at runtime.

This enables the "late sourcing" pattern where a running Python process
can discover newly built ROS2 packages without restarting.
"""
from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


def update_ament_prefix_path(workspace_install_dir: str | Path) -> bool:
    """
    Add workspace install directory to AMENT_PREFIX_PATH.
    
    This makes ROS2 packages in the workspace discoverable to
    rosidl_runtime_py utilities like get_message(), get_service().
    
    Args:
        workspace_install_dir: Path to workspace install directory
            (e.g., "/workspace/install")
    
    Returns:
        True if path was added, False if already present
    
    Examples:
        >>> update_ament_prefix_path("/workspace/install")
        True
    """
    install_path = Path(workspace_install_dir).resolve()
    
    if not install_path.exists():
        logger.warning(
            f"⚠️ Workspace install directory does not exist: {install_path}"
        )
        return False
    
    install_str = str(install_path)
    current_ament_path = os.environ.get('AMENT_PREFIX_PATH', '')
    
    # Check if already in path
    if install_str in current_ament_path.split(':'):
        logger.debug(f"AMENT_PREFIX_PATH already contains: {install_str}")
        return False
    
    # Prepend to path (highest priority)
    if current_ament_path:
        new_path = f"{install_str}:{current_ament_path}"
    else:
        new_path = install_str
    
    os.environ['AMENT_PREFIX_PATH'] = new_path
    logger.info(f"✓ Added to AMENT_PREFIX_PATH: {install_str}")
    
    return True


def update_python_path(
    package_site_packages_dir: str | Path,
    prepend: bool = True
) -> bool:
    """
    Add package site-packages directory to sys.path for Python imports.
    
    This enables importing generated Python modules (*_pb2.py, ROS2 .py files)
    from newly built packages.
    
    Args:
        package_site_packages_dir: Path to site-packages directory
            (e.g., "/workspace/install/v2_modulemanager_interfaces/lib/python3.12/site-packages")
        prepend: If True, add to front of sys.path (default), else append
    
    Returns:
        True if path was added, False if already present
    
    Examples:
        >>> update_python_path("/workspace/install/mypackage/lib/python3.12/site-packages")
        True
    """
    site_packages = Path(package_site_packages_dir).resolve()
    
    if not site_packages.exists():
        logger.warning(
            f"⚠️ Site-packages directory does not exist: {site_packages}"
        )
        return False
    
    site_packages_str = str(site_packages)
    
    # Check if already in sys.path
    if site_packages_str in sys.path:
        logger.debug(f"sys.path already contains: {site_packages_str}")
        return False
    
    # Add to sys.path
    if prepend:
        sys.path.insert(0, site_packages_str)
        logger.info(f"✓ Prepended to sys.path: {site_packages_str}")
    else:
        sys.path.append(site_packages_str)
        logger.info(f"✓ Appended to sys.path: {site_packages_str}")
    
    return True


def detect_python_version() -> str:
    """
    Detect current Python version (e.g., "3.12").
    
    Returns:
        Python version string in format "X.Y"
    """
    version_info = sys.version_info
    return f"{version_info.major}.{version_info.minor}"


def find_package_site_packages(
    workspace_install_dir: str | Path,
    package_name: str,
    python_version: Optional[str] = None
) -> Optional[Path]:
    """
    Find site-packages directory for a package in workspace.
    
    Searches for typical ROS2 package structure:
    <workspace>/install/<package>/lib/python<version>/site-packages
    
    Args:
        workspace_install_dir: Workspace install directory
        package_name: Name of the package
        python_version: Python version (e.g., "3.12"), auto-detected if None
    
    Returns:
        Path to site-packages if found, None otherwise
    
    Examples:
        >>> find_package_site_packages("/workspace/install", "v2_modulemanager_interfaces")
        Path('/workspace/install/v2_modulemanager_interfaces/lib/python3.12/site-packages')
    """
    if python_version is None:
        python_version = detect_python_version()
    
    install_dir = Path(workspace_install_dir).resolve()
    package_dir = install_dir / package_name
    
    if not package_dir.exists():
        logger.debug(f"Package directory not found: {package_dir}")
        return None
    
    # Try standard ROS2 path
    site_packages = package_dir / "lib" / f"python{python_version}" / "site-packages"
    
    if site_packages.exists():
        return site_packages
    
    # Try without version-specific subdirectory (some packages)
    site_packages_alt = package_dir / "lib" / "site-packages"
    if site_packages_alt.exists():
        return site_packages_alt
    
    logger.debug(f"Site-packages not found for package: {package_name}")
    return None


def ensure_interface_package_discoverable(
    workspace_install_dir: str | Path,
    package_name: str,
    python_version: Optional[str] = None
) -> bool:
    """
    Make an interface package discoverable to running Python process.
    
    Updates both AMENT_PREFIX_PATH (for ROS2 introspection) and
    sys.path (for Python imports of generated modules).
    
    Args:
        workspace_install_dir: Workspace install directory
        package_name: Name of the interface package
        python_version: Python version (auto-detected if None)
    
    Returns:
        True if package is now discoverable, False on failure
    
    Examples:
        >>> ensure_interface_package_discoverable(
        ...     "/workspace/install",
        ...     "v2_modulemanager_interfaces"
        ... )
        True
    """
    install_dir = Path(workspace_install_dir).resolve()
    
    # Update AMENT_PREFIX_PATH with workspace install dir
    ament_updated = update_ament_prefix_path(install_dir)
    
    # Find and update Python path for package
    site_packages = find_package_site_packages(
        install_dir, package_name, python_version
    )
    
    python_updated = False
    if site_packages:
        python_updated = update_python_path(site_packages)
    else:
        logger.warning(
            f"⚠️ Could not find site-packages for package: {package_name}"
        )
    
    success = ament_updated or python_updated
    
    if success:
        logger.info(
            f"✓ Package '{package_name}' is now discoverable "
            f"(AMENT: {ament_updated}, Python: {python_updated})"
        )
    else:
        logger.debug(
            f"Package '{package_name}' was already discoverable or not found"
        )
    
    return success


def ensure_workspace_discoverable(
    workspace_install_dir: str | Path,
    python_version: Optional[str] = None
) -> int:
    """
    Make all packages in a workspace install directory discoverable.
    
    Scans workspace install directory and updates paths for all found packages.
    
    Args:
        workspace_install_dir: Workspace install directory
        python_version: Python version (auto-detected if None)
    
    Returns:
        Number of packages made discoverable
    
    Examples:
        >>> ensure_workspace_discoverable("/workspace/install")
        5  # Made 5 packages discoverable
    """
    install_dir = Path(workspace_install_dir).resolve()
    
    if not install_dir.exists():
        logger.error(f"❌ Workspace install directory not found: {install_dir}")
        return 0
    
    # Update AMENT_PREFIX_PATH once for entire workspace
    update_ament_prefix_path(install_dir)
    
    # Find all packages (directories with share/ subdirectory typically)
    count = 0
    for package_dir in install_dir.iterdir():
        if not package_dir.is_dir():
            continue
        
        # Skip common non-package directories
        if package_dir.name in ['ament_index', 'colcon-core', 'setup']:
            continue
        
        package_name = package_dir.name
        
        # Try to add Python path for this package
        site_packages = find_package_site_packages(
            install_dir, package_name, python_version
        )
        
        if site_packages:
            if update_python_path(site_packages):
                count += 1
    
    logger.info(
        f"✓ Workspace discoverable: {count} package(s) with Python modules"
    )
    
    return count


def get_current_paths_info() -> dict[str, str]:
    """
    Get current AMENT_PREFIX_PATH and sys.path for debugging.
    
    Returns:
        Dict with 'ament_prefix_path' and 'sys_path' entries
    """
    return {
        'ament_prefix_path': os.environ.get('AMENT_PREFIX_PATH', ''),
        'sys_path': ':'.join(sys.path),
        'python_version': detect_python_version()
    }
