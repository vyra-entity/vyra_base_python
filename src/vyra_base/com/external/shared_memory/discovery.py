"""
Shared Memory Discovery Service

PID-tracked filesystem-based discovery for shared memory segments.
Implements deterministic discovery without network dependencies.
"""
import json
import logging
import os
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional

from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.exceptions import TransportError

logger = logging.getLogger(__name__)

# Discovery directory in /tmp for POSIX compatibility
DISCOVERY_DIR = Path("/tmp/vyra_sockets")


@dataclass
class SegmentDiscoveryInfo:
    """Discovery information for shared memory segment."""
    pid: int
    segment_name: str
    size: int
    created: float
    protocol: str
    module_name: str
    interface_name: str
    last_heartbeat: float = 0.0
    
    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'SegmentDiscoveryInfo':
        """Create from dictionary."""
        return cls(**data)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'SegmentDiscoveryInfo':
        """Create from JSON string."""
        return cls.from_dict(json.loads(json_str))


class SharedMemoryDiscovery:
    """
    Filesystem-based discovery service for shared memory segments.
    
    Features:
    - PID-based registration
    - Automatic stale entry cleanup
    - No network dependencies
    - Industrial-grade reliability
    
    Discovery File Format:
        Location: /tmp/vyra_sockets/<module_name>.<interface_name>.sm.pid
        Content: JSON with SegmentDiscoveryInfo
    
    Example:
        >>> discovery = SharedMemoryDiscovery()
        >>> discovery.register("motor_control", "start", "/vyra_motor_start", 4096)
        >>> segments = discovery.discover_all()
        >>> discovery.unregister("motor_control", "start")
    """
    
    def __init__(self, discovery_dir: Optional[Path] = None):
        """
        Initialize discovery service.
        
        Args:
            discovery_dir: Discovery directory (default: /tmp/vyra_sockets)
        """
        self.discovery_dir = discovery_dir or DISCOVERY_DIR
        self._ensure_discovery_dir()
    
    def _ensure_discovery_dir(self) -> None:
        """Create discovery directory if it doesn't exist."""
        try:
            self.discovery_dir.mkdir(parents=True, exist_ok=True)
            logger.debug(f"✅ Discovery directory ready: {self.discovery_dir}")
        except Exception as e:
            raise TransportError(
                f"Failed to create discovery directory {self.discovery_dir}: {e}"
            )
    
    def _get_discovery_path(self, module_name: str, interface_name: str) -> Path:
        """
        Get discovery file path.
        
        Args:
            module_name: Module name (e.g., 'motor_control')
            interface_name: Interface name (e.g., 'start')
            
        Returns:
            Path to discovery file
        """
        filename = f"{module_name}.{interface_name}.sm.pid"
        return self.discovery_dir / filename
    
    def register(
        self,
        module_name: str,
        interface_name: str,
        segment_name: str,
        size: int,
        pid: Optional[int] = None
    ) -> bool:
        """
        Register a shared memory segment.
        
        Args:
            module_name: Module name
            interface_name: Interface name
            segment_name: Shared memory segment name (must start with /)
            size: Segment size in bytes
            pid: Process ID (default: current process)
            
        Returns:
            bool: True if registration successful
            
        Example:
            >>> discovery.register(
            ...     "motor_control",
            ...     "start",
            ...     "/vyra_motor_start",
            ...     4096
            ... )
            True
        """
        pid = pid or os.getpid()
        discovery_path = self._get_discovery_path(module_name, interface_name)
        
        info = SegmentDiscoveryInfo(
            pid=pid,
            segment_name=segment_name,
            size=size,
            created=time.time(),
            protocol=ProtocolType.SHARED_MEMORY.value,
            module_name=module_name,
            interface_name=interface_name,
            last_heartbeat=time.time()
        )
        
        try:
            discovery_path.write_text(info.to_json())
            logger.info(
                f"✅ Registered shared memory segment: "
                f"{module_name}.{interface_name} -> {segment_name} (PID: {pid})"
            )
            return True
        except Exception as e:
            logger.error(f"❌ Failed to register segment: {e}")
            return False
    
    def unregister(self, module_name: str, interface_name: str) -> bool:
        """
        Unregister a shared memory segment.
        
        Args:
            module_name: Module name
            interface_name: Interface name
            
        Returns:
            bool: True if unregistration successful
        """
        discovery_path = self._get_discovery_path(module_name, interface_name)
        
        try:
            if discovery_path.exists():
                discovery_path.unlink()
                logger.info(
                    f"✅ Unregistered: {module_name}.{interface_name}"
                )
            return True
        except Exception as e:
            logger.error(f"❌ Failed to unregister: {e}")
            return False
    
    def discover(
        self,
        module_name: str,
        interface_name: str
    ) -> Optional[SegmentDiscoveryInfo]:
        """
        Discover a specific shared memory segment.
        
        Args:
            module_name: Module name
            interface_name: Interface name
            
        Returns:
            SegmentDiscoveryInfo if found, None otherwise
        """
        discovery_path = self._get_discovery_path(module_name, interface_name)
        
        if not discovery_path.exists():
            return None
        
        try:
            json_str = discovery_path.read_text()
            info = SegmentDiscoveryInfo.from_json(json_str)
            
            # Check if process is alive
            if not self._is_process_alive(info.pid):
                logger.warning(
                    f"⚠️ Stale segment found: {module_name}.{interface_name} "
                    f"(PID {info.pid} not running)"
                )
                self.unregister(module_name, interface_name)
                return None
            
            return info
        except Exception as e:
            logger.error(f"❌ Failed to discover segment: {e}")
            return None
    
    def discover_all(self) -> List[SegmentDiscoveryInfo]:
        """
        Discover all registered shared memory segments.
        
        Returns:
            List of SegmentDiscoveryInfo
        """
        segments = []
        
        if not self.discovery_dir.exists():
            return segments
        
        for file_path in self.discovery_dir.glob("*.sm.pid"):
            try:
                json_str = file_path.read_text()
                info = SegmentDiscoveryInfo.from_json(json_str)
                
                # Check if process is alive
                if self._is_process_alive(info.pid):
                    segments.append(info)
                else:
                    logger.debug(
                        f"🧹 Cleaning stale segment: {file_path.name}"
                    )
                    file_path.unlink()
            except Exception as e:
                logger.error(f"❌ Failed to read {file_path}: {e}")
        
        logger.debug(f"🔍 Discovered {len(segments)} active segments")
        return segments
    
    def discover_by_module(self, module_name: str) -> List[SegmentDiscoveryInfo]:
        """
        Discover all segments for a specific module.
        
        Args:
            module_name: Module name
            
        Returns:
            List of SegmentDiscoveryInfo
        """
        all_segments = self.discover_all()
        return [s for s in all_segments if s.module_name == module_name]
    
    def update_heartbeat(
        self,
        module_name: str,
        interface_name: str
    ) -> bool:
        """
        Update heartbeat timestamp for a segment.
        
        Args:
            module_name: Module name
            interface_name: Interface name
            
        Returns:
            bool: True if update successful
        """
        info = self.discover(module_name, interface_name)
        if not info:
            return False
        
        info.last_heartbeat = time.time()
        discovery_path = self._get_discovery_path(module_name, interface_name)
        
        try:
            discovery_path.write_text(info.to_json())
            return True
        except Exception as e:
            logger.error(f"❌ Failed to update heartbeat: {e}")
            return False
    
    def cleanup_stale_segments(self, max_age: float = 300.0) -> int:
        """
        Clean up segments with stale heartbeats or dead processes.
        
        Args:
            max_age: Maximum age in seconds (default: 5 minutes)
            
        Returns:
            int: Number of cleaned segments
        """
        cleaned = 0
        current_time = time.time()
        
        for segment in self.discover_all():
            # Check heartbeat age
            age = current_time - segment.last_heartbeat
            if age > max_age:
                logger.info(
                    f"🧹 Cleaning stale segment: "
                    f"{segment.module_name}.{segment.interface_name} "
                    f"(age: {age:.1f}s)"
                )
                if self.unregister(segment.module_name, segment.interface_name):
                    cleaned += 1
        
        if cleaned > 0:
            logger.info(f"✅ Cleaned {cleaned} stale segments")
        
        return cleaned
    
    @staticmethod
    def _is_process_alive(pid: int) -> bool:
        """
        Check if a process is alive.
        
        Args:
            pid: Process ID
            
        Returns:
            bool: True if process is running
        """
        try:
            # Send signal 0 to check if process exists
            os.kill(pid, 0)
            return True
        except OSError:
            return False
    
    def get_statistics(self) -> Dict:
        """
        Get discovery statistics.
        
        Returns:
            Dict with statistics
        """
        segments = self.discover_all()
        
        stats: Dict = {
            'total_segments': len(segments),
            'total_size_bytes': sum(s.size for s in segments),
            'modules': {},
            'oldest_segment': None,
            'newest_segment': None
        }
        
        if not segments:
            return stats
        
        # Group by module
        for segment in segments:
            module = segment.module_name
            if module not in stats['modules']:
                stats['modules'][module] = {
                    'count': 0,
                    'size_bytes': 0,
                    'interfaces': []
                }
            
            stats['modules'][module]['count'] += 1
            stats['modules'][module]['size_bytes'] += segment.size
            stats['modules'][module]['interfaces'].append(segment.interface_name)
        
        # Find oldest and newest
        sorted_by_age = sorted(segments, key=lambda s: s.created)
        stats['oldest_segment'] = {
            'module': sorted_by_age[0].module_name,
            'interface': sorted_by_age[0].interface_name,
            'age_seconds': time.time() - sorted_by_age[0].created
        }
        stats['newest_segment'] = {
            'module': sorted_by_age[-1].module_name,
            'interface': sorted_by_age[-1].interface_name,
            'age_seconds': time.time() - sorted_by_age[-1].created
        }
        
        return stats


# Global singleton instance
_discovery_instance: Optional[SharedMemoryDiscovery] = None


def get_discovery() -> SharedMemoryDiscovery:
    """
    Get global discovery instance.
    
    Returns:
        SharedMemoryDiscovery singleton
    """
    global _discovery_instance
    if _discovery_instance is None:
        _discovery_instance = SharedMemoryDiscovery()
    return _discovery_instance
