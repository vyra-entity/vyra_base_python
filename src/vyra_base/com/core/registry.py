"""
DataSpace Registry

Central registry for all VYRA communication interfaces (Callables, Speakers, Jobs).
Thread-safe singleton pattern for global access.
"""
import logging
from typing import Dict, List, Optional
from threading import RLock

from vyra_base.com.core.types import (
    VyraInterface,
    VyraCallable,
    VyraSpeaker,
    VyraJob,
    InterfaceType,
)
from vyra_base.com.core.exceptions import InterfaceError

logger = logging.getLogger(__name__)


class DataSpaceRegistry:
    """
    Central registry for all communication interfaces.
    
    This singleton manages all callables, speakers, and jobs across all protocols.
    It provides thread-safe registration, lookup, and lifecycle management.
    """
    
    _instance: Optional['DataSpaceRegistry'] = None
    _lock = RLock()
    
    def __new__(cls):
        """Singleton pattern implementation."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        """Initialize registry (only once)."""
        if self._initialized:
            return
        
        self._callables: Dict[str, VyraCallable] = {}
        self._speakers: Dict[str, VyraSpeaker] = {}
        self._jobs: Dict[str, VyraJob] = {}
        self._all_interfaces: Dict[str, VyraInterface] = {}
        self._initialized = True
        
        logger.info("✅ DataSpace Registry initialized")
    
    def register_callable(self, callable_obj: VyraCallable) -> None:
        """
        Register a callable interface.
        
        Args:
            callable_obj: Callable to register
            
        Raises:
            InterfaceError: If callable with same name already exists
        """
        with self._lock:
            if callable_obj.name in self._callables:
                logger.warning(
                    f"⚠️ Callable '{callable_obj.name}' already registered, replacing"
                )
            
            self._callables[callable_obj.name] = callable_obj
            self._all_interfaces[callable_obj.name] = callable_obj
            
            logger.debug(
                f"✅ Callable '{callable_obj.name}' registered "
                f"(protocol: {callable_obj.protocol})"
            )
    
    def register_speaker(self, speaker_obj: VyraSpeaker) -> None:
        """
        Register a speaker interface.
        
        Args:
            speaker_obj: Speaker to register
        """
        with self._lock:
            if speaker_obj.name in self._speakers:
                logger.warning(
                    f"⚠️ Speaker '{speaker_obj.name}' already registered, replacing"
                )
            
            self._speakers[speaker_obj.name] = speaker_obj
            self._all_interfaces[speaker_obj.name] = speaker_obj
            
            logger.debug(
                f"✅ Speaker '{speaker_obj.name}' registered "
                f"(protocol: {speaker_obj.protocol})"
            )
    
    def register_job(self, job_obj: VyraJob) -> None:
        """
        Register a job interface.
        
        Args:
            job_obj: Job to register
        """
        with self._lock:
            if job_obj.name in self._jobs:
                logger.warning(
                    f"⚠️ Job '{job_obj.name}' already registered, replacing"
                )
            
            self._jobs[job_obj.name] = job_obj
            self._all_interfaces[job_obj.name] = job_obj
            
            logger.debug(
                f"✅ Job '{job_obj.name}' registered "
                f"(protocol: {job_obj.protocol})"
            )
    
    def get_callable(self, name: str) -> Optional[VyraCallable]:
        """Get callable by name."""
        return self._callables.get(name)
    
    def get_speaker(self, name: str) -> Optional[VyraSpeaker]:
        """Get speaker by name."""
        return self._speakers.get(name)
    
    def get_job(self, name: str) -> Optional[VyraJob]:
        """Get job by name."""
        return self._jobs.get(name)
    
    def get_interface(self, name: str) -> Optional[VyraInterface]:
        """Get any interface by name."""
        return self._all_interfaces.get(name)
    
    def list_callables(self) -> List[VyraCallable]:
        """Get all registered callables."""
        with self._lock:
            return list(self._callables.values())
    
    def list_speakers(self) -> List[VyraSpeaker]:
        """Get all registered speakers."""
        with self._lock:
            return list(self._speakers.values())
    
    def list_jobs(self) -> List[VyraJob]:
        """Get all registered jobs."""
        with self._lock:
            return list(self._jobs.values())
    
    def list_all(self) -> List[VyraInterface]:
        """Get all registered interfaces."""
        with self._lock:
            return list(self._all_interfaces.values())
    
    def remove_callable(self, name: str) -> bool:
        """Remove callable by name."""
        with self._lock:
            if name in self._callables:
                del self._callables[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed callable '{name}'")
                return True
            return False
    
    def remove_speaker(self, name: str) -> bool:
        """Remove speaker by name."""
        with self._lock:
            if name in self._speakers:
                del self._speakers[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed speaker '{name}'")
                return True
            return False
    
    def remove_job(self, name: str) -> bool:
        """Remove job by name."""
        with self._lock:
            if name in self._jobs:
                del self._jobs[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed job '{name}'")
                return True
            return False
    
    def clear_all(self) -> None:
        """Clear all registered interfaces."""
        with self._lock:
            self._callables.clear()
            self._speakers.clear()
            self._jobs.clear()
            self._all_interfaces.clear()
            logger.info("🗑️ All interfaces cleared from registry")
    
    def get_statistics(self) -> Dict[str, int]:
        """Get registry statistics."""
        with self._lock:
            return {
                "callables": len(self._callables),
                "speakers": len(self._speakers),
                "jobs": len(self._jobs),
                "total": len(self._all_interfaces),
            }


# Global singleton instance
DataSpace = DataSpaceRegistry()

# Legacy compatibility functions (will be deprecated)
def add_callable(callable_obj: VyraCallable) -> None:
    """Legacy: Register callable."""
    DataSpace.register_callable(callable_obj)

def add_speaker(speaker_obj: VyraSpeaker) -> None:
    """Legacy: Register speaker."""
    DataSpace.register_speaker(speaker_obj)

def add_job(job_obj: VyraJob) -> None:
    """Legacy: Register job."""
    DataSpace.register_job(job_obj)

def get_callable(name: str) -> Optional[VyraCallable]:
    """Legacy: Get callable."""
    return DataSpace.get_callable(name)

def get_speaker(name: str) -> Optional[VyraSpeaker]:
    """Legacy: Get speaker."""
    return DataSpace.get_speaker(name)

def get_job(name: str) -> Optional[VyraJob]:
    """Legacy: Get job."""
    return DataSpace.get_job(name)
