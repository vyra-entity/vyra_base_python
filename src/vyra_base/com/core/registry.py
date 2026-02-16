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
    InterfaceType,
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraClient,
    VyraActionServer,
    VyraActionClient
)

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
        
        self._publishers: Dict[str, VyraPublisher] = {}
        self._subscribers: Dict[str, VyraSubscriber] = {}
        self._servers: Dict[str, VyraServer] = {}
        self._clients: Dict[str, VyraClient] = {}
        self._action_servers: Dict[str, VyraActionServer] = {}
        self._action_clients: Dict[str, VyraActionClient] = {}
        self._all_interfaces: Dict[str, VyraInterface] = {}
        self._initialized = True
        
        logger.info("✅ DataSpace Registry initialized")
    
    def register_publisher(self, publisher_obj: VyraPublisher) -> None:
        """
        Register a publisher interface.
        
        Args:
            publisher_obj: Publisher to register
            
        Raises:
            InterfaceError: If publisher with same name already exists
        """
        with self._lock:
            if publisher_obj.name in self._publishers:
                logger.warning(
                    f"⚠️ Publisher '{publisher_obj.name}' already registered, replacing"
                )
            
            self._publishers[publisher_obj.name] = publisher_obj
            self._all_interfaces[publisher_obj.name] = publisher_obj
            
            logger.debug(
                f"✅ Publisher '{publisher_obj.name}' registered "
                f"(protocol: {publisher_obj.protocol})"
            )
    
    def register_subscriber(self, subscriber_obj: VyraSubscriber) -> None:
        """
        Register a subscriber interface.
        
        Args:
            subscriber_obj: Subscriber to register
        """
        with self._lock:
            if subscriber_obj.name in self._subscribers:
                logger.warning(
                    f"⚠️ Subscriber '{subscriber_obj.name}' already registered, replacing"
                )
            
            self._subscribers[subscriber_obj.name] = subscriber_obj
            self._all_interfaces[subscriber_obj.name] = subscriber_obj
            
            logger.debug(
                f"✅ Subscriber '{subscriber_obj.name}' registered "
                f"(protocol: {subscriber_obj.protocol})"
            )

    def register_server(self, server_obj: VyraServer) -> None:
        """
        Register a server interface.
        
        Args:
            server_obj: Server to register
        """
        with self._lock:
            if server_obj.name in self._servers:
                logger.warning(
                    f"⚠️ Server '{server_obj.name}' already registered, replacing"
                )
            
            self._servers[server_obj.name] = server_obj
            self._all_interfaces[server_obj.name] = server_obj
            
            logger.debug(
                f"✅ Server '{server_obj.name}' registered "
                f"(protocol: {server_obj.protocol})"
            )
    
    def register_client(self, client_obj: VyraClient) -> None:
        """
        Register a client interface.
        
        Args:
            client_obj: Client to register
        """
        with self._lock:
            if client_obj.name in self._clients:
                logger.warning(
                    f"⚠️ Client '{client_obj.name}' already registered, replacing"
                )
            
            self._clients[client_obj.name] = client_obj
            self._all_interfaces[client_obj.name] = client_obj
            
            logger.debug(
                f"✅ Client '{client_obj.name}' registered "
                f"(protocol: {client_obj.protocol})"
            )

    def register_action_server(self, action_server_obj: VyraActionServer) -> None:
        """
        Register an action server interface.
        
        Args:
            action_server_obj: Action server to register
        """
        with self._lock:
            if action_server_obj.name in self._action_servers:
                logger.warning(
                    f"⚠️ Action Server '{action_server_obj.name}' already registered, replacing"
                )
            
            self._action_servers[action_server_obj.name] = action_server_obj
            self._all_interfaces[action_server_obj.name] = action_server_obj
            
            logger.debug(
                f"✅ Action Server '{action_server_obj.name}' registered "
                f"(protocol: {action_server_obj.protocol})"
            )

    def register_action_client(self, action_client_obj: VyraActionClient) -> None:
        """
        Register an action client interface.
        
        Args:
            action_client_obj: Action client to register
        """
        with self._lock:
            if action_client_obj.name in self._action_clients:
                logger.warning(
                    f"⚠️ Action Client '{action_client_obj.name}' already registered, replacing"
                )
            
            self._action_clients[action_client_obj.name] = action_client_obj
            self._all_interfaces[action_client_obj.name] = action_client_obj
            
            logger.debug(
                f"✅ Action Client '{action_client_obj.name}' registered "
                f"(protocol: {action_client_obj.protocol})"
            )
    
    def get_publisher(self, name: str) -> Optional[VyraPublisher]:
        """Get publisher by name."""
        return self._publishers.get(name)
    
    def get_subscriber(self, name: str) -> Optional[VyraSubscriber]:
        """Get subscriber by name."""
        return self._subscribers.get(name)
    
    def get_server(self, name: str) -> Optional[VyraServer]:
        """Get server by name."""
        return self._servers.get(name)
    
    def get_client(self, name: str) -> Optional[VyraClient]:
        """Get client by name."""
        return self._clients.get(name)
    
    def get_action_server(self, name: str) -> Optional[VyraActionServer]:
        """Get action server by name."""
        return self._action_servers.get(name)
    
    def get_action_client(self, name: str) -> Optional[VyraActionClient]:
        """Get action client by name."""
        return self._action_clients.get(name)
    
    def get_interface(self, name: str) -> Optional[VyraInterface]:
        """Get any interface by name."""
        return self._all_interfaces.get(name)
    
    def list_publishers(self) -> List[VyraPublisher]:
        """Get all registered publishers."""
        with self._lock:
            return list(self._publishers.values())
    
    def list_subscribers(self) -> List[VyraSubscriber]:
        """Get all registered subscribers."""
        with self._lock:
            return list(self._subscribers.values())
        
    def list_servers(self) -> List[VyraServer]:
        """Get all registered servers."""
        with self._lock:
            return list(self._servers.values())

    def list_clients(self) -> List[VyraClient]:
        """Get all registered clients."""
        with self._lock:
            return list(self._clients.values())
    
    def list_action_servers(self) -> List[VyraActionServer]:
        """Get all registered action servers."""
        with self._lock:
            return list(self._action_servers.values())
    
    def list_action_clients(self) -> List[VyraActionClient]:
        """Get all registered action clients."""
        with self._lock:
            return list(self._action_clients.values())
    
    def list_all(self) -> List[VyraInterface]:
        """Get all registered interfaces."""
        with self._lock:
            return list(self._all_interfaces.values())
    
    def remove_publisher(self, name: str) -> bool:
        """Remove publisher by name."""
        with self._lock:
            if name in self._publishers:
                del self._publishers[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed publisher '{name}'")
                return True
            return False

    def remove_subscriber(self, name: str) -> bool:
        """Remove subscriber by name."""
        with self._lock:
            if name in self._subscribers:
                del self._subscribers[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed subscriber '{name}'")
                return True
            return False
        
    def remove_server(self, name: str) -> bool:
        """Remove server by name."""
        with self._lock:
            if name in self._servers:
                del self._servers[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed server '{name}'")
                return True
            return False

    def remove_client(self, name: str) -> bool:
        """Remove client by name."""
        with self._lock:
            if name in self._clients:
                del self._clients[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed client '{name}'")
                return True
            return False
    
    def remove_action_server(self, name: str) -> bool:
        """Remove action server by name."""
        with self._lock:
            if name in self._action_servers:
                del self._action_servers[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed action server '{name}'")
                return True
            return False
    
    def remove_action_client(self, name: str) -> bool:
        """Remove action client by name."""
        with self._lock:
            if name in self._action_clients:
                del self._action_clients[name]
                del self._all_interfaces[name]
                logger.debug(f"Removed action client '{name}'")
                return True
            return False
    
    def clear_all(self) -> None:
        """Clear all registered interfaces."""
        with self._lock:
            self._publishers.clear()
            self._subscribers.clear()
            self._servers.clear()
            self._clients.clear()
            self._action_servers.clear()
            self._action_clients.clear()
            self._all_interfaces.clear()
            logger.info("🗑️ All interfaces cleared from registry")
    
    def get_statistics(self) -> Dict[str, int]:
        """Get registry statistics."""
        with self._lock:
            return {
                "publishers": len(self._publishers),
                "subscribers": len(self._subscribers),
                "servers": len(self._servers),
                "clients": len(self._clients),
                "action_servers": len(self._action_servers),
                "action_clients": len(self._action_clients),
                "total": len(self._all_interfaces),
            }


# Global singleton instance
DataSpace = DataSpaceRegistry()

# Legacy compatibility functions (will be deprecated)
def add_publisher(publisher_obj: VyraPublisher) -> None:
    """Legacy: Register publisher."""
    DataSpace.register_publisher(publisher_obj)

def add_subscriber(subscriber_obj: VyraSubscriber) -> None:
    """Legacy: Register subscriber."""
    DataSpace.register_subscriber(subscriber_obj)

def add_server(server_obj: VyraServer) -> None:
    """Legacy: Register server."""
    DataSpace.register_server(server_obj)

def add_client(client_obj: VyraClient) -> None:
    """Legacy: Register client."""
    DataSpace.register_client(client_obj)

def add_action_server(action_server_obj: VyraActionServer) -> None:
    """Legacy: Register action server."""
    DataSpace.register_action_server(action_server_obj)

def add_action_client(action_client_obj: VyraActionClient) -> None:
    """Legacy: Register action client."""
    DataSpace.register_action_client(action_client_obj)

