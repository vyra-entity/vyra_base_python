"""
Converter Registry

Manages registration and lookup of converters.
"""
import logging
from typing import Dict, Optional

from vyra_base.com.converters.interface import ConverterInterface

logger = logging.getLogger(__name__)


class ConverterRegistry:
    """
    Singleton registry for all converters.
    
    Manages converter instances and provides lookup by name.
    """
    
    _instance: Optional['ConverterRegistry'] = None
    _converters: Dict[str, ConverterInterface] = {}
    
    def __new__(cls):
        """Singleton pattern."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    @classmethod
    def register(cls, converter: ConverterInterface) -> None:
        """
        Register a converter.
        
        Args:
            converter: Converter instance to register
        """
        instance = cls()
        name = converter.name
        
        if name in instance._converters:
            logger.warning(f"Converter '{name}' already registered, overwriting")
        
        instance._converters[name] = converter
        logger.debug(f"✅ Registered converter: {name}")
    
    @classmethod
    def get(cls, name: str) -> Optional[ConverterInterface]:
        """
        Get converter by name.
        
        Args:
            name: Converter name
            
        Returns:
            Converter instance or None if not found
        """
        instance = cls()
        return instance._converters.get(name)
    
    @classmethod
    def list_converters(cls) -> list[str]:
        """
        List all registered converter names.
        
        Returns:
            List of converter names
        """
        instance = cls()
        return list(instance._converters.keys())
    
    @classmethod
    def clear(cls) -> None:
        """Clear all registered converters."""
        instance = cls()
        instance._converters.clear()
        logger.debug("Cleared all converters")
