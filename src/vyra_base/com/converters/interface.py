"""
Converter Interface

Abstract base class for all communication converters.
"""
from abc import ABC, abstractmethod
from typing import Any, Optional, Type


class ConverterInterface(ABC):
    """
    Abstract base class for all converters.
    
    All converters (Protobuf, MessagePack, etc.) must implement this interface.
    """
    
    @property
    @abstractmethod
    def name(self) -> str:
        """Get converter name."""
        pass
    
    @abstractmethod
    def convert_to(self, data: Any, target_type: Optional[Type] = None) -> Any:
        """
        Convert data to target format.
        
        Args:
            data: Source data
            target_type: Optional target type
            
        Returns:
            Converted data
        """
        pass
    
    @abstractmethod
    def convert_from(self, data: Any, source_type: Optional[Type] = None) -> Any:
        """
        Convert data from source format.
        
        Args:
            data: Source data
            source_type: Optional source type
            
        Returns:
            Converted data
        """
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """
        Check if converter is available (dependencies installed).
        
        Returns:
            bool: True if converter can be used
        """
        pass
