"""
Converter Factory

Factory for creating and accessing converters.
"""
import logging
from typing import Optional

from vyra_base.com.converters.interface import ConverterInterface
from vyra_base.com.converters.registry import ConverterRegistry

logger = logging.getLogger(__name__)


class ConverterFactory:
    """
    Factory for creating and accessing converters.
    
    Provides convenient access to registered converters with automatic
    lazy initialization.
    """
    
    _initialized = False
    
    @classmethod
    def _initialize_default_converters(cls) -> None:
        """Initialize and register default converters."""
        if cls._initialized:
            return
        
        # Register Protobuf converter
        try:
            from vyra_base.com.converters.protobuf_converter import ProtobufConverter
            converter = ProtobufConverter()
            if converter.is_available():
                ConverterRegistry.register(converter)
                logger.debug("✅ Registered Protobuf converter")
        except ImportError as e:
            logger.debug(f"Protobuf converter not available: {e}")
        
        # Future converters can be added here
        # - MessagePackConverter
        # - AvroConverter
        # - FlatBuffersConverter
        # etc.
        
        cls._initialized = True
    
    @classmethod
    def get_converter(cls, name: str) -> Optional[ConverterInterface]:
        """
        Get converter by name.
        
        Args:
            name: Converter name (e.g., "protobuf")
            
        Returns:
            Converter instance or None
        """
        cls._initialize_default_converters()
        return ConverterRegistry.get(name)
    
    @classmethod
    def list_available_converters(cls) -> list[str]:
        """
        List all available converter names.
        
        Returns:
            List of converter names
        """
        cls._initialize_default_converters()
        return ConverterRegistry.list_converters()
    
    @classmethod
    def register_custom_converter(cls, converter: ConverterInterface) -> None:
        """
        Register a custom converter.
        
        Args:
            converter: Custom converter instance
        """
        ConverterRegistry.register(converter)
