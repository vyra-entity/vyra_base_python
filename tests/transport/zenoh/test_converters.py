"""
Unit tests for Zenoh communication converters.
"""
import pytest
from unittest.mock import Mock

from vyra_base.com.converters import (
    ConverterFactory,
    ConverterRegistry,
    ProtobufConverter
)


@pytest.mark.unit
class TestConverterFactory:
    """Test converter factory functionality."""
    
    def test_get_converter(self):
        """Test getting converter by name."""
        converter = ConverterFactory.get_converter("protobuf")
        
        if converter:
            assert converter.name == "protobuf"
            assert isinstance(converter, ProtobufConverter)
    
    def test_list_converters(self):
        """Test listing available converters."""
        converters = ConverterFactory.list_available_converters()
        
        assert isinstance(converters, list)
        # Should have at least protobuf if available
    
    def test_get_nonexistent_converter(self):
        """Test getting non-existent converter."""
        converter = ConverterFactory.get_converter("nonexistent")
        
        assert converter is None


@pytest.mark.unit
class TestConverterRegistry:
    """Test converter registry."""
    
    def test_register_converter(self):
        """Test registering a converter."""
        # Create mock converter
        mock_converter = Mock()
        mock_converter.name = "test_converter"
        
        ConverterRegistry.register(mock_converter)
        
        retrieved = ConverterRegistry.get("test_converter")
        assert retrieved == mock_converter
    
    def test_list_converters(self):
        """Test listing converters."""
        converters = ConverterRegistry.list_converters()
        
        assert isinstance(converters, list)
    
    def test_clear_registry(self):
        """Test clearing registry."""
        mock_converter = Mock()
        mock_converter.name = "temp_converter"
        
        ConverterRegistry.register(mock_converter)
        ConverterRegistry.clear()
        
        retrieved = ConverterRegistry.get("temp_converter")
        assert retrieved is None


@pytest.mark.unit
class TestProtobufConverter:
    """Test Protobuf converter."""
    
    @pytest.fixture
    def converter(self):
        """Create converter instance."""
        return ProtobufConverter()
    
    def test_converter_name(self, converter):
        """Test converter name."""
        assert converter.name == "protobuf"
    
    def test_is_available(self, converter):
        """Test availability check."""
        available = converter.is_available()
        
        assert isinstance(available, bool)
        # Will be True if protobuf is installed
