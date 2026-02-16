"""
Comprehensive test suite for vyra_base.defaults module
Tests default configurations, entries, exceptions, and constants
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime
from typing import Any, Dict
from uuid import uuid4, UUID

from vyra_base.defaults.entries import (
    StateEntry, NewsEntry, ErrorEntry, ModuleEntry,
    FunctionConfigEntry, FunctionConfigBaseTypes, FunctionConfigDisplaystyle
)
from vyra_base.defaults.exceptions import (
    FeederException
)


class TestStateEntry:
    """Test StateEntry functionality"""
    
    def test_state_entry_creation(self):
        """Test StateEntry creation with required parameters"""
        test_time = datetime.now()
        
        state_entry = StateEntry(
            _type=None,
            previous="initial",
            trigger="start",
            current="running",
            module_id="test_module_123",
            module_name="test_module",
            timestamp=test_time
        )
        
        assert state_entry.previous == "initial"
        assert state_entry.trigger == "start"
        assert state_entry.current == "running" 
        assert state_entry.module_id == "test_module_123"
        assert state_entry.module_name == "test_module"
        assert state_entry.timestamp == test_time

    def test_state_entry_defaults(self):
        """Test StateEntry with default values"""
        test_time = datetime.now()
        
        state_entry = StateEntry(
            _type=None,
            trigger="start",
            current="running",
            module_id="test_123",
            module_name="test_module",
            timestamp=test_time
        )
        
        assert state_entry.previous == "N/A"  # Default value

    def test_state_entry_representation(self):
        """Test StateEntry string representation"""
        state_entry = StateEntry(
            _type=None,
            previous="initial",
            trigger="start", 
            current="running",
            module_id="test_123",
            module_name="test_module",
            timestamp=datetime.now()
        )
        
        repr_str = repr(state_entry)
        assert "StateEntry" in repr_str
        assert "current=running" in repr_str
        assert "trigger=start" in repr_str
        assert "previous=initial" in repr_str

    def test_state_transitions(self):
        """Test various state transitions"""
        test_time = datetime.now()
        transitions = [
            ("initial", "start", "running"),
            ("running", "pause", "paused"),
            ("paused", "resume", "running"),
            ("running", "stop", "stopped")
        ]
        
        for previous, trigger, current in transitions:
            state_entry = StateEntry(
                _type=None,
                previous=previous,
                trigger=trigger,
                current=current,
                module_id="test_123",
                module_name="test_module", 
                timestamp=test_time
            )
            assert state_entry.previous == previous
            assert state_entry.trigger == trigger
            assert state_entry.current == current


class TestNewsEntry:
    """Test NewsEntry functionality"""
    
    def test_news_entry_creation(self):
        """Test NewsEntry creation"""
        test_time = datetime.now()
        test_uuid = uuid4()
        
        news_entry = NewsEntry(
            _type=None,
            level=NewsEntry.MESSAGE_LEVEL.INFO,
            message="Test news message",
            timestamp=test_time,
            uuid=test_uuid,
            module_name="test_module",
            module_id="test_module_123"
        )
        
        assert news_entry.level == NewsEntry.MESSAGE_LEVEL.INFO
        assert news_entry.message == "Test news message"
        assert news_entry.timestamp == test_time
        assert news_entry.uuid == test_uuid
        assert news_entry.module_name == "test_module"
        assert news_entry.module_id == "test_module_123"

    def test_news_entry_defaults(self):
        """Test NewsEntry with default values"""
        news_entry = NewsEntry(_type=None)
        
        assert news_entry.level == 2  # Default INFO level
        assert news_entry.message == ""
        assert news_entry.module_name == "N/A"
        assert isinstance(news_entry.timestamp, datetime)
        assert isinstance(news_entry.uuid, UUID)

    def test_news_message_levels(self):
        """Test different news message levels"""
        levels = [
            (NewsEntry.MESSAGE_LEVEL.ACTION, 0),
            (NewsEntry.MESSAGE_LEVEL.DEBUG, 1), 
            (NewsEntry.MESSAGE_LEVEL.INFO, 2),
            (NewsEntry.MESSAGE_LEVEL.HINT, 3),
            (NewsEntry.MESSAGE_LEVEL.WARNING, 4)
        ]
        
        for level_enum, level_value in levels:
            news_entry = NewsEntry(
                _type=None,
                level=level_enum,
                message=f"Test {level_enum.name} message"
            )
            assert news_entry.level == level_value
            assert level_enum.name.lower() in news_entry.message.lower()

    def test_news_entry_representation(self):
        """Test NewsEntry string representation"""
        news_entry = NewsEntry(
            _type=None,
            level=NewsEntry.MESSAGE_LEVEL.WARNING,
            message="Test warning message"
        )
        
        repr_str = repr(news_entry)
        assert "NewsEntry" in repr_str
        assert "level=4" in repr_str
        assert "message=Test warning message" in repr_str


class TestErrorEntry:
    """Test ErrorEntry functionality"""
    
    def test_error_entry_creation(self):
        """Test ErrorEntry creation"""
        test_uuid = uuid4()
        test_time = "2024-01-01T12:00:00"
        
        error_entry = ErrorEntry(
            _type=None,
            level=ErrorEntry.ERROR_LEVEL.MAJOR_FAULT,
            code=0x00001001,
            uuid=test_uuid,
            timestamp=test_time,
            description="Test error description",
            solution="Test error solution",
            miscellaneous="Additional error info",
            module_name="test_module",
            module_id="test_module_123"
        )
        
        assert error_entry.level == ErrorEntry.ERROR_LEVEL.MAJOR_FAULT
        assert error_entry.code == 0x00001001
        assert error_entry.uuid == test_uuid
        assert error_entry.timestamp == test_time
        assert error_entry.description == "Test error description"
        assert error_entry.solution == "Test error solution"
        assert error_entry.miscellaneous == "Additional error info"
        assert error_entry.module_name == "test_module"
        assert error_entry.module_id == "test_module_123"

    def test_error_entry_defaults(self):
        """Test ErrorEntry with default values"""
        error_entry = ErrorEntry(_type=None)
        
        assert error_entry.level == 0  # MINOR_FAULT
        assert error_entry.code == 0x00000000
        assert error_entry.timestamp == ""
        assert error_entry.description == ""
        assert error_entry.solution == ""
        assert error_entry.miscellaneous == ""
        assert error_entry.module_name == "N/A"
        assert error_entry.module_id == "N/A"
        assert isinstance(error_entry.uuid, UUID)

    def test_error_levels(self):
        """Test different error levels"""
        levels = [
            (ErrorEntry.ERROR_LEVEL.MINOR_FAULT, 0),
            (ErrorEntry.ERROR_LEVEL.MAJOR_FAULT, 1),
            (ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT, 2),
            (ErrorEntry.ERROR_LEVEL.EMERGENCY_FAULT, 3)
        ]
        
        for level_enum, level_value in levels:
            error_entry = ErrorEntry(
                _type=None,
                level=level_enum,
                description=f"Test {level_enum.name} error"
            )
            assert error_entry.level == level_value

    def test_error_entry_representation(self):
        """Test ErrorEntry string representation"""
        error_entry = ErrorEntry(
            _type=None,
            level=ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT,
            code=0x00001234,
            description="Critical system failure",
            solution="Restart required"
        )
        
        repr_str = repr(error_entry)
        assert "ErrorEntry" in repr_str
        assert "level=2" in repr_str
        assert "code=4660" in repr_str  # 0x1234 in decimal
        assert "description=Critical system failure" in repr_str
        assert "solution=Restart required" in repr_str


class TestModuleEntry:
    """Test ModuleEntry functionality"""
    
    def test_module_entry_creation(self):
        """Test ModuleEntry creation"""
        module_entry = ModuleEntry(
            uuid="12345678-1234-1234-1234-123456789abc",
            name="test_module",
            template="vyra_module_template", 
            description="A test module for testing",
            version="1.0.0"
        )
        
        assert module_entry.uuid == "12345678-1234-1234-1234-123456789abc"
        assert module_entry.name == "test_module"
        assert module_entry.template == "vyra_module_template"
        assert module_entry.description == "A test module for testing"
        assert module_entry.version == "1.0.0"

    def test_module_entry_to_dict(self):
        """Test ModuleEntry to_dict method"""
        module_entry = ModuleEntry(
            uuid="12345678-1234-1234-1234-123456789abc",
            name="test_module",
            template="vyra_module_template",
            description="A test module for testing", 
            version="1.2.3"
        )
        
        result = module_entry.to_dict()
        
        assert isinstance(result, dict)
        assert result["uuid"] == "12345678-1234-1234-1234-123456789abc"
        assert result["name"] == "test_module"
        assert result["template"] == "vyra_module_template"
        assert result["description"] == "A test module for testing"
        assert result["version"] == "1.2.3"

    def test_module_entry_gen_uuid(self):
        """Test ModuleEntry UUID generation"""
        generated_uuid = ModuleEntry.gen_uuid()
        
        assert isinstance(generated_uuid, str)
        assert len(generated_uuid) == 32  # hex UUID without dashes
        
        # Generate multiple UUIDs to ensure uniqueness
        uuids = set(ModuleEntry.gen_uuid() for _ in range(10))
        assert len(uuids) == 10  # All should be unique

    def test_module_entry_restructure_uuid(self):
        """Test ModuleEntry UUID restructuring"""
        hex_uuid = "12345678123412341234123456789abc"
        module_entry = ModuleEntry(
            uuid=hex_uuid,
            name="test_module",
            template="vyra_module_template",
            description="Test description",
            version="1.0.0"
        )
        
        restructured = module_entry.restructure_uuid()
        
        assert isinstance(restructured, UUID)
        assert restructured.hex == hex_uuid

    def test_module_entry_representation(self):
        """Test ModuleEntry string representation"""
        module_entry = ModuleEntry(
            uuid="12345678-1234-1234-1234-123456789abc",
            name="test_module",
            template="vyra_module_template",
            description="A test module",
            version="2.1.0"
        )
        
        repr_str = repr(module_entry)
        assert "ModuleEntry" in repr_str
        assert "uuid=12345678-1234-1234-1234-123456789abc" in repr_str
        assert "name=test_module" in repr_str
        assert "template=vyra_module_template" in repr_str
        assert "version=2.1.0" in repr_str


class TestFeederException:
    """Test FeederException functionality"""
    
    def test_feeder_exception_creation(self):
        """Test FeederException creation"""
        exception = FeederException("Test error message")
        
        assert exception.message == "Test error message"
        assert str(exception) == "Feeder: Test error message"

    def test_feeder_exception_inheritance(self):
        """Test FeederException inheritance"""
        exception = FeederException("Test message")
        
        assert isinstance(exception, Exception)
        assert isinstance(exception, FeederException)

    def test_feeder_exception_raise(self):
        """Test raising FeederException"""
        with pytest.raises(FeederException) as exc_info:
            raise FeederException("Critical feeder error")
        
        assert exc_info.value.message == "Critical feeder error"
        assert str(exc_info.value) == "Feeder: Critical feeder error"


class TestFunctionConfig:
    """Test Function Configuration classes"""
    
    def test_function_config_base_types(self):
        """Test FunctionConfigBaseTypes enumeration"""
        assert FunctionConfigBaseTypes.publisher.value == 'speaker'
        assert FunctionConfigBaseTypes.service.value == 'callable'
        assert FunctionConfigBaseTypes.action.value == 'job'

    def test_function_config_displaystyle(self):
        """Test FunctionConfigDisplaystyle"""
        display_style = FunctionConfigDisplaystyle()
        
        assert display_style.visible == False  # Default value
        assert display_style.published == False  # Default value

        display_style_custom = FunctionConfigDisplaystyle(
            visible=True,
            published=True
        )
        
        assert display_style_custom.visible == True
        assert display_style_custom.published == True

    def test_function_config_displaystyle_asdict(self):
        """Test FunctionConfigDisplaystyle asdict method"""
        display_style = FunctionConfigDisplaystyle(
            visible=True,
            published=False
        )
        
        result = display_style.asdict()
        
        assert isinstance(result, dict)
        assert result["visible"] == True
        assert result["published"] == False


if __name__ == "__main__":
    pytest.main([__file__])