"""
Unit tests for TopicBuilder

Tests the generic naming convention for all transport protocols.
"""
import pytest
from vyra_base.com.core.topic_builder import (
    TopicBuilder,
    TopicComponents,
    InterfaceType,
    create_topic_builder,
    build_topic,
    parse_topic,
)


class TestTopicBuilder:
    """Test TopicBuilder functionality."""
    
    def test_initialization(self):
        """Test basic initialization."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        assert builder.module_name == "v2_modulemanager"
        assert builder.module_id == "abc123"
        assert builder.module_prefix == "v2_modulemanager_abc123"
    
    def test_invalid_module_name(self):
        """Test that invalid module names raise ValueError."""
        with pytest.raises(ValueError, match="Invalid module_name"):
            TopicBuilder("invalid-name!", "abc123")
    
    def test_invalid_module_id(self):
        """Test that invalid module IDs raise ValueError."""
        with pytest.raises(ValueError, match="Invalid module_id"):
            TopicBuilder("v2_modulemanager", "invalid-id!")
    
    def test_build_simple(self):
        """Test building simple topic name."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        topic = builder.build("get_modules")
        assert topic == "v2_modulemanager_abc123/get_modules"
    
    def test_build_with_subsection(self):
        """Test building topic with subsection."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        topic = builder.build("set_config", subsection="theme")
        assert topic == "v2_modulemanager_abc123/set_config/theme"
    
    def test_build_with_interface_type(self):
        """Test building with interface type (for logging)."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        # Should work and log (we can't test logging easily here)
        topic = builder.build("get_modules", interface_type=InterfaceType.SERVER)
        assert topic == "v2_modulemanager_abc123/get_modules"
    
    def test_build_with_namespace(self):
        """Test building with namespace."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        topic = builder.build("update", namespace="state")
        assert topic == "v2_modulemanager_abc123/state/update"
    
    def test_build_with_namespace_and_subsection(self):
        """Test building with namespace and subsection."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        topic = builder.build("temperature", namespace="data", subsection="celsius")
        assert topic == "v2_modulemanager_abc123/data/temperature/celsius"
    
    def test_invalid_function_name(self):
        """Test that invalid function names raise ValueError."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        with pytest.raises(ValueError, match="Invalid function_name"):
            builder.build("invalid-name!")
    
    def test_invalid_subsection(self):
        """Test that invalid subsections raise ValueError."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        with pytest.raises(ValueError, match="Invalid subsection"):
            builder.build("get_modules", subsection="invalid name!")
    
    def test_parse_simple(self):
        """Test parsing simple topic name."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        components = builder.parse("v2_modulemanager_abc123/get_modules")
        
        assert components.module_name == "v2_modulemanager"
        assert components.module_id == "abc123"
        assert components.function_name == "get_modules"
        assert components.subsection is None
    
    def test_parse_with_subsection(self):
        """Test parsing topic with subsection."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        components = builder.parse("v2_modulemanager_abc123/set_config/theme")
        
        assert components.module_name == "v2_modulemanager"
        assert components.module_id == "abc123"
        assert components.function_name == "set_config"
        assert components.subsection == "theme"
    
    def test_parse_complex_module_name(self):
        """Test parsing with complex module name containing underscores."""
        builder = TopicBuilder("v2_module_manager", "abc123")
        
        components = builder.parse("v2_module_manager_abc123/get_modules")
        
        assert components.module_name == "v2_module_manager"
        assert components.module_id == "abc123"
        assert components.function_name == "get_modules"
    
    def test_parse_invalid_format(self):
        """Test that invalid format raises ValueError."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        with pytest.raises(ValueError, match="Invalid topic format"):
            builder.parse("invalid_format")
    
    def test_parse_invalid_prefix(self):
        """Test that invalid prefix raises ValueError."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        with pytest.raises(ValueError, match="Invalid module prefix"):
            builder.parse("invalid/get_modules")
    
    def test_validate_correct_topic(self):
        """Test validation of correct topic."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        assert builder.validate("v2_modulemanager_abc123/get_modules") is True
    
    def test_validate_wrong_module(self):
        """Test validation fails for wrong module."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        assert builder.validate("v2_dashboard_xyz789/get_modules") is False
    
    def test_validate_invalid_format(self):
        """Test validation fails for invalid format."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        assert builder.validate("invalid_format") is False
    
    def test_topic_components_str(self):
        """Test string representation of TopicComponents."""
        components = TopicComponents(
            module_name="v2_modulemanager",
            module_id="abc123",
            function_name="get_modules",
            subsection="detail"
        )
        
        assert str(components) == "v2_modulemanager_abc123/get_modules/detail"
    
    def test_topic_components_without_subaction(self):
        """Test TopicComponents without subaction."""
        components = TopicComponents(
            module_name="v2_modulemanager",
            module_id="abc123",
            function_name="get_modules"
        )
        
        assert str(components) == "v2_modulemanager_abc123/get_modules"


class TestFactoryFunctions:
    """Test factory and convenience functions."""
    
    def test_create_topic_builder(self):
        """Test factory function."""
        builder = create_topic_builder("v2_modulemanager", "abc123")
        
        assert isinstance(builder, TopicBuilder)
        assert builder.module_name == "v2_modulemanager"
        assert builder.module_id == "abc123"
    
    def test_build_topic(self):
        """Test convenience function for building."""
        topic = build_topic("v2_modulemanager", "abc123", "get_modules")
        
        assert topic == "v2_modulemanager_abc123/get_modules"
    
    def test_build_topic_with_subsection(self):
        """Test convenience function with subsection."""
        topic = build_topic(
            "v2_modulemanager",
            "abc123",
            "set_config",
            subsection="theme"
        )
        
        assert topic == "v2_modulemanager_abc123/set_config/theme"
    
    def test_parse_topic(self):
        """Test convenience function for parsing."""
        components = parse_topic("v2_modulemanager_abc123/get_modules")
        
        assert components.module_name == "v2_modulemanager"
        assert components.module_id == "abc123"
        assert components.function_name == "get_modules"


class TestRealWorldExamples:
    """Test real-world usage examples."""
    
    def test_modulemanager_example(self):
        """Test v2_modulemanager example."""
        builder = TopicBuilder(
            "v2_modulemanager",
            "733256b82d6b48a48bc52b5ec73ebfff"
        )
        
        # Service names
        assert builder.build("get_modules") == \
            "v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/get_modules"
        assert builder.build("install_module") == \
            "v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/install_module"
        
        # With subsections
        assert builder.build("get_module_info", subsection="details") == \
            "v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/get_module_info/details"
    
    def test_dashboard_example(self):
        """Test v2_dashboard example."""
        builder = TopicBuilder(
            "v2_dashboard",
            "aef036f639d3486a985b65ee25df8fec"
        )
        
        # Configuration topics
        assert builder.build("theme", namespace="config") == \
            "v2_dashboard_aef036f639d3486a985b65ee25df8fec/config/theme"
        assert builder.build("language", namespace="config") == \
            "v2_dashboard_aef036f639d3486a985b65ee25df8fec/config/language"
    
    def test_sensor_example(self):
        """Test sensor module example."""
        builder = TopicBuilder("sensor_node", "def456")
        
        # Sensor data topics
        assert builder.build("temperature", namespace="data") == \
            "sensor_node_def456/data/temperature"
        assert builder.build("pressure", namespace="data") == \
            "sensor_node_def456/data/pressure"
        
        # With units
        assert builder.build("temperature", namespace="data", subsection="celsius") == \
            "sensor_node_def456/data/temperature/celsius"
    
    def test_roundtrip(self):
        """Test building and parsing roundtrip."""
        builder = TopicBuilder("v2_modulemanager", "abc123")
        
        # Build topic
        original = builder.build("get_modules", subsection="detailed")
        
        # Parse it back
        components = builder.parse(original)
        
        # Rebuild from components
        rebuilt = str(components)
        
        assert rebuilt == original
        assert rebuilt == "v2_modulemanager_abc123/get_modules/detailed"


class TestInterfaceTypes:
    """Test InterfaceType enum."""
    
    def test_interface_types(self):
        """Test all interface types."""
        assert InterfaceType.SERVER.value == "server"
        assert InterfaceType.PUBLISHER.value == "publisher"
        assert InterfaceType.ACTION_SERVER.value == "action_server"
    
    def test_builder_with_interface_types(self):
        """Test builder with different interface types."""
        builder = TopicBuilder("test_module", "test123")
        
        # All should produce same output (interface_type is just for logging)
        server_topic = builder.build("service", interface_type=InterfaceType.SERVER)
        publisher_topic = builder.build("topic", interface_type=InterfaceType.PUBLISHER)
        action_topic = builder.build("action", interface_type=InterfaceType.ACTION_SERVER)
        
        assert server_topic == "test_module_test123/service"
        assert publisher_topic == "test_module_test123/topic"
        assert action_topic == "test_module_test123/action"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
