"""
Topic Builder for Vyra Communication Framework

Provides generic naming conventions for all transport protocols.
Ensures consistent topic/service/key naming across ROS2, Zenoh, Redis, and UDS.

Naming Convention:
    <module_name>_<module_id>/<function_name>{/<optional_subaction>}

Examples:
    - v2_modulemanager_abc123/get_modules
    - v2_dashboard_xyz789/set_config/theme
    - sensor_node_def456/data/temperature
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from typing import Optional
from enum import Enum

logger = logging.getLogger(__name__)


class InterfaceType(Enum):
    """Type of communication interface."""
    CALLABLE = "callable"  # Service/Request-Response (ROS2 Service, Zenoh Query, Redis Callable)
    SPEAKER = "speaker"    # Pub/Sub (ROS2 Topic, Zenoh Pub/Sub, Redis Pub/Sub)
    JOB = "job"           # Action/Long-running task (ROS2 Action)


@dataclass
class TopicComponents:
    """Components of a topic name."""
    module_name: str
    module_id: str
    function_name: str
    subaction: Optional[str] = None
    
    def __str__(self) -> str:
        """String representation of topic components."""
        return f"{self.module_name}_{self.module_id}/{self.function_name}" + \
               (f"/{self.subaction}" if self.subaction else "")


class TopicBuilder:
    """
    Builder for consistent topic/service names across transport protocols.
    
    Features:
        - Validates module names and IDs
        - Ensures naming consistency
        - Supports optional subactions
        - Protocol-agnostic design
    
    Examples:
        >>> builder = TopicBuilder("v2_modulemanager", "abc123")
        >>> builder.build("get_modules")
        'v2_modulemanager_abc123/get_modules'
        
        >>> builder.build("set_config", subaction="theme")
        'v2_modulemanager_abc123/set_config/theme'
        
        >>> builder.build_with_prefix("state", function_name="update")
        'v2_modulemanager_abc123/state/update'
    """
    
    # Validation patterns
    MODULE_NAME_PATTERN = re.compile(r"^[a-zA-Z0-9_]+$")
    MODULE_ID_PATTERN = re.compile(r"^[a-zA-Z0-9_]+$")
    FUNCTION_NAME_PATTERN = re.compile(r"^[a-zA-Z0-9_]+$")
    SUBACTION_PATTERN = re.compile(r"^[a-zA-Z0-9_/]+$")
    
    def __init__(self, module_name: str, module_id: str):
        """
        Initialize topic builder.
        
        Args:
            module_name: Name of the module (e.g., "v2_modulemanager")
            module_id: Unique module instance ID (e.g., "abc123" or full hash)
            
        Raises:
            ValueError: If module_name or module_id contains invalid characters
        """
        if not self.MODULE_NAME_PATTERN.match(module_name):
            raise ValueError(
                f"Invalid module_name '{module_name}'. "
                f"Must contain only alphanumeric characters and underscores."
            )
        
        if not self.MODULE_ID_PATTERN.match(module_id):
            raise ValueError(
                f"Invalid module_id '{module_id}'. "
                f"Must contain only alphanumeric characters and underscores."
            )
        
        self.module_name = module_name
        self.module_id = module_id
        self._module_prefix = f"{module_name}_{module_id}"
        
        logger.debug(f"TopicBuilder initialized: {self._module_prefix}")
    
    @property
    def module_prefix(self) -> str:
        """Get the module prefix (<module_name>_<module_id>)."""
        return self._module_prefix
    
    def build(
        self,
        function_name: str,
        subaction: Optional[str] = None,
        interface_type: Optional[InterfaceType] = None
    ) -> str:
        """
        Build a topic/service name.
        
        Args:
            function_name: Name of the function/interface
            subaction: Optional subaction or subcategory
            interface_type: Type of interface (for logging/validation)
            
        Returns:
            Complete topic/service name
            
        Examples:
            >>> builder.build("get_modules")
            'v2_modulemanager_abc123/get_modules'
            
            >>> builder.build("set_config", subaction="theme")
            'v2_modulemanager_abc123/set_config/theme'
        """
        if not self.FUNCTION_NAME_PATTERN.match(function_name):
            raise ValueError(
                f"Invalid function_name '{function_name}'. "
                f"Must contain only alphanumeric characters and underscores."
            )
        
        if subaction and not self.SUBACTION_PATTERN.match(subaction):
            raise ValueError(
                f"Invalid subaction '{subaction}'. "
                f"Must contain only alphanumeric characters, underscores, and slashes."
            )
        
        components = TopicComponents(
            module_name=self.module_name,
            module_id=self.module_id,
            function_name=function_name,
            subaction=subaction
        )
        
        topic_name = str(components)
        
        if interface_type:
            logger.debug(
                f"Built {interface_type.value} topic: {topic_name} "
                f"(module: {self.module_prefix})"
            )
        
        return topic_name
    
    def build_with_prefix(
        self,
        prefix: str,
        function_name: str,
        subaction: Optional[str] = None,
        interface_type: Optional[InterfaceType] = None
    ) -> str:
        """
        Build a topic name with an additional prefix level.
        
        Useful for categorizing interfaces (e.g., state/, data/, control/).
        
        Args:
            prefix: Category prefix (becomes part of the path)
            function_name: Name of the function/interface
            subaction: Optional subaction
            interface_type: Type of interface (for logging/validation)
            
        Returns:
            Complete topic name with prefix
            
        Examples:
            >>> builder.build_with_prefix("state", "update")
            'v2_modulemanager_abc123/state/update'
            
            >>> builder.build_with_prefix("data", "temperature", subaction="celsius")
            'v2_modulemanager_abc123/data/temperature/celsius'
        """
        if not self.FUNCTION_NAME_PATTERN.match(prefix):
            raise ValueError(
                f"Invalid prefix '{prefix}'. "
                f"Must contain only alphanumeric characters and underscores."
            )
        
        if not self.FUNCTION_NAME_PATTERN.match(function_name):
            raise ValueError(
                f"Invalid function_name '{function_name}'. "
                f"Must contain only alphanumeric characters and underscores."
            )
        
        # Build the complete path directly
        topic = f"{self._module_prefix}/{prefix}/{function_name}"
        if subaction:
            if not self.SUBACTION_PATTERN.match(subaction):
                raise ValueError(
                    f"Invalid subaction '{subaction}'. "
                    f"Must contain only alphanumeric characters, underscores, and slashes."
                )
            topic = f"{topic}/{subaction}"
        
        if interface_type:
            logger.debug(
                f"Built {interface_type.value} topic with prefix: {topic} "
                f"(module: {self.module_prefix})"
            )
        
        return topic
    
    def parse(self, topic_name: str) -> TopicComponents:
        """
        Parse a topic name back into components.
        
        Args:
            topic_name: Complete topic name to parse
            
        Returns:
            TopicComponents with parsed values
            
        Raises:
            ValueError: If topic name doesn't match expected format
            
        Examples:
            >>> builder.parse("v2_modulemanager_abc123/get_modules")
            TopicComponents(module_name='v2_modulemanager', module_id='abc123', 
                          function_name='get_modules', subaction=None)
        """
        # Expected format: <module_name>_<module_id>/<function_name>{/<subaction>}
        parts = topic_name.split("/", 1)
        
        if len(parts) < 2:
            raise ValueError(
                f"Invalid topic format '{topic_name}'. "
                f"Expected format: <module_name>_<module_id>/<function_name>"
            )
        
        # Parse module prefix
        module_prefix = parts[0]
        if "_" not in module_prefix:
            raise ValueError(
                f"Invalid module prefix '{module_prefix}'. "
                f"Expected format: <module_name>_<module_id>"
            )
        
        # Split module_name and module_id (last underscore is separator)
        module_parts = module_prefix.rsplit("_", 1)
        if len(module_parts) != 2:
            raise ValueError(
                f"Could not parse module_name and module_id from '{module_prefix}'"
            )
        
        module_name, module_id = module_parts
        
        # Parse function and optional subaction
        function_parts = parts[1].split("/", 1)
        function_name = function_parts[0]
        subaction = function_parts[1] if len(function_parts) > 1 else None
        
        return TopicComponents(
            module_name=module_name,
            module_id=module_id,
            function_name=function_name,
            subaction=subaction
        )
    
    def validate(self, topic_name: str) -> bool:
        """
        Validate if a topic name follows the naming convention.
        
        Args:
            topic_name: Topic name to validate
            
        Returns:
            True if valid, False otherwise
        """
        try:
            components = self.parse(topic_name)
            return (
                components.module_name == self.module_name and
                components.module_id == self.module_id
            )
        except ValueError:
            return False


def create_topic_builder(module_name: str, module_id: str) -> TopicBuilder:
    """
    Factory function to create a TopicBuilder instance.
    
    Args:
        module_name: Name of the module
        module_id: Unique module instance ID
        
    Returns:
        Configured TopicBuilder instance
    """
    return TopicBuilder(module_name, module_id)


# Convenience functions for quick topic building without creating a builder instance
def build_topic(
    module_name: str,
    module_id: str,
    function_name: str,
    subaction: Optional[str] = None
) -> str:
    """
    Build a topic name without creating a builder instance.
    
    Convenience function for one-off topic generation.
    
    Args:
        module_name: Name of the module
        module_id: Unique module instance ID
        function_name: Name of the function/interface
        subaction: Optional subaction
        
    Returns:
        Complete topic name
    """
    builder = TopicBuilder(module_name, module_id)
    return builder.build(function_name, subaction)


def parse_topic(topic_name: str) -> TopicComponents:
    """
    Parse a topic name without a builder instance.
    
    Args:
        topic_name: Topic name to parse
        
    Returns:
        TopicComponents with parsed values
    """
    # Extract module info from topic
    parts = topic_name.split("/", 1)
    if len(parts) < 2:
        raise ValueError(f"Invalid topic format: {topic_name}")
    
    module_parts = parts[0].rsplit("_", 1)
    if len(module_parts) != 2:
        raise ValueError(f"Could not parse module info from: {parts[0]}")
    
    builder = TopicBuilder(module_parts[0], module_parts[1])
    return builder.parse(topic_name)
