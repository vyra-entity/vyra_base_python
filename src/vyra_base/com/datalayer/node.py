import re
import asyncio

from dataclasses import dataclass, field
from rclpy.node import Node


@dataclass
class NodeSettings:
    """
    Settings for a Vyra node.

    :ivar name: Name of the node.
    :vartype name: str
    :ivar parameters: Dictionary of parameters for the node.
    :vartype parameters: dict
    """
    name: str = 'Vyra_node'
    parameters: dict = field(default_factory=dict)


class VyraNode(Node):
    """
    Vyra node class.

    :param node_settings: NodeSettings object containing the node's settings.
    :type node_settings: NodeSettings
    """
    def __init__(self, node_settings: NodeSettings) -> None:
        super().__init__(node_settings.name)
        self._node_settings: NodeSettings = node_settings
        self.reload_event = asyncio.Event()

    def set_reload(self) -> None:
        """
        Set the reload event to notify that the node settings have changed.
        """
        self.reload_event.set()

    @property
    def node_settings(self) -> NodeSettings:
        """
        Get the node settings.

        :return: NodeSettings object containing the node's settings.
        :rtype: NodeSettings
        """
        return self._node_settings
    

class CheckerNode(Node):
    """
    Node to check the availability of other nodes.
    """
    def __init__(self):
        super().__init__('checker_node')

    def is_node_available(self, node_name: str) -> bool:
        """
        Check if a node with the given name is available.

        :param node_name: Name of the node to check.
        :type node_name: str
        :return: True if the node is available, False otherwise.
        :rtype: bool
        """
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        return any(name == node_name for name, _ in node_names_and_namespaces)

    @staticmethod
    def check_node_name(node_name: str) -> bool:
        """
        Check if the node name is valid.

        :param node_name: Name of the node to check.
        :type node_name: str
        :return: True if the node name is valid, False otherwise.
        :rtype: bool
        """
        return bool(re.match(r'^[a-zA-Z0-9_/]*$', node_name))