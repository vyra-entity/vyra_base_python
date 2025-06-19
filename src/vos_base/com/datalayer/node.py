import rclpy
from dataclasses import dataclass, field
from rclpy.node import Node

@dataclass
class NodeSettings:
    name: str = 'VOS_node'
    parameters: dict = field(default_factory=dict)


class VOSNode(Node):
    def __init__(self, node_settings: NodeSettings) -> None:
        super().__init__(node_settings.name)
        
        self._node_settings: NodeSettings = node_settings

    @property
    def node_settings(self) -> NodeSettings:
        """
        Get the node settings.
        :return: NodeSettings object containing the node's settings.
        """
        return self._node_settings
    

class CheckerNode(Node):
    def __init__(self):
        super().__init__('checker_node')

    def is_node_available(self, node_name: str) -> bool:
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        return any(name == node_name for name, _ in node_names_and_namespaces)
