"""
OPC UA Callable Implementation

Request/response interface for OPC UA node operations.
"""

import logging
from typing import Dict, Any, Optional
from vyra_base.com.core.types import VyraCallable
from vyra_base.com.core.exceptions import CallableError

logger = logging.getLogger(__name__)


class OpcuaCallable(VyraCallable):
    """
    OPC UA Callable Interface
    
    Supports reading/writing OPC UA nodes and calling methods via RPC-style calls.
    
    Request Format:
        {
            "operation": "read"|"write"|"browse"|"call_method",
            "node_id": str,
            "value": Any (for write),
            "method_id": str (for call_method),
            "args": list (for call_method)
        }
    
    Response Format:
        {
            "success": bool,
            "data": Any,
            "error": str (if success=False)
        }
    
    Example:
        # Read node
        result = await callable.call({
            "operation": "read",
            "node_id": "ns=2;s=MyVariable"
        })
        # Returns: {"success": True, "data": 42}
        
        # Write node
        result = await callable.call({
            "operation": "write",
            "node_id": "ns=2;s=MyVariable",
            "value": 100
        })
        # Returns: {"success": True, "data": None}
        
        # Browse node
        result = await callable.call({
            "operation": "browse",
            "node_id": "ns=2;s=MyFolder"
        })
        # Returns: {"success": True, "data": [{...}, {...}]}
        
        # Call method
        result = await callable.call({
            "operation": "call_method",
            "node_id": "ns=2;s=MyObject",
            "method_id": "ns=2;s=MyMethod",
            "args": [1, 2, 3]
        })
        # Returns: {"success": True, "data": result_value}
    """
    
    def __init__(self, client: Any, name: str):
        self._client = client
        self._name = name
        self._initialized = False
    
    async def initialize(self) -> None:
        """Initialize callable (no-op for OPC UA)"""
        self._initialized = True
    
    async def shutdown(self) -> None:
        """Shutdown callable (no-op for OPC UA)"""
        self._initialized = False
    
    async def call(
        self,
        request: Dict[str, Any],
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """
        Execute OPC UA operation
        
        Args:
            request: Operation parameters
            timeout: Not used (OPC UA has its own timeout)
        
        Returns:
            Operation result
        """
        if not self._initialized:
            raise CallableError("Callable not initialized")
        
        operation = request.get("operation")
        if not operation:
            return {
                "success": False,
                "data": None,
                "error": "Missing 'operation' field"
            }
        
        try:
            node_id = request.get("node_id")
            if not node_id and operation != "get_root":
                return {
                    "success": False,
                    "data": None,
                    "error": "Missing 'node_id' field"
                }
            
            if operation == "read":
                # Read node value
                node = self._client.get_node(node_id)
                value = await node.read_value()
                
                # Convert to JSON-serializable
                if hasattr(value, "to_python"):
                    value = value.to_python()
                
                return {
                    "success": True,
                    "data": value,
                    "error": None
                }
            
            elif operation == "write":
                # Write node value
                value = request.get("value")
                if value is None:
                    return {
                        "success": False,
                        "data": None,
                        "error": "Missing 'value' field"
                    }
                
                node = self._client.get_node(node_id)
                await node.write_value(value)
                
                return {
                    "success": True,
                    "data": None,
                    "error": None
                }
            
            elif operation == "browse":
                # Browse node children
                node = self._client.get_node(node_id)
                children = await node.get_children()
                
                results = []
                for child in children:
                    browse_name = await child.read_browse_name()
                    node_class = await child.read_node_class()
                    
                    results.append({
                        "node_id": str(child.nodeid),
                        "browse_name": browse_name.Name,
                        "node_class": str(node_class)
                    })
                
                return {
                    "success": True,
                    "data": results,
                    "error": None
                }
            
            elif operation == "call_method":
                # Call OPC UA method
                method_id = request.get("method_id")
                if not method_id:
                    return {
                        "success": False,
                        "data": None,
                        "error": "Missing 'method_id' field"
                    }
                
                args = request.get("args", [])
                
                obj_node = self._client.get_node(node_id)
                method_node = self._client.get_node(method_id)
                
                result = await obj_node.call_method(method_node, *args)
                
                # Convert to JSON-serializable
                if hasattr(result, "to_python"):
                    result = result.to_python()
                
                return {
                    "success": True,
                    "data": result,
                    "error": None
                }
            
            elif operation == "get_root":
                # Get root node
                root = self._client.get_root_node()
                return {
                    "success": True,
                    "data": str(root.nodeid),
                    "error": None
                }
            
            elif operation == "get_objects":
                # Get objects node
                objects = self._client.get_objects_node()
                return {
                    "success": True,
                    "data": str(objects.nodeid),
                    "error": None
                }
            
            elif operation == "read_attributes":
                # Read multiple node attributes
                node = self._client.get_node(node_id)
                
                attributes = {}
                try:
                    attributes["browse_name"] = (
                        await node.read_browse_name()
                    ).Name
                except Exception:
                    pass
                
                try:
                    attributes["display_name"] = (
                        await node.read_display_name()
                    ).Text
                except Exception:
                    pass
                
                try:
                    attributes["node_class"] = str(
                        await node.read_node_class()
                    )
                except Exception:
                    pass
                
                try:
                    attributes["data_type"] = str(
                        await node.read_data_type()
                    )
                except Exception:
                    pass
                
                try:
                    attributes["value"] = await node.read_value()
                    if hasattr(attributes["value"], "to_python"):
                        attributes["value"] = attributes["value"].to_python()
                except Exception:
                    pass
                
                return {
                    "success": True,
                    "data": attributes,
                    "error": None
                }
            
            else:
                return {
                    "success": False,
                    "data": None,
                    "error": f"Unknown operation: {operation}"
                }
        
        except Exception as e:
            logger.error(f"OPC UA operation failed: {e}")
            return {
                "success": False,
                "data": None,
                "error": str(e)
            }
