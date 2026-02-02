"""
OPC UA Speaker Implementation

Subscription interface for monitoring OPC UA node changes.
"""

import logging
from typing import Dict, Any, Optional, Callable
from vyra_base.com.core.types import VyraSpeaker
from vyra_base.com.core.exceptions import SpeakerError

logger = logging.getLogger(__name__)


class OpcuaSpeaker(VyraSpeaker):
    """
    OPC UA Speaker Interface
    
    Subscribes to OPC UA node data changes and calls callback on updates.
    
    Args:
        client: OPC UA client instance
        subscription: OPC UA subscription instance
        node_id: Node ID to monitor
        callback: Callback function for data changes
    
    Callback Format:
        callback({
            "node_id": str,
            "value": Any,
            "timestamp": float,
            "status": str
        })
    
    Example:
        def on_change(data):
            print(f"Node {data['node_id']} changed to {data['value']}")
        
        speaker = await provider.create_speaker(
            "ns=2;s=Temperature",
            callback=on_change
        )
    """
    
    def __init__(
        self,
        client: Any,
        subscription: Any,
        node_id: str,
        callback: Optional[Callable] = None
    ):
        self._client = client
        self._subscription = subscription
        self._node_id = node_id
        self._callback = callback
        self._handle = None
        self._initialized = False
    
    async def initialize(self) -> None:
        """Initialize subscription to node"""
        if self._initialized:
            return
        
        if not self._callback:
            logger.warning(
                f"OPC UA speaker for {self._node_id} has no callback"
            )
            self._initialized = True
            return
        
        try:
            # Get node
            node = self._client.get_node(self._node_id)
            
            # Create subscription handler
            class SubscriptionHandler:
                def __init__(self, callback):
                    self.callback = callback
                
                def datachange_notification(self, node, val, data):
                    """Called when monitored data changes"""
                    try:
                        # Convert value
                        if hasattr(val, "to_python"):
                            val = val.to_python()
                        
                        # Call user callback
                        self.callback({
                            "node_id": str(node.nodeid),
                            "value": val,
                            "timestamp": data.monitored_item.Value.SourceTimestamp.timestamp(),
                            "status": str(data.monitored_item.Value.StatusCode)
                        })
                    except Exception as e:
                        logger.error(f"OPC UA callback error: {e}")
            
            # Subscribe to node
            handler = SubscriptionHandler(self._callback)
            self._handle = await self._subscription.subscribe_data_change(
                node,
                handler
            )
            
            logger.info(f"✅ Subscribed to OPC UA node: {self._node_id}")
            self._initialized = True
        
        except Exception as e:
            raise SpeakerError(f"Failed to subscribe to {self._node_id}: {e}")
    
    async def shutdown(self) -> None:
        """Unsubscribe from node"""
        if self._handle:
            try:
                await self._subscription.unsubscribe(self._handle)
                logger.info(f"Unsubscribed from OPC UA node: {self._node_id}")
            except Exception as e:
                logger.error(f"Error unsubscribing: {e}")
            
            self._handle = None
        
        self._initialized = False
    
    async def shout(self, message: Dict[str, Any]) -> None:
        """
        Write value to subscribed node
        
        Args:
            message: {"value": Any}
        
        Note:
            This is a convenience method for writing to the monitored node.
            Not a typical pub/sub "shout" operation.
        """
        if not self._initialized:
            raise SpeakerError("Speaker not initialized")
        
        value = message.get("value")
        if value is None:
            raise SpeakerError("Missing 'value' in message")
        
        try:
            node = self._client.get_node(self._node_id)
            await node.write_value(value)
            logger.debug(f"Wrote value to {self._node_id}: {value}")
        
        except Exception as e:
            raise SpeakerError(f"Failed to write to {self._node_id}: {e}")
