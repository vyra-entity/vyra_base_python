"""
MQTT Speaker Implementation

Pub/Sub communication via MQTT with QoS support.
"""
import asyncio
import json
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import SpeakerError

logger = logging.getLogger(__name__)


class MqttSpeaker(VyraSpeaker):
    """
    MQTT Speaker for pub/sub messaging.
    
    Features:
    - Quality of Service (QoS 0, 1, 2)
    - Topic wildcards (+ and #)
    - Retained messages
    - Automatic JSON serialization
    - Message statistics
    
    Example:
        >>> # Publisher
        >>> speaker = MqttSpeaker("sensor/temp", client=mqtt_client)
        >>> await speaker.initialize()
        >>> await speaker.shout({"value": 23.5, "unit": "°C"})
        >>> 
        >>> # Subscriber
        >>> async def handle_temp(message):
        ...     print(f"Temperature: {message['value']} {message['unit']}")
        >>> 
        >>> speaker = MqttSpeaker(
        ...     "sensor/+",  # Wildcard
        ...     client=mqtt_client,
        ...     callback=handle_temp
        ... )
        >>> await speaker.initialize()
        >>> await speaker.listen(handle_temp)
    """
    
    def __init__(
        self,
        name: str,
        client: Any,  # paho.mqtt.client.Client
        callback: Optional[Callable] = None,
        qos: int = 1,
        retain: bool = False,
        **kwargs
    ):
        """
        Initialize MQTT speaker.
        
        Args:
            name: Topic name (can use wildcards: +, #)
            client: paho-mqtt Client instance
            callback: Callback for received messages
            qos: Quality of Service (0, 1, or 2)
            retain: Whether to retain published messages
            **kwargs: Additional metadata
        """
        super().__init__(name, protocol=ProtocolType.MQTT, **kwargs)
        self._client = client
        self.qos = qos
        self.retain = retain
        self._subscribed = False
        self._stats = {
            "publish_count": 0,
            "receive_count": 0,
            "qos": qos,
            "retain": retain,
        }
        self._callback = callback
    
    async def initialize(self) -> bool:
        """Initialize MQTT speaker."""
        if not self._client:
            raise SpeakerError("MQTT client not provided")
        
        # If callback provided, auto-subscribe
        if self._callback:
            await self.listen(self._callback)
        
        self._initialized = True
        logger.debug(f"✅ MQTT speaker initialized: {self.name}")
        return True
    
    async def shutdown(self) -> None:
        """Shutdown MQTT speaker."""
        if self._subscribed:
            try:
                self._client.unsubscribe(self.name)
                self._subscribed = False
                logger.debug(f"✅ Unsubscribed from {self.name}")
            except Exception as e:
                logger.error(f"❌ Unsubscribe error: {e}")
        
        self._initialized = False
    
    async def shout(self, message: Any) -> bool:
        """
        Publish message to MQTT topic.
        
        Args:
            message: Message to publish (dict/list serialized to JSON)
            
        Returns:
            bool: True if publish successful
            
        Raises:
            SpeakerError: If not initialized or publish fails
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        try:
            # Serialize message
            if isinstance(message, (dict, list)):
                payload = json.dumps(message)
            else:
                payload = str(message)
            
            # Publish to topic
            result = self._client.publish(
                topic=self.name,
                payload=payload,
                qos=self.qos,
                retain=self.retain
            )
            
            # Check if published successfully
            if result.rc == 0:
                self._stats["publish_count"] += 1
                logger.debug(f"📢 Published to '{self.name}': {payload[:100]}...")
                return True
            else:
                logger.error(f"❌ Publish failed with code {result.rc}")
                return False
                
        except Exception as e:
            raise SpeakerError(f"MQTT publish to '{self.name}' failed: {e}")
    
    async def listen(self, callback: Callable) -> None:
        """
        Subscribe to MQTT topic.
        
        Args:
            callback: Async callback function(message) to handle received messages
            
        Raises:
            SpeakerError: If subscription fails
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        try:
            # Define message handler wrapper
            def _on_message(client, userdata, msg):
                """Internal handler that wraps async callback."""
                try:
                    # Parse JSON if possible
                    try:
                        payload = json.loads(msg.payload.decode())
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        payload = msg.payload.decode()
                    
                    # Update stats
                    self._stats["receive_count"] += 1
                    
                    # Call user callback
                    if asyncio.iscoroutinefunction(callback):
                        # Schedule async callback
                        asyncio.create_task(callback(payload))
                    else:
                        # Call sync callback directly
                        callback(payload)
                        
                except Exception as e:
                    logger.error(f"❌ Message handler error: {e}")
            
            # Subscribe to topic
            result, mid = self._client.subscribe(self.name, qos=self.qos)
            
            if result == 0:
                # Set message callback for this topic
                self._client.message_callback_add(self.name, _on_message)
                self._subscribed = True
                logger.info(f"✅ Subscribed to '{self.name}' (QoS {self.qos})")
            else:
                raise SpeakerError(f"Subscribe failed with code {result}")
                
        except Exception as e:
            raise SpeakerError(f"MQTT subscribe to '{self.name}' failed: {e}")
    
    def get_statistics(self) -> dict:
        """
        Get speaker statistics.
        
        Returns:
            dict: Statistics (publish_count, receive_count, qos, retain)
        """
        return self._stats.copy()
