"""
MQTT Client for vyra_base

High-level wrapper for MQTT communication.
Provides protocol-specific operations similar to RedisClient.

Example:
    >>> client = MqttClient(broker="mqtt.example.com", port=1883)
    >>> await client.connect()
    >>> await client.publish("topic/data", {"sensor": "temperature", "value": 23.5})
    >>> 
    >>> async def on_message(topic, payload):
    ...     print(f"Received: {topic} -> {payload}")
    >>> await client.subscribe("topic/#", on_message)
    >>> await client.close()
"""
from __future__ import annotations

import logging
import asyncio
from typing import Any, Optional, Callable, Dict
import json

from vyra_base.helper.logger import logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if paho-mqtt is available
try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.client import MQTTMessage
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


class MqttClient:
    """
    High-level MQTT Client wrapper.
    
    Features:
    - Publish/Subscribe messaging
    - QoS support (0, 1, 2)
    - TLS support
    - Automatic reconnection
    - JSON serialization
    
    Args:
        broker: MQTT broker hostname
        port: MQTT broker port (default: 1883, TLS: 8883)
        client_id: Optional client ID
        username: Optional username for authentication
        password: Optional password for authentication
        use_tls: Enable TLS/SSL
        ca_cert: Path to CA certificate for TLS
        keepalive: Keepalive interval in seconds
    
    Example:
        >>> client = MqttClient(
        ...     broker="mqtt.example.com",
        ...     port=8883,
        ...     use_tls=True,
        ...     username="user",
        ...     password="pass"
        ... )
        >>> await client.connect()
        >>> await client.publish("sensors/temp", {"value": 23.5}, qos=1)
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        broker: str,
        port: int = 1883,
        client_id: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        use_tls: bool = False,
        ca_cert: Optional[str] = None,
        keepalive: int = 60,
    ):
        """Initialize MQTT client."""
        if not MQTT_AVAILABLE:
            raise ImportError("paho-mqtt not installed. Install with: pip install paho-mqtt")
        
        self.broker = broker
        self.port = port
        self.client_id = client_id or f"vyra_mqtt_{id(self)}"
        self.username = username
        self.password = password
        self.use_tls = use_tls
        self.ca_cert = ca_cert
        self.keepalive = keepalive
        
        self._client: Optional[mqtt.Client] = None
        self._connected = False
        self._subscriptions: Dict[str, Callable] = {}
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish connection to MQTT broker."""
        try:
            logger.info(f"🔌 Connecting to MQTT broker: {self.broker}:{self.port}")
            
            # Create MQTT client
            self._client = mqtt.Client(client_id=self.client_id)
            
            # Set callbacks
            self._client.on_connect = self._on_connect
            self._client.on_disconnect = self._on_disconnect
            self._client.on_message = self._on_message
            
            # Set authentication
            if self.username and self.password:
                self._client.username_pw_set(self.username, self.password)
            
            # Set TLS
            if self.use_tls:
                import ssl
                self._client.tls_set(
                    ca_certs=self.ca_cert,
                    cert_reqs=ssl.CERT_REQUIRED,
                    tls_version=ssl.PROTOCOL_TLSv1_2
                )
            
            # Connect
            self._client.connect(self.broker, self.port, self.keepalive)
            self._client.loop_start()
            
            # Wait for connection
            await asyncio.sleep(0.5)
            
            self._connected = True
            logger.info(f"✅ Connected to MQTT broker: {self.broker}:{self.port}")
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to MQTT broker: {e}")
            raise
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT on_connect callback."""
        if rc == 0:
            logger.debug("MQTT connected successfully")
            self._connected = True
            
            # Resubscribe to topics
            for topic in self._subscriptions.keys():
                client.subscribe(topic)
        else:
            logger.error(f"MQTT connection failed with code {rc}")
            self._connected = False
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT on_disconnect callback."""
        logger.warning(f"MQTT disconnected with code {rc}")
        self._connected = False
    
    def _on_message(self, client, userdata, msg: MQTTMessage):
        """MQTT on_message callback."""
        topic = msg.topic
        
        # Find matching subscription
        for pattern, callback in self._subscriptions.items():
            if mqtt.topic_matches_sub(pattern, topic):
                try:
                    # Try to decode as JSON
                    try:
                        payload = json.loads(msg.payload.decode())
                    except:
                        payload = msg.payload.decode()
                    
                    # Call callback
                    asyncio.create_task(callback(topic, payload))
                except Exception as e:
                    logger.error(f"❌ Error in MQTT message callback: {e}")
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close connection to MQTT broker."""
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None
        
        self._connected = False
        logger.debug(f"MQTT connection closed: {self.broker}")
    
    async def _ensure_connected(self) -> mqtt.Client:
        """Ensure client is connected."""
        if not self._connected or self._client is None:
            await self.connect()
        
        if self._client is None:
            raise RuntimeError("MQTT connection failed")
        
        return self._client
    
    @ErrorTraceback.w_check_error_exist
    async def publish(
        self,
        topic: str,
        payload: Any,
        qos: int = 0,
        retain: bool = False
    ) -> None:
        """
        Publish message to MQTT topic.
        
        Args:
            topic: MQTT topic
            payload: Message payload (will be JSON-encoded if dict)
            qos: Quality of Service (0, 1, or 2)
            retain: Retain message flag
        """
        client = await self._ensure_connected()
        
        try:
            # Serialize payload
            if isinstance(payload, dict):
                message = json.dumps(payload)
            elif isinstance(payload, str):
                message = payload
            else:
                message = str(payload)
            
            # Publish
            result = client.publish(topic, message, qos=qos, retain=retain)
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                raise Exception(f"MQTT publish failed with code {result.rc}")
            
            logger.debug(f"📤 MQTT published to {topic}")
            
        except Exception as e:
            logger.error(f"❌ MQTT publish failed: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def subscribe(
        self,
        topic: str,
        callback: Callable,
        qos: int = 0
    ) -> None:
        """
        Subscribe to MQTT topic.
        
        Args:
            topic: MQTT topic (supports wildcards: +, #)
            callback: Async callback function(topic, payload)
            qos: Quality of Service (0, 1, or 2)
        """
        client = await self._ensure_connected()
        
        try:
            # Subscribe
            result, mid = client.subscribe(topic, qos=qos)
            
            if result != mqtt.MQTT_ERR_SUCCESS:
                raise Exception(f"MQTT subscribe failed with code {result}")
            
            # Store subscription
            self._subscriptions[topic] = callback
            
            logger.info(f"📥 MQTT subscribed to {topic}")
            
        except Exception as e:
            logger.error(f"❌ MQTT subscribe failed: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def unsubscribe(self, topic: str) -> None:
        """
        Unsubscribe from MQTT topic.
        
        Args:
            topic: MQTT topic
        """
        client = await self._ensure_connected()
        
        try:
            result, mid = client.unsubscribe(topic)
            
            if result != mqtt.MQTT_ERR_SUCCESS:
                raise Exception(f"MQTT unsubscribe failed with code {result}")
            
            # Remove subscription
            if topic in self._subscriptions:
                del self._subscriptions[topic]
            
            logger.info(f"🚫 MQTT unsubscribed from {topic}")
            
        except Exception as e:
            logger.error(f"❌ MQTT unsubscribe failed: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def health_check(self) -> bool:
        """
        Check if MQTT broker is reachable.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected and self._client is not None
