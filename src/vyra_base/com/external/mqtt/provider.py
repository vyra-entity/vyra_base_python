"""
MQTT Protocol Provider

Implements AbstractProtocolProvider for MQTT pub/sub messaging.
Uses paho-mqtt with optional TLS and authentication.
"""
import logging
from typing import Any, Callable, Optional, Dict

from vyra_base.com.core.types import (
    ProtocolType,
    VyraCallable,
    VyraSpeaker,
    VyraJob,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

logger = logging.getLogger(__name__)

# Check if paho-mqtt is available
try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.client import Client as mqtt_client
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    mqtt_client = None  # type: ignore
    class MockMQTT:
        def __getattr__(self, name):
            raise ImportError("paho-mqtt is not installed on this system.")
    mqtt = MockMQTT() # type: ignore
    logger.warning(
        "⚠️ MQTT not available. Install with: pip install paho-mqtt"
    )


class MqttProvider(AbstractProtocolProvider):
    """
    Protocol provider for MQTT pub/sub messaging.
    
    Features:
    - Quality of Service (QoS 0, 1, 2)
    - Topic wildcards (+ for single level, # for multi-level)
    - Retained messages
    - Last Will and Testament (LWT)
    - TLS encryption
    - Username/password authentication
    
    Example:
        >>> # Initialize provider
        >>> provider = MqttProvider(
        ...     broker="localhost",
        ...     port=1883,
        ...     client_id="vyra_module_1",
        ...     username="user",
        ...     password="pass"
        ... )
        >>> await provider.initialize()
        >>> 
        >>> # Create speaker for pub/sub
        >>> async def handle_message(message):
        ...     print(f"Received: {message}")
        >>> 
        >>> speaker = await provider.create_speaker(
        ...     "sensor/temperature",
        ...     callback=handle_message
        ... )
        >>> 
        >>> # Publish message
        >>> await speaker.shout({"value": 23.5, "unit": "°C"})
    """
    
    def __init__(
        self,
        protocol: ProtocolType = ProtocolType.MQTT,
        broker: str = "localhost",
        port: int = 1883,
        client_id: str = "vyra_mqtt_client",
        username: Optional[str] = None,
        password: Optional[str] = None,
        use_tls: bool = False,
        tls_ca_certs: Optional[str] = None,
        keepalive: int = 60,
        **kwargs
    ):
        """
        Initialize MQTT provider.
        
        Args:
            protocol: Protocol type (must be MQTT)
            broker: MQTT broker hostname/IP
            port: MQTT broker port (1883 default, 8883 for TLS)
            client_id: Unique client identifier
            username: Optional username for authentication
            password: Optional password for authentication
            use_tls: Enable TLS encryption
            tls_ca_certs: Path to CA certificate file
            keepalive: Keep-alive interval in seconds
            **kwargs: Additional mqtt.Client parameters
        """
        super().__init__(protocol)
        self.broker = broker
        self.port = port
        self.client_id = client_id
        self.username = username
        self.password = password
        self.use_tls = use_tls
        self.tls_ca_certs = tls_ca_certs
        self.keepalive = keepalive
        self._client: Optional[mqtt_client] = None
        self._connected = False
    
    async def check_availability(self) -> bool:
        """
        Check if MQTT is available.
        
        Returns:
            bool: True if paho-mqtt can be imported
        """
        if not MQTT_AVAILABLE:
            logger.warning("❌ MQTT not available - install paho-mqtt")
            return False
        
        logger.debug("✅ MQTT available")
        return True
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize MQTT client and connect to broker.
        
        Args:
            config: Optional configuration overrides
            
        Returns:
            bool: True if connection successful
            
        Raises:
            ProviderError: If connection fails
        """
        if not await self.check_availability():
            raise ProtocolUnavailableError("MQTT not available")
        
        try:
            # Apply config overrides
            if config:
                self.broker = config.get("broker", self.broker)
                self.port = config.get("port", self.port)
                self.username = config.get("username", self.username)
                self.password = config.get("password", self.password)
                self.use_tls = config.get("use_tls", self.use_tls)
            
            # Create MQTT client
            self._client = mqtt.Client(client_id=self.client_id)
            
            if self._client is None:
                raise TypeError("self._client is None")
            
            # Set authentication
            if self.username and self.password:
                self._client.username_pw_set(self.username, self.password)
            
            # Configure TLS
            if self.use_tls:
                self._client.tls_set(ca_certs=self.tls_ca_certs)
            
            # Set callbacks
            self._client.on_connect = self._on_connect
            self._client.on_disconnect = self._on_disconnect
            self._client.on_message = self._on_message
            
            # Connect to broker
            self._client.connect(self.broker, self.port, self.keepalive)
            self._client.loop_start()
            
            # Wait for connection (max 5 seconds)
            import asyncio
            for _ in range(50):
                if self._connected:
                    break
                await asyncio.sleep(0.1)
            
            if not self._connected:
                raise ProviderError(f"Failed to connect to MQTT broker {self.broker}:{self.port}")
            
            logger.info(f"✅ MQTT connected to {self.broker}:{self.port}")
            self._initialized = True
            return True
            
        except Exception as e:
            raise ProviderError(f"MQTT initialization failed: {e}")
    
    def _on_connect(self, client, userdata, flags, rc):
        """Callback when connected to broker."""
        if rc == 0:
            self._connected = True
            logger.debug("✅ MQTT connection established")
        else:
            logger.error(f"❌ MQTT connection failed with code: {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        """Callback when disconnected from broker."""
        self._connected = False
        if rc != 0:
            logger.warning(f"⚠️ MQTT unexpected disconnect: {rc}")
    
    def _on_message(self, client, userdata, msg):
        """Callback for received messages (handled by speakers)."""
        pass  # Individual speakers handle their own topics
    
    async def shutdown(self) -> None:
        """Shutdown MQTT client."""
        if self._client:
            try:
                self._client.loop_stop()
                self._client.disconnect()
                logger.info("✅ MQTT provider shutdown complete")
            except Exception as e:
                logger.error(f"❌ MQTT shutdown error: {e}")
        
        self._initialized = False
        self._connected = False
        self._client = None
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create MQTT Callable.
        
        Note:
            MQTT is pub/sub only, no request/response pattern.
            Use create_speaker() instead or consider gRPC/Redis for RPC.
        """
        raise NotImplementedError(
            "MQTT Callable not supported. "
            "MQTT is pub/sub only. Use create_speaker() or consider gRPC/Redis for RPC."
        )
    
    async def create_speaker(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create MQTT Speaker for pub/sub.
        
        Args:
            name: Topic name
            callback: Callback for received messages
            **kwargs: Additional parameters (qos, retain)
            
        Returns:
            VyraSpeaker: MQTT speaker instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        from vyra_base.com.external.mqtt.speaker import MqttSpeaker
        
        speaker = MqttSpeaker(
            name=name,
            client=self._client,
            callback=callback,
            **kwargs
        )
        
        await speaker.initialize()
        return speaker
    
    async def create_job(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create MQTT Job.
        
        Note:
            MQTT doesn't support job/task patterns natively.
            Consider using Job patterns with Redis or gRPC instead.
        """
        raise NotImplementedError(
            "MQTT Job not supported. "
            "Consider Redis Streams or gRPC streaming for job/task patterns."
        )
    
    def get_client(self) -> mqtt_client:
        """
        Get underlying MQTT client for advanced operations.
        
        Returns:
            paho.mqtt.client.Client instance
            
        Raises:
            ProviderError: If provider not initialized
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        return self._client
