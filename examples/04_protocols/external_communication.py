#!/usr/bin/env python3
"""
External Communication Example

Demonstrates how to use VYRA communication layer with external systems:
- External REST API integration
- External gRPC service communication
- External MQTT broker connection
- HTTP webhook handling

This shows how VYRA modules can interact with non-VYRA systems.
"""
import asyncio
import logging
from typing import Any, Dict

# External libraries (optional, install as needed)
try:
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False
    print("⚠️ aiohttp not available. Install: pip install aiohttp")

try:
    import grpc
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False
    print("⚠️ grpc not available. Install: pip install grpcio")

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("⚠️ paho-mqtt not available. Install: pip install paho-mqtt")

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ============================================================================
# Example 1: REST API Integration
# ============================================================================

class ExternalRESTClient:
    """Wrapper for external REST API communication."""
    
    def __init__(self, base_url: str):
        self.base_url = base_url
        self.session = None
    
    async def initialize(self):
        """Initialize HTTP session."""
        if not AIOHTTP_AVAILABLE:
            raise ImportError("aiohttp required for REST communication")
        
        self.session = aiohttp.ClientSession()
        logger.info(f"✅ REST client initialized: {self.base_url}")
    
    async def get_data(self, endpoint: str) -> Dict[str, Any]:
        """
        GET request to external API.
        
        Example:
            data = await client.get_data("/api/sensors/temperature")
        """
        url = f"{self.base_url}{endpoint}"
        
        async with self.session.get(url) as response:
            response.raise_for_status()
            data = await response.json()
            logger.info(f"📥 Received from {endpoint}: {data}")
            return data
    
    async def post_data(self, endpoint: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        POST request to external API.
        
        Example:
            result = await client.post_data("/api/commands", {"action": "start"})
        """
        url = f"{self.base_url}{endpoint}"
        
        async with self.session.post(url, json=payload) as response:
            response.raise_for_status()
            data = await response.json()
            logger.info(f"📤 Sent to {endpoint}: {payload} → Response: {data}")
            return data
    
    async def shutdown(self):
        """Close HTTP session."""
        if self.session:
            await self.session.close()
            logger.info("🛑 REST client shutdown")


# ============================================================================
# Example 2: MQTT Broker Integration
# ============================================================================

class ExternalMQTTClient:
    """Wrapper for external MQTT broker communication."""
    
    def __init__(self, broker: str, port: int = 1883):
        self.broker = broker
        self.port = port
        self.client = None
        self.loop = None
        self.message_queue = asyncio.Queue()
    
    async def initialize(self):
        """Initialize MQTT client."""
        if not MQTT_AVAILABLE:
            raise ImportError("paho-mqtt required for MQTT communication")
        
        self.loop = asyncio.get_event_loop()
        self.client = mqtt.Client()
        
        # Set callbacks
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        
        # Connect to broker
        await self.loop.run_in_executor(
            None,
            lambda: self.client.connect(self.broker, self.port, 60)
        )
        
        # Start network loop in background
        self.client.loop_start()
        
        logger.info(f"✅ MQTT client connected: {self.broker}:{self.port}")
    
    def _on_connect(self, client, userdata, flags, rc):
        """Callback when connected to broker."""
        if rc == 0:
            logger.info("✅ MQTT connected successfully")
        else:
            logger.error(f"❌ MQTT connection failed: {rc}")
    
    def _on_message(self, client, userdata, msg):
        """Callback when message received."""
        # Put message in async queue
        asyncio.run_coroutine_threadsafe(
            self.message_queue.put((msg.topic, msg.payload)),
            self.loop
        )
    
    async def publish(self, topic: str, payload: str):
        """
        Publish message to MQTT topic.
        
        Example:
            await mqtt.publish("factory/sensor/temperature", "23.5")
        """
        result = await self.loop.run_in_executor(
            None,
            lambda: self.client.publish(topic, payload)
        )
        logger.info(f"📤 Published to {topic}: {payload}")
        return result
    
    async def subscribe(self, topic: str):
        """
        Subscribe to MQTT topic.
        
        Example:
            await mqtt.subscribe("factory/commands/#")
        """
        await self.loop.run_in_executor(
            None,
            lambda: self.client.subscribe(topic)
        )
        logger.info(f"👂 Subscribed to topic: {topic}")
    
    async def receive_message(self, timeout: float = None):
        """
        Wait for and receive next message.
        
        Returns:
            (topic, payload) tuple or None if timeout
        """
        try:
            if timeout:
                return await asyncio.wait_for(
                    self.message_queue.get(),
                    timeout=timeout
                )
            else:
                return await self.message_queue.get()
        except asyncio.TimeoutError:
            return None
    
    async def shutdown(self):
        """Disconnect from broker."""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            logger.info("🛑 MQTT client disconnected")


# ============================================================================
# Example 3: Integration with VYRA Module
# ============================================================================

class ExternalBridgeComponent:
    """
    VYRA component that bridges external systems with VYRA communication.
    
    This component:
    - Receives data from external REST API
    - Publishes to VYRA via Speaker
    - Receives VYRA commands via Callable
    - Sends commands to external MQTT broker
    """
    
    def __init__(self):
        self.rest_client = None
        self.mqtt_client = None
        self.running = False
    
    async def initialize(
        self,
        rest_api_url: str,
        mqtt_broker: str,
        mqtt_port: int = 1883
    ):
        """Initialize external connections."""
        # Initialize REST client
        self.rest_client = ExternalRESTClient(rest_api_url)
        await self.rest_client.initialize()
        
        # Initialize MQTT client
        self.mqtt_client = ExternalMQTTClient(mqtt_broker, mqtt_port)
        await self.mqtt_client.initialize()
        
        # Subscribe to command topic
        await self.mqtt_client.subscribe("vyra/commands/#")
        
        logger.info("✅ External bridge initialized")
    
    async def run_polling_loop(self, vyra_speaker):
        """
        Poll external REST API and publish to VYRA.
        
        This would be a background task in a real VYRA module.
        """
        self.running = True
        
        while self.running:
            try:
                # Get data from external API
                data = await self.rest_client.get_data("/api/sensors/latest")
                
                # Publish to VYRA network via Speaker
                await vyra_speaker.shout({
                    "source": "external_api",
                    "data": data,
                    "timestamp": asyncio.get_event_loop().time()
                })
                
                logger.info(f"🔄 Bridged REST → VYRA: {data}")
                
            except Exception as e:
                logger.error(f"❌ Polling error: {e}")
            
            await asyncio.sleep(5.0)  # Poll every 5 seconds
    
    async def run_mqtt_listener(self, vyra_speaker):
        """
        Listen to MQTT and forward to VYRA.
        
        This would be a background task in a real VYRA module.
        """
        self.running = True
        
        while self.running:
            try:
                # Wait for MQTT message
                message = await self.mqtt_client.receive_message(timeout=1.0)
                
                if message:
                    topic, payload = message
                    
                    # Publish to VYRA network
                    await vyra_speaker.shout({
                        "source": "mqtt",
                        "topic": topic,
                        "payload": payload.decode('utf-8'),
                        "timestamp": asyncio.get_event_loop().time()
                    })
                    
                    logger.info(f"🔄 Bridged MQTT → VYRA: {topic} = {payload}")
                
            except Exception as e:
                logger.error(f"❌ MQTT listener error: {e}")
    
    async def handle_vyra_command(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle VYRA callable and forward to external system.
        
        This would be registered as a VYRA callable:
            @remote_service
            async def send_external_command(self, request, response=None):
                return await bridge.handle_vyra_command(request)
        """
        command = request.get("command")
        target = request.get("target", "mqtt")
        
        if target == "mqtt":
            # Send command via MQTT
            topic = request.get("topic", "vyra/external/command")
            payload = request.get("payload", "")
            
            await self.mqtt_client.publish(topic, payload)
            
            return {
                "status": "sent",
                "target": "mqtt",
                "topic": topic
            }
        
        elif target == "rest":
            # Send command via REST API
            endpoint = request.get("endpoint", "/api/commands")
            payload = request.get("payload", {})
            
            result = await self.rest_client.post_data(endpoint, payload)
            
            return {
                "status": "sent",
                "target": "rest",
                "result": result
            }
        
        else:
            return {
                "status": "error",
                "message": f"Unknown target: {target}"
            }
    
    async def shutdown(self):
        """Stop bridge and cleanup."""
        self.running = False
        
        if self.rest_client:
            await self.rest_client.shutdown()
        
        if self.mqtt_client:
            await self.mqtt_client.shutdown()
        
        logger.info("🛑 External bridge shutdown")


# ============================================================================
# Demo Functions
# ============================================================================

async def demo_rest_api():
    """Demo: External REST API communication."""
    logger.info("\n" + "="*60)
    logger.info("Demo 1: External REST API")
    logger.info("="*60)
    
    if not AIOHTTP_AVAILABLE:
        logger.warning("⚠️ Skipping - aiohttp not available")
        return
    
    # Example with JSONPlaceholder API
    client = ExternalRESTClient("https://jsonplaceholder.typicode.com")
    await client.initialize()
    
    try:
        # GET request
        users = await client.get_data("/users/1")
        logger.info(f"User: {users['name']}")
        
        # POST request
        new_post = await client.post_data("/posts", {
            "title": "VYRA Integration",
            "body": "Testing external REST communication",
            "userId": 1
        })
        logger.info(f"Created post ID: {new_post.get('id')}")
        
    finally:
        await client.shutdown()


async def demo_mqtt():
    """Demo: External MQTT communication."""
    logger.info("\n" + "="*60)
    logger.info("Demo 2: External MQTT")
    logger.info("="*60)
    
    if not MQTT_AVAILABLE:
        logger.warning("⚠️ Skipping - paho-mqtt not available")
        return
    
    # Example with public MQTT broker (requires internet)
    client = ExternalMQTTClient("test.mosquitto.org", 1883)
    
    try:
        await client.initialize()
        
        # Subscribe to test topic
        await client.subscribe("vyra/test/#")
        
        # Publish test message
        await client.publish("vyra/test/demo", "Hello from VYRA!")
        
        # Wait for message (should receive own message)
        message = await client.receive_message(timeout=5.0)
        if message:
            topic, payload = message
            logger.info(f"📨 Received: {topic} = {payload.decode('utf-8')}")
        else:
            logger.warning("⏰ No message received (timeout)")
        
    except Exception as e:
        logger.error(f"❌ MQTT demo failed: {e}")
    finally:
        await client.shutdown()


async def demo_bridge():
    """Demo: Integration bridge (mock VYRA components)."""
    logger.info("\n" + "="*60)
    logger.info("Demo 3: External Bridge Integration")
    logger.info("="*60)
    
    logger.info("""
This demonstrates how a VYRA module would:
1. Poll external REST API → Publish to VYRA
2. Listen to external MQTT → Publish to VYRA  
3. Receive VYRA commands → Forward to external systems

In a real VYRA module, you would:
- Use @remote_service for handling commands
- Use Speaker.shout() to publish to VYRA network
- Run polling/listening as background tasks
    """)
    
    if not (AIOHTTP_AVAILABLE and MQTT_AVAILABLE):
        logger.warning("⚠️ Skipping - requires aiohttp and paho-mqtt")
        return
    
    # Mock VYRA speaker
    class MockVyraSpeaker:
        async def shout(self, data):
            logger.info(f"📣 VYRA Speaker: {data}")
    
    vyra_speaker = MockVyraSpeaker()
    
    bridge = ExternalBridgeComponent()
    
    try:
        # Initialize bridge
        await bridge.initialize(
            rest_api_url="https://jsonplaceholder.typicode.com",
            mqtt_broker="test.mosquitto.org"
        )
        
        # Demo: Send VYRA command to external MQTT
        result = await bridge.handle_vyra_command({
            "command": "test",
            "target": "mqtt",
            "topic": "vyra/external/test",
            "payload": "Command from VYRA"
        })
        logger.info(f"Command result: {result}")
        
        # Demo: Send VYRA command to external REST API
        result = await bridge.handle_vyra_command({
            "command": "create_post",
            "target": "rest",
            "endpoint": "/posts",
            "payload": {"title": "From VYRA", "body": "Test", "userId": 1}
        })
        logger.info(f"Command result: {result}")
        
    finally:
        await bridge.shutdown()


# ============================================================================
# Main
# ============================================================================

async def main():
    """Run all examples."""
    logger.info("""
╔════════════════════════════════════════════════════════════╗
║      VYRA External Communication Examples                  ║
║                                                             ║
║  Demonstrates integration with external systems:           ║
║  - REST APIs (HTTP/HTTPS)                                  ║
║  - MQTT brokers                                            ║
║  - Bridging external → VYRA                                ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    try:
        await demo_rest_api()
        await demo_mqtt()
        await demo_bridge()
        
        logger.info("\n✅ All examples completed")
        
    except KeyboardInterrupt:
        logger.info("\n⏸️  Examples interrupted by user")
    except Exception as e:
        logger.error(f"\n❌ Error: {e}", exc_info=True)


if __name__ == "__main__":
    asyncio.run(main())
