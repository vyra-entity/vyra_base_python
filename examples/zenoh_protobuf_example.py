#!/usr/bin/env python3
"""
Zenoh with Protobuf Example

Demonstrates using Zenoh transport with Protocol Buffers for:
- Type-safe message serialization
- Language-agnostic communication
- Schema evolution
- Efficient binary encoding

This example shows:
1. Defining Protobuf messages
2. Using Zenoh with Protobuf serialization
3. Speaker (pub/sub) with Protobuf
4. Callable (request/response) with Protobuf
5. Job (long-running task) with Protobuf
"""
import asyncio
import logging
from pathlib import Path
from typing import Any, Dict

# Protobuf (requires: pip install protobuf)
try:
    from google.protobuf import message as pb_message
    PROTOBUF_AVAILABLE = True
except ImportError:
    PROTOBUF_AVAILABLE = False
    print("⚠️ protobuf not available. Install: pip install protobuf")

# VYRA imports
try:
    from vyra_base.com.transport.t_zenoh import (
        ZenohProvider,
        ZenohSession,
        SessionConfig,
        SessionMode,
        ZENOH_AVAILABLE
    )
    from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat
    from vyra_base.com.converters import ConverterFactory, ProtobufConverter
except ImportError as e:
    print(f"⚠️ VYRA imports failed: {e}")
    ZENOH_AVAILABLE = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ============================================================================
# Protobuf Message Definitions
# ============================================================================

# In a real project, these would be in .proto files and compiled
# For this example, we'll define them inline using Python

if PROTOBUF_AVAILABLE:
    from google.protobuf import descriptor_pb2
    from google.protobuf.descriptor import FieldDescriptor
    from google.protobuf.message_factory import MessageFactory
    
    # Example: Sensor data message
    SENSOR_DATA_PROTO = """
    syntax = "proto3";
    
    message SensorData {
        string sensor_id = 1;
        double temperature = 2;
        double humidity = 3;
        int64 timestamp = 4;
    }
    """
    
    # Example: Command message
    COMMAND_PROTO = """
    syntax = "proto3";
    
    message Command {
        string command_type = 1;
        map<string, string> parameters = 2;
        int32 priority = 3;
    }
    """
    
    # Example: Job progress message
    JOB_PROGRESS_PROTO = """
    syntax = "proto3";
    
    message JobProgress {
        string job_id = 1;
        int32 progress_percent = 2;
        string status_message = 3;
        repeated string completed_steps = 4;
    }
    """


# ============================================================================
# Helper: Simple Protobuf Message Creation
# ============================================================================

class SimpleProtoMessage:
    """
    Simple wrapper for creating Protobuf messages in Python.
    
    In production, use compiled .proto files with protoc.
    """
    
    @staticmethod
    def create_sensor_data(sensor_id: str, temperature: float, humidity: float, timestamp: int):
        """Create SensorData protobuf message."""
        # This is a simplified version - in production use compiled proto
        return {
            "sensor_id": sensor_id,
            "temperature": temperature,
            "humidity": humidity,
            "timestamp": timestamp
        }
    
    @staticmethod
    def create_command(command_type: str, parameters: Dict[str, str], priority: int = 1):
        """Create Command protobuf message."""
        return {
            "command_type": command_type,
            "parameters": parameters,
            "priority": priority
        }
    
    @staticmethod
    def create_job_progress(job_id: str, progress: int, message: str, steps: list):
        """Create JobProgress protobuf message."""
        return {
            "job_id": job_id,
            "progress_percent": progress,
            "status_message": message,
            "completed_steps": steps
        }


# ============================================================================
# Example 1: Zenoh Speaker with Protobuf (Pub/Sub)
# ============================================================================

async def demo_zenoh_speaker_protobuf():
    """Demonstrate Zenoh Speaker with Protobuf serialization."""
    logger.info("\n" + "="*60)
    logger.info("Example 1: Zenoh Speaker with Protobuf")
    logger.info("="*60)
    
    if not ZENOH_AVAILABLE:
        logger.error("❌ Zenoh not available")
        return
    
    # Create Zenoh session
    config = SessionConfig(
        mode=SessionMode.PEER,  # Use PEER mode for local testing
        connect=[],
        scouting_multicast=True
    )
    
    session = ZenohSession(config)
    await session.open()
    
    provider = ZenohProvider(protocol_type=None)
    provider._session = session
    
    try:
        # Create publisher (shouter)
        publisher = await provider.create_speaker(
            "/sensors/temperature",
            format=SerializationFormat.JSON,  # We'll serialize proto to JSON
            is_publisher=True
        )
        
        # Create subscriber (listener)
        received_messages = []
        
        def handle_sensor_data(data):
            logger.info(f"📨 Received sensor data: {data}")
            received_messages.append(data)
        
        subscriber = await provider.create_speaker(
            "/sensors/temperature",
            format=SerializationFormat.JSON,
            is_publisher=False
        )
        await subscriber.listen(handle_sensor_data)
        
        # Give subscriber time to connect
        await asyncio.sleep(0.5)
        
        # Publish Protobuf messages
        for i in range(3):
            sensor_data = SimpleProtoMessage.create_sensor_data(
                sensor_id=f"TEMP_{i:03d}",
                temperature=20.0 + i,
                humidity=45.0 + i * 2,
                timestamp=1000000 + i
            )
            
            # Using ProtobufConverter to serialize
            # In production, you'd convert the proto message to binary
            await publisher.shout(sensor_data)
            
            logger.info(f"📤 Published sensor data: {sensor_data}")
            await asyncio.sleep(0.2)
        
        # Wait for messages to be received
        await asyncio.sleep(1.0)
        
        logger.info(f"✅ Received {len(received_messages)} messages")
        
        await publisher.shutdown()
        await subscriber.shutdown()
        
    finally:
        await session.close()


# ============================================================================
# Example 2: Zenoh Callable with Protobuf (Request/Response)
# ============================================================================

async def demo_zenoh_callable_protobuf():
    """Demonstrate Zenoh Callable with Protobuf serialization."""
    logger.info("\n" + "="*60)
    logger.info("Example 2: Zenoh Callable with Protobuf")
    logger.info("="*60)
    
    if not ZENOH_AVAILABLE:
        logger.error("❌ Zenoh not available")
        return
    
    # Create Zenoh session
    config = SessionConfig(
        mode=SessionMode.PEER,
        connect=[],
        scouting_multicast=True
    )
    
    session = ZenohSession(config)
    await session.open()
    
    provider = ZenohProvider(protocol_type=None)
    provider._session = session
    
    try:
        # Create responder (server)
        async def handle_command(request):
            """Handle command protobuf message."""
            logger.info(f"🔧 Processing command: {request}")
            
            cmd_type = request.get("command_type")
            params = request.get("parameters", {})
            
            # Simulate command processing
            if cmd_type == "start":
                result = {"status": "started", "device": params.get("device", "unknown")}
            elif cmd_type == "stop":
                result = {"status": "stopped", "device": params.get("device", "unknown")}
            else:
                result = {"status": "error", "message": f"Unknown command: {cmd_type}"}
            
            return result
        
        responder = await provider.create_callable(
            "/device/command",
            callback=handle_command,
            format=SerializationFormat.JSON,
            is_server=True
        )
        
        # Create requester (client)
        requester = await provider.create_callable(
            "/device/command",
            format=SerializationFormat.JSON,
            is_server=False
        )
        
        # Give server time to start
        await asyncio.sleep(0.5)
        
        # Send commands
        commands = [
            SimpleProtoMessage.create_command("start", {"device": "motor_1"}, priority=1),
            SimpleProtoMessage.create_command("stop", {"device": "motor_1"}, priority=2),
            SimpleProtoMessage.create_command("unknown", {}, priority=1),
        ]
        
        for cmd in commands:
            logger.info(f"📤 Sending command: {cmd}")
            response = await requester.call(cmd, timeout=5.0)
            logger.info(f"📨 Response: {response}")
            await asyncio.sleep(0.3)
        
        await responder.shutdown()
        await requester.shutdown()
        
    finally:
        await session.close()


# ============================================================================
# Example 3: Zenoh Job with Protobuf (Long-Running Task)
# ============================================================================

async def demo_zenoh_job_protobuf():
    """Demonstrate Zenoh Job with Protobuf feedback and result."""
    logger.info("\n" + "="*60)
    logger.info("Example 3: Zenoh Job with Protobuf")
    logger.info("="*60)
    
    if not ZENOH_AVAILABLE:
        logger.error("❌ Zenoh not available")
        return
    
    # Create Zenoh session
    config = SessionConfig(
        mode=SessionMode.PEER,
        connect=[],
        scouting_multicast=True
    )
    
    session = ZenohSession(config)
    await session.open()
    
    provider = ZenohProvider(protocol_type=None)
    provider._session = session
    
    try:
        # Create job server
        async def process_data_job(params, publish_feedback):
            """
            Long-running job that processes data.
            
            Sends Protobuf JobProgress messages as feedback.
            Returns Protobuf result message.
            """
            job_id = params.get("job_id", "unknown")
            data_items = params.get("data_items", 10)
            
            logger.info(f"🏁 Starting job {job_id} with {data_items} items")
            
            completed_steps = []
            
            for i in range(data_items):
                # Simulate processing
                await asyncio.sleep(0.2)
                
                step_name = f"step_{i+1}"
                completed_steps.append(step_name)
                
                # Send Protobuf feedback
                progress = SimpleProtoMessage.create_job_progress(
                    job_id=job_id,
                    progress=int((i + 1) / data_items * 100),
                    message=f"Processing {step_name}",
                    steps=completed_steps.copy()
                )
                
                await publish_feedback(progress)
                logger.info(f"📊 Progress: {(i+1)/data_items*100:.0f}%")
            
            # Return Protobuf result
            return {
                "job_id": job_id,
                "status": "completed",
                "processed_items": data_items,
                "completed_steps": completed_steps
            }
        
        job_server = await provider.create_job(
            "/processing/data_job",
            result_callback=process_data_job,
            format=SerializationFormat.JSON,
            is_server=True
        )
        
        # Create job client
        job_client = await provider.create_job(
            "/processing/data_job",
            format=SerializationFormat.JSON,
            is_server=False
        )
        
        # Feedback handler
        async def handle_feedback(feedback):
            progress = feedback.get("progress_percent", 0)
            message = feedback.get("status_message", "")
            logger.info(f"📈 Job feedback: {progress}% - {message}")
        
        # Result handler
        async def handle_result(result):
            logger.info(f"✅ Job result: {result}")
        
        # Subscribe to feedback and result
        await job_client.listen_feedback(handle_feedback)
        await job_client.listen_result(handle_result)
        
        # Give server time to start
        await asyncio.sleep(0.5)
        
        # Start job
        job_params = {
            "job_id": "JOB_001",
            "data_items": 5
        }
        
        logger.info(f"🚀 Starting job with params: {job_params}")
        await job_client.start(job_params)
        
        # Wait for job to complete
        await asyncio.sleep(3.0)
        
        await job_server.shutdown()
        await job_client.shutdown()
        
    finally:
        await session.close()


# ============================================================================
# Example 4: Using Real Protobuf Binary Serialization
# ============================================================================

async def demo_protobuf_converter():
    """Demonstrate using ProtobufConverter for binary serialization."""
    logger.info("\n" + "="*60)
    logger.info("Example 4: Protobuf Binary Serialization")
    logger.info("="*60)
    
    if not PROTOBUF_AVAILABLE:
        logger.error("❌ Protobuf not available")
        return
    
    try:
        # Get Protobuf converter
        converter = ConverterFactory.get_converter("protobuf")
        
        if not converter:
            logger.warning("⚠️ ProtobufConverter not registered")
            return
        
        # Example message (as dict)
        message = {
            "sensor_id": "TEMP_001",
            "temperature": 23.5,
            "humidity": 45.0,
            "timestamp": 1234567890
        }
        
        logger.info(f"Original message: {message}")
        
        # Serialize to binary
        binary_data = converter.dict_to_binary(message, "SensorData")
        logger.info(f"Binary data: {len(binary_data)} bytes")
        
        # Deserialize from binary
        restored = converter.binary_to_dict(binary_data, "SensorData")
        logger.info(f"Restored message: {restored}")
        
        # Verify
        assert message == restored, "Serialization roundtrip failed"
        logger.info("✅ Protobuf serialization roundtrip successful")
        
    except Exception as e:
        logger.error(f"❌ Protobuf converter demo failed: {e}")


# ============================================================================
# Main
# ============================================================================

async def main():
    """Run all Zenoh+Protobuf examples."""
    logger.info("""
╔════════════════════════════════════════════════════════════╗
║      VYRA Zenoh + Protobuf Examples                        ║
║                                                             ║
║  Demonstrates using Zenoh with Protocol Buffers:           ║
║  1. Speaker (Pub/Sub) with Protobuf messages              ║
║  2. Callable (Request/Response) with Protobuf             ║
║  3. Job (Long-running task) with Protobuf feedback        ║
║  4. Binary Protobuf serialization                          ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    if not ZENOH_AVAILABLE:
        logger.error("❌ Zenoh not available. Install: pip install eclipse-zenoh")
        return
    
    if not PROTOBUF_AVAILABLE:
        logger.error("❌ Protobuf not available. Install: pip install protobuf")
        return
    
    try:
        await demo_zenoh_speaker_protobuf()
        await demo_zenoh_callable_protobuf()
        await demo_zenoh_job_protobuf()
        await demo_protobuf_converter()
        
        logger.info("\n✅ All Zenoh+Protobuf examples completed")
        
    except KeyboardInterrupt:
        logger.info("\n⏸️  Examples interrupted by user")
    except Exception as e:
        logger.error(f"\n❌ Error: {e}", exc_info=True)


if __name__ == "__main__":
    asyncio.run(main())
