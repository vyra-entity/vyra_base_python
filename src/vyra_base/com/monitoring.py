"""
Communication Monitoring with Prometheus

Provides automatic performance monitoring for all communication interfaces.
Tracks latency, call count, and errors per protocol.
"""
import logging
import time
import functools
from typing import Callable, Optional, Any
from contextlib import contextmanager

try:
    from prometheus_client import Counter, Histogram, Gauge, Info
    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False

from vyra_base.com.core.types import ProtocolType, InterfaceType

logger = logging.getLogger(__name__)


class CommunicationMetrics:
    """
    Prometheus metrics for communication monitoring.
    
    Tracks:
    - Call count per protocol/interface
    - Latency distribution
    - Error rate
    - Active connections
    """
    
    _instance: Optional['CommunicationMetrics'] = None
    _initialized = False
    
    def __new__(cls):
        """Singleton pattern."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize metrics (only once)."""
        if self._initialized:
            return
        
        if not PROMETHEUS_AVAILABLE:
            logger.warning(
                "⚠️ prometheus_client not installed. "
                "Install with: pip install prometheus-client"
            )
            self._enabled = False
            self._initialized = True
            return
        
        # Call counters
        self.call_total = Counter(
            'vyra_com_calls_total',
            'Total number of communication calls',
            ['protocol', 'interface_type', 'interface_name', 'status']
        )
        
        # Latency histogram
        self.call_latency = Histogram(
            'vyra_com_call_duration_seconds',
            'Communication call duration in seconds',
            ['protocol', 'interface_type', 'interface_name'],
            buckets=(0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0)
        )
        
        # Active connections
        self.active_connections = Gauge(
            'vyra_com_active_connections',
            'Number of active connections',
            ['protocol']
        )
        
        # Protocol info
        self.protocol_info = Info(
            'vyra_com_protocol',
            'Information about available protocols'
        )
        
        # Error counter
        self.error_total = Counter(
            'vyra_com_errors_total',
            'Total number of communication errors',
            ['protocol', 'interface_type', 'error_type']
        )
        
        # Message size
        self.message_size = Histogram(
            'vyra_com_message_size_bytes',
            'Message size in bytes',
            ['protocol', 'interface_type', 'direction'],
            buckets=(64, 256, 1024, 4096, 16384, 65536, 262144, 1048576)
        )
        
        # ROS2/DDS specific metrics
        self.ros2_topic_throughput = Counter(
            'vyra_ros2_topic_messages_total',
            'Total messages published/received on ROS2 topics',
            ['topic_name', 'direction']
        )
        
        self.ros2_message_drops = Counter(
            'vyra_ros2_message_drops_total',
            'Total dropped messages due to QoS policy',
            ['topic_name', 'qos_policy']
        )
        
        self.ros2_qos_violations = Counter(
            'vyra_ros2_qos_violations_total',
            'Total QoS policy violations',
            ['topic_name', 'violation_type']
        )
        
        self.ros2_discovery_nodes = Gauge(
            'vyra_ros2_discovered_nodes',
            'Number of discovered ROS2 nodes in domain'
        )
        
        self.ros2_discovery_topics = Gauge(
            'vyra_ros2_discovered_topics',
            'Number of discovered ROS2 topics in domain'
        )
        
        self.ros2_service_latency = Histogram(
            'vyra_ros2_service_call_duration_seconds',
            'ROS2 service call duration',
            ['service_name'],
            buckets=(0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0)
        )
        
        # DDS specific metrics
        self.dds_writer_liveliness = Gauge(
            'vyra_dds_writer_liveliness_lost',
            'DDS DataWriter liveliness lost events',
            ['topic_name']
        )
        
        self.dds_reader_sample_lost = Counter(
            'vyra_dds_reader_samples_lost_total',
            'DDS DataReader samples lost',
            ['topic_name']
        )
        
        self.dds_deadline_missed = Counter(
            'vyra_dds_deadline_missed_total',
            'DDS deadline missed events',
            ['topic_name', 'entity_type']
        )
        
        self._enabled = True
        self._initialized = True
        logger.info("✅ Communication metrics initialized (including ROS2/DDS)")
    
    def is_enabled(self) -> bool:
        """Check if metrics are enabled."""
        return self._enabled
    
    def record_call(
        self,
        protocol: ProtocolType,
        interface_type: InterfaceType,
        interface_name: str,
        duration: float,
        success: bool = True
    ) -> None:
        """Record a communication call."""
        if not self._enabled:
            return
        
        status = 'success' if success else 'error'
        
        self.call_total.labels(
            protocol=protocol.value,
            interface_type=interface_type.value,
            interface_name=interface_name,
            status=status
        ).inc()
        
        self.call_latency.labels(
            protocol=protocol.value,
            interface_type=interface_type.value,
            interface_name=interface_name
        ).observe(duration)
    
    def record_error(
        self,
        protocol: ProtocolType,
        interface_type: InterfaceType,
        error_type: str
    ) -> None:
        """Record an error."""
        if not self._enabled:
            return
        
        self.error_total.labels(
            protocol=protocol.value,
            interface_type=interface_type.value,
            error_type=error_type
        ).inc()
    
    def record_message_size(
        self,
        protocol: ProtocolType,
        interface_type: InterfaceType,
        size: int,
        direction: str = 'send'
    ) -> None:
        """Record message size."""
        if not self._enabled:
            return
        
        self.message_size.labels(
            protocol=protocol.value,
            interface_type=interface_type.value,
            direction=direction
        ).observe(size)
    
    @contextmanager
    def track_connection(self, protocol: ProtocolType):
        """Context manager to track active connections."""
        if self._enabled:
            self.active_connections.labels(protocol=protocol.value).inc()
        
        try:
            yield
        finally:
            if self._enabled:
                self.active_connections.labels(protocol=protocol.value).dec()
    
    def update_protocol_info(self, protocols: list[str]) -> None:
        """Update protocol information."""
        if not self._enabled:
            return
        
        self.protocol_info.info({
            'available_protocols': ','.join(protocols),
            'count': str(len(protocols))
        })
    
    def record_ros2_topic_message(
        self,
        topic_name: str,
        direction: str = 'send'
    ) -> None:
        """Record ROS2 topic message."""
        if not self._enabled:
            return
        
        self.ros2_topic_throughput.labels(
            topic_name=topic_name,
            direction=direction
        ).inc()
    
    def record_ros2_message_drop(
        self,
        topic_name: str,
        qos_policy: str
    ) -> None:
        """Record ROS2 message drop."""
        if not self._enabled:
            return
        
        self.ros2_message_drops.labels(
            topic_name=topic_name,
            qos_policy=qos_policy
        ).inc()
    
    def record_ros2_qos_violation(
        self,
        topic_name: str,
        violation_type: str
    ) -> None:
        """Record ROS2 QoS violation."""
        if not self._enabled:
            return
        
        self.ros2_qos_violations.labels(
            topic_name=topic_name,
            violation_type=violation_type
        ).inc()
    
    def update_ros2_discovery(
        self,
        nodes_count: int,
        topics_count: int
    ) -> None:
        """Update ROS2 discovery metrics."""
        if not self._enabled:
            return
        
        self.ros2_discovery_nodes.set(nodes_count)
        self.ros2_discovery_topics.set(topics_count)
    
    def record_dds_writer_liveliness_lost(self, topic_name: str) -> None:
        """Record DDS writer liveliness lost."""
        if not self._enabled:
            return
        
        self.dds_writer_liveliness.labels(topic_name=topic_name).inc()
    
    def record_dds_reader_sample_lost(self, topic_name: str, count: int = 1) -> None:
        """Record DDS reader samples lost."""
        if not self._enabled:
            return
        
        self.dds_reader_sample_lost.labels(topic_name=topic_name).inc(count)
    
    def record_dds_deadline_missed(
        self,
        topic_name: str,
        entity_type: str = 'publisher'
    ) -> None:
        """Record DDS deadline missed."""
        if not self._enabled:
            return
        
        self.dds_deadline_missed.labels(
            topic_name=topic_name,
            entity_type=entity_type
        ).inc()


# Global metrics instance
metrics = CommunicationMetrics()


def monitored_callable(
    protocol: Optional[ProtocolType] = None,
    interface_name: Optional[str] = None,
    track_message_size: bool = False
):
    """
    Decorator for automatic callable monitoring.
    
    Usage:
        @monitored_callable(protocol=ProtocolType.ROS2, interface_name="motor_start")
        async def motor_start(self, request, response):
            # Your code here
            return response
    
    Args:
        protocol: Protocol type (auto-detected if None)
        interface_name: Interface name (uses function name if None)
        track_message_size: If True, track request/response sizes
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        async def async_wrapper(*args, **kwargs):
            nonlocal interface_name, protocol
            
            # Auto-detect interface name
            if interface_name is None:
                interface_name = func.__name__
            
            # Auto-detect protocol from instance
            if protocol is None and len(args) > 0:
                instance = args[0]
                if hasattr(instance, 'protocol'):
                    protocol = instance.protocol
            
            if protocol is None:
                protocol = ProtocolType.ROS2  # Default fallback
            
            start_time = time.time()
            success = True
            
            try:
                # Track message size
                if track_message_size and len(args) > 1:
                    try:
                        request = args[1]
                        request_size = len(str(request).encode('utf-8'))
                        metrics.record_message_size(
                            protocol,
                            InterfaceType.CALLABLE,
                            request_size,
                            'receive'
                        )
                    except:
                        pass
                
                # Execute function
                result = await func(*args, **kwargs)
                
                # Track response size
                if track_message_size and result is not None:
                    try:
                        response_size = len(str(result).encode('utf-8'))
                        metrics.record_message_size(
                            protocol,
                            InterfaceType.CALLABLE,
                            response_size,
                            'send'
                        )
                    except:
                        pass
                
                return result
                
            except Exception as e:
                success = False
                metrics.record_error(
                    protocol,
                    InterfaceType.CALLABLE,
                    type(e).__name__
                )
                raise
            finally:
                duration = time.time() - start_time
                metrics.record_call(
                    protocol,
                    InterfaceType.CALLABLE,
                    interface_name,
                    duration,
                    success
                )
        
        @functools.wraps(func)
        def sync_wrapper(*args, **kwargs):
            nonlocal interface_name, protocol
            
            if interface_name is None:
                interface_name = func.__name__
            
            if protocol is None and len(args) > 0:
                instance = args[0]
                if hasattr(instance, 'protocol'):
                    protocol = instance.protocol
            
            if protocol is None:
                protocol = ProtocolType.ROS2
            
            start_time = time.time()
            success = True
            
            try:
                result = func(*args, **kwargs)
                return result
            except Exception as e:
                success = False
                metrics.record_error(
                    protocol,
                    InterfaceType.CALLABLE,
                    type(e).__name__
                )
                raise
            finally:
                duration = time.time() - start_time
                metrics.record_call(
                    protocol,
                    InterfaceType.CALLABLE,
                    interface_name,
                    duration,
                    success
                )
        
        # Return appropriate wrapper
        if functools.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper
    
    return decorator


def monitored_speaker(
    protocol: Optional[ProtocolType] = None,
    speaker_name: Optional[str] = None
):
    """
    Decorator for automatic speaker monitoring.
    
    Usage:
        @monitored_speaker(protocol=ProtocolType.MQTT)
        async def publish_status(self, message):
            # Your code here
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            nonlocal speaker_name, protocol
            
            if speaker_name is None:
                speaker_name = func.__name__
            
            if protocol is None and len(args) > 0:
                instance = args[0]
                if hasattr(instance, 'protocol'):
                    protocol = instance.protocol
            
            if protocol is None:
                protocol = ProtocolType.ROS2
            
            start_time = time.time()
            success = True
            
            try:
                result = await func(*args, **kwargs)
                return result
            except Exception as e:
                success = False
                metrics.record_error(
                    protocol,
                    InterfaceType.SPEAKER,
                    type(e).__name__
                )
                raise
            finally:
                duration = time.time() - start_time
                metrics.record_call(
                    protocol,
                    InterfaceType.SPEAKER,
                    speaker_name,
                    duration,
                    success
                )
        
        return wrapper
    
    return decorator
