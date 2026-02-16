"""
UDS VYRA Models Layer

VYRA abstractions for Unix Domain Socket communication.

Legacy patterns (deprecated):
- UDSCallable: Request-response pattern via UDS
- UDSSpeaker: Publish-subscribe pattern via UDS
- UDSJob: Long-running task pattern via UDS

Unified transport layer:
- Publisher/Subscriber (datagram sockets)
- Server/Client (stream sockets)
- ActionServer/ActionClient (stream sockets + state messages)

Usage:
    >>> from vyra_base.com.transport.t_uds.vyra_models import UdsServerImpl
    >>> 
    >>> # Server side
    >>> async def handle_request(request):
    ...     return {"result": request["value"] * 2}
    >>> 
    >>> server = UdsServerImpl(
    ...     name="calculate",
    ...     topic_builder=builder,
    ...     response_callback=handle_request,
    ...     service_type=CalcService,
    ...     module_name="math_service"
    ... )
    >>> await server.initialize()
"""
import logging

logger = logging.getLogger(__name__)

try:
    
    # Unified transport layer imports
    from vyra_base.com.transport.t_uds.vyra_models.publisher import UdsPublisherImpl
    from vyra_base.com.transport.t_uds.vyra_models.subscriber import UdsSubscriberImpl
    from vyra_base.com.transport.t_uds.vyra_models.server import UdsServerImpl
    from vyra_base.com.transport.t_uds.vyra_models.client import VyraClientImpl
    from vyra_base.com.transport.t_uds.vyra_models.action_server import VyraActionServerImpl
    from vyra_base.com.transport.t_uds.vyra_models.action_client import VyraActionClientImpl
    
    UDS_MODELS_AVAILABLE = True
    logger.debug("✅ UDS VYRA models layer available")
    
except ImportError as e:
    # Unified
    UdsPublisherImpl = None
    UdsSubscriberImpl = None
    UdsServerImpl = None
    VyraClientImpl = None
    VyraActionServerImpl = None
    VyraActionClientImpl = None
    UDS_MODELS_AVAILABLE = False
    logger.debug(f"⚠️  UDS VYRA models layer unavailable: {e}")

__all__ = [
    # Unified transport layer
    "UdsPublisherImpl",
    "UdsSubscriberImpl",
    "UdsServerImpl",
    "VyraClientImpl",
    "VyraActionServerImpl",
    "VyraActionClientImpl",
    # Availability flag
    "UDS_MODELS_AVAILABLE",
]
