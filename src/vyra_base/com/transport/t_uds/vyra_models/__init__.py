"""
UDS VYRA Models Layer

VYRA abstractions for Unix Domain Socket communication.
This layer wraps UDS functionality into VYRA's unified interface.

Components:
    - UDSCallable: Request-response pattern via UDS
    - UDSSpeaker: Publish-subscribe pattern via UDS
    - UDSJob: Long-running task pattern via UDS

Usage:
    >>> from vyra_base.com.transport.t_uds.vyra_models import UDSCallable
    >>> 
    >>> # Server side
    >>> async def handle_request(request):
    ...     return {"result": request["value"] * 2}
    >>> 
    >>> callable = UDSCallable(
    ...     "calculate",
    ...     callback=handle_request,
    ...     module_name="math_service"
    ... )
    >>> await callable.initialize()
    >>> 
    >>> # Client side
    >>> callable = UDSCallable("calculate", module_name="math_service")
    >>> result = await callable.call({"value": 21})
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.transport.t_uds.vyra_models.callable import UDSCallable
    from vyra_base.com.transport.t_uds.vyra_models.speaker import UDSSpeaker
    from vyra_base.com.transport.t_uds.vyra_models.job import UDSJob
    
    UDS_MODELS_AVAILABLE = True
    logger.debug("✅ UDS VYRA models layer available")
    
except ImportError as e:
    UDSCallable = None
    UDSSpeaker = None
    UDSJob = None
    UDS_MODELS_AVAILABLE = False
    logger.debug(f"⚠️  UDS VYRA models layer unavailable: {e}")

__all__ = [
    "UDSCallable",
    "UDSSpeaker",
    "UDSJob",
    "UDS_MODELS_AVAILABLE",
]
