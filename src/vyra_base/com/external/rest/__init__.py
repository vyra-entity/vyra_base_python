"""
REST External Communication

HTTP REST API integration with FastAPI/aiohttp.
"""
from vyra_base.com.external.rest.rest_client import RestClient, REST_AVAILABLE

__all__ = [
    "RestClient",
    "REST_AVAILABLE",
]
