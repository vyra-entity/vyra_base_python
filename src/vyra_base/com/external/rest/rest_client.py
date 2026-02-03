"""
REST Client for vyra_base

High-level wrapper for HTTP/REST API communication.

Example:
    >>> client = RestClient(base_url="https://api.example.com")
    >>> response = await client.get("/users/123")
    >>> await client.post("/users", json={"name": "John"})
"""
from __future__ import annotations

import logging
from typing import Any, Optional, Dict
import json

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if httpx is available
try:
    import httpx
    REST_AVAILABLE = True
except ImportError:
    REST_AVAILABLE = False


class RestClient:
    """
    High-level REST/HTTP Client wrapper.
    
    Features:
    - GET, POST, PUT, DELETE, PATCH methods
    - JSON serialization
    - Headers management
    - Timeout support
    - TLS/SSL support
    
    Args:
        base_url: Base URL for API
        timeout: Default timeout in seconds
        headers: Default headers
        verify_ssl: Verify SSL certificates
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        base_url: str,
        timeout: float = 30.0,
        headers: Optional[Dict[str, str]] = None,
        verify_ssl: bool = True,
    ):
        """Initialize REST client."""
        if not REST_AVAILABLE:
            raise ImportError("httpx not installed. Install with: pip install httpx")
        
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.headers = headers or {}
        self.verify_ssl = verify_ssl
        
        self._client: Optional[httpx.AsyncClient] = None
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Create HTTP client."""
        self._client = httpx.AsyncClient(
            base_url=self.base_url,
            timeout=self.timeout,
            headers=self.headers,
            verify=self.verify_ssl
        )
        Logger.debug(f"REST client initialized: {self.base_url}")
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close HTTP client."""
        if self._client:
            await self._client.aclose()
            self._client = None
        Logger.debug(f"REST client closed: {self.base_url}")
    
    async def _ensure_connected(self) -> httpx.AsyncClient:
        """Ensure client is created."""
        if self._client is None:
            await self.connect()
        
        if self._client is None:
            raise RuntimeError("REST client creation failed")
        
        return self._client
    
    @ErrorTraceback.w_check_error_exist
    async def get(
        self,
        path: str,
        params: Optional[Dict] = None,
        headers: Optional[Dict] = None,
        timeout: Optional[float] = None
    ) -> Any:
        """HTTP GET request."""
        client = await self._ensure_connected()
        
        try:
            response = await client.get(
                path,
                params=params,
                headers=headers,
                timeout=timeout or self.timeout
            )
            response.raise_for_status()
            return response.json()
        except Exception as e:
            Logger.error(f"❌ REST GET failed: {path}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def post(
        self,
        path: str,
        json: Optional[Any] = None,
        data: Optional[Any] = None,
        headers: Optional[Dict] = None,
        timeout: Optional[float] = None
    ) -> Any:
        """HTTP POST request."""
        client = await self._ensure_connected()
        
        try:
            response = await client.post(
                path,
                json=json,
                data=data,
                headers=headers,
                timeout=timeout or self.timeout
            )
            response.raise_for_status()
            return response.json()
        except Exception as e:
            Logger.error(f"❌ REST POST failed: {path}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def put(
        self,
        path: str,
        json: Optional[Any] = None,
        data: Optional[Any] = None,
        headers: Optional[Dict] = None,
        timeout: Optional[float] = None
    ) -> Any:
        """HTTP PUT request."""
        client = await self._ensure_connected()
        
        try:
            response = await client.put(
                path,
                json=json,
                data=data,
                headers=headers,
                timeout=timeout or self.timeout
            )
            response.raise_for_status()
            return response.json()
        except Exception as e:
            Logger.error(f"❌ REST PUT failed: {path}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def delete(
        self,
        path: str,
        headers: Optional[Dict] = None,
        timeout: Optional[float] = None
    ) -> Any:
        """HTTP DELETE request."""
        client = await self._ensure_connected()
        
        try:
            response = await client.delete(
                path,
                headers=headers,
                timeout=timeout or self.timeout
            )
            response.raise_for_status()
            return response.json() if response.content else None
        except Exception as e:
            Logger.error(f"❌ REST DELETE failed: {path}: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def health_check(self) -> bool:
        """Check if REST endpoint is reachable."""
        try:
            await self.get("/health")
            return True
        except:
            return False
