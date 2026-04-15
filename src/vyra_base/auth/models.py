"""
Shared Pydantic models for authentication used across all VYRA modules.
"""

from typing import Optional
from pydantic import BaseModel


class AuthToken(BaseModel):
    """Represents a session token stored in Redis."""

    token: str
    username: str
    user_id: int
    role: str
    level: int
    created_at: str
    expires_at: str
    module_id: str


class LoginRequest(BaseModel):
    """Login request payload."""

    username: str
    password: str
    auth_mode: str = "local"
    module_name: Optional[str] = None


class LoginResponse(BaseModel):
    """Login response payload."""

    success: bool
    token: str = ""
    username: str = ""
    auth_mode: str = "local"
    message: str = ""


class ChangePasswordRequest(BaseModel):
    """Change-password request payload."""

    old_password: str
    new_password: str


class CreateUserRequest(BaseModel):
    """Create-local-user request payload."""

    username: str
    password: str
    role: str = "viewer"
    level: int = 0
    permissions: str = "{}"
