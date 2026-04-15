"""
Shared authentication exceptions for all VYRA modules.
"""


class UsermanagerUnavailableError(Exception):
    """Raised when the external v2_usermanager service cannot be reached or is not registered."""
