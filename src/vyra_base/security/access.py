from enum import Enum
import uuid
from typing import Any, Literal, LiteralString, Union

from vyra_base.helper.logger import Logger
from vyra_base.security.trust import (
    TrustlevelManager, 
    TRUST_LEVEL, 
    TRUST_STATUS
)
from vyra_base.com.datalayer.interface_factory import remote_callable

class ACCESS_STATUS(Enum, int):
    """
    Enum for access levels.
    """
    ACCESS_GRANTED = 0
    NOT_ALLOWED = 1
    MISSING_ACCESS_PARAMETERS = 2
    INTERNAL_ERROR = 3

    def __repr__(self) -> LiteralString:
        return (
            "Access granted and allowed(0), "
            "Access not allowed(1), "
            "Access not allowed due to missing parameters(2), "
            "Access not allowed due to internal error(3)"
        )

class ACCESS_LEVEL(Enum, int):
    """
    Enum for access levels.
    """
    READ_ONLY = 1
    EXTENDED_READ = 2
    CONTROL_AND_PARAM = 3
    EXTENDED_CONTROL = 4
    FULL_ACCESS = 5

    @staticmethod
    def parse_access_level(level: int) -> 'ACCESS_LEVEL':
        """
        Parse an integer to an ACCESS_LEVEL enum.
        """
        try:
            return ACCESS_LEVEL(level)
        except ValueError:
            raise ValueError(f"Invalid access level: {level}")

class AccessManager:
    """ 
    AccessManager class to manage access levels.
    """
    
    def __init__(self):
        """
        Initialize the AccessManager class.
        """
        self.tlm = TrustlevelManager()

    @remote_callable
    def request_access(self,
                       certificate: str,
                       module_id: uuid.UUID,
                       requested_level: Union[int, ACCESS_LEVEL],
                       level_4_psw: str = None,
                       level_5_passkey: bytes = None) -> int:
        """
        Request access to a module.
        """
        if not self.tlm.verify_id(module_id):
            raise ValueError("Module ID is not verified.")

        if not isinstance(requested_level, (int, ACCESS_LEVEL)):
            Logger.warn(f"Invalid access level: {requested_level}")
            raise ValueError(
                "access_level must be an integer or an ACCESS_LEVEL enum.")

        if not self._check_certificate_signature(certificate):
            Logger.error(f"Invalid certificate signature for module {module_id}.")
            return ACCESS_STATUS.NOT_ALLOWED.value

        func = None
        args: list[Any] = []

        match ACCESS_LEVEL(requested_level):
            case ACCESS_LEVEL.READ_ONLY:
                Logger.info(f"Read-only access granted for module {module_id}.")
                return ACCESS_STATUS.ACCESS_GRANTED.value
            case ACCESS_LEVEL.EXTENDED_READ:
                Logger.info(f"Extended read access granted for module {module_id}.")
                return ACCESS_STATUS.ACCESS_GRANTED.value
            case ACCESS_LEVEL.CONTROL_AND_PARAM:
                Logger.info(f"Control and parameter access granted for module {module_id}.")
                return ACCESS_STATUS.ACCESS_GRANTED.value
            case ACCESS_LEVEL.EXTENDED_CONTROL:
                if level_4_psw is None:
                    Logger.error("Level 4 password required for extended control access.")
                    return ACCESS_STATUS.MISSING_ACCESS_PARAMETERS.value
                func = self._verify_extended_control
                args = [level_4_psw]
            case ACCESS_LEVEL.FULL_ACCESS:
                if level_4_psw is None:
                    Logger.error("Level 4 password required for extended control access.")
                    return ACCESS_STATUS.MISSING_ACCESS_PARAMETERS.value
                if level_5_passkey is None:
                    Logger.error("Level 5 passkey required for extended control access.")
                    return ACCESS_STATUS.MISSING_ACCESS_PARAMETERS.value
                func = self._verify_full_access
                args = [level_4_psw, level_5_passkey]
                Logger.info(f"Full access granted for module {module_id}.")
            case _:
                Logger.error(f"Invalid access level: {requested_level}")
                return ACCESS_STATUS.NOT_ALLOWED.value()

        try:            
            if func(*args):
                if self.tlm.check_trust_access(module_id) != TRUST_STATUS.SUCCEED:
                    Logger.error(f"Access denied for module {module_id}.")
                    return ACCESS_STATUS.NOT_ALLOWED.value
                Logger.info(
                    f"Access granted for module {module_id} "
                    f"with level {requested_level}.")
                return ACCESS_STATUS.ACCESS_GRANTED.value
            else:
                Logger.error(
                    f"Access denied for module {module_id} "
                    f"with level {requested_level}.")
                return ACCESS_STATUS.NOT_ALLOWED.value
        except Exception as e:
            Logger.error(f"Error occurred while verifying access for module {module_id}: {e}")
            return ACCESS_STATUS.INTERNAL_ERROR.value

    def _verify_extended_control(
            self, level_4_psw: str) -> bool:
        pass

    def _verify_full_access(
            self, level_4_psw: str, level_5_passkey: bytes) -> bool:
        pass

    def _check_certificate_signature(self, certificate: str) -> bool:
        """
        Check the signature of the certificate.
        This is a placeholder for actual implementation.
        """
        pass