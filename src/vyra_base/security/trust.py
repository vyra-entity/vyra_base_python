from enum import Enum
from pathlib import Path

from vyra_base.helper.env_handler import EnvHandler
from vyra_base.helper.logger import Logger

class TRUST_LEVEL(Enum, int):
    TRUST_NONE = 0
    TRUST_ONE_RESERVE = 1
    TRUST_ONE_RELATED = 2
    TRUST_ALL = 3
    
class TRUST_STATUS(Enum, int):
    SUCCEED = 0
    FAILED = 1
    UNKNOWN = 2

class TrustlevelManager:
    def __init__(self):
        """
        Initialize the TrustlevelManager class.
        """
        self.level = self.get_level()
        self.ENV_PATH: Path = Path('.')
        self.trusted_ids: list = []
        self.related_ids: list[set] = []

    def get_level(self) -> TRUST_LEVEL:
        """
        Reads the trust level from a ENV file.
        """
        eh = EnvHandler()
        eh.load_env()

        if 'TRUST_LEVEL' not in eh.env.keys():
            raise ValueError("TRUST_LEVEL not found in environment variables.")
        
        return TRUST_LEVEL(eh.env["TRUST_LEVEL"])

    def check_trust_access(self, trust_id, related_ids: list = []) -> TRUST_STATUS:
        """
        Check if the given id has access based on the trust level.
        
        :param id: The identifier to check access for.
        :type id: str
        :return: TRUST_STATUS indicating whether the id is trusted or untrusted.
        """
        if not self.verify_id(trust_id):     
            Logger.warn(f"ID {trust_id} is not valid, trust failed.")   
            return TRUST_STATUS.FAILED

        match self.level:
            case TRUST_LEVEL.TRUST_ALL:
                if trust_id in self.trusted_ids:
                    Logger.info(f"ID {trust_id} is already trusted.")
                    return TRUST_STATUS.SUCCEED

                Logger.info(f"ID {trust_id} is added and trusted.")
                self.trusted_ids.append(trust_id)
                return TRUST_STATUS.SUCCEED
            case TRUST_LEVEL.TRUST_ONE_RESERVE:
                if len(self.trusted_ids) < 1:
                    Logger.info(f"ID {trust_id} is added and trusted.")
                    self.trusted_ids.append(trust_id)
                else:
                    Logger.warn(f"Access denied, module already reserved.")
                    return TRUST_STATUS.FAILED
            case TRUST_LEVEL.TRUST_ONE_RELATED:
                
                if (len(self.trusted_ids) > 0 and 
                    trust_id not in self.trusted_ids):
                    Logger.warn(f"Access denied, module already reserved.")
                    return TRUST_STATUS.FAILED
                elif (len(self.trusted_ids) > 0 and 
                    trust_id in self.trusted_ids):
                    Logger.info(f"ID {trust_id} is already trusted.")
                    
                for related_id in related_ids:
                    if not self.verify_id(related_id):
                        Logger.warn(f"Related ID {related_id} is not valid, access denied.")
                        return TRUST_STATUS.FAILED
                    if related_id not in self.related_ids:
                        Logger.info(f"Related ID {related_id} is added and trusted.")
                        self.related_ids.append((trust_id, related_id))
                    else:
                        Logger.info(f"Related ID {related_id} is already trusted.")

                Logger.info(f"ID {trust_id} is added and trusted.")
                return TRUST_STATUS.SUCCEED
            case _:
                Logger.warn(f"Unknown trust level {self.level}")
                return TRUST_STATUS.UNKNOWN
            
    def verify_id(self, id) -> bool:
        if (False):
            Logger.warn(f"ID {id} is not valid, access denied.")
            return False
        else:
            return True