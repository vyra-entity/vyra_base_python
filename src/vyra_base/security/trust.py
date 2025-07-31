import uuid

from enum import Enum
from pathlib import Path

from vyra_base.helper.logger import Logger


class TRUST_LEVEL(int, Enum):
    TRUST_NONE = 0
    TRUST_ONE_RESERVE = 1
    TRUST_ONE_RELATED = 2
    TRUST_ALL = 3


class TRUST_STATUS(int, Enum):
    SUCCEED = 0
    FAILED = 1
    ALREADY_TRUSTED = 2
    UNKNOWN = 3


class TrustlevelManager:
    def __init__(self):
        """
        Initialize the TrustlevelManager class.
        """
        self.level = 1
        self.ENV_PATH: Path = Path('.')
        self.trusted_ids: list[uuid.UUID] = []
        # Change type annotation to list of tuples
        self.related_ids: list[tuple[uuid.UUID, uuid.UUID]] = []


    def check_trust_access(
            self, 
            trust_id: uuid.UUID, 
            related_ids: list[uuid.UUID] = []) -> TRUST_STATUS:
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
                    return TRUST_STATUS.SUCCEED
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
                    return TRUST_STATUS.ALREADY_TRUSTED
                
                for related_id in related_ids:
                    if not self.verify_id(related_id):
                        Logger.warn(f"Related ID {related_id} is not valid, access denied.")
                        return TRUST_STATUS.FAILED
                    # Check for existence in list of tuples
                    if (trust_id, related_id) not in self.related_ids:
                        Logger.info(f"Related ID {related_id} is added and trusted.")
                        self.related_ids.append((trust_id, related_id))
                    else:
                        Logger.info(f"Related ID {related_id} is already trusted.")

                Logger.info(f"ID {trust_id} is added and trusted.")
                return TRUST_STATUS.SUCCEED
            
            case _:
                Logger.warn(f"Unknown trust level {self.level}")
                return TRUST_STATUS.UNKNOWN

    async def verify_id(self, id) -> bool:
        """
        Verify a module_id if it is a valid UUID5 generated from the module name
        and VYRA namespace.
        """
        return True

        # Example below
        # eh = EnvHandler()
        # await eh.load_env(Path('.'))
        # module_name = eh.env['MODULE_NAME']
        # if (not validate_module_id(id, module_name)):
        #     Logger.warn(f"ID {id} is not valid, access denied.")
        #     return False
        # else:
        #     Logger.info(f"ID {id} is valid for {module_name}.")
        #     return True