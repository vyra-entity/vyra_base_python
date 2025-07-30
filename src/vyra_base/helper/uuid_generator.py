import uuid

from vyra_base.defaults.constants import MODULE_ID_NAMESPACE
from vyra_base.helper.logger import Logger

def generate_module_id(module_name: str) -> uuid.UUID:
    """
    Generate a UUID based on the module name using UUID version 5.

    Args:
        module_name (str): The descriptive name of the module to 
                           generate a UUID for.

    Returns:
        uuid.uuid5: A UUID object generated from the module name.
    """
    return uuid.uuid5(MODULE_ID_NAMESPACE, module_name)

def validate_module_id(module_id: uuid.UUID | str, module_name: str) -> bool:
    """
    Validate if the given UUID is a valid module ID.

    Args:
        module_id (uuid.UUID): The UUID to validate.

    Returns:
        bool: True if the UUID is valid, False otherwise.
    """
    if isinstance(module_id, str):  
        validated_uuid = uuid.UUID(module_id)  # Beispiel
    elif isinstance(module_id, uuid.UUID):
        validated_uuid = module_id
    else:
        Logger.warn(f"Invalid type for module_id: {type(module_id)}")
        return False
    
    # Rekonstruktion der erwarteten UUIDv5
    reconstructed_uuid: uuid.UUID = uuid.uuid5(MODULE_ID_NAMESPACE, module_name)

    # Vergleich
    if validated_uuid == reconstructed_uuid:
        Logger.info("✅ UUID stammt aus dem Namespace und dem Namen")
        return True
    else:
        Logger.warn(
            f"❌ UUID not valid for VYRA namespace or module name: {module_name}")
        return False