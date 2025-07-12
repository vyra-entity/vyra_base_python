import uuid

from vyra_base.defaults.constants import MODULE_ID_NAMESPACE
from vyra_base.helper.logger import Logger

def generate_module_id(module_name: str) -> uuid.uuid5:
    """
    Generate a UUID based on the module name using UUID version 5.

    Args:
        module_name (str): The descriptive name of the module to 
                           generate a UUID for.

    Returns:
        uuid.uuid5: A UUID object generated from the module name.
    """
    return uuid.uuid5(MODULE_ID_NAMESPACE, module_name)

def validate_module_id(module_id: uuid.UUID, module_name: str) -> bool:
    """
    Validate if the given UUID is a valid module ID.

    Args:
        module_id (uuid.UUID): The UUID to validate.

    Returns:
        bool: True if the UUID is valid, False otherwise.
    """
    validated_uuid = uuid.UUID(module_id)  # Beispiel

    # Rekonstruktion der erwarteten UUIDv5
    reconstructed_uuid = uuid.uuid5(MODULE_ID_NAMESPACE, module_name)

    # Vergleich
    if validated_uuid == reconstructed_uuid:
        Logger.info("✅ UUID stammt aus dem Namespace und dem Namen")
    else:
        Logger.warn(
            f"❌ UUID not valid for VYRA namespace or module name: {module_name}")