from dataclasses import dataclass
from enum import Enum
from typing import Final, NamedTuple
import uuid

# Namespace to generate unique uuidv5 for a specific module id
MODULE_ID_NAMESPACE: uuid.UUID = uuid.uuid5(uuid.NAMESPACE_DNS, "VYRA")
