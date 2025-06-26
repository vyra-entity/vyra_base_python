from __future__ import annotations

from typing import Any

class VyraJob:
    """
    Class representing a job in the data layer.
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None

    def merge(self, other: Any) -> VyraJob:
        """
        Merge another Job into this one, combining their attributes.
        :param other: Another Job instance to merge with.
        :return: A new Job instance with merged attributes.
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return

        return self