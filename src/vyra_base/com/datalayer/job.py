from __future__ import annotations

from typing import Any

from vyra_base.helper.logger import Logger

class VyraJob:
    """
    Represents a job in the data layer.

    :ivar name: The name of the job.
    :vartype name: str
    :ivar type: The type of the job.
    :vartype type: Any
    :ivar description: The description of the job.
    :vartype description: str
    :ivar last_return: The last return value of the job.
    :vartype last_return: Any
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None

    def merge(self, other: Any) -> VyraJob:
        """
        Merges another Job into this one, combining their attributes.

        :param other: Another Job instance to merge with.
        :type other: Any
        :return: A new Job instance with merged attributes.
        :rtype: VyraJob
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return

        return self
    
    def __del__(self):
        """
        Destructor to clean up the job.
        If the job has a service server, it will be destroyed.
        """
        Logger.log(f"<NOT IMPLEMENTED> VyraJob '{self.name}' destroyed.")
        pass
        # if hasattr(self, 'service_server') and self.service_server:
        #     self.service_server.destroy_service()
        #     self.service_server = None
        #     Logger.log(f"VyraJob '{self.name}' destroyed.")


class VyraJobRunner:
    """
    Represents a job in the data layer.

    :ivar name: The name of the job.
    :vartype name: str
    :ivar type: The type of the job.
    :vartype type: Any
    :ivar description: The description of the job.
    :vartype description: str
    :ivar last_return: The last return value of the job.
    :vartype last_return: Any
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None

    def merge(self, other: Any) -> VyraJobRunner:
        """
        Merges another Job into this one, combining their attributes.

        :param other: Another Job instance to merge with.
        :type other: Any
        :return: A new Job instance with merged attributes.
        :rtype: VyraJob
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return

        return self
    
    def __del__(self):
        """
        Destructor to clean up the job.
        If the job has a service server, it will be destroyed.
        """
        Logger.log(f"<NOT IMPLEMENTED> VyraJob '{self.name}' destroyed.")
        pass
        # if hasattr(self, 'service_server') and self.service_server:
        #     self.service_server.destroy_service()
        #     self.service_server = None
        #     Logger.log(f"VyraJob '{self.name}' destroyed.")