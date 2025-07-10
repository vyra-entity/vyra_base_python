from pathlib import Path
from vyra_base.helper.env_handler import EnvHandler

class ACCESS_LEVEL:
    """
    Enum for access levels.
    """
    NO_ACCESS = 0
    READ_ONLY = 1
    EXTENDED_READ = 2
    CONTROL_AND_PARAM = 3
    EXTENDED_CONTROL = 4
    FULL_ACCESS = 5

    def __repr__(self):
        pass

class AccessManager:
    """
    AccessManager class to manage access levels.
    """
    
    def __init__(self):
        """
        Initialize the AccessManager class.
        """
        pass


    def blabli(self):
        pass