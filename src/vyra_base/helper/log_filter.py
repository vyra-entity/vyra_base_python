from dataclasses import dataclass
import logging

@dataclass(slots=True)
class LogFilter(logging.Filter):
    """
    Custom logging filter for filtering log records based on parameters.
    
    Extends Python's logging.Filter to provide VYRA-specific log filtering
    capabilities.
    
    :ivar param: Filter parameter string.
    :type param: str
    """
    
    param: str = ''

        

    