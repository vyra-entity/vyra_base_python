from dataclasses import dataclass
import logging

@dataclass(slots=True)
class LogFilter(logging.Filter):
    
    param: str = ''

        

    