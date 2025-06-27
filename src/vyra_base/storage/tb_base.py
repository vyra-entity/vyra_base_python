from sqlalchemy.ext.asyncio import AsyncAttrs
from sqlalchemy.orm import DeclarativeBase
from sqlalchemy.types import TypeDecorator, Integer

class Base(AsyncAttrs, DeclarativeBase):
    """
    Base class for all SQLAlchemy models in the application.
    """
    pass

# TypeDecorator für int-Enums
class IntEnum(TypeDecorator):
    """
    A SQLAlchemy TypeDecorator for storing Python IntEnum values as integers in the database.

    This class allows seamless conversion between Python IntEnum members and their integer
    representation in the database. When binding parameters, it converts IntEnum members
    to their integer values. When retrieving results, it converts integers back to the
    corresponding IntEnum members.

    :param enumtype: The IntEnum class to use for conversion.
    :type enumtype: Type[IntEnum]
    """
    impl = Integer

    def __init__(self, enumtype, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._enumtype = enumtype

    def process_bind_param(self, value, dialect):
        if value is None:
            return None
        if isinstance(value, int):
            return value
        return value.value

    def process_result_value(self, value, dialect):
        if value is None:
            return None
        return self._enumtype(value)