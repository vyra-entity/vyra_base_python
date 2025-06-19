from sqlalchemy.ext.asyncio import AsyncAttrs
from sqlalchemy.orm import DeclarativeBase
from sqlalchemy.types import TypeDecorator, Integer

class Base(AsyncAttrs, DeclarativeBase):
    pass

# TypeDecorator für int-Enums
class IntEnum(TypeDecorator):
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