
from enum import Enum
from sqlalchemy import JSON, String
from sqlalchemy import Enum as SQLEnum
from sqlalchemy.orm import Mapped
from sqlalchemy.orm import mapped_column
from typing import Any, Optional

from vyra_base.storage.tb_base import Base


class TypeEnum(str, Enum):
    """
    Enumeration of supported parameter data types in VYRA.
    
    Defines the types that can be stored and validated in the parameter system.
    
    :cvar integer: Integer type.
    :cvar string: String type.
    :cvar boolean: Boolean type.
    :cvar float: Floating point type.
    :cvar list: List/array type.
    :cvar dict: Dictionary/object type.
    """
    integer = "int"
    string = "string"
    boolean = "bool"
    float = "float"
    list = "list"
    dict = "dict"


class Parameter(Base):
    """
    SQLAlchemy model for storing module parameters with metadata.
    
    Stores runtime-configurable parameters with type information, constraints,
    display metadata, and visibility/editability flags. Parameters can be
    used for module configuration and are accessible via the VYRA API.
    
    :ivar name: Unique parameter identifier.
    :ivar value: Current parameter value (JSON-serialized).
    :ivar default_value: Default value for reset operations.
    :ivar type: Parameter data type (TypeEnum).
    :ivar visible: Whether parameter appears in UI.
    :ivar editable: Whether parameter can be modified.
    :ivar displayname: Human-readable display name.
    :ivar description: Parameter description for documentation.
    :ivar min_value: Minimum allowed value (for numeric types).
    :ivar max_value: Maximum allowed value (for numeric types).
    :ivar range_value: Allowed value range/options (for constrained types).
    """
    __tablename__ = "parameter"

    name: Mapped[str] = mapped_column(
        primary_key=True,
        unique=True)
    value: Mapped[dict[str, Any]] = mapped_column(
        JSON(), nullable=False)
    default_value: Mapped[dict[str, Any]] = mapped_column(
        JSON(), nullable=False, default={})
    type: Mapped[TypeEnum] = mapped_column(
        SQLEnum(TypeEnum, values_callable=lambda enum: [e.value for e in enum]),
        nullable=False, unique=True)
    visible: Mapped[bool] = mapped_column(
        nullable=False, default=True)
    editable: Mapped[bool] = mapped_column(
        nullable=False, default=True)
    displayname: Mapped[str] = mapped_column(
        nullable=False, default="")
    description: Mapped[str] = mapped_column(
        nullable=False, default="")
    min_value: Mapped[Optional[str]] = mapped_column(
        String(), nullable=True, default=None)
    max_value: Mapped[Optional[str]] = mapped_column(
        String(), nullable=True, default=None)
    range_value: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), nullable=True, default=None)
