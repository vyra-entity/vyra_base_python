
from enum import Enum
from sqlalchemy import JSON, String
from sqlalchemy import Enum as SQLEnum
from sqlalchemy.orm import Mapped
from sqlalchemy.orm import mapped_column
from typing import Any, Optional

from vyra_base.storage.tb_base import Base


class TypeEnum(str, Enum):
    integer = "int"
    string = "string"
    boolean = "bool"
    float = "float"
    list = "list"
    dict = "dict"


class Parameter(Base):
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
