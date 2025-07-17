
from email.policy import default
from enum import Enum
from sqlalchemy import Enum as SQLEnum
from sqlalchemy import JSON
from sqlalchemy import JSON

from sqlalchemy.orm import Mapped
from sqlalchemy.orm import mapped_column
from sqlalchemy.orm import relationship

from typing import Any

from vyra_base.storage.tb_base import Base


class Parameter(Base):
    __tablename__ = "parameters"

    name: Mapped[int] = mapped_column(
        primary_key=True,
        unique=True)
    #############################################################################
    value: Mapped[dict[str, Any]] = mapped_column(
        JSON(), nullable=False)
    default_value: Mapped[dict[str, Any]] = mapped_column(
        JSON(), nullable=False, default={})
    type: Mapped[str] = mapped_column(
        nullable=True, default="string")
    visible: Mapped[bool] = mapped_column(
        nullable=False, default=True)
    editable: Mapped[bool] = mapped_column(
        nullable=False, default=True)
    displayname: Mapped[str] = mapped_column(
        nullable=False, default="")
    description: Mapped[str] = mapped_column(
        nullable=False, default="")
    min_value: Mapped[Any] = mapped_column(
        nullable=True, default=None)
    max_value: Mapped[Any] = mapped_column(
        nullable=True, default=None)
    range_value: Mapped[tuple[Any, ...]] = mapped_column(
        JSON(), nullable=True, default=None)
