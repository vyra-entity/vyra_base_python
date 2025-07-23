
from sqlalchemy import JSON, String
from sqlalchemy.orm import Mapped
from sqlalchemy.orm import mapped_column
from typing import Any, Optional

from vyra_base.storage.tb_base import Base


class Parameter(Base):
    __tablename__ = "parameters"

    name: Mapped[str] = mapped_column(
        primary_key=True,
        unique=True)
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
    min_value: Mapped[Optional[str]] = mapped_column(
        String(), nullable=True, default=None)
    max_value: Mapped[Optional[str]] = mapped_column(
        String(), nullable=True, default=None)
    range_value: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), nullable=True, default=None)
