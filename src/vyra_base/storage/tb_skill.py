from datetime import datetime
from typing import Any, Optional

from sqlalchemy import JSON, Boolean, DateTime, String, func
from sqlalchemy.orm import Mapped, mapped_column

from vyra_base.storage.tb_base import Base


class Skill(Base):
    """
    SQLAlchemy model for storing skill instances of a VYRA module.

    A skill instance maps a logical skill type (defined in a blueprint) to
    concrete module resources: parameters, volatiles, and interfaces.  Multiple
    instances with the same ``skill_type`` are supported to represent operational
    profiles (e.g. *motion_slow*, *motion_normal*, *motion_fast* all map to the
    ``motion_control`` skill type).

    :ivar id: Unique skill instance identifier within the module (e.g. ``motion_slow``).
    :ivar skill_type: Logical skill type from the blueprint (e.g. ``motion_control``).
    :ivar is_enabled: Whether this skill instance is active.
    :ivar parameter_mapping: Maps skill parameter names to module parameter keys.
        Format: ``{skill_param_name: module_param_key}``
    :ivar volatile_mapping: Maps skill volatile names to module volatile keys.
        Format: ``{skill_volatile_name: module_volatile_key}``
    :ivar interface_mapping: Maps skill interface names to module function names.
        Format: ``{skill_interface_name: module_functionname}``
    :ivar local_defaults: Instance-specific default value overrides applied on top
        of module-level defaults.
        Format: ``{param_key: default_value}``
    :ivar displayname: Optional human-readable label.
    :ivar description: Optional description.
    :ivar tags: Optional list of string tags for grouping/filtering.
    :ivar created_at: Timestamp of creation (auto-set on insert).
    :ivar updated_at: Timestamp of last update (auto-set on insert and update).
    """

    __tablename__ = "skill"

    id: Mapped[str] = mapped_column(
        String(), primary_key=True, unique=True, nullable=False
    )
    skill_type: Mapped[str] = mapped_column(
        String(), nullable=False
    )
    is_enabled: Mapped[bool] = mapped_column(
        Boolean(), nullable=False, default=True
    )
    parameter_mapping: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), nullable=True, default={}
    )
    volatile_mapping: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), nullable=True, default={}
    )
    interface_mapping: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), nullable=True, default={}
    )
    local_defaults: Mapped[Optional[dict[str, Any]]] = mapped_column(
        JSON(), nullable=True, default={}
    )
    displayname: Mapped[Optional[str]] = mapped_column(
        String(), nullable=True, default=None
    )
    description: Mapped[Optional[str]] = mapped_column(
        String(), nullable=True, default=None
    )
    tags: Mapped[Optional[list[str]]] = mapped_column(
        JSON(), nullable=True, default=None
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(), nullable=False, server_default=func.now()
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(), nullable=False, server_default=func.now(), onupdate=func.now()
    )
