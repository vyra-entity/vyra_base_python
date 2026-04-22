import json
import logging
from typing import Any, Optional

from vyra_base.com import remote_service
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.storage.db_access import DBSTATUS, DbAccess
from vyra_base.storage.db_manipulator import DBReturnValue, DbManipulator
from vyra_base.storage.tb_skill import Skill as DbSkill

logger = logging.getLogger(__name__)


class SkillManager:
    """
    Manages skill instances for a VYRA module.

    A skill instance binds a logical skill type (required by a blueprint) to
    concrete module resources: parameters, volatiles, and interfaces.  This class
    provides both internal CRUD helpers and the Zenoh service endpoints exposed
    via the ``vyra_skills.meta.json`` interface config.

    **Zenoh services** (bound automatically via ``VyraEntity.register_service_callbacks``):

    * ``read_all_skills``  — returns all skill instances as a JSON array
    * ``get_skill``        — returns a single skill by ID
    * ``add_skill``        — creates a new skill instance
    * ``update_skill``     — partially updates an existing skill
    * ``delete_skill``     — deletes a skill by ID

    **Internal helpers** (for use inside module code and blueprint verification):

    * :meth:`list_skills`           — async list of all Skill rows
    * :meth:`get_skill_impl`        — async dict-or-None lookup
    * :meth:`get_skills_by_type`    — async filter by ``skill_type``
    * :meth:`add_skill_impl`        — async insert
    * :meth:`update_skill_impl`     — async partial update
    * :meth:`delete_skill_impl`     — async delete by ID

    :param database_access: Persistent database access object.
    :type database_access: DbAccess
    """

    def __init__(self, database_access: DbAccess) -> None:
        """
        Initialize the SkillManager.

        :param database_access: Persistent database access for the module's SQLite DB.
        :type database_access: DbAccess
        :raises TypeError: If ``database_access`` is not a :class:`~vyra_base.storage.db_access.DbAccess`.
        """
        if not isinstance(database_access, DbAccess):
            raise TypeError("database_access must be of type DbAccess.")

        self._manipulator = DbManipulator(database_access, DbSkill)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @ErrorTraceback.w_check_error_exist
    async def list_skills(self) -> list[dict[str, Any]]:
        """
        Return all skill instances as a list of dicts.

        :return: List of skill dicts; empty list if none exist.
        :rtype: list[dict[str, Any]]
        """
        result: DBReturnValue = await self._manipulator.get_all()
        if result.status != DBSTATUS.SUCCESS:
            logger.warning("list_skills: DB query failed — %s", result.details)
            return []
        if not isinstance(result.value, list):
            return []
        return [self._manipulator.to_dict(s) for s in result.value]

    @ErrorTraceback.w_check_error_exist
    async def get_skill_impl(self, skill_id: str) -> Optional[dict[str, Any]]:
        """
        Return a single skill instance dict by its ``id``, or ``None`` if not found.

        :param skill_id: The unique skill instance identifier.
        :type skill_id: str
        :return: Skill dict or ``None``.
        :rtype: Optional[dict[str, Any]]
        """
        result: DBReturnValue = await self._manipulator.get_all(
            filters={"id": skill_id}
        )
        if result.status == DBSTATUS.NOT_FOUND or result.status != DBSTATUS.SUCCESS:
            return None
        if not isinstance(result.value, list) or len(result.value) == 0:
            return None
        return self._manipulator.to_dict(result.value[0])

    @ErrorTraceback.w_check_error_exist
    async def get_skills_by_type(self, skill_type: str) -> list[dict[str, Any]]:
        """
        Return all skill instances that match a given ``skill_type``.

        Used by the blueprint verifier to check whether a module satisfies
        a required skill type.

        :param skill_type: The logical skill type to filter by (e.g. ``motion_control``).
        :type skill_type: str
        :return: List of matching skill dicts; empty list if none.
        :rtype: list[dict[str, Any]]
        """
        result: DBReturnValue = await self._manipulator.get_all(
            filters={"skill_type": skill_type}
        )
        if result.status == DBSTATUS.NOT_FOUND or result.status != DBSTATUS.SUCCESS:
            return []
        if not isinstance(result.value, list):
            return []
        return [self._manipulator.to_dict(s) for s in result.value]

    @ErrorTraceback.w_check_error_exist
    async def add_skill_impl(
        self,
        id: str,
        skill_type: str,
        parameter_mapping: Optional[dict[str, Any]] = None,
        volatile_mapping: Optional[dict[str, Any]] = None,
        interface_mapping: Optional[dict[str, Any]] = None,
        local_defaults: Optional[dict[str, Any]] = None,
        displayname: Optional[str] = None,
        description: Optional[str] = None,
        tags: Optional[list[str]] = None,
    ) -> dict[str, Any]:
        """
        Create a new skill instance.

        :param id: Unique skill instance ID within the module.
        :param skill_type: Logical skill type from the blueprint.
        :param parameter_mapping: ``{skill_param: module_param_key}`` mapping.
        :param volatile_mapping: ``{skill_volatile: module_volatile_key}`` mapping.
        :param interface_mapping: ``{skill_interface: module_functionname}`` mapping.
        :param local_defaults: Instance-specific default overrides.
        :param displayname: Optional human-readable label.
        :param description: Optional description.
        :param tags: Optional list of string tags.
        :return: ``{"success": bool, "message": str}``
        :rtype: dict[str, Any]
        """
        data = {
            "id": id,
            "skill_type": skill_type,
            "is_enabled": True,
            "parameter_mapping": parameter_mapping or {},
            "volatile_mapping": volatile_mapping or {},
            "interface_mapping": interface_mapping or {},
            "local_defaults": local_defaults or {},
            "displayname": displayname,
            "description": description,
            "tags": tags,
        }
        result: DBReturnValue = await self._manipulator.add(data)
        if result.status != DBSTATUS.SUCCESS:
            msg = f"add_skill: failed to create skill '{id}' — {result.details}"
            logger.warning(msg)
            return {"success": False, "message": msg}
        return {"success": True, "message": f"Skill '{id}' created successfully."}

    @ErrorTraceback.w_check_error_exist
    async def update_skill_impl(self, id: str, **kwargs: Any) -> dict[str, Any]:
        """
        Partially update an existing skill instance.

        Only the fields present in ``kwargs`` are updated.  Supported fields:
        ``skill_type``, ``is_enabled``, ``parameter_mapping``, ``volatile_mapping``,
        ``interface_mapping``, ``local_defaults``, ``displayname``, ``description``,
        ``tags``.

        :param id: Skill instance ID to update.
        :param kwargs: Fields to update.
        :return: ``{"success": bool, "message": str}``
        :rtype: dict[str, Any]
        """
        allowed = {
            "skill_type", "is_enabled", "parameter_mapping", "volatile_mapping",
            "interface_mapping", "local_defaults", "displayname", "description", "tags",
        }
        update_data = {k: v for k, v in kwargs.items() if k in allowed}
        update_data["id"] = id

        result: DBReturnValue = await self._manipulator.update(
            update_data, filters={"id": id}
        )
        if result.status != DBSTATUS.SUCCESS:
            msg = f"update_skill: failed to update skill '{id}' — {result.details}"
            logger.warning(msg)
            return {"success": False, "message": msg}
        return {"success": True, "message": f"Skill '{id}' updated successfully."}

    @ErrorTraceback.w_check_error_exist
    async def delete_skill_impl(self, id: str) -> dict[str, Any]:
        """
        Delete a skill instance by its ID.

        :param id: Skill instance ID to delete.
        :return: ``{"success": bool, "message": str}``
        :rtype: dict[str, Any]
        """
        skill = await self.get_skill_impl(id)
        if skill is None:
            msg = f"delete_skill: skill '{id}' not found."
            logger.warning(msg)
            return {"success": False, "message": msg}

        result: DBReturnValue = await self._manipulator.delete(id)
        if result.status != DBSTATUS.SUCCESS:
            msg = f"delete_skill: failed to delete skill '{id}' — {result.details}"
            logger.warning(msg)
            return {"success": False, "message": msg}
        return {"success": True, "message": f"Skill '{id}' deleted successfully."}

    # ------------------------------------------------------------------
    # Zenoh service endpoints  (bound by VyraEntity.register_service_callbacks)
    # ------------------------------------------------------------------

    @remote_service()
    async def read_all_skills(self, request: Any, response: Any) -> bool:
        """
        Zenoh service: return all skill instances as a JSON array.

        Maps to ``read_all_skills`` in ``vyra_skills.meta.json``.

        :param request: Incoming service request (no parameters expected).
        :param response: Service response; ``all_skills_json`` is set.
        :return: Always ``True``.
        :rtype: bool
        """
        skills = await self.list_skills()
        response.all_skills_json = json.dumps(skills, default=str)
        return True

    @remote_service()
    async def get_skill(self, request: Any, response: Any) -> bool:
        """
        Zenoh service: return a single skill instance by ID.

        Maps to ``get_skill`` in ``vyra_skills.meta.json``.

        :param request: Service request; must have ``id`` attribute.
        :param response: Service response; ``success``, ``message``, ``skill_json`` are set.
        :return: ``True`` if found, ``False`` otherwise.
        :rtype: bool
        """
        skill_id = request.id
        skill = await self.get_skill_impl(skill_id)
        if skill is None:
            response.success = False
            response.message = f"Skill '{skill_id}' not found."
            response.skill_json = ""
            return False
        response.success = True
        response.message = f"Skill '{skill_id}' retrieved successfully."
        response.skill_json = json.dumps(skill, default=str)
        return True

    @remote_service()
    async def add_skill(self, request: Any, response: Any) -> bool:
        """
        Zenoh service: create a new skill instance.

        Maps to ``add_skill`` in ``vyra_skills.meta.json``.

        :param request: Service request with fields: ``id``, ``skill_type``,
            ``parameter_mapping`` (JSON str), ``volatile_mapping`` (JSON str),
            ``interface_mapping`` (JSON str), ``local_defaults`` (JSON str),
            ``displayname``, ``description``, ``tags`` (JSON str).
        :param response: Service response; ``success``, ``message`` are set.
        :return: ``True`` on success.
        :rtype: bool
        """
        def _parse(value: str) -> Any:
            """Parse optional JSON string, returning empty dict/list on blank input."""
            if not value:
                return {}
            try:
                return json.loads(value)
            except (json.JSONDecodeError, TypeError):
                return {}

        tags_raw = getattr(request, "tags", "")
        tags = _parse(tags_raw) if tags_raw else None
        if isinstance(tags, dict):
            tags = None  # tags must be a list or None

        result = await self.add_skill_impl(
            id=request.id,
            skill_type=request.skill_type,
            parameter_mapping=_parse(getattr(request, "parameter_mapping", "")),
            volatile_mapping=_parse(getattr(request, "volatile_mapping", "")),
            interface_mapping=_parse(getattr(request, "interface_mapping", "")),
            local_defaults=_parse(getattr(request, "local_defaults", "")),
            displayname=getattr(request, "displayname", None) or None,
            description=getattr(request, "description", None) or None,
            tags=tags,
        )
        response.success = result["success"]
        response.message = result["message"]
        return result["success"]

    @remote_service()
    async def update_skill(self, request: Any, response: Any) -> bool:
        """
        Zenoh service: partially update an existing skill instance.

        Maps to ``update_skill`` in ``vyra_skills.meta.json``.

        :param request: Service request with ``id`` and any combination of
            ``skill_type``, ``is_enabled``, ``parameter_mapping``, ``volatile_mapping``,
            ``interface_mapping``, ``local_defaults``, ``displayname``,
            ``description``, ``tags`` (JSON strings for mapping fields).
        :param response: Service response; ``success``, ``message`` are set.
        :return: ``True`` on success.
        :rtype: bool
        """
        def _parse(value: Any, fallback: Any = None) -> Any:
            if value is None or value == "":
                return fallback
            if isinstance(value, str):
                try:
                    return json.loads(value)
                except (json.JSONDecodeError, TypeError):
                    return fallback
            return value

        kwargs: dict[str, Any] = {}

        skill_type = getattr(request, "skill_type", "")
        if skill_type:
            kwargs["skill_type"] = skill_type

        is_enabled_raw = getattr(request, "is_enabled", None)
        if is_enabled_raw is not None and is_enabled_raw != "":
            kwargs["is_enabled"] = bool(is_enabled_raw)

        for field in ("parameter_mapping", "volatile_mapping", "interface_mapping", "local_defaults"):
            raw = getattr(request, field, "")
            parsed = _parse(raw)
            if parsed is not None:
                kwargs[field] = parsed

        displayname = getattr(request, "displayname", "")
        if displayname:
            kwargs["displayname"] = displayname

        desc = getattr(request, "description", "")
        if desc:
            kwargs["description"] = desc

        tags_raw = getattr(request, "tags", "")
        if tags_raw:
            tags = _parse(tags_raw, fallback=[])
            kwargs["tags"] = tags if isinstance(tags, list) else []

        result = await self.update_skill_impl(request.id, **kwargs)
        response.success = result["success"]
        response.message = result["message"]
        return result["success"]

    @remote_service()
    async def delete_skill(self, request: Any, response: Any) -> bool:
        """
        Zenoh service: delete a skill instance by ID.

        Maps to ``delete_skill`` in ``vyra_skills.meta.json``.

        :param request: Service request with ``id`` attribute.
        :param response: Service response; ``success``, ``message`` are set.
        :return: ``True`` on success.
        :rtype: bool
        """
        result = await self.delete_skill_impl(request.id)
        response.success = result["success"]
        response.message = result["message"]
        return result["success"]
