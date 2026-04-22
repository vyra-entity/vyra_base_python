"""
Unit tests for vyra_base.core.skill_manager.SkillManager

Tests cover:
- Constructor validation
- Internal CRUD helpers (list_skills, get_skill_impl, get_skills_by_type,
  add_skill_impl, update_skill_impl, delete_skill_impl)
- Zenoh service endpoints (read_all_skills, get_skill, add_skill,
  update_skill, delete_skill)

All database calls are mocked — no real DB required.
"""

import json
import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from vyra_base.storage.db_access import DBSTATUS, DbAccess
from vyra_base.storage.db_manipulator import DBReturnValue


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_db_access() -> MagicMock:
    """Return a MagicMock that satisfies isinstance(x, DbAccess)."""
    mock = MagicMock(spec=DbAccess)
    return mock


def _success(value=None) -> DBReturnValue:
    rv = DBReturnValue()
    rv.status = DBSTATUS.SUCCESS
    rv.value = value if value is not None else []
    return rv


def _not_found() -> DBReturnValue:
    rv = DBReturnValue()
    rv.status = DBSTATUS.NOT_FOUND
    rv.value = []
    rv.details = "not found"
    return rv


def _error(details: str = "DB error") -> DBReturnValue:
    rv = DBReturnValue()
    rv.status = DBSTATUS.ERROR
    rv.value = "error"
    rv.details = details
    return rv


def _fake_skill_row(
    id: str = "motion_slow",
    skill_type: str = "motion_control",
    is_enabled: bool = True,
    parameter_mapping: dict = None,
    volatile_mapping: dict = None,
    interface_mapping: dict = None,
    local_defaults: dict = None,
    displayname: str = "Motion Slow",
    description: str = "Test skill",
    tags: list = None,
) -> MagicMock:
    """Return a MagicMock that looks like a Skill ORM row."""
    row = MagicMock()
    row.id = id
    row.skill_type = skill_type
    row.is_enabled = is_enabled
    row.parameter_mapping = parameter_mapping or {}
    row.volatile_mapping = volatile_mapping or {}
    row.interface_mapping = interface_mapping or {}
    row.local_defaults = local_defaults or {}
    row.displayname = displayname
    row.description = description
    row.tags = tags or []
    row.created_at = "2026-01-01T00:00:00"
    row.updated_at = "2026-01-01T00:00:00"
    return row


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def db_access():
    return _make_db_access()


@pytest.fixture
def skill_manager(db_access):
    """Return a SkillManager with a mocked DbManipulator."""
    from vyra_base.core.skill_manager import SkillManager

    with patch("vyra_base.core.skill_manager.DbManipulator") as MockManipulator:
        mock_manipulator = MagicMock()
        MockManipulator.return_value = mock_manipulator
        manager = SkillManager(database_access=db_access)
        manager._manipulator = mock_manipulator
        mock_manipulator.to_dict = lambda obj: {
            "id": obj.id,
            "skill_type": obj.skill_type,
            "is_enabled": obj.is_enabled,
            "parameter_mapping": obj.parameter_mapping,
            "volatile_mapping": obj.volatile_mapping,
            "interface_mapping": obj.interface_mapping,
            "local_defaults": obj.local_defaults,
            "displayname": obj.displayname,
            "description": obj.description,
            "tags": obj.tags,
            "created_at": str(obj.created_at),
            "updated_at": str(obj.updated_at),
        }
        yield manager


# ---------------------------------------------------------------------------
# Constructor
# ---------------------------------------------------------------------------

class TestSkillManagerInit:
    """Tests for SkillManager.__init__"""

    def test_init_requires_db_access(self):
        """TypeError when database_access is not DbAccess."""
        from vyra_base.core.skill_manager import SkillManager

        with pytest.raises(TypeError):
            SkillManager(database_access="not_a_db_access")

    def test_init_with_mock_db_access(self, db_access):
        """SkillManager constructs successfully with a valid DbAccess mock."""
        from vyra_base.core.skill_manager import SkillManager

        with patch("vyra_base.core.skill_manager.DbManipulator"):
            manager = SkillManager(database_access=db_access)
            assert manager is not None


# ---------------------------------------------------------------------------
# list_skills
# ---------------------------------------------------------------------------

class TestListSkills:
    """Tests for SkillManager.list_skills"""

    @pytest.mark.asyncio
    async def test_list_skills_empty(self, skill_manager):
        """Returns empty list when no skills exist."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([]))
        result = await skill_manager.list_skills()
        assert result == []

    @pytest.mark.asyncio
    async def test_list_skills_returns_dicts(self, skill_manager):
        """Returns list of dicts for existing rows."""
        row = _fake_skill_row()
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        result = await skill_manager.list_skills()
        assert len(result) == 1
        assert result[0]["id"] == "motion_slow"
        assert result[0]["skill_type"] == "motion_control"

    @pytest.mark.asyncio
    async def test_list_skills_db_error_returns_empty(self, skill_manager):
        """Returns empty list on DB error (does not raise)."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_error())
        result = await skill_manager.list_skills()
        assert result == []


# ---------------------------------------------------------------------------
# get_skill_impl
# ---------------------------------------------------------------------------

class TestGetSkillImpl:
    """Tests for SkillManager.get_skill_impl"""

    @pytest.mark.asyncio
    async def test_get_skill_found(self, skill_manager):
        """Returns dict when skill exists."""
        row = _fake_skill_row(id="motion_slow")
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        result = await skill_manager.get_skill_impl("motion_slow")
        assert result is not None
        assert result["id"] == "motion_slow"

    @pytest.mark.asyncio
    async def test_get_skill_not_found(self, skill_manager):
        """Returns None when skill does not exist."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_not_found())
        result = await skill_manager.get_skill_impl("nonexistent")
        assert result is None

    @pytest.mark.asyncio
    async def test_get_skill_empty_list(self, skill_manager):
        """Returns None when DB returns SUCCESS but empty list."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([]))
        result = await skill_manager.get_skill_impl("motion_slow")
        assert result is None


# ---------------------------------------------------------------------------
# get_skills_by_type
# ---------------------------------------------------------------------------

class TestGetSkillsByType:
    """Tests for SkillManager.get_skills_by_type"""

    @pytest.mark.asyncio
    async def test_returns_matching_skills(self, skill_manager):
        """Returns skills matching the given skill_type."""
        rows = [_fake_skill_row(id=f"motion_{i}", skill_type="motion_control") for i in range(3)]
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success(rows))
        result = await skill_manager.get_skills_by_type("motion_control")
        assert len(result) == 3
        assert all(r["skill_type"] == "motion_control" for r in result)

    @pytest.mark.asyncio
    async def test_returns_empty_when_none_match(self, skill_manager):
        """Returns empty list when no skills match."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_not_found())
        result = await skill_manager.get_skills_by_type("unknown_type")
        assert result == []


# ---------------------------------------------------------------------------
# add_skill_impl
# ---------------------------------------------------------------------------

class TestAddSkillImpl:
    """Tests for SkillManager.add_skill_impl"""

    @pytest.mark.asyncio
    async def test_add_skill_success(self, skill_manager):
        """Returns success dict on successful insert."""
        skill_manager._manipulator.add = AsyncMock(return_value=_success())
        result = await skill_manager.add_skill_impl(
            id="motion_slow", skill_type="motion_control"
        )
        assert result["success"] is True
        assert "motion_slow" in result["message"]

    @pytest.mark.asyncio
    async def test_add_skill_db_error(self, skill_manager):
        """Returns failure dict when DB insert fails."""
        skill_manager._manipulator.add = AsyncMock(return_value=_error("unique constraint"))
        result = await skill_manager.add_skill_impl(
            id="motion_slow", skill_type="motion_control"
        )
        assert result["success"] is False

    @pytest.mark.asyncio
    async def test_add_skill_defaults_empty_mappings(self, skill_manager):
        """Mappings default to empty dicts when None is passed."""
        skill_manager._manipulator.add = AsyncMock(return_value=_success())
        await skill_manager.add_skill_impl(id="s1", skill_type="t1")
        call_kwargs = skill_manager._manipulator.add.call_args[0][0]
        assert call_kwargs["parameter_mapping"] == {}
        assert call_kwargs["volatile_mapping"] == {}
        assert call_kwargs["interface_mapping"] == {}
        assert call_kwargs["local_defaults"] == {}

    @pytest.mark.asyncio
    async def test_add_skill_with_all_fields(self, skill_manager):
        """All optional fields are forwarded to the DB."""
        skill_manager._manipulator.add = AsyncMock(return_value=_success())
        await skill_manager.add_skill_impl(
            id="motion_slow",
            skill_type="motion_control",
            parameter_mapping={"max_speed": "drive.max_speed_mps"},
            volatile_mapping={"pos": "robot.pos"},
            interface_mapping={"move": "drive_move"},
            local_defaults={"drive.max_speed_mps": 0.3},
            displayname="Motion Slow",
            description="Safe speed",
            tags=["motion", "safe"],
        )
        call_data = skill_manager._manipulator.add.call_args[0][0]
        assert call_data["parameter_mapping"] == {"max_speed": "drive.max_speed_mps"}
        assert call_data["tags"] == ["motion", "safe"]


# ---------------------------------------------------------------------------
# update_skill_impl
# ---------------------------------------------------------------------------

class TestUpdateSkillImpl:
    """Tests for SkillManager.update_skill_impl"""

    @pytest.mark.asyncio
    async def test_update_skill_success(self, skill_manager):
        """Returns success dict on successful update."""
        skill_manager._manipulator.update = AsyncMock(return_value=_success())
        result = await skill_manager.update_skill_impl(
            "motion_slow", is_enabled=False
        )
        assert result["success"] is True

    @pytest.mark.asyncio
    async def test_update_skill_db_error(self, skill_manager):
        """Returns failure dict when DB update fails."""
        skill_manager._manipulator.update = AsyncMock(return_value=_error("row not found"))
        result = await skill_manager.update_skill_impl("motion_slow", displayname="New Name")
        assert result["success"] is False

    @pytest.mark.asyncio
    async def test_update_filters_unknown_fields(self, skill_manager):
        """Unknown kwargs are silently ignored; only allowed fields are passed."""
        skill_manager._manipulator.update = AsyncMock(return_value=_success())
        await skill_manager.update_skill_impl(
            "motion_slow", is_enabled=True, unknown_field="ignored"
        )
        update_data = skill_manager._manipulator.update.call_args[0][0]
        assert "unknown_field" not in update_data
        assert "is_enabled" in update_data


# ---------------------------------------------------------------------------
# delete_skill_impl
# ---------------------------------------------------------------------------

class TestDeleteSkillImpl:
    """Tests for SkillManager.delete_skill_impl"""

    @pytest.mark.asyncio
    async def test_delete_skill_success(self, skill_manager):
        """Returns success dict when skill is deleted."""
        row = _fake_skill_row()
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        skill_manager._manipulator.delete = AsyncMock(return_value=_success())
        result = await skill_manager.delete_skill_impl("motion_slow")
        assert result["success"] is True

    @pytest.mark.asyncio
    async def test_delete_skill_not_found(self, skill_manager):
        """Returns failure dict when skill does not exist."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_not_found())
        result = await skill_manager.delete_skill_impl("nonexistent")
        assert result["success"] is False
        assert "not found" in result["message"]

    @pytest.mark.asyncio
    async def test_delete_skill_db_error(self, skill_manager):
        """Returns failure dict when DB delete fails."""
        row = _fake_skill_row()
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        skill_manager._manipulator.delete = AsyncMock(return_value=_error("constraint"))
        result = await skill_manager.delete_skill_impl("motion_slow")
        assert result["success"] is False


# ---------------------------------------------------------------------------
# Zenoh service: read_all_skills
# ---------------------------------------------------------------------------

class TestReadAllSkillsService:
    """Zenoh service endpoint: read_all_skills"""

    @pytest.mark.asyncio
    async def test_returns_json_array(self, skill_manager):
        """Response contains a valid JSON array string."""
        row = _fake_skill_row()
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        request = MagicMock()
        response = MagicMock()
        await skill_manager.read_all_skills(request, response)
        payload = json.loads(response.all_skills_json)
        assert isinstance(payload, list)
        assert payload[0]["id"] == "motion_slow"

    @pytest.mark.asyncio
    async def test_returns_empty_array_when_no_skills(self, skill_manager):
        """Response is an empty JSON array when no skills exist."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([]))
        request = MagicMock()
        response = MagicMock()
        await skill_manager.read_all_skills(request, response)
        assert response.all_skills_json == "[]"


# ---------------------------------------------------------------------------
# Zenoh service: get_skill
# ---------------------------------------------------------------------------

class TestGetSkillService:
    """Zenoh service endpoint: get_skill"""

    @pytest.mark.asyncio
    async def test_get_skill_found(self, skill_manager):
        """Sets success=True and skill_json when skill exists."""
        row = _fake_skill_row()
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        request = MagicMock()
        request.id = "motion_slow"
        response = MagicMock()
        returned = await skill_manager.get_skill(request, response)
        assert returned is True
        assert response.success is True
        parsed = json.loads(response.skill_json)
        assert parsed["id"] == "motion_slow"

    @pytest.mark.asyncio
    async def test_get_skill_not_found(self, skill_manager):
        """Sets success=False and empty skill_json when skill not found."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_not_found())
        request = MagicMock()
        request.id = "nonexistent"
        response = MagicMock()
        returned = await skill_manager.get_skill(request, response)
        assert returned is False
        assert response.success is False
        assert response.skill_json == ""


# ---------------------------------------------------------------------------
# Zenoh service: add_skill
# ---------------------------------------------------------------------------

class TestAddSkillService:
    """Zenoh service endpoint: add_skill"""

    def _make_request(
        self,
        skill_id: str = "motion_slow",
        skill_type: str = "motion_control",
        parameter_mapping: str = "{}",
        volatile_mapping: str = "{}",
        interface_mapping: str = "{}",
        local_defaults: str = "{}",
        displayname: str = "",
        description: str = "",
        tags: str = "",
    ) -> MagicMock:
        req = MagicMock()
        req.id = skill_id
        req.skill_type = skill_type
        req.parameter_mapping = parameter_mapping
        req.volatile_mapping = volatile_mapping
        req.interface_mapping = interface_mapping
        req.local_defaults = local_defaults
        req.displayname = displayname
        req.description = description
        req.tags = tags
        return req

    @pytest.mark.asyncio
    async def test_add_skill_success(self, skill_manager):
        """Sets response.success=True on successful creation."""
        skill_manager._manipulator.add = AsyncMock(return_value=_success())
        request = self._make_request()
        response = MagicMock()
        await skill_manager.add_skill(request, response)
        assert response.success is True

    @pytest.mark.asyncio
    async def test_add_skill_parses_json_fields(self, skill_manager):
        """JSON string fields are deserialized before insert."""
        skill_manager._manipulator.add = AsyncMock(return_value=_success())
        request = self._make_request(
            parameter_mapping='{"max_speed": "drive.max_speed_mps"}',
            tags='["motion", "safe"]',
        )
        response = MagicMock()
        await skill_manager.add_skill(request, response)
        call_data = skill_manager._manipulator.add.call_args[0][0]
        assert call_data["parameter_mapping"] == {"max_speed": "drive.max_speed_mps"}
        assert call_data["tags"] == ["motion", "safe"]


# ---------------------------------------------------------------------------
# Zenoh service: update_skill
# ---------------------------------------------------------------------------

class TestUpdateSkillService:
    """Zenoh service endpoint: update_skill"""

    @pytest.mark.asyncio
    async def test_update_skill_success(self, skill_manager):
        """Sets response.success=True on successful update."""
        skill_manager._manipulator.update = AsyncMock(return_value=_success())
        request = MagicMock()
        request.id = "motion_slow"
        request.skill_type = "motion_control"
        request.is_enabled = True
        request.parameter_mapping = ""
        request.volatile_mapping = ""
        request.interface_mapping = ""
        request.local_defaults = ""
        request.displayname = "Updated Name"
        request.description = ""
        request.tags = ""
        response = MagicMock()
        await skill_manager.update_skill(request, response)
        assert response.success is True

    @pytest.mark.asyncio
    async def test_update_skill_empty_fields_ignored(self, skill_manager):
        """Empty optional fields are not forwarded to update_skill_impl."""
        skill_manager._manipulator.update = AsyncMock(return_value=_success())
        request = MagicMock()
        request.id = "motion_slow"
        request.skill_type = ""
        request.is_enabled = None
        request.parameter_mapping = ""
        request.volatile_mapping = ""
        request.interface_mapping = ""
        request.local_defaults = ""
        request.displayname = ""
        request.description = ""
        request.tags = ""
        response = MagicMock()
        await skill_manager.update_skill(request, response)
        update_data = skill_manager._manipulator.update.call_args[0][0]
        assert "skill_type" not in update_data
        assert "displayname" not in update_data


# ---------------------------------------------------------------------------
# Zenoh service: delete_skill
# ---------------------------------------------------------------------------

class TestDeleteSkillService:
    """Zenoh service endpoint: delete_skill"""

    @pytest.mark.asyncio
    async def test_delete_skill_success(self, skill_manager):
        """Sets response.success=True when skill is deleted."""
        row = _fake_skill_row()
        skill_manager._manipulator.get_all = AsyncMock(return_value=_success([row]))
        skill_manager._manipulator.delete = AsyncMock(return_value=_success())
        request = MagicMock()
        request.id = "motion_slow"
        response = MagicMock()
        returned = await skill_manager.delete_skill(request, response)
        assert returned is True
        assert response.success is True

    @pytest.mark.asyncio
    async def test_delete_skill_not_found(self, skill_manager):
        """Sets response.success=False when skill does not exist."""
        skill_manager._manipulator.get_all = AsyncMock(return_value=_not_found())
        request = MagicMock()
        request.id = "nonexistent"
        response = MagicMock()
        returned = await skill_manager.delete_skill(request, response)
        assert returned is False
        assert response.success is False
