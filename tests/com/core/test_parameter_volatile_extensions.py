import pytest
from typing import Any, cast

from vyra_base.core.parameter import Parameter
from vyra_base.core.volatile import Volatile
from vyra_base.storage.db_access import DBSTATUS
from vyra_base.storage.db_manipulator import DBReturnValue
from vyra_base.storage.tb_params import TypeEnum
from vyra_base.com.transport.t_redis import REDIS_TYPE


class _FakeManipulator:
    def __init__(self):
        self._items = []

    async def get_all(self, filters=None):
        if not filters:
            return DBReturnValue(status=DBSTATUS.SUCCESS, value=self._items)
        name = filters.get("name")
        found = [item for item in self._items if item["name"] == name]
        if found:
            return DBReturnValue(status=DBSTATUS.SUCCESS, value=found)
        return DBReturnValue(status=DBSTATUS.NOT_FOUND, value=[])

    async def add(self, item):
        self._items.append(item)
        return DBReturnValue(status=DBSTATUS.SUCCESS, value=item)


class _FakeRedis:
    def __init__(self):
        self._data = {}

    async def get_all_keys(self):
        return list(self._data.keys())

    async def set(self, key, value):
        self._data[key] = value

    async def get(self, key):
        return self._data.get(key)

    async def exists(self, key):
        return key in self._data

    async def get_type(self, key):
        if key not in self._data:
            return None
        return REDIS_TYPE.STRING

    async def subscribe_to_key(self, key):
        return None

    async def unsubscribe_from_key(self, key):
        return None


@pytest.mark.asyncio
async def test_create_new_parameter_impl_is_strict():
    parameter = Parameter.__new__(Parameter)
    parameter.persistant_manipulator = cast(Any, _FakeManipulator())

    result_create = await parameter.create_new_parameter_impl("demo", "1")
    assert result_create is not None
    assert result_create["success"] is True

    result_duplicate = await parameter.create_new_parameter_impl("demo", "2")
    assert result_duplicate is not None
    assert result_duplicate["success"] is False
    assert "already exists" in result_duplicate["message"]


@pytest.mark.asyncio
async def test_volatile_uses_fixed_address_prefix_and_create_is_strict():
    fake_redis = _FakeRedis()
    volatile = Volatile(
        storage_access_transient=cast(Any, fake_redis),
        module_name="v2_demo",
        module_id="abc123",
        node=None,
        transient_base_types={
            "VolatileString": str,
            "VolatileHash": dict,
            "VolatileList": list,
            "VolatileSet": set,
        },
    )

    create_result = await volatile.create_new_volatile_impl("temperature", "42")
    assert create_result["success"] is True

    duplicate_result = await volatile.create_new_volatile_impl("temperature", "84")
    assert duplicate_result["success"] is False

    read_all = await volatile.read_all_volatiles_impl()
    assert "v2_demo_abc123/volatile/temperature" in read_all["all_volatiles_json"]
