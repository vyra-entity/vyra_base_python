"""
Example: Storage – DbAccess and DbManipulator

Demonstrates:
- DbAccess with SQLite (in-memory or file)
- Custom table with tb_ prefix (required by VYRA)
- DbManipulator: add(), get_all(), get_by_id()

Usage:
  python examples/storage_example.py
"""

import asyncio
import tempfile
from pathlib import Path

from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy import String, Float, DateTime
import uuid
from datetime import datetime

from vyra_base.storage import (
    Base,
    DbAccess,
    DbManipulator,
    DBTYPE,
    DBSTATUS,
)


# Custom table – MUST use tb_ prefix (VYRA requirement)
class tb_sensor_log(Base):
    """Example sensor log table."""
    __tablename__ = "tb_sensor_log"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    sensor_name: Mapped[str] = mapped_column(String(100), nullable=False)
    value: Mapped[float] = mapped_column(nullable=False)
    timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)


async def main():
    print("=" * 60)
    print("Storage Example: DbAccess + DbManipulator")
    print("=" * 60)

    tmpdir = Path(tempfile.mkdtemp(prefix="vyra_storage_example_"))
    db_path = tmpdir / "data"
    db_path.mkdir(parents=True, exist_ok=True)

    db_config = {
        "sqlite": {
            "path": str(db_path) + "/",
            "database": "example.db",
        }
    }

    db = DbAccess(
        module_name="example_module",
        db_config=db_config,
        db_type=DBTYPE.SQLITE,
    )
    await db.create_all_tables()
    print("\n1. DbAccess created (SQLite), tables created.")

    manipulator = DbManipulator(db_access=db, model=tb_sensor_log)
    print("2. DbManipulator created for tb_sensor_log.")

    # Insert
    r = await manipulator.add({
        "sensor_name": "temperature_1",
        "value": 23.5,
    })
    print(f"\n3. add() result: status={r.status}, value={r.value}")

    await manipulator.add({"sensor_name": "humidity_1", "value": 65.0})
    await manipulator.add({"sensor_name": "temperature_1", "value": 24.1})
    print("4. Added two more rows.")

    # Get all
    all_result = await manipulator.get_all()
    print(f"\n5. get_all(): status={all_result.status}, count={len(all_result.details) if all_result.details else 0}")
    if all_result.details:
        for row in all_result.details:
            print(f"   - {row.sensor_name}: {row.value} @ {row.timestamp}")

    # Get by id (last row)
    last_id = all_result.details[-1].id if all_result.details else None
    if last_id:
        one = await manipulator.get_by_id(last_id)
        row = one.value
        print(f"\n6. get_by_id(last): {row.sensor_name}: {row.value}" if row else "\n6. get_by_id: no row")

    print("\n" + "=" * 60)
    print("Storage example completed.")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
