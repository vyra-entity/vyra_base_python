"""
VYRA Base Storage Module

Provides access to SQLite database and Redis storage.

Public API for external developers

Important: All database tables MUST use the ``tb_`` prefix!
Example: tb_parameters, tb_sensor_data, tb_logs
"""

# Base classes for table definition
from .tb_base import Base, IntEnum

# Database access
from .db_access import DbAccess, DBTYPE, DBSTATUS, DBMESSAGE

# Database CRUD operations
from .db_manipulator import DbManipulator, DBReturnValue

# Redis client
from .redis_client import RedisClient

# Storage base class
from .storage import Storage

# Example table definitions (for reference)
from .tb_params import Parameter as tb_parameters

__all__ = [
    # Base classes
    "Base",
    "IntEnum",
    
    # Database
    "DbAccess",
    "DBTYPE",
    "DBSTATUS",
    "DBMESSAGE",
    "DbManipulator",
    "DBReturnValue",
    
    # Redis
    "RedisClient",
    
    # Storage base
    "Storage",
    
    # Example tables
    "tb_parameters",
]

# Usage example for developers:
"""
Creating a custom table:

from vyra_base.storage import Base
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy import String, Float, DateTime
import uuid
from datetime import datetime

class tb_sensor_data(Base):
    '''Custom sensor data table - NOTE: tb_ prefix is required!'''
    __tablename__ = 'tb_sensor_data'
    
    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    sensor_name: Mapped[str] = mapped_column(String(100), nullable=False)
    value: Mapped[float] = mapped_column(nullable=False)
    timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

Using the table:

from vyra_base.storage import DbManipulator

# Create manipulator
manipulator = DbManipulator(
    db_access=entity.storage.db_access,
    table_structure=tb_sensor_data
)

# Insert data
await manipulator.insert({
    "sensor_name": "temperature_1",
    "value": 23.5
})

# Query data
result = await manipulator.get_all()
for entry in result.details:
    print(f"{entry.sensor_name}: {entry.value}")
"""