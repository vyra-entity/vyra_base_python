import configparser
import os

from enum import Enum
from pathlib import Path
from typing import Union

from sqlalchemy import inspect, MetaData, Table
from sqlalchemy.ext.asyncio import AsyncEngine, async_sessionmaker, create_async_engine
from sqlalchemy.orm import sessionmaker

from vyra_base.helper.logger import Logger, LogEntry, LogMode
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.storage.storage import Storage
from vyra_base.storage.tb_base import Base

meta = MetaData()


class DBSTATUS(str, Enum):
    SUCCESS = "success"
    ERROR = "error"
    NOT_FOUND = "not_found"
    NOT_ALLOWED = "not_allowed"
    NOT_AUTHORIZED = "not_authorized"
    CONFLICT = "conflict"


class DBTYPE(str, Enum):
    SQLITE = "sqlite"
    MYSQL = "mysql"
    POSTGRESQL = "postgresql"


class DBMESSAGE:
    DEFAULT_ERROR = 'Something went wrong while processing the query. See the details.'


class DbAccess(Storage):
    """Baseclass for database access."""

    def __init__(self, module_name: str, db_config_path: str = None, 
                 db_config: dict = None, db_type: DBTYPE = DBTYPE.SQLITE) -> None:
        """
        Initialize database object.

        :param module_name: The id of the V.Y.R.A. module.
        :type module_name: str
        :param db_config_path: Path to the ini file of your sqlalchemy config. 
                               Defaults to WORKING_PATH+PATH_DB_CONFIG.
        :type db_config_path: str, optional
        :param db_config: Dictionary with database configuration.
        :type db_config: dict, optional
        :raises ValueError: If neither db_config_path nor db_config is provided, 
                            or if db_config is not a dict.
        """

        self.module_name = module_name

        try:
            if db_config_path is not None and db_config is None:
                self._config = configparser.ConfigParser()
                self._config.read(db_config_path)
            elif db_config is not None:
                db_config = db_config[0] if isinstance(db_config, list) else db_config

                if not isinstance(db_config, dict):
                    raise ValueError("db_config must be a dictionary.")

                self._config = db_config
            else:
                raise ValueError("Either db_config_path or db_config must be provided.")
            

            if db_type not in (item.value for item in DBTYPE):
                raise ValueError(f"Unsupported database type: {db_type}. Supported types: {list(DBTYPE)}")
            
            self.db_type = db_type
            self._user: str | None = os.environ.get("USER")
            self._path: str | None = self._config[self.db_type]['path']
            self._port: str | None = self._config[self.db_type].get('port', None)
            self._host: str | None = self._config[self.db_type].get('host', None)
            
            self._path = self._path.replace("${user}", str(self._user))

            self._database = self._config[self.db_type]['database']
            self._database = self._database.replace("${module_name}", self.module_name)

            if not self._path.endswith('/'):
                self._path += '/'

            Path(self._path).mkdir(parents=True, exist_ok=True)

            self.db_engine: AsyncEngine = self._build_engine()

            Logger.debug(f"Database config: {self._config}")
            Logger.add_external('sqlalchemy.engine')

        finally:
            ErrorTraceback.check_error_exist()

    def _build_engine(self) -> AsyncEngine:
        """
        Build the database engine based on the configuration.

        :returns: An instance of AsyncEngine.
        :rtype: AsyncEngine
        """
        if self.db_type == DBTYPE.SQLITE:
            return create_async_engine(
                f"sqlite+aiosqlite:///{self._path}{self._database}",
                echo=True,
            )
        elif self.db_type == DBTYPE.MYSQL:
            return create_async_engine(
                f"mysql+aiomysql://{self._user}@{self._host}:{self._port}/{self._database}",
                echo=True,
            )
        elif self.db_type == DBTYPE.POSTGRESQL:
            return create_async_engine(
                f"postgresql+asyncpg://{self._user}@{self._host}:{self._port}/{self._database}",
                echo=True,
            )
        else:
            raise ValueError(f"Unsupported database type: {self.db_type}")

    def session(self) -> Union[sessionmaker, async_sessionmaker]:
        """
        Create a session for the database.

        :returns: A session object.
        :rtype: sessionmaker or async_sessionmaker
        """
        if isinstance(self.db_engine, AsyncEngine):
            return async_sessionmaker(self.db_engine, expire_on_commit=False)
        else:
            return sessionmaker(self.db_engine, expire_on_commit=False)

    async def create_all_tables(self) -> str:
        """
        Create all database tables that are children of the SQLAlchemy Base class.

        This method will create all tables that are defined in the SQLAlchemy Base class.
        It will also create the tables if they do not exist.
        This method will not create the tables if they already exist.

        :returns: Status of the operation.
        :rtype: str (see DBSTATUS class)
        """
        try:
            async with self.db_engine.begin() as conn:
                await conn.run_sync(Base.metadata.create_all)
                Logger.log(
                    LogEntry(f'Successfully created all defined tables', mode=LogMode.INFO))

            return DBSTATUS.SUCCESS

        finally:
            if ErrorTraceback.check_error_exist():
                return DBSTATUS.ERROR

    async def create_selected_table(self, table_structs: list[Base]) -> str:
        """
        Create new database table.

        :param table_structs: Table configurations as python classes (SQLAlchemy declarative base style).
        :type table_structs: list[Base]
        :returns: Status of the operation.
        :rtype: str (see DBSTATUS class)
        """
        try:
            meta = MetaData()

            def load_table(sync_conn):
                Table(table_name, meta, autoload_with=sync_conn)

            async with self.db_engine.connect() as async_conn:
                for table_struct in table_structs:
                    table_name = table_struct.__tablename__
                    
                    if table_name in meta.tables:
                        continue
                    
                    await async_conn.run_sync(load_table)

            async with self.db_engine.begin() as conn:
                await conn.run_sync(meta.create_all)

                Logger.log(LogEntry(
                    f'Successfully created table {table_name}'))

            return DBSTATUS.SUCCESS

        finally:
            if ErrorTraceback.check_error_exist():
                return DBSTATUS.ERROR

    async def drop_table(self, table: Base) -> str:
        """
        Delete table from database.

        :param table: Table class (SQLAlchemy declarative base).
        :type table: Base
        :returns: Status of the operation.
        :rtype: str (see DBSTATUS class)
        """
        try:
            table_name = table.__tablename__

            async with self.db_engine.connect() as async_conn:
                def check_and_reflect(sync_conn):
                    inspector = inspect(sync_conn)
                    if not inspector.has_table(table_name):
                        return None
                    meta = MetaData()
                    return Table(table_name, meta, autoload_with=sync_conn)

                table_obj = await async_conn.run_sync(check_and_reflect)

                if table_obj is not None:
                    await async_conn.run_sync(lambda sync_conn: table_obj.drop(sync_conn))
                    Logger.log(LogEntry(f"Tabelle '{table_name}' wurde gelöscht."))
                else:
                    Logger.log(LogEntry(f"Tabelle '{table_name}' existiert nicht."))
                    return DBSTATUS.NOT_FOUND

            return DBSTATUS.SUCCESS
        
        finally:
            if ErrorTraceback.check_error_exist():
                return DBSTATUS.ERROR
