import configparser
import os

from enum import Enum
from typing import Union

from sqlalchemy import inspect
from sqlalchemy import MetaData
from sqlalchemy import Table
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio import async_sessionmaker
from sqlalchemy.orm import sessionmaker

from vos_base.helper.logger import Logger
from vos_base.helper.logger import LogEntry
from vos_base.helper.logger import LogMode
from vos_base.storage.tb_base import Base
from vos_base.helper.error_handler import ErrorTraceback

meta = MetaData()


class DBSTATUS(str, Enum):
    SUCCESS = "success"
    ERROR = "error"
    NOT_FOUND = "not_found"
    NOT_ALLOWED = "not_allowed"
    NOT_AUTHORIZED = "not_authorized"
    CONFLICT = "conflict"

class DBMESSAGE:
    DEFAULT_ERROR = 'Something went wrong while processing the query. See the details.'

class DbAccess:
    """Baseclass for database access."""

    def __init__(self, module_id: str, db_config_path: str):
        """Initialize database object.

        Args:
            module_id (str): The id of the varioboticOS module.
            config_path (str, optional): Path to the ini file of your sqlalchemy config. 
                                         Defaults to WORKING_PATH+PATH_DB_CONFIG.
        """        
        try:
            self.module_id = module_id

            self._config = configparser.ConfigParser()
            self._config.read(db_config_path)

            self._user: str|None = os.environ.get("USER")
            self._path = self._config['sqlite']['path']
            self._path = self._path.replace("${user}", str(self._user))

            self._database = self._config['sqlite']['database']
            self._database = self._database.replace("${module_id}", self.module_id)

            self.db_engine: AsyncEngine = create_async_engine(
                f"sqlite+aiosqlite:///{self._path}{self._database}",
                echo=True,
            )

            Logger.add_external('sqlalchemy.engine')

        finally:
            ErrorTraceback.check_error_exist()

    def session(self) -> Union[sessionmaker, async_sessionmaker]:
        """Create a session for the database.

        Args:
            async_session (bool, optional): If True, create an async session. 
                                            Defaults to False.

        Returns:
            Session: A session object.
        """
        if isinstance(self.db_engine, AsyncEngine):
            return async_sessionmaker(self.db_engine, expire_on_commit=False)
        else:
            return sessionmaker(self.db_engine, expire_on_commit=False)

    async def create_all(self) -> str:
        """Create all database tables that are childen of the SQLAlchemy Base class.
        This method will create all tables that are defined in the SQLAlchemy Base class.
        It will also create the tables if they do not exist.
        This method will not create the tables if they already exist.            

        Returns:
            DBSTATUS: see DBSTATUS class
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

    async def create_selective(self, table_structs: list[Base]) -> str:
        """Create new database table.

        Args:
            table_structs (dlist[object]ict): 
                table configurations as python classes 
                style -> sqlalchemy.ext.declarative_base

        Returns:
            str: Element of DBSTATUS class
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

            # conn is an instance of AsyncConnection
            async with self.db_engine.begin() as conn:
                # to support SQLAlchemy DDL methods as well as legacy functions, the
                # AsyncConnection.run_sync() awaitable method will pass a "sync"
                # version of the AsyncConnection object to any synchronous method,
                # where synchronous IO calls will be transparently translated for
                # await.

                await conn.run_sync(meta.create_all)

                Logger.log(LogEntry(
                    f'Successfully created table {table_name}'))

            return DBSTATUS.SUCCESS

        finally:
            if ErrorTraceback.check_error_exist():
                return DBSTATUS.ERROR

    async def drop(self, table: Base) -> str:
        """Delete table from database.

        Args:
            table (str): table name

        Returns:
            str: Element of DBSTATUS class
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