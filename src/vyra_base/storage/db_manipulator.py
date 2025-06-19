"""Handling database datatables"""

import uuid

from dataclasses import dataclass

from sqlalchemy import select
from sqlalchemy import inspect
from sqlalchemy import update as sqlalchemy_update
from sqlalchemy import delete as sqlalchemy_delete
from sqlalchemy import func as sqlalchemy_func

from typing import Union
from typing import Type
from typing import Any

from vyra_base.helper.logger import Logger
from vyra_base.helper.logger import LogEntry
from vyra_base.storage.tb_base import Base
from vyra_base.storage.db_access import DBSTATUS
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_access import DBMESSAGE
from vyra_base.helper.error_handler import ErrorTraceback


@dataclass
class DBReturnValue:
    status: Union[str, None] = None
    value: Union[bool, str, list, dict] = ''
    details: Union[bool, str, list, dict] = ''

    def __init__(self, status: Union[None, str]=None, 
                 value: Union[bool, str, list, dict]='', 
                 details: Union[bool, str, list, dict]=''):
        self.status = status
        self.value = value
        self.details = details
    
    def error_return(self, details: Union[bool, str, list, dict]=""):
        self.status = DBSTATUS.ERROR
        
        if self.value == "":
            self.value = DBMESSAGE.DEFAULT_ERROR
        
        if details != "":
            self.details = details

        return self
    
    def success_return(self):
        self.status = DBSTATUS.SUCCESS
        return self


class DBTableManipulator:
    """Datatable class manipulator"""

    @ErrorTraceback.w_check_error_exist
    def __init__(
            self, db_access: DbAccess, model: Type[Base], module_id: str):
        """Initialize datatable.

        Args:
            db_access (DbAccess): Database access object (using SQLAlchemy).
            table_name (str): Name of the datatable.
            module_id (str): The id of the varioboticOS module.
        """
        if isinstance(db_access, DbAccess) is False:
            raise TypeError('db_access must be of type DbAccess.')
        
        if not issubclass(model, Base):
            Logger.log(LogEntry(
                ('Model is no subclass of Base (SQLAlchemy). '
                    'This could lead to errors')).warn()
            )

        self._db = db_access
        self.module_id = module_id
        self.model = model
        self.table_name = model.__tablename__


    def _read_pkey(self) -> str:
        return inspect(self.model).primary_key[0].name

    def get_table_structure(self) -> DBReturnValue:
        """Read the datatable structure from the config file.

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        try:
            # inspector = inspect(self._db.db_engine)
            # columns = inspector.get_columns(self.model.__tablename__)

            columns = [column.name for column in self.model.__table__.columns]

            return DBReturnValue(
                value=columns).success_return()
        
        finally:
            error_details: list = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    ('Could not get tablestructure from '
                     f'{self.model.__tablename__}.')).error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def get_by_id(self, id: Union[uuid.UUID, int]=-1) -> DBReturnValue:
        """Read line from datatable of database by 'id'.

        If the id is -1, the last line will be read.
        If the id is None, the last line will be read.

        Args:
            id (int, optional): Private key of the table element to 
                                select the row to be read. Defaults to -1.

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """

        try:
            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        stmt = select(self.model)
                        if id in ['-1', -1, 'None', None]:
                            stmt = (stmt
                                .order_by(getattr(
                                self.model, self._read_pkey()).desc())
                                .limit(1)
                            )
                        else:
                            stmt = stmt.where(getattr(
                                self.model, self._read_pkey()) == id)
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in get_by_id: {e}').error()
                        )
                        raise e
                    
            result = await session.execute(stmt)
            row = result.fetchone()

            if not row:
                return DBReturnValue(
                    status=DBSTATUS.NOT_FOUND,
                    value=row,
                    details='No data found')

            return DBReturnValue(
                value=row,
                details='Data found').success_return()

            # return_value = {}
            # conf = self.db_conf['tables_config']['tables'][self.table_name]['columns']
            # for idx, col in enumerate(conf.values()):
            #     return_value[col['name']] = row[idx]

            # return DBReturnValue(
            #     value=return_value,
            #     details='Data found').success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not get data by id from {self.model.__tablename__}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def get_all(self, filters: dict=None, order_by=None, limit=None) -> DBReturnValue:
        """Read all lines from datatable of database.
        Args:
            filters (dict, optional): add filter elements to identify the rows 
                                      to be read. Defaults to None.
            order_by (str, optional): column name to order the result. Defaults to None.
            limit (int, optional): number of lines to be read. Defaults to None.

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        try:
            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        stmt = select(self.model)
                        if filters:
                            for key, value in filters.items():
                                if not hasattr(self.model, key):
                                    raise ValueError(
                                        f"Model '{self.model.__name__}' has no column '{key}'")
                                
                                if isinstance(value, list):
                                    stmt = stmt.where(getattr(self.model, key).in_(value))
                                else:
                                    stmt = stmt.where(getattr(self.model, key) == value)
                        
                        if order_by:
                            stmt = stmt.order_by(getattr(self.model, order_by))
                        
                        if limit:
                            stmt = stmt.limit(limit)

                        result = await session.execute(stmt)
                        value = result.scalars().all()

                        if len(value) == 0:
                            return DBReturnValue(
                                status=DBSTATUS.NOT_FOUND,
                                value=value,
                                details='No data found')

                        return DBReturnValue(
                            value=value,
                            details='Data found').success_return()
                    except Exception as e:
                        Logger.log(LogEntry(f'Error in get_all: {e}').error())
                        raise e
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not get all data from {self.model.__tablename__}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

    async def update(self, data: dict, filters: Union[dict, None]=None) -> DBReturnValue:
        """Add new entry to datatable of database on the bottom.

        If the table config has a field 'max_lines' and the
        number of lines are greater than this field, the lowest 'id' value will 
        be deleted and the new entry will be added by a incremented 'id' value.

        Args:
            data (dict): Update data in an existing entry
            id (int): Private key of the table element to select the row to be updated

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        try:
            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        stmt = (
                            sqlalchemy_update(self.model)
                            .values(**data)
                            .execution_options(synchronize_session="fetch")
                        )

                        if filters:
                            for key, value in filters.items():
                                stmt = stmt.where(getattr(self.model, key) == value)

                        exec_ret = await session.execute(stmt)
                        await session.flush()
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in update: {e}').error()
                        )
                        raise e

            if exec_ret.rowcount == 0:
                return DBReturnValue(
                    status=DBSTATUS.NOT_FOUND,
                    value=str(exec_ret.rowcount),
                    details='No data found')

            return DBReturnValue(
                value=f"Successfully updated entry in {self.model.__tablename__}.",
                details=f"Updated element <{data}> by filters: {filters}"
                ).success_return()

        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not update data to {self.model.__tablename__}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def add(self, data: dict) -> DBReturnValue:
        """Add a new a row in a datatable.

        Args:
            data (dict): Content of the new entry to be added to the table

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        
        try:
            obj = self.model(**data)
            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        session.add(obj)
                        await session.flush()
                        await session.refresh(obj)
                    except Exception as e:
                        Logger.log(LogEntry(str(e)).error())
                        raise e
            
            return DBReturnValue(
                value=f"Successfully added new entry to {self.model.__tablename__}.",
                details={
                    'id': getattr(obj, self._read_pkey()),
                    'data': self.to_dict(obj)
                }
                ).success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not add tabledata to {self.table_name}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

            
    async def delete(self, id: Any) -> DBReturnValue:
        """Update a line in a datatable of database by a given 'id'.

        Args:
            id (any): Private key of the table element to select the row to be deleted

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        try:
            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        stmt = sqlalchemy_delete(self.model).where(
                            getattr(self.model, self._read_pkey()) == id)
                        await session.execute(stmt)
                        await session.flush()
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in delete: {e}').error()
                        )
                        raise e

            return DBReturnValue(
                value=f"Successfully deleted entry from {self.model.__tablename__}.",
                details={
                    'id': id,
                    'data': f"Deleted element by id: {id}"
                }
                ).success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not delete tabledata from {self.table_name}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def bulk_add(self, data: list[dict]) -> DBReturnValue:
        """Update a line in a datatable of database by a given 'id'.

        Args:
            data (list[dict]): data list to be added to the table 

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        try:
            objs = [self.model(**d) for d in data]

            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        session.add_all(objs)
                        await session.flush()
                        for obj in objs:
                            await session.refresh(obj)
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in bulk_add: {e}').error()
                        )
                        raise e

            return DBReturnValue(
                value=f"Successfully added new entrys to {self.model.__tablename__}.",
                details={
                    'ids': [getattr(obj, self._read_pkey()) for obj in objs],
                    'data': [self.to_dict(obj) for obj in objs]
                }
                ).success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not bulk add tabledata to {self.table_name}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

            
    async def bulk_delete(self, filters: dict) -> DBReturnValue:
        """Update a line in a datatable of database by a given 'id'.

        Args:
            filters (dict): add filter elements to identify the rows to be deleted

        Returns:
            dict: {Status: DBSTATUS, value: Short info, details: Detailed info}
        """
        try:
            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        stmt = sqlalchemy_delete(self.model)
                        for key, value in filters.items():
                            stmt = stmt.where(getattr(self.model, key) == value)
                        await session.execute(stmt)
                        await session.flush()
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in bulk_delete: {e}').error()
                        )
                        raise e

            return DBReturnValue(
                value=f"Successfully deleted entry from {self.model.__tablename__}.",
                details=f"Deleted element by filters: {filters}"
                ).success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not bulk delete tabledata from {self.table_name}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def exists(self, id: int) -> DBReturnValue:
        """Update a line in a datatable of database by a given 'id'.

        Args:
            new_entry (dict): _description_
            ident (str): _description_

        Returns:
            dict: {Status: HTML Status, msg: Success or not, details: If an error occures}
        """
        try:
            ret_val: bool = False

            async with self._db.session()() as session:
                async with session.begin():
                    try:
                        stmt = select(self.model)
                        
                        stmt = stmt.where(
                            getattr(self.model, self._read_pkey()) == id)
                        
                        result = await session.execute(stmt)

                        ret_val = result.scalar_one_or_none() is not None
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in exists: {e}').error()
                        )
                        raise e

            return DBReturnValue(
                value=ret_val,
                details='').success_return()

        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not check existance of {self.table_name}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

            
    async def count(self, filters: dict) -> DBReturnValue:
        """Update a line in a datatable of database by a given 'id'.

        Args:
            new_entry (dict): _description_
            ident (str): _description_

        Returns:
            dict: {Status: HTML Status, msg: Success or not, details: If an error occures}
        """
        try:
            async with (self._db.session())() as session:
                async with session.begin():
                    try:
                        stmt = select(sqlalchemy_func.count()).select_from(self.model)
                        if filters:
                            for key, value in filters.items():
                                stmt = stmt.where(getattr(self.model, key) == value)
                        result = await session.execute(stmt)
                    
                        return DBReturnValue(
                            value=result.scalar_one(),
                            details='').success_return() 
                    except Exception as e:
                        Logger.log(LogEntry(
                            f'Error in count: {e}').error()
                        )
                        raise e
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                Logger.log(LogEntry(
                    f'Could not count tabledata from {self.table_name}.').error()
                )
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

    def to_dict(self, obj, exclude=None):
        return {
            c.name: getattr(obj, c.name) for 
                c in obj.__table__.columns if 
                    exclude is None or c.name not in exclude}

# EOF
