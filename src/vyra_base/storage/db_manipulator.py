"""Handling database datatables"""

import uuid
import logging
logger = logging.getLogger(__name__)

from dataclasses import dataclass

from sqlalchemy import select
from sqlalchemy import inspect
from sqlalchemy import update as sqlalchemy_update
from sqlalchemy import delete as sqlalchemy_delete
from sqlalchemy import func as sqlalchemy_func

from typing import Union
from typing import Type
from typing import Any

from vyra_base.storage.tb_base import Base
from vyra_base.storage.db_access import DBSTATUS
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_access import DBMESSAGE
from vyra_base.helper.error_handler import ErrorTraceback


@dataclass
class DBReturnValue:
    """
    Standardized return value container for database operations.
    
    Encapsulates the result of database operations with status, value, and details.
    Provides convenience methods for setting error/success states.
    
    :ivar status: Operation status (see DBSTATUS enum).
    :type status: Union[str, None]
    :ivar value: Main return value (data or error message).
    :type value: Union[bool, str, list, dict]
    :ivar details: Additional details about the operation.
    :type details: Union[bool, str, list, dict]
    """
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
        """
        Set the return value to error status.

        :param details: Additional error details.
        :type details: Union[bool, str, list, dict]
        :return: Self with error status set.
        :rtype: DBReturnValue
        """
        self.status = DBSTATUS.ERROR
        
        if self.value == "":
            self.value = DBMESSAGE.DEFAULT_ERROR
        
        if details != "":
            self.details = details

        return self
    
    def success_return(self):
        """
        Set the return value to success status.

        :return: Self with success status set.
        :rtype: DBReturnValue
        """
        self.status = DBSTATUS.SUCCESS
        return self


class DbManipulator:
    """Datatable class manipulator"""

    @ErrorTraceback.w_check_error_exist
    def __init__(
            self, db_access: DbAccess, model: Type[Base]):
        """
        Initialize datatable.

        :param db_access: Database access object (using SQLAlchemy).
        :type db_access: DbAccess
        :param model: SQLAlchemy model class.
        :type model: Type[Base]
        :param module_id: The id of the V.Y.R.A. module.
        :type module_id: str
        """
        if isinstance(db_access, DbAccess) is False:
            raise TypeError('db_access must be of type DbAccess.')
        
        if not issubclass(model, Base):
            logger.warning('Model is no subclass of Base (SQLAlchemy). This could lead to errors')

        self._db = db_access
        self.model = model
        self.table_name = model.__tablename__


    def _read_pkey(self) -> str:
        """
        Get the name of the primary key column.

        :return: Primary key column name.
        :rtype: str
        """
        return inspect(self.model).primary_key[0].name

    def get_table_structure(self) -> DBReturnValue:
        """
        Read the datatable structure from the config file.

        :return: Table structure information.
        :rtype: DBReturnValue
        """
        try:
            columns = [column.name for column in self.model.__table__.columns]

            return DBReturnValue(
                value=columns).success_return()
        
        finally:
            error_details: list = []
            if ErrorTraceback.check_error_exist(error_details):
                logger.error('Could not get tablestructure from '
                     f'{self.model.__tablename__}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def get_by_id(self, id: Union[uuid.UUID, int]=-1) -> DBReturnValue:
        """
        Read line from datatable of database by 'id'.

        If the id is -1 or None, the last line will be read.

        :param id: Private key of the table element to select the row to be read.
        :type id: Union[uuid.UUID, int], optional
        :return: Row data or not found status.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in get_by_id: {e}')
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

        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                logger.error(f'Could not get data by id from {self.model.__tablename__}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def get_all(self, filters: dict=None, order_by=None, limit=None) -> DBReturnValue:
        """
        Read all lines from datatable of database.

        :param filters: Filter elements to identify the rows to be read.
        :type filters: dict, optional
        :param order_by: Column name to order the result.
        :type order_by: str, optional
        :param limit: Number of lines to be read.
        :type limit: int, optional
        :return: List of rows or not found status.
        :rtype: DBReturnValue
        """
        try:
            async with self._db.session()() as session:
                async with session.begin():
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

                    logger.debug("Start executing query")
                    try:
                        result = await session.execute(stmt)
                    finally:
                        logger.debug("Ran execute on session")
                        ErrorTraceback.check_error_exist()
                    logger.debug(f"Executed query: {stmt}: {result}")
                    value = result.scalars().all()

                    if len(value) == 0:
                        return DBReturnValue(
                            status=DBSTATUS.NOT_FOUND,
                            value=value,
                            details='No data found')

                    return DBReturnValue(
                        value=value,
                        details='Data found').success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                logger.error(f'Could not get all data from {self.model.__tablename__}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

    async def update(self, data: dict, filters: Union[dict, None]=None) -> DBReturnValue:
        """
        Update a line in a datatable of database by given data.

        If the table config has a field 'max_lines' and the number of lines are greater than this field,
        the lowest 'id' value will be deleted and the new entry will be added by a incremented 'id' value.

        :param data: Update data in an existing entry.
        :type data: dict
        :param filters: Filter to select the row to be updated.
        :type filters: dict, optional
        :return: Update status.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in update: {e}')
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
                logger.error(f'Could not update data to {self.model.__tablename__}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def add(self, data: dict) -> DBReturnValue:
        """
        Add a new a row in a datatable.

        :param data: Content of the new entry to be added to the table.
        :type data: dict
        :return: Add status and details.
        :rtype: DBReturnValue
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
                        logger.error(str(e))
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
                logger.error(f'Could not add tabledata to {self.table_name}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

            
    async def delete(self, id: Any) -> DBReturnValue:
        """
        Update a line in a datatable of database by a given 'id'.

        :param id: Private key of the table element to select the row to be deleted.
        :type id: Any
        :return: Delete status and details.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in delete: {e}')
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
                logger.error(f'Could not delete tabledata from {self.table_name}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def bulk_add(self, data: list[dict]) -> DBReturnValue:
        """
        Add multiple rows to a datatable.

        :param data: Data list to be added to the table.
        :type data: list[dict]
        :return: Bulk add status and details.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in bulk_add: {e}')
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
                logger.error(f'Could not bulk add tabledata to {self.table_name}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

            
    async def bulk_delete(self, filters: dict) -> DBReturnValue:
        """
        Delete multiple rows in a datatable by given filters.

        :param filters: Filter elements to identify the rows to be deleted.
        :type filters: dict
        :return: Bulk delete status and details.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in bulk_delete: {e}')
                        raise e

            return DBReturnValue(
                value=f"Successfully deleted entry from {self.model.__tablename__}.",
                details=f"Deleted element by filters: {filters}"
                ).success_return()
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                logger.error(f'Could not bulk delete tabledata from {self.table_name}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)


    async def exists(self, id: int) -> DBReturnValue:
        """
        Check if a row exists in the datatable by a given 'id'.

        :param id: Private key of the table element to check.
        :type id: int
        :return: Existence status.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in exists: {e}')
                        raise e

            return DBReturnValue(
                value=ret_val,
                details='').success_return()

        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                logger.error(f'Could not check existance of {self.table_name}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

            
    async def count(self, filters: dict) -> DBReturnValue:
        """
        Count the number of rows in a datatable matching given filters.

        :param filters: Filter elements to identify the rows to be counted.
        :type filters: dict
        :return: Number of matching rows.
        :rtype: DBReturnValue
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
                        logger.error(f'Error in count: {e}')
                        raise e
        finally:
            error_details = []
            if ErrorTraceback.check_error_exist(error_details):
                logger.error(f'Could not count tabledata from {self.table_name}.')
                error_ret = DBReturnValue()
                return error_ret.error_return(error_details)

    def to_dict(self, obj, exclude=None):
        """
        Convert a SQLAlchemy model instance to a dictionary.

        :param obj: SQLAlchemy model instance.
        :type obj: Base
        :param exclude: List of column names to exclude.
        :type exclude: list, optional
        :return: Dictionary representation of the object.
        :rtype: dict
        """
        return {
            c.name: getattr(obj, c.name) for 
                c in obj.__table__.columns if 
                    exclude is None or c.name not in exclude}

# EOF

