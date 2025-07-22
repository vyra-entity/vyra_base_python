import json
from pathlib import Path

from vyra_base.helper._aiopath import AsyncPath
from vyra_base.helper.file_lock import get_lock_for_file, release_lock_for_file


class FileWriter:
    """File writer.

    Writes Python objects to a file.
    """

    @classmethod
    async def write_json_file(cls, file: Path, file_content: dict) -> bool:
        """
        Writes a JSON file from a dictionary.

        :param file: The path to the file to write.
        :type file: Path
        :param file_content: The dictionary to serialize as JSON.
        :type file_content: dict
        :returns: True if writing finished successfully, else False.
        :rtype: bool
        """
        try:
            lock = await get_lock_for_file(file)
            async with lock:
                async with await AsyncPath(file).open(mode='w') as writer:
                    await writer.write(json.dumps(file_content, indent=4))
                    return True
            release_lock_for_file(file)
        except (IOError, UnicodeDecodeError, json.decoder.JSONDecodeError, 
                 TypeError, FileNotFoundError) as error:
            if isinstance(error, IOError):
                raise IOError('An unexpected io error occured!')
            if isinstance(error, UnicodeDecodeError):
                raise UnicodeDecodeError('A decoding error occured!', b'', 0, 0, '')
            if isinstance(error, json.decoder.JSONDecodeError):
                raise json.decoder.JSONDecodeError('JSON decoding error:', '', 0)
            if isinstance(error, TypeError):
                raise TypeError("An unexpected type error uccured!")
            if isinstance(error, FileNotFoundError):
                raise FileNotFoundError("File not found error!")
        return False
            
    @classmethod
    async def write_binary_file(cls, file: Path, content: bytes) -> bool:
        """
        Writes binary content to a file.

        :param file: The path to the file to write.
        :type file: Path
        :param content: The binary content to write.
        :type content: bytes
        :returns: True if writing finished successfully, else False.
        :rtype: bool
        """
        try:
            async with await AsyncPath(file).open(mode='wb') as binary_file:
                await binary_file.write(content)
                return True 
        except (FileNotFoundError, IOError):  
            if FileNotFoundError:
                raise FileNotFoundError("File not found!")
            elif IOError:
                raise IOError("An unexpected io error occured.")
