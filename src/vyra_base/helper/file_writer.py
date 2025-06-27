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
                async with AsyncPath(file).open(mode='w') as writer:
                    await writer.write(json.dumps(file_content, indent=4))
                    return True
            release_lock_for_file(file)
        except (IOError, UnicodeDecodeError, json.decoder.JSONDecodeError, 
                 TypeError, FileNotFoundError) as error:
            if error == IOError():
                raise IOError('An unexpected io error occured!')
            if error == UnicodeDecodeError('', b'', 0, 0, ''):
                raise UnicodeDecodeError('A decoding error occured!', writer, 0, 0, '')
            if error == json.decoder.JSONDecodeError('', '', 0):
                raise json.decoder.JSONDecodeError('JSON decoding error:', '', 0)
            if error == TypeError():
                raise TypeError("An unexpected type error uccured!")
            if error == FileNotFoundError():
                raise FileNotFoundError("File not found error!")
        return False
            
    @classmethod
    async def write_binary_file(cls, content: str) -> bool:
        """
        Writes binary content to a file.

        :param content: The content to write.
        :type content: str
        :returns: True if writing finished successfully, else False.
        :rtype: bool
        """
        try:
            async with AsyncPath(content).open(mode='wb') as binary_file:
                await binary_file.write(binary_file)
                return True 
        except (FileNotFoundError, IOError):  
            if FileNotFoundError:
                raise FileNotFoundError("File not found!")
            elif IOError:
                raise IOError("An unexpected io error occured.")
