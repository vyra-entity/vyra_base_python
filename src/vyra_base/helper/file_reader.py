import json
import os
import sys
from asyncio import Lock
from pathlib import Path
from typing import Any, Optional, Union

from dotenv import load_dotenv

from vyra_base.helper._aiopath import AsyncPath
from vyra_base.helper.file_lock import get_lock_for_file, release_lock_for_file

class FileReader:
    """File reader.

    Reads file content from locally stored files into the module.
    """

    @classmethod    
    async def open_json_file(
        cls, config_file: Path, config_default: Path=Path('')) -> Any: 
        """Reads a JSON file.

        To ensure cross platform compatibility, JSON files are opened with
        utf8-sig encoding.

        :param config_file: JSON formatted file.
        :type config_file: Path
        :param config_default: JSON file as a default configuration for the module.
        :type config_default: Path
        :returns: Parsed JSON content.
        :rtype: Any
        :raises IOError: If an unexpected IO error occurs.
        :raises UnicodeDecodeError: If a decoding error occurs.
        :raises json.decoder.JSONDecodeError: If a JSON decoding error occurs.
        :raises TypeError: If an unexpected type error occurs.
        :raises FileNotFoundError: If the file is not found.
        """
        try:
            lock = await get_lock_for_file(config_file)
            async with lock:
                async with await AsyncPath(str(config_file)).open(mode='r', 
                                                encoding='utf-8') as file:
                    json_content = json.loads(await file.read())
                    if json_content:
                        return json_content
                    else:
                        async with await AsyncPath(str(config_default)) \
                                    .open(mode='r', encoding='utf-8-sig') as file:
                            await AsyncPath(str(config_file)).write_text(str(file))
                        return json.loads(await file.read())
            release_lock_for_file(config_file)
        except (IOError, UnicodeDecodeError, json.decoder.JSONDecodeError, 
                 TypeError, FileNotFoundError) as error:
                if error == IOError():
                    raise IOError('An unexpected io error occured!')
                if error == UnicodeDecodeError('', b'', 0, 0, ''):  
                    raise UnicodeDecodeError(
                        'utf-8', b'', 0, 0, 'A decoding error occured!') # type: ignore
                if error == json.decoder.JSONDecodeError('', '', 0): 
                    raise json.decoder.JSONDecodeError('JSON decoding error:', '', 0)

    @classmethod    
    async def open_markdown_file(cls, config_file: Path, config_default='') -> Any:
        """Reads a markdown file.

        :param config_file: Markdown formatted file.
        :type config_file: Path
        :param config_default: Markdown file as a default configuration for the module.
        :type config_default: str
        :returns: File content.
        :rtype: Any
        :raises IOError: If an unexpected IO error occurs.
        :raises UnicodeDecodeError: If a decoding error occurs.
        :raises json.decoder.JSONDecodeError: If a JSON decoding error occurs.
        :raises TypeError: If an unexpected type error occurs.
        :raises FileNotFoundError: If the file is not found.
        """
        try:  
            lock = await get_lock_for_file(config_file)
            async with lock:
                async with await AsyncPath(str(config_file)) \
                            .open(mode='r', encoding='utf-8') as file:
                    content = await file.read()
                    if content:
                        return content
                    else:
                        async with await AsyncPath(str(config_default)) \
                                    .open(mode='r', encoding='utf-8-sig') as file:
                            await AsyncPath(str(config_file)).write_text(str(file))
                        return await file.read()
                    
            release_lock_for_file(config_file)

        except (IOError, UnicodeDecodeError, json.decoder.JSONDecodeError, 
                 TypeError, FileNotFoundError) as error:
            if error == UnicodeDecodeError: # type: ignore
                raise UnicodeDecodeError(
                    'utf-8', b'', 0, 0, 'A decoding error occured!') # type: ignore
            if error == json.decoder.JSONDecodeError: # type: ignore
                raise json.decoder.JSONDecodeError('JSON decoding error.', '', 0)
        finally:
            await release_lock_for_file(config_file)

    @classmethod
    async def open_env_file(cls, env_path: Path) -> Optional[dict]:
        """Reads an environment (.env) file.

        :param env_path: Path to the directory containing the .env file.
        :type env_path: Path
        :returns: Dictionary of environment variables.
        :rtype: dict
        :raises IOError: If an unexpected IO error occurs.
        :raises TypeError: If an unexpected type error occurs.
        :raises FileNotFoundError: If the file is not found.
        """
        try:
            lock: Lock = await get_lock_for_file(env_path / '.env')
            async with lock:
                load_dotenv(dotenv_path=env_path)
                env_content: list = []
                env_vars: dict = dict(os.environ)
                return env_vars
        finally:
            await release_lock_for_file(env_path / '.env')

    @classmethod
    async def open_toml_file(cls, config_file: Path) -> Optional[Any]:
        """Reads a TOML file.

        :param config_file: TOML formatted file.
        :type config_file: Path
        :returns: Parsed TOML content as a dictionary.
        :rtype: dict[str, Any]
        :raises ImportError: If tomli is not installed for Python < 3.11.
        :raises IOError: If an unexpected IO error occurs.
        :raises TypeError: If an unexpected type error occurs.
        :raises FileNotFoundError: If the file is not found.
        """
        try:
            if sys.version_info >= (3, 11):
                import tomllib
            else:
                try:
                    import tomli as tomllib
                except ImportError:
                    raise ImportError(
                        "Python < 3.11 benötigt das Paket 'tomli' zum Parsen "
                        "von TOML-Dateien. Installiere es mit: pip install tomli"
                    )
            
            lock: Lock = await get_lock_for_file(config_file)
            async with lock:
                toml_content: dict = {}
                async with await AsyncPath(config_file) \
                            .open(mode='r', encoding='utf-8-sig') as file:
                    raw_content = await file.read()
                    
                    if not isinstance(raw_content, str):
                        raise TypeError(
                            f"{config_file} must be a string. Red: {raw_content}"
                        )

                    toml_content = tomllib.loads(raw_content)
                return toml_content
        finally:
            await release_lock_for_file(config_file)

    @classmethod
    async def open_ini_file(cls, config_file: Path) -> Optional[Any]:
        """Reads an INI file.

        :param config_file: INI formatted file.
        :type config_file: Path
        :returns: Parsed INI content as a dictionary.
        :rtype: dict[str, Any]
        :raises ImportError: If configparser is not available.
        :raises IOError: If an unexpected IO error occurs.
        :raises TypeError: If an unexpected type error occurs.
        :raises FileNotFoundError: If the file is not found.
        """
        try:
            import configparser
            
            lock: Lock = await get_lock_for_file(config_file)
            async with lock:
                config = configparser.ConfigParser()
                config.read(str(config_file))
            
            return {s: dict(config.items(s)) for s in config.sections()}
        finally:
            await release_lock_for_file(config_file)

    @classmethod
    async def open_yaml_file(cls, config_file: Path) -> Optional[Any]:
        """Reads a YAML file.

        :param config_file: YAML formatted file.
        :type config_file: Path
        :returns: Parsed YAML content as a dictionary.
        :rtype: dict[str, Any]
        :raises ImportError: If PyYAML is not installed.
        :raises IOError: If an unexpected IO error occurs.
        :raises TypeError: If an unexpected type error occurs.
        :raises FileNotFoundError: If the file is not found.
        """
        try:
            import yaml
            
            lock: Lock = await get_lock_for_file(config_file)
            async with lock:
                async with await AsyncPath(config_file) \
                            .open(mode='r', encoding='utf-8-sig') as file:
                    yaml_content = yaml.safe_load(await file.read())
                    return yaml_content
        finally:
            await release_lock_for_file(config_file)