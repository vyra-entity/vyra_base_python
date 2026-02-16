import os

from pathlib import Path
from vyra_base.helper.file_reader import FileReader


class EnvHandler:
    """
    Class to handle the environment variables for the project.
    """
    env: dict = {}

    def __init__(self):
        """
        Initialize the EnvHandler class.
        """
        self.env = {}

    async def load_env(self, env_path: Path):
        """
        Load the environment variables from a file.

        :param env_path: Path to the environment file directory.
        :type env_path: Path
        """
        EnvHandler.env = await FileReader.open_env_file(env_path / '.env')

def get_env_required(name: str) -> str:
    value = os.getenv(name)
    if value is None:
        raise EnvironmentError(f"Kritischer Fehler: Die Umgebungsvariable '{name}' ist nicht gesetzt!")
    return value