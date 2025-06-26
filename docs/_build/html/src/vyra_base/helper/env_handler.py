
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

    def load_env(self, env_path: Path):
        """
        Load the environment variables from a file.

        Args:
            env_file (str): Path to the environment file.
        """
        EnvHandler.env = FileReader.open_env_file(env_path / '.env')