class FeederException(Exception):
    """
    Exception raised for errors in the feeder.

    :param message: Explanation of the error.
    :type message: str
    """
    def __init__(self, message):
        """
        Initialize FeederException.

        :param message: Explanation of the error.
        :type message: str
        """
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        """
        Return the formatted error message.

        :return: Formatted error message.
        :rtype: str
        """
        return f"Feeder: {self.message}"