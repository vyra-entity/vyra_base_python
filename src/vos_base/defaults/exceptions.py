
class FeederException(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

    def __str__(self):  # Optional: Formatierung der Fehlermeldung
        return f"Feeder: {self.message}"