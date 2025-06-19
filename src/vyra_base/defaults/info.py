from dataclasses import dataclass


class AuthorInfo:
    """ Info about the development team.

    Contains nested @dataclasses classes which are logically related to the 
    AuthorInfo class. It adds some additional information about the 
    develoment team and the company behind the software solution.
    """
    @dataclass(slots=True, frozen=True)
    class Author():
        """Author class for setting Author information."""

        creator: str = 'Variobotic'
        phone: str = '+49 731 85 07 22 0'
        mail: str = 'info@variobotic.de'


    @dataclass(slots=True, frozen=True)
    class FileChange():
        """File change class for setting"""

        date: str
        time: str

        def __str__(self):
            return f'{self.date}:{self.time}'


    @dataclass(slots=True, frozen=True)
    class Version():
        """Version class for setting

        Attributes:
            major (int): Major version number.
            minor (int): Minor version number.
            patch (int): Patch version number.
        """

        major: int = 0
        minor: int = 1
        patch: int = 0

        def __str__(self) -> str:
            return f'{self.major}.{self.minor}.{self.patch}'
