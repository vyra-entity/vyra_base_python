from dataclasses import dataclass


class AuthorInfo:
    """
    Info about the development team.

    Contains nested :py:class:`dataclasses` classes which are logically related to the 
    AuthorInfo class. It adds some additional information about the 
    development team and the company behind the software solution.
    """

    @dataclass(slots=True, frozen=True)
    class Author:
        """
        Author class for setting Author information.

        :cvar creator: Name of the creator or team.
        :cvar phone: Contact phone number.
        :cvar mail: Contact email address.
        """

        creator: str = 'V.Y.R.A. Development Team'
        phone: str = '+49 731 85 07 22 0'
        mail: str = 'vyra.0a1@gmail.com'

    @dataclass(slots=True, frozen=True)
    class FileChange:
        """
        File change class for setting file change information.

        :ivar date: Date of the file change.
        :ivar time: Time of the file change.
        """

        date: str
        time: str

        def __str__(self):
            """
            Returns a string representation of the file change.

            :return: String in the format 'date:time'.
            :rtype: str
            """
            return f'{self.date}:{self.time}'

    @dataclass(slots=True, frozen=True)
    class Version:
        """
        Version class for setting version information.

        :cvar major: Major version number.
        :cvar minor: Minor version number.
        :cvar patch: Patch version number.
        """

        major: int = 0
        minor: int = 1
        patch: int = 0

        def __str__(self) -> str:
            """
            Returns the version as a string.

            :return: Version string in the format 'major.minor.patch'.
            :rtype: str
            """
            return f'{self.major}.{self.minor}.{self.patch}'
