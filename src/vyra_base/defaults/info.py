# Unused structure (no references in src/vyra_base outside defaults).
# Kept commented for traceability; see defaults/change-file.md.
#
# from dataclasses import dataclass
#
#
# class AuthorInfo:
#     """Meta information about author and version."""
#
#     @dataclass(slots=True, frozen=True)
#     class Author:
#         creator: str = 'V.Y.R.A. Development Team'
#         phone: str = '+49 731 85 07 22 0'
#         mail: str = 'vyra.0a1@gmail.com'
#
#     @dataclass(slots=True, frozen=True)
#     class FileChange:
#         date: str
#         time: str
#
#         def __str__(self):
#             return f'{self.date}:{self.time}'
#
#     @dataclass(slots=True, frozen=True)
#     class Version:
#         major: int = 0
#         minor: int = 1
#         patch: int = 0
#
#         def __str__(self) -> str:
#             return f'{self.major}.{self.minor}.{self.patch}'
