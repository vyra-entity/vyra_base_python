from dataclasses import dataclass, field


@dataclass(slots=True)
class FunctionConfigEntry:
    tags: list[str]

    # def __dict__(self) -> dict:
    #     return {
    #         "tags": self.tags
    #     }
    
fce = FunctionConfigEntry(tags=["example", "test"])

print(fce)