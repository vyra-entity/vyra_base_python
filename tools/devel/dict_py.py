from dataclasses import dataclass, field, asdict

def foo():
    pass

@dataclass(slots=True)
class Plugins:
    name: str
    version: str = field(default="1.0")
    enabled: bool = field(default=True)

@dataclass(slots=True)
class FunctionConfigEntry:
    tags: list[str]
    params: list[Plugins] = field(default_factory=list)
    callables: list[callable]
    # def __dict__(self) -> dict:
    #     return {
    #         "tags": self.tags
    #     }

fce = FunctionConfigEntry(
    tags=["example", "test"], 
    params=[{'name': 'plugin1'}, {'name': 'plugin2'}],
    callables=[foo]
)

# fce = FunctionConfigEntry(
#     tags=["example", "test"], 
#     params=[Plugins(name="plugin1"), Plugins(name="plugin2")]
# )

print(asdict(fce))