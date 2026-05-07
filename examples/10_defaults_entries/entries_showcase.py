"""
Showcase for vyra_base.defaults.entries
=======================================

Demonstrates:
- FunctionConfigBaseTypes backward-compatible values
- Dataclass-based entry construction
- Recursive serialization via asdict()
"""

from vyra_base.defaults.entries import (
    FunctionConfigBaseTypes,
    FunctionConfigDisplaystyle,
    FunctionConfigParamTypes,
    FunctionConfigBaseParams,
    FunctionConfigBaseReturn,
    FunctionConfigEntry,
)


def main() -> None:
    # Enum compatibility (both values are supported)
    print("message:", FunctionConfigBaseTypes.message.value)

    entry = FunctionConfigEntry(
        tags=[],
        type=FunctionConfigBaseTypes.message,
        interfacetypes=dict,
        functionname="StatusFeed",
        displayname="Status Feed",
        description="Publishes status updates",
        displaystyle=FunctionConfigDisplaystyle(visible=True, published=True),
        params=[
            FunctionConfigBaseParams(
                datatype=FunctionConfigParamTypes.STRING,
                displayname="status",
                description="Operational status",
            )
        ],
        returns=[
            FunctionConfigBaseReturn(
                datatype=FunctionConfigParamTypes.BOOL,
                displayname="accepted",
                description="Whether message was accepted",
            )
        ],
        namespace="feeder",
    )

    print("\nSerialized FunctionConfigEntry:")
    print(entry.asdict())


if __name__ == "__main__":
    main()
