# VolatileHash.msg

KeyValue[] entries

# This message is used to represent a volatile hash parameter in the system.
# The entries field contains the key-value pairs that can change over time.
# It is typically used for parameters that are expected to change frequently
# and need to be communicated between different components or modules.
# Example usage:
# - entries: [{key: "field1", value: "value1"}, {key: "field2", value: "value2"}]