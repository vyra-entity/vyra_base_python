# VolatileSet.msg

string[] values

# This message is used to represent a volatile set parameter in the system.
# The values field contains the list of string data that can change over time.
# It is typically used for parameters that are expected to change frequently
# and need to be communicated between different components or modules.
# Example usage:
# - values: ["item1", "item2", "item3"]