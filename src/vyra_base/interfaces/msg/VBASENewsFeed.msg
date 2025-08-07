# NewsFeed.msg

uint8 level
string message
builtin_interfaces/Time timestamp
unique_identifier_msgs/UUID uuid
string module_name
string module_id

# Possible Level:
# Action message (0)
# Debug message (1)
# Info message (2)
# Hint message (3)
# Warning message (4)