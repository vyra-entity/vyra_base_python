# LoggerStream.msg

string log_level
string log_message
builtin_interfaces/Time timestamp

# This message is used to represent a log entry in the vos system.
# The log_level field indicates the severity of the log message (e.g., INFO, WARNING, ERROR).
# The log_message field contains the actual log message.
# The timestamp field indicates when the log entry was created.
# Example usage:
# - log_level: "INFO"
# - log_message: "System started successfully"
# - timestamp: "2023-10-01T12:00:00Z"