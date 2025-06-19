# ErrorFeed.msg

uint32 error_code
string module_id
string description
string solution
string miscellaneous
builtin_interfaces/Time timestamp

# The error code representing the specific error
# A description of the error
# A list of asset IDs related to the error
# The timestamp when the error occurred
# This message is used to communicate error information in the system
# between different components or modules.
# It can be used for logging, debugging, or alerting purposes.
# The error_code is a unique identifier for the error type (hex-foramt).
# The error_description provides additional context about the error.
# The assets array contains objects (e.g., pictures) to illustrate the error.
