# Error Feed
# Communicate error information in the system between different components or modules.
# Generated from JSON metadata - DO NOT EDIT MANUALLY

uint32 error_code  # The error code representing the specific error
string module_id  # The module ID where the error occurred
string description  # A description of the error
string solution  # A solution or suggestion for the error
string miscellaneous  # Additional information
builtin_interfaces/Time timestamp  # The timestamp when the error occurred
