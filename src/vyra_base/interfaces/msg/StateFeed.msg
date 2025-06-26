# StateFeeder.msg

string previous
string current
string trigger
unique_identifier_msgs/UUID module_id
string module_name
builtin_interfaces/Time timestamp


# The previous state of the system
# The current state of the system
# The timestamp when the state change occurred
# This message is used to communicate state changes in the system
# between different components or modules.