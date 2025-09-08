# VBASEUpdateParamEvent.msg

string param_key
string changed_from
string changed_to
string id
time timestamp

# Event message for parameter updates in the vyra system.
# 'param_key' is the key of the parameter that was changed.
# 'changed_from' indicates the previous value of the parameter.
# 'changed_to' indicates the new value of the parameter.
# 'id' is the unique identifier for the parameter.
# 'timestamp' records the time when the change occurred.

# This message is published whenever a parameter is updated in the system.
