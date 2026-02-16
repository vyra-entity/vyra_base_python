# News Feed
# Communicate news in the system between different components or modules.
# Generated from JSON metadata - DO NOT EDIT MANUALLY

string type  # The type of news (e.g., 'info', 'warning', 'error', 'action', 'debug')
string message  # The content of the news
builtin_interfaces/Time timestamp  # The timestamp when the news was published
string module_id  # The ID of the module that published the news
string module_name  # The name of the module that published the news
