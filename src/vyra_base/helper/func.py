# Providing helper function to be used anywhere

def deep_merge(d1: dict, d2: dict) -> dict:
    """Recursively merges two dictionaries, combining nested dictionaries.
    If a key exists in both dictionaries and both values are dictionaries,
    they are merged recursively. Otherwise, the value from the second dictionary
    overwrites the value from the first dictionary. 
    :param d1: The first dictionary.
    :param d2: The second dictionary.
    :return: A new dictionary that is the result of merging d1 and d2."""
    if not isinstance(d1, dict) or not isinstance(d2, dict):
        raise ValueError("Both inputs must be dictionaries.")

    result = d1.copy()
    for k, v in d2.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = v
    return result



