# Providing helper function to be used anywhere

import difflib
from typing import Sequence


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


def fuzzy_match(
    word: str,
    possibilities: Sequence[str],
    n: int = 3,
    cutoff: float = 0.6,
) -> list[str]:
    """Find the closest matches to *word* from a list of *possibilities*.

    Uses :func:`difflib.get_close_matches` under the hood, providing a
    convenient wrapper with sensible defaults for industrial automation
    diagnostics (e.g. suggesting valid interface names when a lookup fails).

    :param word: The string to match against the possibilities.
    :type word: str
    :param possibilities: The pool of candidate strings to search in.
    :type possibilities: Sequence[str]
    :param n: Maximum number of close matches to return. Defaults to 3.
    :type n: int
    :param cutoff: Minimum similarity score in the range ``[0, 1]``. Strings
        with a ratio below this threshold are excluded. Defaults to 0.6.
    :type cutoff: float
    :return: A list of the best matches (may be empty if nothing is close
        enough), sorted by decreasing similarity.
    :rtype: list[str]

    Usage example::

        from vyra_base.helper.func import fuzzy_match

        candidates = ["state_feeder", "news_feeder", "error_feeder"]
        suggestions = fuzzy_match("stae_feeder", candidates)
        # → ["state_feeder"]
    """
    return difflib.get_close_matches(word, possibilities, n=n, cutoff=cutoff)



