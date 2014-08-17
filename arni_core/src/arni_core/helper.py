import re

"""
This helper module contains useful functions and variables that
are needed throughout the whole arni package.
"""

#: the delimiter the seuid uses
SEUID_DELIMITER = "!"


def is_seuid(seuid):
    """
    Determines whether the given parameter is a valid seuid.
    :param seuid: A string presenting a seuid to validate.
    :return: True, if the given parameter is a valid seuid, false if not.
    """
    p = re.compile('^([nhtc])' + SEUID_DELIMITER + '([_a-z].*)')
    m = p.match(seuid)
    if m is None:
        return False
    arglen = len(m.group(2).strip().split(SEUID_DELIMITER))
    if m.group(1) == "c":
        return arglen == 3
    else:
        return arglen == 1
