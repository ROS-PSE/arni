class Outcome(object):
    """An enumeration of all states an rated statistic can have."""

    HIGH = 0
    LOW = 1
    UNKNOWN = 2
    OK = 3
    OUT_OF_BOUNDS = 4

    def is_valid(out):
        """Check if a outcome (numeric representation) is valid.

        :param out: The outcome in numeric format.
        :type out:  int

        :return:    whether out is a valid outcome or not.
        :rtype:     boolean
        """
        if (out >= 0 & out <= 4):
            return True
        return False
