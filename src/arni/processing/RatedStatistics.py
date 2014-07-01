class RatedStatistics:
    """
    Wraps the result of the comparison between the actual metadata and the specification.
    """
    metatype = []
    actual = []
    expected = []
    state = []

    def __init__(self, seuid):
        """
        Creates a new RatedStatistics object for the given connection identifier..
        :param seuid: Identifies a host/node/connection.
        :type seuid: str.
        """
        self.seuid = seuid

    def add_value(self, metatype, actual, expected, state):
        """
        Adds a group of values for a metatype.
        :param metatype: The measured field.
        :type metatype: str.
        :param actual: The actual value.
        :param expected: The expected value.
        :param state: An error state based on the difference between actual and expected.
        """
        self.metatype.append(metatype)
        self.actual.append(actual)
        self.expected.append(expected)
        self.state.append(state)

    def get_value(self, metatype):
        """
        Returns values of the given metatype.
        :param metatype: The metatype to return the values for.
        :type metatype: str.
        :returns: A dictionary with the keys *metatype*, *actual*, *expected* and *state*, each field containing it's respective values.
        """
        if metatype in self.metatype:
            index = self.metatype.index(metatype)
            return {"metatype": metatype, "actual": self.metatype[index], "expected": self.expected[index], "state": self.state[index] }
        else:
            return False