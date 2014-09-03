class Specification:
    """
    An object loaded from the specification configurations
    and basis for comparison of Metadata
    with desired values.
    """

    def add_tuple(self, t):
        """
        Adds a Tuple to the bundle.

        :param t: (key, value)
        :type t: tuple
        """
        self.__values[t[0]] = t

    def keys(self):
        """
        Returns all stored keys.

        :return: list(str)
        """
        return self.__values.keys()

    def has_field(self, field):
        """
        Returns whether a field is defined in this specification.

        :param field: The metadata field to check for.
        :type field: str
        :return: bool
        """
        return field in self.__values.keys()

    def get(self, key):
        """
        Returns the value of the specification with the given key.

        :param key: The identifier for the stored measurement.
        :type key: str
        :return: A tuple containing the limits in its value field.
        """
        if key in self.__values:
            return self.__values[key]
        return False

    def __init__(self, tuples=[]):
        """
        A new Specification object to bundle multiple specifications.

        :param tuples: Optionally give tuples on creation. Use add_tuple(tuple) afterwards.
        :type tuples: list(tuple(str, object)
        """
        self.__values = {}
        for t in tuples:
            self.add_tuple(t)
        self.seuid = ""

    def __str__(self):
        output = ""
        for k, t in self.__values.iteritems():
            output += "- %s: %s\n" % (str(k), str(t))
        return output