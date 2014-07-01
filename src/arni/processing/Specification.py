from MetadataTuple import MetadataTuple

class Specification:
    """
    An object loaded from the specification configurations
    and basis for comparison of Metadata
    with desired values.
    """

    __values = {}

    def add_tuple(self, tuple):
        """
        Adds a MetadataTuple to the bundle.

        :param tuple: The Tuple to be added.
        :type tuple: MetadataTuple
        """
        self.__values[tuple.key] = tuple

    def get(self, key):
        """
        Returns the value of the MetadataTuple with the given key.

        :param key: The identifier for the stored measurement.
        :type key: str
        :return: A list containing limit values for the measured fields. False, if the key does not exist.
        """
        if key in self.__values:
            return self.__values[key]
        return False

    def __init__(self, tuples = {}):
        """
        A new Specification object to bundle multiple specifications.

        :param tuples: Optionally give tuples on creation. Use add_tuple(MetadataTuple) afterwards.
        :type tuples: dict(str, MetadataTuple)
        """
        self.__values = tuples