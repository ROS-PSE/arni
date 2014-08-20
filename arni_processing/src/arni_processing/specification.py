from metadata_tuple import MetadataTuple


class Specification:
    """
    An object loaded from the specification configurations
    and basis for comparison of Metadata
    with desired values.
    """

    __values = {}

    def add_tuple(self, t):
        """
        Adds a MetadataTuple to the bundle.

        :param t: The Tuple to be added.
        :type t: MetadataTuple
        """
        self.__values[t.key] = t

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
        Returns the value of the MetadataTuple with the given key.

        :param key: The identifier for the stored measurement.
        :type key: str
        :return: A MetadataTuple containing the limits in its value field.
        """
        if key in self.__values:
            return self.__values[key]
        return False

    def __init__(self, tuples={}):
        """
        A new Specification object to bundle multiple specifications.

        :param tuples: Optionally give tuples on creation. Use add_tuple(MetadataTuple) afterwards.
        :type tuples: dict(str, MetadataTuple)
        """
        self.__values = tuples

    def __str__(self):
        for t in self.__values:
            print("- " + str(t))