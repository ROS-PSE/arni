class MetadataTuple:
    """
    Stores any kind of value for a certain key.
    Specifications storing values indicating limits,
    Metadata storing absolute actual values.
    """

    def __init__(self, key, value):
        """
        Give the key and the value on initiating.

        :param key: The identifier for the stored measurement.
        :type key: str
        :param value: Either a tuple for Specification objects or an absolute value.
        :type value: object
        :raise ValueError: If the key is null or empty.
        """
        if key == None or key.trim() == "":
            raise ValueError("Key cannot be empty!")
        self.key = key.trim()
        self.value = value