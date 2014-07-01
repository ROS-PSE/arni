class SpecificationHandler:
    """
    Loads the specifications from the parameter server, stores and compares them to the actual metadata.
    """

    __specifications = {}

    def __load_specifications(self):
        """
        Loads Specifications from the configurations and stores them in the
        internal storage.
        """
        pass

    def get(self, identifier):
        """
        Returns the Specification object from the internal storage.
        :param identifier: The seuid describing the desired Specification object.
        :return: The Specification object with the given identifier, None if it was not found.
        """
        if identifier in self.__specifications:
            return self.__specifications[identifier]
        return None

    def compare(self, metadata, specification = None):
        """
        Compares a Metadata object with a Specification object regarding all available matching fields.
        :param metadata: The Metadata object
        :type metadata: Metadata.
        :param specification: The Specification object, alternatively a string identifying it.
        :type specification: Specification or str.
        :returns: A RatedStatistics object representing the result.
        """
        pass

    def __init__(self):
        """
        Initiates the SpecificationHandler kicking off the loading of available specifications.
        """
        self.__load_specifications()