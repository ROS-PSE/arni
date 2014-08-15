import rosparam
from specification import Specification
from metadata_tuple import MetadataTuple
from rated_statistics import RatedStatistics
from arni_core.helper import is_seuid


class SpecificationHandler:
    """
    Loads the specifications from the parameter server, stores and compares them to the actual metadata.
    """

    __namespace = '/arni/specifications'

    __specifications = {}

    def __load_specifications(self):
        """
        Loads Specifications from the configurations and stores them in the
        internal storage.
        """

        '''
        params = rosparam.list_params(self.__namespace)
        nslen = len(self.__namespace.split('/'))
        last_seuid = None
        current = None
        for p in params:
            path = p.split('/')
            seuid = path[nslen]
            if seuid != last_seuid:
                if last_seuid != None:
                    self.__specifications[seuid] = current
                last_seuid = seuid
                current = Specification()
            current.add_tuple(MetadataTuple(path[nslen + 1], rosparam.get_param(p)))
        '''
        params = rosparam.get_param(self.__namespace)
        for seuid in params.keys():
            if is_seuid(seuid):
                spec = Specification()
                for k in params[seuid].keys():
                    spec.add_tuple(MetadataTuple(k, params[seuid][k]))
                self.__specifications[seuid] = spec

        print("[SpecificationHandler] Loaded parameters")
        print(self.__specifications)

    def get(self, identifier):
        """
        Returns the Specification object from the internal storage.

        :param identifier: The seuid describing the desired Specification object.
        :return: The Specification object with the given identifier, None if it was not found.
        """
        if identifier in self.__specifications.keys():
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

    def reload_specifications(self):
        """
        Reloads all specifications loaded into the namespace /arni/specifications
        """
        self.__load_specifications()

    def __init__(self):
        """
        Initiates the SpecificationHandler kicking off the loading of available specifications.
        """
        self.__load_specifications()