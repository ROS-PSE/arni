import rospy
from specification import Specification
from metadata_tuple import MetadataTuple
from rated_statistics import RatedStatistics
from arni_core.helper import is_seuid


class SpecificationHandler:
    """
    Loads the specifications from the parameter server, stores and compares them to the actual metadata.
    """

    __namespace = '/arni/specifications'

    def __load_specifications(self):
        """
        Loads Specifications from the configurations and stores them in the
        internal storage.
        """
        try:
            params = rospy.get_param(self.__namespace)
            for seuid in params.keys():
                if is_seuid(seuid):
                    spec = Specification()
                    for k in params[seuid].keys():
                        spec.add_tuple(MetadataTuple(k, params[seuid][k]))
                    self.__specifications[seuid] = spec
                else:
                    rospy.logdebug("[SpecificationHandler][__load_specifications] %s is not a valid seuid." % seuid)
        except KeyError as err:
            pass
        rospy.loginfo("[SpecificationHandler] Loaded %s parameters." % str(len(self.__specifications.keys())))

    def loaded_specifications(self):
        """
        Returns a list containing all seuids of loaded specifications.

        :return: A list of strings.
        """
        return self.__specifications.keys()

    def get(self, identifier):
        """
        Returns the Specification object from the internal storage.

        :param identifier: The seuid describing the desired Specification object.
        :type identifier: str
        :return: The Specification object with the given identifier, None if it was not found.
        """
        if identifier in self.__specifications.keys():
            return self.__specifications[identifier]
        return None

    def compare(self, data, identifier, specification=None):
        """
        Compares a Metadata object with a Specification object regarding all available matching fields.

        :param data: The Metadata
        :type data: object.
        :param identifier: The identifier of the metadata package.
        :type identifier: str
        :param specification: The Specification object, alternatively a string identifying it.
        :type specification: Specification or str.
        :returns: A RatedStatistics object representing the result.
        """
        result = RatedStatistics()
        if specification is None:
            if identifier in self.__specifications.keys():
                specification = self.__specifications[identifier]
            else:
                if identifier[0] == "t":
                    pass
                else:
                    errmsg = "[SpecificationHandler] No Specification available for " + identifier
                    rospy.loginfo(errmsg)
                return None
                status = 2
        for field in dir(data):
            current_obj = {}
            value = getattr(data, field)
            if specification.has_field(field):
                specs = specification.get(field).value
                limits = specs[0:1]
                if len(specs) > 2 and specs[2][0].lower() == "r":
                    if limits[1] > 1:
                        limits[1] -= 1
                    m = limits[0]
                    r = limits[1]
                    limits[0] = m - m * r
                    limits[1] = m + m * r
                if isinstance(value, list):
                    current_obj["state"] = []
                    current_obj["actual"] = []
                    current_obj["expected"] = []
                    for v in value:
                        current_obj["actual"].append(v)
                        current_obj["state"].append(self.__compare(v, limits))
                        current_obj["expected"].append(limits)
                else:
                    status = self.__compare(value, limits)
                    current_obj["state"] = status
                    current_obj["actual"] = value
                    current_obj["expected"] = limits
            else:
                current_obj["state"] = 2
            result.add_value(field, current_obj["actual"], current_obj["expected"], current_obj["state"])
        return result

    def __compare(self, value, reference):
        if not isinstance(reference, list) or len(reference) < 2 or\
            not isinstance(reference[0], (int, long, float, complex)) or\
            not isinstance(reference[1], (int, long, float, complex)):
            return 2
        reference.sort()
        r = (reference[0] - reference[1]) / 2
        m = reference[0] + r
        if reference[0] > value:
            state = 1
        elif reference[1] < value:
            state = 0
        else:
            state = 3
        return state

    def reload_specifications(self):
        """
        Reloads all specifications loaded into the namespace /arni/specifications
        """
        self.__specifications = {}
        self.__load_specifications()

    def __init__(self):
        """
        Initiates the SpecificationHandler kicking off the loading of available specifications.
        """
        self.__specifications = {}
        self.__load_specifications()