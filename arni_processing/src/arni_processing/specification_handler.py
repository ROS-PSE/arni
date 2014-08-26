import rospy
from specification import Specification
from rated_statistics import RatedStatisticsContainer
from arni_core.helper import *
import arni_msgs
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity


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
            for o in params:
                for seuid in o.keys():
                    if SEUID().is_valid(seuid):
                        spec = Specification()
                        for k in o[seuid].keys():
                            spec.add_tuple((k, o[seuid][k]))
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
        Compares a Message object with a Specification object regarding all available matching fields.

        :param data: The actual data.
        :type data: object.
        :param identifier: The identifier of the metadata package.
        :type identifier: str
        :param specification: The Specification object, alternatively a string identifying it.
        :type specification: Specification or str.
        :returns: A RatedStatisticsContainer object representing the result.
        """
        if identifier is None:
            rospy.logdebug("[SpecificationHandler][compare] No identifier given.")
            return None
        if not SEUID().is_valid(identifier):
            rospy.logdebug("[SpecificationHandler][compare] Given identifier is invalid.")
            return None
        if data is None:
            rospy.logdebug("[SpecificationHandler][compare] No data given.")
            return None
        result = RatedStatisticsContainer(identifier)
        if specification is None:
            if identifier in self.__specifications.keys():
                specification = self.__specifications[identifier]
            else:
                if identifier[0] == "c":
                    if SEUID(identifier).topic in self.__specifications.keys():
                        specification = self.__specifications[SEUID(identifier).topic]
        # if specification is None:
        # rospy.logdebug("[SpecificationHandler][compare] No Specification available for %s" % identifier)
        for field in dir(data):
            if field[0] == "_" or "serialize" in field:
                continue
            current_obj = {}
            value = getattr(data, field)
            try:
                limits = self.__get_limits(specification, field)
            except TypeError:
                limits = None
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
            result.add_value(field, current_obj["actual"], current_obj["expected"], current_obj["state"])
        return result

    def compare_topic(self, data=[]):
        """
        Compares Messages about one topic

        :param data: List of Statistics messages
        :return: list of RatedStatistics messages
        """
        by_topic = {}
        result = []
        for message in data:
            seuid = SEUID(message)
            if not seuid.get_seuid("topic") in by_topic.keys():
                by_topic[seuid.get_seuid("topic")] = {
                    "window_min": rospy.Time.now(),
                    "window_max": rospy.Time(0),
                    "delivered_msgs": 0,
                    "dropped_msgs": 0,
                    "traffic": 0,
                    "stamp_age_mean": [],
                    "stamp_age_stddev": [],
                    "stamp_age_max": rospy.Duration(0),
                    "packages": 0
                }
            window_len = message.window_stop - message.window_start
            scale = window_len.to_sec() / window_len.to_sec()
            by_topic[seuid.get_seuid("topic")]["window_min"] = min(message.window_start,
                                                                   by_topic[seuid.get_seuid("topic")]["window_min"])
            by_topic[seuid.get_seuid("topic")]["window_max"] = max(message.window_stop,
                                                                   by_topic[seuid.get_seuid("topic")]["window_max"])
            # by_topic[seuid.get_seuid("topic")]["delivered_msgs"] += message.delivered_msgs * scale
            by_topic[seuid.get_seuid("topic")]["dropped_msgs"] += message.dropped_msgs * scale
            by_topic[seuid.get_seuid("topic")]["traffic"] += message.traffic * scale
            by_topic[seuid.get_seuid("topic")]["stamp_age_max"] = max(message.stamp_age_max,
                                                                      by_topic[seuid.get_seuid("topic")][
                                                                          "stamp_age_max"])
            by_topic[seuid.get_seuid("topic")]["stamp_age_mean"].append(message.stamp_age_mean * scale)
            by_topic[seuid.get_seuid("topic")]["packages"] += 1
        for topic, data in by_topic.iteritems():
            if topic in self.__specifications.keys():
                specification = self.__specifications[topic]
            else:
                specification = None
            r = RatedStatistics()
            r.window_start = data["window_min"]
            r.window_stop = data["window_max"]
            window_len = data["window_max"] - data["window_min"]
            r.seuid = topic
            fields = ("dropped_msgs", "traffic", "traffic_per_second", "stamp_age_max", "stamp_age_mean", "packages",
                      "packages_per_second")
            alt_names = {"packages_per_second": "bandwidth"}
            # fields = ("delivered_msgs", "dropped_msgs", "traffic", "stamp_age_max", "stamp_age_mean")
            for f in fields:
                limits = self.__get_limits(specification, f)
                re = RatedStatisticsEntity()
                re.statistic_type = f
                if f in alt_names.keys():
                    re.statistic_type = alt_names[f]
                if f[-10:] == "per_second":
                    value = data[f[0:-11]] / window_len.to_sec()
                elif f == "stamp_age_mean":
                    x = data[f]
                    value = sum(y.to_sec() for y in x) / len(x)
                else:
                    value = data[f]
                re.actual_value.append(str(value))
                re.expected_value.append(str(limits))
                re.state = [self.__compare(value, limits)]
                r.rated_statistics_entity.append(re)
            result.append(r)
            # print(result)
        return result

    def __get_limits(self, specification, field):
        if specification is None or field is None:
            return None
        try:
            specs = specification.get(field)[1]
            limits = specs[0:2]
            if len(specs) > 2 and specs[2][0].lower() == "r":
                if limits[1] > 1:
                    limits[1] -= 1
                m = limits[0]
                r = limits[1]
                limits[0] = m - m * r
                limits[1] = m + m * r
        except TypeError, AttributeError:
            limits = None
        return limits

    def __compare(self, value, reference):
        if not isinstance(reference, list) or len(reference) < 2 or \
                not isinstance(reference[0], (int, long, float, complex)) or \
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