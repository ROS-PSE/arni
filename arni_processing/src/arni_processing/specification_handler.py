import rospy
from specification import Specification
from rated_statistics import RatedStatisticsContainer
from arni_core.helper import *
import arni_msgs
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
#from arni_gui.helper_functions import prepare_number_for_representation


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
            if isinstance(params, dict):
                specifications = []
                for x in params.values():
                    for y in x:
                        specifications.append(y)
            else:
                specifications = params
            for o in specifications:
                for seuid in o.keys():
                    if SEUID().is_valid(seuid):
                        spec = Specification()
                        spec.seuid = seuid
                        for k in o[seuid].keys():
                            spec.add_tuple((k, o[seuid][k]))
                        self.__specifications[seuid] = spec
                    else:
                        rospy.logdebug("[SpecificationHandler][__load_specifications] %s is not a valid seuid." % seuid)
        except KeyError:
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
        if identifier[0] == "c":
            if SEUID(identifier).topic in self.__specifications.keys():
                return self.__specifications[SEUID(identifier).topic]
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
        if identifier[0] == "n":
            result.host = data.host
        if specification is None:
            specification = self.get(identifier)
        window_len = data.window_stop - data.window_start
        if window_len.to_sec() == 0:
            window_len = rospy.Duration(1)
        fields = dir(data)
        exclude = ("delivered_msgs", "traffic")
        for x in exclude:
            if x in fields:
                fields.remove(x)
        if identifier[0] == "c":
            fields.append("bandwidth")
            fields.append("frequency")
        for field in fields:
            value = None
            if field[0] == "_" or "serialize" in field:
                continue
            current_obj = {}
            if field == "bandwidth":
                value = data.traffic / window_len.to_sec()
            elif field == "frequency":
               value = data.delivered_msgs / window_len.to_sec()
            else:
                value = getattr(data, field)
            if value is not None:
                limits = self.__get_limits(specification, field)
                if isinstance(value, (list, tuple)):
                    current_obj["state"] = []
                    current_obj["actual"] = []
                    current_obj["expected"] = []
                    for i, v in enumerate(value):
                        limits = self.__get_limits(specification, field, i)
                        current_obj["actual"].append(v)
                        current_obj["state"].append(self.__compare(v, limits))
                        current_obj["expected"].append(limits)
                else:
                    status = self.__compare(value, limits)
                    current_obj["state"] = status
                    current_obj["actual"] = value
                    current_obj["expected"] = limits
                result.add_value(field, current_obj["actual"], current_obj["expected"], current_obj["state"])
        result.add_value("alive", ["True"], ["True"], [2])
        return result

    def compare_topic(self, data=None):
        """
        Compares Messages about one topic

        :param data: List of Statistics messages
        :return: list of RatedStatistics messages
        """
        if not data:
            data = []
        by_connection = {}
        by_topic = {}
        result = []
        frequency_set = False
        for message in data:
            seuid = SEUID(message)
            if not seuid.identifier in by_connection.keys():
                by_connection[seuid.identifier] = {
                    "window_min": rospy.Time.now(),
                    "window_max": rospy.Time(0),
                    "dropped_msgs": 0,
                    "frequency": 0,
                    "traffic": 0,
                    "bandwidth": 0,
                    "stamp_age_mean": rospy.Duration(0),
                    "stamp_age_stddev": rospy.Duration(0),
                    "stamp_age_max": rospy.Duration(0),
                    "count": 0,
                }
            by_connection[seuid.identifier]["count"] += 1
            window_len = message.window_stop - message.window_start
            if window_len.to_sec() == 0:
                window_len = rospy.Duration(1)
            by_connection[seuid.identifier]["window_min"] = min(message.window_start, by_connection[seuid.identifier]["window_min"])
            by_connection[seuid.identifier]["window_max"] = max(message.window_stop, by_connection[seuid.identifier]["window_max"])
            if hasattr(message, "delivered_msgs"):
                frequency_set = True
                by_connection[seuid.identifier]["frequency"] += message.delivered_msgs / float(window_len.to_sec())
            by_connection[seuid.identifier]["dropped_msgs"] += message.dropped_msgs
            by_connection[seuid.identifier]["traffic"] += message.traffic
            by_connection[seuid.identifier]["bandwidth"] += message.traffic / float(window_len.to_sec())
            by_connection[seuid.identifier]["stamp_age_max"] = max(message.stamp_age_max, by_connection[seuid.identifier]["stamp_age_max"])
            by_connection[seuid.identifier]["stamp_age_mean"] += message.stamp_age_mean
            #TODO by_connection[seuid.identifier]["stamp_age_stddev"]
        for connection in by_connection:
            seuid = SEUID(connection)
            for key in 'frequency', 'bandwidth', 'dropped_msgs', 'traffic', 'stamp_age_mean': # average
              by_connection[connection][key] /= by_connection[connection]['count']

            topic = seuid.get_seuid('topic')
            if not topic in by_topic.keys():
                by_topic[topic] = {
                    "window_min": rospy.Time.now(),
                    "window_max": rospy.Time(0),
                    "dropped_msgs": 0,
                    "frequency": 0,
                    "traffic": 0,
                    "bandwidth": 0,
                    "stamp_age_mean": rospy.Duration(0),
                    "stamp_age_stddev": rospy.Duration(0),
                    "stamp_age_max": rospy.Duration(0),
                    'count': 0,
                }
            by_topic[topic]['count'] += 1
            by_topic[topic]["window_min"] = min(by_connection[connection]['window_min'], by_topic[topic]["window_min"])
            by_topic[topic]["window_max"] = max(by_connection[connection]['window_max'], by_topic[topic]["window_max"])
            if "frequency" in by_connection[connection]:
                frequency_set = True
                by_topic[topic]["frequency"] += by_connection[connection]['frequency']
            by_topic[topic]["dropped_msgs"] += by_connection[connection]['dropped_msgs']
            by_topic[topic]["traffic"] += by_connection[connection]['traffic']
            by_topic[topic]["bandwidth"] += by_connection[connection]['bandwidth']
            by_topic[topic]["stamp_age_max"] = max(by_connection[connection]['stamp_age_max'], by_topic[topic]["stamp_age_max"])
            by_topic[topic]["stamp_age_mean"] +=  by_connection[connection]['stamp_age_mean']
            #TODO by_connection[seuid.identifier]["stamp_age_stddev"]
        for topic in by_topic:
          by_topic[topic]['stamp_age_mean'] /= by_topic[topic]['count']
        for topic, data in by_topic.iteritems():
            specification = self.get(topic)
            r = RatedStatistics()
            r.window_start = data["window_min"]
            r.window_stop = data["window_max"]
            window_len = data["window_max"] - data["window_min"]
            if window_len.to_sec() == 0:
                window_len = rospy.Duration(1)
            r.seuid = topic
            fields = ["dropped_msgs", "traffic", "bandwidth", "stamp_age_max", "stamp_age_mean"]
            fields.remove("traffic")
            if frequency_set:
                fields.append("frequency")
            for f in fields:
                re = RatedStatisticsEntity()
                re.statistic_type = f
                value = data[f]
                re.actual_value.append(str(value))
                limits = self.__get_limits(specification, re.statistic_type)
                re.expected_value.append(str(limits))
                re.state = [self.__compare(value, limits)]
                r.rated_statistics_entity.append(re)
            re = RatedStatisticsEntity()
            re.statistic_type = "alive"
            re.expected_value = ["True"]
            re.actual_value = ["True"]
            re.state = [2]
            r.rated_statistics_entity.append(re)
            result.append(r)
        return result

    def __get_limits(self, specification, field, offset=0):
        if specification is None or field is None:
            return None
        key = "%s_%s_%s" % (specification.seuid, field, str(offset))
        if key in self.__limit_cache.keys():
            return self.__limit_cache[key]
        try:
            specs = specification.get(field)[1]
            if isinstance(specs, list) and len(specs) > 0 and isinstance(specs[0], list):
                if len(specs) > offset:
                    specs = specs[offset]
                else:
                    return None
            limits = specs[0:2]
            if len(specs) > 2 and specs[2][0].lower() == "r":
                if limits[1] > 1:
                    limits[1] -= 1
                m = limits[0]
                r = limits[1]
                limits[0] = m - m * r
                limits[1] = m + m * r
        except TypeError:
            limits = None
        except AttributeError:
            limits = None
        convert_to_duration = ["stamp_age_max", "stamp_age_mean", "stamp_age_stddev", "period_max", "period_mean", "period_stddev"]
        if limits is not None and field in convert_to_duration:
          limits = [rospy.Duration.from_sec(limits[0]), rospy.Duration.from_sec(limits[1])]
        self.__limit_cache[key] = limits
        return limits

    def __compare(self, value, reference):
        if not isinstance(reference, list) or len(reference) < 2 or \
                not isinstance(reference[0], (int, long, float, complex)) or \
                not isinstance(reference[1], (int, long, float, complex)):
            return 2
        reference.sort()
        # r = (reference[0] - reference[1]) / 2
        # m = reference[0] + r
        if reference[0] > value:
            state = 1
        elif reference[1] < value:
            state = 0
        else:
            state = 3
        return state

    def reload_specifications(self, msg=None):
        """
        Reloads all specifications loaded into the namespace /arni/specifications
        """
        self.__limit_cache = {}
        self.__specifications = {}
        self.__load_specifications()
        return []

    def __init__(self):
        """
        Initiates the SpecificationHandler kicking off the loading of available specifications.
        """
        self.__limit_cache = {}
        self.__specifications = {}
        self.reload_specifications()
