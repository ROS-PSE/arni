import re
import rospy
import arni_msgs
from arni_msgs.msg import HostStatistics, NodeStatistics, MasterApiEntity
import rosgraph_msgs
from rosgraph_msgs.msg import TopicStatistics
from rospy.exceptions import ROSInitException


"""
This helper module contains useful functions and variables that
are needed throughout the whole arni package.
"""


class SEUID:

    """
    A class providing several actions for a seuid.
    """

    DELIMITER = "!"

    def __init__(self, identifier=None):
        """
        Creates a new instance for a given identifier or a Message object

        :param identifier: Either a seuid string or a Message object.
        :raises TypeError: If the given parameter is not an expected message type.
        :raises NameError: If the seuid from the parameter or the message is invalid.
        """
        if identifier is not None:
            if not isinstance(identifier, str):
                self.from_message(identifier)
            else:
                self.identifier = self.__deserialize(identifier)
            if not self.is_valid():
                raise NameError("Given seuid is not of valid form.")
            else:
                self.set_fields()
        else:
            self.identifier = identifier

    def set_fields(self):
        self.host = None
        """The host name if SEUID describes a host."""
        self.topic = None
        """The topic name if SEUID describes a topic or a connection."""
        self.subscriber = None
        """The subscribing node's name if SEUID describes a connection."""
        self.publisher = None
        """The publishing node's name if SEUID describes a connection."""
        self.node = None
        """The node name if SEUID describes a node."""
        parts = self.identifier.split(SEUID.DELIMITER)
        if parts[0] == "n":
            self.node = parts[1]
        elif parts[0] == "h":
            self.host = parts[1]
        elif parts[0] == "t":
            self.topic = parts[1]
        elif parts[0] == "c":
            self.subscriber = parts[1]
            self.topic = parts[2]
            self.publisher = parts[3]

    def get_seuid(self, field):
        """
        Returns the seuid for a specified component.

        :param field: Can be "host", "topic", "subscriber", "publisher" or "node".
            Only if the SEUID object contains this key
        :return: A seuid (str)
        :raises KeyError: When the current SEUID does not contain the given field since it has a different type
        :raises AttributeError: When the given parameter is none of the above.
        """
        fields = ("host", "topic", "subscriber", "node", "publisher")
        if field in fields and hasattr(self, field):
            if getattr(self, field) is None:
                raise KeyError(
                    "[SEUID] this seuid does not contain the field %s" % field)
                return None
            else:
                starts = "htnnn"
                return starts[fields.index(field)] + SEUID().DELIMITER + getattr(self, field)
        else:
            raise AttributeError("[SEUID] %s is not a public field" % field)

    def from_string(self, type, string, topic="", pub=""):
        if string is None:
            raise UserWarning("from_string: string was None")

        if type == "h":
            self.identifier = "h" + SEUID_DELIMITER + string
        elif type == "n":
            self.identifier = "n" + SEUID_DELIMITER + string
        elif type == "t":
            self.identifier = "t" + SEUID_DELIMITER + string
        elif type == "c":
            if topic is "" or pub is "":
                raise UserWarning("empty strings to from_string")
            self.identifier = "c" + SEUID_DELIMITER + string \
                      + SEUID_DELIMITER + topic \
                      + SEUID_DELIMITER + pub
        else:
            raise UserWarning()
        return self.identifier

    def get_field(self, type, seuid):
        if not self.is_valid(seuid):
            raise UserWarning("seuid was not valid")
        self.identifier = seuid
        self.set_fields()

        if type == "h":
            return self.host
        elif type == "n":
            return self.node
        elif type == "t":
            return self.topic
        elif type == "s":
            return self.subscriber
        elif type == "p":
            return self.publisher
        else:
            raise UserWarning("invalid type was chosen.")

    def from_message(self, msg, topic=False, sub=False):
        """
        Creates a SEUID from a Message object.

        :param msg: A Message object of type arni_msgs.msg.HostStatistics, arni_msgs.msg.NodeStatistics or
            rosgraph_msgs.msg.TopicStatistics
        :param topic: if true a topic seuid will be return instead of a connection seuid
        :param sub: if true "--sub" will be appended to the connection seuid
        :raises TypeError: If the parameter is none of the mentioned messagestypes.
        """
        if isinstance(msg, arni_msgs.msg.HostStatistics):
            self.identifier = "h" + SEUID_DELIMITER + msg.host
        elif isinstance(msg, arni_msgs.msg.NodeStatistics):
            self.identifier = "n" + SEUID_DELIMITER + msg.node
        elif isinstance(msg, rosgraph_msgs.msg.TopicStatistics):
            if topic:
                self.identifier = "t" + SEUID_DELIMITER + msg.topic
            else:
                if sub:
                    self.identifier = "c" + SEUID_DELIMITER + msg.node_sub \
                              + SEUID_DELIMITER + msg.topic \
                              + SEUID_DELIMITER + msg.node_pub + "--sub"
                else:
                    self.identifier = "c" + SEUID_DELIMITER + msg.node_sub \
                              + SEUID_DELIMITER + msg.topic \
                              + SEUID_DELIMITER + msg.node_pub
        else:
            raise TypeError("Cannot create SEUID from an object other than a HostStatistics,\
                NodeStatistics or TopicStatistics message.")
        return self.identifier


    def is_valid(self, identifier=None):
        """
        Determines whether the given parameter is a valid seuid.

        :param identifier: A string presenting a seuid to validate.
        :return: True, if the given parameter is a valid seuid, false if not.
        """
        check = None
        if identifier is not None:
            check = identifier
        else:
            check = self.identifier
        if check is None:
            return False
        p = re.compile(
            '^([nhtc])' + SEUID.DELIMITER + '([A-Za-z0-9/]('
            + SEUID.DELIMITER + '?[a-zA-Z0-9_/.])*)$')
        m = p.match(check)
        if m is None:
            return False
        arglen = len(m.group(2).strip().split(SEUID.DELIMITER))
        if m.group(1) == "c":
            valid = arglen == 3
        else:
            valid = arglen == 1
        if m.group(1) != "h":
            valid &= not m.group(2)[0].isdigit()
        return valid

    def __str__(self):
        return self.identifier

    def serialize(self):
        """
        Serializes the seuid for use in topics and parameters.

        :return: Escapes !, / and .
        """
        out = self.identifier
        out = out.replace("!", "___xm___")
        out = out.replace("/", "___sl___")
        out = out.replace(".", "___dt___")
        return out

    def __deserialize(self, serialized):
        """

        :rtype : str
        """
        out = serialized
        out = out.replace("___xm___", "!")
        out = out.replace("___sl___", "/")
        out = out.replace("___dt___", ".")
        return out


# : the delimiter the seuid uses
SEUID_DELIMITER = SEUID.DELIMITER


def is_seuid(seuid):
    """
    Determines whether the given parameter is a valid seuid.

    :param seuid: A string presenting a seuid to validate.
    :return: True, if the given parameter is a valid seuid, false if not.
    """
    return SEUID().is_valid(seuid)


def underscore_ip(ip):
    """
    Takes an ipv4 adress and replaces dots with underscores.
    """
    if (ip is None):
        return None

    return ip.replace(".", "_")


def older_than(time1, duration, time2=None):
    """
    Compares a given rospy.Time to now or a second given time and returns whether it exceeds the given duration.

    :param time1: A rospy.Time object.
    :param duration: A rospy.Duration object.
    :param time2:  Optionally a rospy.Time object if not to assume rospy.Time.now()
    :return: True, if the difference between the given times exceeds the duration and a valid time2 could be found.
    """
    try:
        if time2 is None:
            time2 = rospy.Time.now()
    except ROSInitException:
        return False
    return time2 > time1 and time2 - time1 > duration


def generate_seuids_from_master_api_data(master_api_data):
    """
    Will only return the seuids of subscribers and publisher, not the of services

    :param master_api_data:
    :return:
    """
    pubs, subs, srvs = master_api_data.pubs, master_api_data.subs, master_api_data.srvs

    conn = TopicStatistics()
    seuid_helper = SEUID()

    seuids = []
    found = False

    for pub in pubs:
        # add topic
        for sub in subs:
            if pub is sub:
                # found the subscribers and publishers of the same topic, now create n*m connections
                for real_pub in pub.content:
                    for real_sub in sub.content:
                        conn.node_sub = real_sub
                        conn.node_pub = real_pub
                        seuid_helper.from_message(conn)
                        seuids.append(seuid_helper.identifier)

    return seuids