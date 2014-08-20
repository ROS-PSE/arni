import re
import arni_msgs
from arni_msgs.msg import HostStatistics, NodeStatistics
import rosgraph_msgs
from rosgraph_msgs.msg import TopicStatistics

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
                self.__set_fields()
        else:
            self.identifier = identifier

    def __set_fields(self):
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

    def from_message(self, msg):
        """
        Creates a SEUID from a Message object.

        :param msg: A Message object of type arni_msgs.msg.HostStatistics, arni_msgs.msg.NodeStatistics or
            rosgraph_msgs.msg.TopicStatistics
        :raises TypeError: If the parameter is none of the mentioned messagestypes.
        """
        if isinstance(msg, arni_msgs.msg.HostStatistics):
            self.identifier = "h" + SEUID_DELIMITER + msg.host
        elif isinstance(msg, arni_msgs.msg.NodeStatistics):
            self.identifier = "n" + SEUID_DELIMITER + msg.node
        elif isinstance(msg, rosgraph_msgs.msg.TopicStatistics):
            self.identifier = "c" + SEUID_DELIMITER + msg.node_sub \
                              + SEUID_DELIMITER + msg.topic \
                              + SEUID_DELIMITER + msg.node_pub
        else:
            raise TypeError("Cannot create SEUID from an object other than a HostStatistics,\
                NodeStatistics or TopicStatistics message.")

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
        p = re.compile('^([nhtc])' + SEUID.DELIMITER + '([A-Za-z0-9/](' + SEUID.DELIMITER + '?[a-zA-Z0-9_/])*)')
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