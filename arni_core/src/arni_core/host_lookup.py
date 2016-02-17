from singleton import *
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
from helper import SEUID_DELIMITER
import helper
import re

import rosnode
import rosgraph


class HostLookup(object):

    """Contains a dictionary of all nodes and the hosts they run on.
    Is a singleton.
    """

    __metaclass__ = Singleton

    def __init__(self):
        super(HostLookup, self).__init__()

        #: Contains all nodes which are on a host who has HostStatisticsNode
        #: running.
        self.__node_dict = dict()

    def get_host(self, node):
        """Return the host the node runs on.

        :param node:    name of the node
        :type:  string

        :return:    the host the node runs on. None if the host is unknown.
        :rtype: string
        """

        # do an call to master if the host is not known yet.
        if not node in self.__node_dict:
            master = rosgraph.Master("")
            node_api = rosnode.get_api_uri(master, node, skip_cache=True)
            # did we get a valid ip?
            if node_api is not None and node_api.startswith("http://"):
                ip = re.search("http://(.*):", str(node_api)).group(1)
                self.__node_dict[node] = ip
            else:
                return None

        return self.__node_dict.get(node)

    def add_node(self, node, host):
        """Add a node - host tuple to the dictionary.

        Overwrites already existing node - host tuple if they have
        the same node name.

        :param node:    name of the node
        :type:  string

        :param host:    ip of the host
        :type:  string
        """
        self.__node_dict[node] = host

    def get_node_list(self, host):
        """Return all nodes of a specific host.

        :param host:    ip of the host
        :type:  string

        :return:    all nodes that are on the specified host.
        :rtype: list
        """
        node_list = list()
        for node, host_in_dict in self.__node_dict.items():
            if host_in_dict == host:
                node_list.append(node)

        return node_list

    def clear(self):
        """Remove all elements from the dict."""
        self.__node_dict = dict()

    def remove_node(self, node):
        """Remove a node - host tuple.

        :param node:    name of the node
        :type:  string
        """
        del self.__node_dict[node]

    @staticmethod
    def get_instance():
        """ Returns the instance of HostLookup.

        # Note: This function is not necessary. It's also possible to
                just call HostLookup() to get a instance of this singleton.

        :return:    The instance of HostLookup.
        """
        return HostLookup()

    def callback_rated(self, message):
        """ Callback for rated statistics. Adds unseen nodes to the dictionary.

        :param message: The message to possibly extract the node - host
                        connection from
        :type message:  RatedStatistics
        """

        # is it about a node?
        if message.seuid.startswith("n") and helper.is_seuid(message.seuid):
            host = message.host
            node = message.seuid.split(SEUID_DELIMITER)[1]
            self.add_node(node, host)
