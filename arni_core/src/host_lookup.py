from singleton import *
from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity
from helper import SEUID_DELIMITER


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
        return self.__node_dict.get(node)

    def add_node(self, node, host):
        self.__node_dict[node] = host

    def get_node_list(self, host):
        node_list = list()
        for node, host_in_dict in self.__node_dict.items():
            if host_in_dict == host:
                node_list.append(node)

        return node_list

    def remove_node(self, node):
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
        if message.seuid.startswith("n"):
            host = message.host
            node = message.seuid.split(SEUID_DELIMITER)[1]
            self.add_node(node, host)
