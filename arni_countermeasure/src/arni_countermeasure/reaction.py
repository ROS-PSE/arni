from abc import ABCMeta, abstractmethod
from arni_core.host_lookup import *


class Reaction(object):

    """Abstract Reaction to an Constraint.
    The Reaction is to be executed
    on the corresponding host of the given node.
    """

    __metaclass__ = ABCMeta

    def __init__(self, node, autonomy_level):
        super(Reaction, self).__init__()

        #: the node to run this reaction on.
        self._node = node

        #: the host on which the node runs on.
        self.__host = None

        #: the autonomy level
        self.autonomy_level = autonomy_level

    @property
    def _host(self):
        return HostLookup().get_host(self._node)

    @abstractmethod
    def execute_reaction(self):
        pass
