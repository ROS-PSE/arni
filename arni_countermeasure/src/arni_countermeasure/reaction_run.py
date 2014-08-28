from reaction import *
import rospy
from arni_msgs.srv import NodeReaction
from rospy import ServiceException


class ReactionRun(Reaction):

    """A Reaction which executes a command
    on the remote machine the specified node runs on.
    """

    def __init__(self, node, autonomy_level, command):
        super(ReactionRun, self).__init__(node, autonomy_level)

        self.__command = command

    def execute_reaction(self):
        host_formatted = helper.underscore_ip(self._host)
        service_name = "/execute_node_reaction/%s" % host_formatted
        try:
            if self._host is not None:
                execute = rospy.ServiceProxy(
                    service_name, NodeReaction)
                resp = execute(
                    self._node, "command", self.__command)
                rospy.logdebug(
                    "sending command '%s' to node %s returned: %s"
                    % (self.__command, self._node, resp.returnmessage))
            else:
                rospy.logdebug(
                    "could not run a command on the host of node %s, "
                    % self._node
                    + "because there is no information about where the node"
                    + " is run from. (possibly because"
                    + " the node never sent any statistics.)")
        except ServiceException:
            rospy.logdebug(
                "could not run a command on node %s: service %s not found"
                % (self._node, service_name))
