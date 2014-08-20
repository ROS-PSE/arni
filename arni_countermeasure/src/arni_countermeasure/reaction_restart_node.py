from reaction import *
import rospy
from arni_msgs.srv import NodeReaction
from rospy import ServiceException


class ReactionRestartNode(Reaction):

    """A reaction that is able to restart a node."""

    def __init__(self, node, autonomy_level):
        super(ReactionRestartNode, self).__init__(node, autonomy_level)

    def execute_reaction(self):
        service_name = "/execute_node_reaction/%s" % self._host
        try:
            if self._host is not None:
                execute = rospy.ServiceProxy(
                    service_name, NodeReaction)
                resp = execute(self._node, "restart", '')
                rospy.logdebug(
                    "restarting node %s returned: %s"
                    % (self._node, resp.returnmessage))
            else:
                rospy.logdebug(
                    "could not restart node %s, " % self._node
                    + "because there is no information about where the node"
                    + " is run from. (possibly because"
                    + " the node never sent any statistics.)")
        except ServiceException:
            rospy.logdebug(
                "could not restart node %s, service %s not found"
                % (self._node, service_name))
