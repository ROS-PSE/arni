from reaction import *
import rospy
from arni_msgs.srv import NodeReaction
from rospy import ServiceException


class ReactionStopNode(Reaction):

    """A reaction that is able to stop a node."""

    def __init__(self, node, autonomy_level):
        super(ReactionStopNode, self).__init__(node, autonomy_level)

    def execute_reaction(self):
        host_formatted = helper.underscore_ip(self._host)
        service_name = "/execute_node_reaction/%s" % host_formatted
        try:
            if self._host is not None:
                execute = rospy.ServiceProxy(
                    service_name, NodeReaction)
                resp = execute(self._node, "stop", '')
                rospy.logdebug(
                    "stopping node %s returned: %s"
                    % (self._node, resp.returnmessage))
            else:
                rospy.logdebug(
                    "could not stop node %s, " % self._node
                    + "because there is no information about where the node"
                    + " is run from. (possibly because"
                    + " the node never sent any statistics.)")
        except ServiceException:
            rospy.logdebug(
                "could not stop node %s, service %s not found"
                % (self._node, service_name))
