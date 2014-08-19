from reaction import *


class ReactionStopNode(Reaction):

    """A reaction that is able to stop a node."""

    def __init__(self, node, autonomy_level):
        super(ReactionStopNode, self).__init__(node, autonomy_level)

    def execute_reaction(self):
        if self._host is not None:
            execute = rospy.ServiceProxy(
                "execute_node_reaction", NodeReaction)
            resp = execute(self._host, self._node, "stop", '')
            rospy.logdebug(
                "Stopping node %s returned: %s"
                % (self._node, resp.returnmessage))
        else:
            rospy.logdebug(
                "Could not stop node %s, " % self._node
                + "because there is no information about where the node"
                + " is run from."
                + " (possibly because the node never sent any statistics.)")
