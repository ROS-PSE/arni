from reaction import *


class ReactionRun(Reaction):

    """A Reaction which executes a command
    on the remote machine the specified node runs on.
    """

    def __init__(self, node, autonomy_level, command):
        super(ReactionRun, self).__init__(node, autonomy_level)

        self.__command = command

    def execute_reaction(self):
        #todo: parallelise the execution,
        # execute_node_reaction has to be a service of the hoststatisticnode
        if self._host is not None:
            execute = rospy.ServiceProxy(
                "execute_node_reaction", NodeReaction)
            resp = execute(self._host, self._node, "command", self.__command)
            rospy.logdebug(
                "Sending command '%s' to node %s returned: %s"
                % (self.__command, self._node, resp.returnmessage))
        else:
            rospy.logdebug(
                "Could not run a command on the host of node %s, " % self._node
                + "because there is no information about where the node"
                + " is run from."
                + " (possibly because the node never sent any statistics.)")
