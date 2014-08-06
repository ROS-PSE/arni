from reaction import *
import rospy


class ReactionPublishRosOutNode(Reaction):

    """A reaction that is able to publish a message on rosout."""

    def __init__(self, message, loglevel):
        super(ReactionPublishRosOutNode, self).__init__()

        #: The message to publish.
        #: :type: string
        self.message = message

        if loglevel == "info":
            log = rospy.loginfo
        elif loglevel == "debug":
            log = rospy.logdebug
        elif loglevel == "err":
            log = rospy.logerr
        elif loglevel == "warn":
            log = rospy.logwarn
        elif loglevel == "fatal":
            log = rospy.logfatal
        else:
            # loglevel does not make sense
            rospy.logwarn(
                "arni_countermeasure: a reaction wants to log on loglevel"
                + " '%s', but that loglevel does not exist." % loglevel
                + " Setting loglevel to info.")
            log = rospy.info

        #: The logging function to use.
        self.__log = log

    def execute_reaction(self):
        """Log the reaction message at a specific loglevel."""
        switch(loglevel)
        self.__log(self.message)
