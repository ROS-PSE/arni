from reaction import *


class ReactionRun(Reaction):

    """A Reaction which executes a command
    on the remote machine the specified node runs on.
    """

    def __init__(self, node, autonomy_level, command):
        super(ReactionRun, self).__init__(node, autonomy_level)

    def execute_reaction(self):
        pass
