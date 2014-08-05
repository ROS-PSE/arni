from reaction import *


class ReactionRun(Reaction):

    """A Reaction which executes a command
    on the remote machine the specified node runs on.
    """

    def __init__(self):
        super(ReactionRun, self).__init__()
