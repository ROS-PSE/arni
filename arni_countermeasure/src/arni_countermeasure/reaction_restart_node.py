from reaction import *


class ReactionRestartNode(Reaction):

    """A reaction that is able to restart a node."""

    def __init__(self, node, autonomy_level):
        super(ReactionRestartNode, self).__init__(node, autonomy_level)

    def execute_reaction(self):
        pass
