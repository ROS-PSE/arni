from reaction import *


class ReactionStopNode(Reaction):

    """A reaction that is able to stop a node."""

    def __init__(self, node, autonomy_level):
        super(ReactionStopNode, self).__init__(node, autonomy_level)

    def execute_reaction(self):
        pass
