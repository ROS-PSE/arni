class Reaction(object):

    """Abstract Reaction to an Constraint.
    The Reaction is to be executed
    on the corresponding host of the given node.
    """

    def __init__(self, node):
        super(Reaction, self).__init__()

        #: the node to run this reaction on.
        self.node = node
