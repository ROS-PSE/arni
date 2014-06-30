class Reaction(object):

    """Abstract Reaction to an Constraint.
    The Reaction is to be executed
    on the corresponding host of the given node.
    """

    def __init__(self, arg):
        super(Reaction, self).__init__()
        self.arg = arg
