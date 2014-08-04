from constraint_item import *


class ConstraintNot(ConstraintItem):

    """An constraints consisting of another constraint negated."""

    def __init__(self):
        super(ConstraintNot, self).__init__()
