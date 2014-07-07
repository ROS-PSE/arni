from constraint_item import *


class ConstraintAnd(ConstraintItem):

    """An constraints consisting of other
    constraints logically and colligated.
    """

    def __init__(self):
        super(ConstraintAnd, self).__init__()
