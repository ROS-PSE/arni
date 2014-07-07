from constraint_item import *


class ConstraintOr(ConstraintItem):

    """An constraints consisting of
    other constraints logically or colligated.
    """

    def __init__(self):
        super(ConstraintOr, self).__init__()
