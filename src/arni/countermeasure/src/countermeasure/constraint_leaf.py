from constraint_item import *


class ConstraintLeaf(ConstraintItem):

    """Contains an actual statistic datapoint
    and the seuid the datapoint belongs to.
    """

    def __init__(self):
        super(ConstraintLeaf, self).__init__()
