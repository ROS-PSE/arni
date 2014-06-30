from ConstraintItem import ConstraintItem


class ConstraintNot(ConstraintItem):

    """An constraints consisting of another constraint negated."""

    def __init__(self):
        super(ConstraintNot, self).__init__()
