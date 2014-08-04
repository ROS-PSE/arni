from constraint_item import *


class ConstraintNot(ConstraintItem):

    """An constraints consisting of another constraint negated."""

    def __init__(self, constraint):
        super(ConstraintNot, self).__init__()

        #: the constraint to be negated
        #: :type:   ConstraintItem
        self.__constraint = constraint

    def evaluate_constraint(self, storage):
        """Evaluate if the constraint inside this constraint is false or not.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage

        :return:    True iff the constraint inside is False.
        """
        return not self.__constraint.evaluate_constraint(storage)
