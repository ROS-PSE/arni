from constraint_item import *


class ConstraintAnd(ConstraintItem):

    """An constraints consisting of other
    constraints logically and colligated.
    """

    def __init__(self, constraint_a, constraint_b):
        super(ConstraintAnd, self).__init__()

        self.__constraint_a = constraint_a

        self.__constraint_b = constraint_b

    def evaluate_constraint(self, storage):
        """Evaluate if both of the constraints inside are true.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage

        :return:    True if both constraints are true.
        """
        return (
            self.__constraint_a.evaluate_constraint(storage) and
            self.__constraint_b.evaluate_constraint(storage))
