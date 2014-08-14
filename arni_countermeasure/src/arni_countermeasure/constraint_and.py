from constraint_item import *


class ConstraintAnd(ConstraintItem):

    """An constraints consisting of other
    constraints logically and colligated.
    """

    def __init__(self, constraints):
        super(ConstraintAnd, self).__init__()

        self.__constraint_list = constraints

    def evaluate_constraint(self, storage):
        """Evaluate if all of the constraints inside are true.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage

        :return:    True if all constraints are true.
        """

        for constraint in self.__constraint_list:
            if not constraint.evaluate_constraint(storage):
                return False

        return True
