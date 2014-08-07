from constraint_item import *


class ConstraintOr(ConstraintItem):

    """An constraints consisting of
    other constraints logically or colligated.
    """

    def __init__(self, constraints):
        super(ConstraintOr, self).__init__()

        self.__constraint_list = constraints

    def evaluate_constraint(self, storage):
        """Evaluate if at least one of the constraints inside is true.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage

        :return:    True if one of the constraints is true.
        """

        for constraint in self.__constraint_list:
            if constraint.evaluate_constraint(storage):
                return True

        return False
