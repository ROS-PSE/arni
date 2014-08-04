from constraint_item import *


class ConstraintOr(ConstraintItem):

    """An constraints consisting of
    other constraints logically or colligated.
    """

    def __init__(self, constraint_a, constraint_b):
        super(ConstraintOr, self).__init__()

        self.__constraint_a = constraint_a

        self.__constraint_b = constraint_b

    def evaluate_constraint(self, storage):
        """Evaluate if at least one of the constraints inside is true.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage

        :return:    True if one of the constraints is true.
        """
        return (
            self.__constraint_a.evaluate_constraint(storage) or
            self.__constraint_b.evaluate_constraint(storage))
