from abc import ABCMeta, abstractmethod


class ConstraintItem(object):

    """Abstract description of a Constraint.
    Can be a logical operation on constraints or an actual constraint.
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        super(ConstraintItem, self).__init__()

    @abstractmethod
    def evaluate_constraint(self, storage):
        """Evaluates if this constraint,
        given the available RatedStatisticStorage.
        Returns wheter the constraint is true or not."""
        pass
