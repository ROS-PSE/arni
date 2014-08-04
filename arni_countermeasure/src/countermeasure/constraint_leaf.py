from constraint_item import *
from rated_statistic_storage import *


class ConstraintLeaf(ConstraintItem):

    """Contains an actual statistic datapoint
    and the seuid the datapoint belongs to.
    """

    def __init__(self, seuid, statistic_type, outcome):
        super(ConstraintLeaf, self).__init__()

        #: The seuid of the entity.
        #: :type:   string
        self.__seuid = seuid

        #: The type of statistic
        #: :type:   string
        self.__statistic_type = statistic_type

        #: The outcome this constraint needs to be true.
        #:  :type:  Outcome
        self.__outcome = outcome

    def evaluate_constraint(self, storage):
        """Evaluate if this constraint is true or not.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage

        :return:    Return if the wanted outcome is the same as the outcome
                    in the storage.
                    Note: having OUT_OF_BOUNDS as constraint and getting
                    HIGH or LOW also evaluates to true.
        """
        real_outcome = storage.get_outcome(
            self.__seuid, self.__statistic_type)

        if real_outcome == self.__outcome:
            return True
        elif (
            self.__outcome == Outcome.OUT_OF_BOUNDS and
            (real_outcome == Outcome.HIGH or
                real_outcome == Outcome.LOW)):
            return True
        return False
