from rated_statistic_storage import *
from constraint import *
from reaction import *


class ConstraintHandler(object):

    """Manages all constraints, checks if they are true
    and executes appropriate reactions if neccessary.
    """

    def __init__(self, rated_statistic_storage):
        super(ConstraintHandler, self).__init__()

        #: Contains a list of all constraints.
        #: :type:   list of Constraints
        self.__constraint_list = list()

        #: Contains all incoming rated statistic.
        #: :type:   RatedStatisticStorage
        self.__rated_statistic_storage = rated_statistic_storage

        #: Only reactions with
        #: an autonomy_level <= reaction_autonomy_level get executed.
        self.__reaction_autonomy_level = None

    def add_constraint(self, constraint):
        """Add an constraint to the list of constraints.

        :param constraint:  The constraint to add.
        :type constraint:   Constraint
        """
        self.__constraint_list.append(constraint)

    def set_statistic_storage(self, rated_statistic_storage):
        """Set the statistic storage to use.
        Should only be needed on initialisation.

        :param rated_statistic_storage: The statistic storage to use.
        :type:  RatedStatisticStorage
        """
        self.__rated_statistic_storage = rated_statistic_storage

    def evaluate_constraints(self):
        """Evaluate every constraint."""

        # should be parallelisable
        for constraint in self.__constraint_list:
            constraint.evaluate_constraint()

    def execute_reactions(self):
        """Check if there are any new reactions to do
        and execute them.
        """

        # should be parallelisable
        for constraint in self.__constraint_list:

            if constraint.evaluation_result is True:
                # reactions need to be done
                for reaction in constraint.planned_reaction:
                    reaction.execute_reaction()

                # tell constraint to reset timers
                constraint.notify_of_execution()

    def _read_param_constraints(self):
        """Read all constraints from the parameter server."""
        # TODO: implement
        pass

    def _read_param_reaction_autonomy_level(self):
        """Read the reaction_autonomy_level from the parameter server."""
        # TODO: implement
        pass
