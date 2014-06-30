class ConstraintHandler(object):

    """Manages all constraints, checks if they are true
    and executes appropriate reactions if neccessary.
    """

    def __init__(self):
        super(ConstraintHandler, self).__init__()

        #: Contains a list of all constraints.
        self.__constraint_list

        #: Contains all incoming rated statistic.
        self.__rated_statistic_storage

        #: Only reactions with
        #: an autonomy_level <= reaction_autonomy_level get executed.
        self.__reaction_autonomy_level

    def add_constraint(self, constraint):
        """Add an constraint to this list."""
        pass

    def set_statistic_storage(self, rated_statistic_storage):
        """Set the Statistic to use.
        Should only be needed on initialisation.
        """
        pass

    def evaluate_constraints(self):
        """Evaluate every constraint."""
        pass

    def execute_reactions(self):
        """Check if there are any new reactions to do
        and execute them.
        """
        pass

    def _react_to_constraint(self, constraint):
        """Execute an single Reaction
        and update the attributes of the Constraint.
        """
        pass

    def _read_param_constraints(self):
        """Read all constraints from the parameter server."""
        pass

    def _read_param_reaction_autonomy_level(self):
        """Read the reaction_autonomy_level from the parameter server."""
        pass
