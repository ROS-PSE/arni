class Constraint(object):

    """Manages all constraints,
    checks if they are true and executes
    appropriate reactions if neccessary.
    """

    def __init__(self):
        super(Constraint, self).__init__()

        self.__constraint

        self.true_since

        self.planned_reaction

        self.__min_reaction_interval

        self.__last_reaction

        self.__reaction_timeout

    def evaluate_constraint(self, storage):
        """Evaluates this constraint and sets the attributes
        according to the result of the evaluation.
        """
        pass
