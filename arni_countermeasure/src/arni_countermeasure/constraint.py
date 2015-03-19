from rated_statistic_storage import *
from constraint_item import *


class Constraint(object):

    """Contains the whole constraint with corresponding reactions.
    """

    def __init__(
            self, name, constraint_root, planned_reaction,
            min_reaction_interval, reaction_timeout):
        super(Constraint, self).__init__()

        #: The name of the constraint. Useful for debugging.
        #: :type:   string
        self.__name = name

        #: The root of the constraint tree.
        #: :type:   ConstraintItem
        self.__constraint_root = constraint_root

        #: Time since when this constraint is true.
        #: # Note if true_since is 0 it has not been true.
        #: :type:   rospy.Time
        self.true_since = rospy.Time(0)

        #: An list of reactions that should be executed if the constraint
        #: has been true longer than min_reaction_interval milliseconds.
        #: :type:   list of Reactions
        self.planned_reaction = planned_reaction

        #: The minimum time needed that the constraint needs to be true to
        #: execute the planned reactions.
        #: :type:   rospy.Duration
        self.__min_reaction_interval = min_reaction_interval

        #: The time this reaction has been executed for the last time.
        #: 0 if it has never been executed.
        #: :type:   rospy.Time
        self.__last_reaction = rospy.Time(0)

        #: Minimum durotation needed before an reaction can happen again.
        #: :type: rospy.Duration
        self.__reaction_timeout = reaction_timeout

        #: True if the current state of the constraint says that
        #: an execution of the reactions is necessary.
        self.evaluation_result = False

    def evaluate_constraint(self, storage):
        """Evaluates this constraint and sets the attributes
        according to the result of the evaluation.

        :param storage: The storage where the incoming statistics are saved.
        :type storage:  RatedStatisticStorage
        """

        evaluation = self.__constraint_root.evaluate_constraint(storage)

        #If the constraint is false only true_since has to be reset.
        if not evaluation:
            self.true_since = rospy.Time(0)
            self.evaluation_result = False
        else:
            if self.true_since == rospy.Time(0):
                self.true_since = rospy.Time.now()

            if (
                (rospy.Time.now() - self.true_since
                    >= self.__min_reaction_interval)
                and
                (rospy.Time.now() - self.__last_reaction
                    >= self.__reaction_timeout)):
                self.evaluation_result = True
        return

    def notify_of_execution(self):
        """ Tells this constraint that its reactions have just been executed.

        # Note: its important to notify this constraint so it knows how long
        the last execution was ago and does not execute too often.
        """
        self.evaluation_result = False
        self.__last_reaction = rospy.Time.now()

    def __str__(self):
        return self.__name
