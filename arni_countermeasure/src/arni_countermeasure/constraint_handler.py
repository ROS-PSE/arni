from rated_statistic_storage import *
from constraint import *
from reaction import *

from constraint_item import *
from constraint_and import *
from constraint_or import *
from constraint_not import *
from constraint_leaf import *

from reaction import *
from reaction_publish_rosout_node import *
from reaction_restart_node import *
from reaction_run import *
from reaction_stop_node import *

import rospy


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
        glob_level = self.__reaction_autonomy_level
        # should be parallelisable
        for constraint in self.__constraint_list:

            if constraint.evaluation_result is True:
                # reactions need to be done
                for reaction in constraint.planned_reaction:
                    if reaction.autonomy_level <= glob_level:
                        reaction.execute_reaction()

                # tell constraint to reset timers
                constraint.notify_of_execution()

    def _read_param_constraints(self):
        """Read all constraints from the parameter server and save them.
        """
        p_constraint = rospy.get_param("/arni_countermeasure/constraints")
        #print constraints
        for name in p_constraint:
            const_dict = p_constraint[name]

            root = self.__create_constraint_tree(
                const_dict['constraint'], name)

            min_reaction_interval, reaction_timeout = (
                self.__parse_interval_and_timeout(const_dict))

            reaction_list = self.__parse_reaction_list(const_dict)

            # is it a valid root?
            if root is not None:
                constraint = Constraint(
                    name, root, reaction_list,
                    min_reaction_interval, reaction_timeout)
                self.__constraint_list.append(constraint)

    def __parse_reaction_list(self, const_dict):
        """Parse the dict to a list of reactions.

        :param const_dict:  The dict from the parameter server holding
                            the reactions of a constraint.
        :type:  dict

        :return:    The parsed list of reactions.
        :type:  list of Reactions
        """

        # TODO: handling missing keys
        p_reaction_list = const_dict['reactions']
        reaction_list = list()

        for r_name in p_reaction_list:
            p_reaction = p_reaction_list[r_name]

            action = p_reaction['action']
            node = p_reaction['node']
            autonomy_level = p_reaction['autonomy_level']

            # find out what kind of reaction it is
            if action == 'publish':
                message = p_reaction['message']
                loglevel = p_reaction['loglevel']

                react_publish = ReactionPublishRosOutNode(
                    node, autonomy_level, message, loglevel)

                reaction_list.append(react_publish)
            elif action == 'stop':
                react_stop = ReactionStopNode(node, autonomy_level)
                reaction_list.append(react_stop)
            elif action == 'restart':
                react_restart = ReactionRestartNode(node, autonomy_level)
                reaction_list.append(react_restart)
            elif action == 'run':
                command = p_reaction['command']
                react_run = ReactionRun(node, autonomy_level, command)
                reaction_list.append(react_run)
            else:
                rospy.logdebug(
                    "arni_countermeasure: The action '%s' " % action
                    + "found on the parameter server is not recognised.")

        return reaction_list

    def __parse_interval_and_timeout(self, const_dict):
        """Parse min_reaction_interval
        and reaction_timeout out of the dictionary.

        If the interval and timeout is not specified in the dict
        default values are assigned.

        :param const_dict:  The dict from the parameter server
                            holding the interval and timeout.
        :type:  dict

        :return:    the min_reaction_interval and reaction_timeout
        :rtype: tuple (rospy.Duration, rospy.Duration)
        """

        # default values
        # TODO: create variables for default
        min_reaction_interval = rospy.Duration(10)
        reaction_timeout = rospy.Duration(10)

        # check if interval and timeout have interger values
        try:
            if hasattr(const_dict, 'min_reaction_interval'):
                min_reaction_interval = rospy.Duration(
                    const_dict[min_reaction_interval])
        except ValueError:
            rospy.logdebug(
                "arni_countermeasure: "
                + "constraint %s - min_reaction_interval '%s'"
                % (name, const_dict[min_reaction_interval])
                + " is an invalid value. (Only integers allowed.)")

        try:
            if hasattr(const_dict, 'reaction_timeout'):
                reaction_timeout = rospy.Duration(
                    const_dict[reaction_timeout])
        except ValueError:
            rospy.logdebug(
                "arni_countermeasure: "
                + "constraint %s - reaction_timeout '%s'"
                % (name, const_dict[reaction_timeout])
                + " is an invalid value. (Only integers allowed.)")

        return (min_reaction_interval, reaction_timeout)

    def __create_constraint_tree(self, constraint_dict, name):
        """Create a constraint tree from a dictionary.
        (the dict is usually from the parameter server.)

        Returns None if the tree is not valid.

        :return:   The constraint item containing the complete tree.
        :rtype: ConstraintItem
        """

        root = None
        # there can be only one root ;-)
        if len(constraint_dict) == 1:
            root = self.__traverse_dict(
                constraint_dict, constraint_dict.keys()[0])
        elif len(constraint_dict) == 0:
            rospy.logdebug(
                "arni_countermeasure: Constraint '%s'" % name
                + " has no constraint items. ")
        else:
            rospy.logdebug(
                "arni_countermeasure: Constraint '%s' is starting" % name
                + "with more than one constraint item."
                + " Use 'and'/'or' as first item to add multiple items.")

        return root

    def __traverse_dict(self, c_dict, item_type):
        """Traverse down the dictionary recursively.

        Creates a constraint item tree from the dictionary.


        :param c_dict:  The dictionary to traverse.
        :type:  dict

        :param item_type:   The kind of item to create.
                            if the type is not 'not','and','or' the
                            item_type stands for the seuid of the leaf.
        :type:  string

        """
        if item_type in ('not', 'and', 'or'):
            constraint_item_list = list()

            # go through all items in the dict
            for sub_item in c_dict[item_type]:
                constraint_item = self.__traverse_dict(
                    c_dict[item_type], sub_item)

                # is this item a list? ('not' returns a list)
                if hasattr(constraint_item, "__iter__"):
                    constraint_item_list.extend(constraint_item)
                else:
                    constraint_item_list.append(constraint_item)

            # create and return constraint items
            return self.__create_constraint_item(
                item_type, constraint_item_list)

        else:
            # lets create a leaf (or a list of leafes)
            leaf_list = list()
            # itemtype is not and,or..? must be a leaf
            seuid = item_type

            for statistic_type in c_dict[item_type]:
                outcome = c_dict[item_type][statistic_type]
                leaf_list.append(
                    ConstraintLeaf(seuid, statistic_type, outcome))

            return leaf_list

    def __create_constraint_item(self, item_type, constraint_list):
        """Creates a constraint item / a list of constraint items.

        Creates a new constraint of type item_type and puts the constraints
        of constraint_list in it.

        If the item_type is 'not' there is a not-constraint createt for every
        constraint in the list.

        :param item_type:   The type of the new constraint item(s).
        :type:  string ('or', 'and', 'not' allowed. no validation)

        :param constraint_list:   Constraints to put inside the new constraint.
        :type:  list of ConstraintItems
        """
        if item_type == 'or':
            return ConstraintOr(constraint_list)
        elif item_type == 'and':
            return ConstraintAnd(constraint_list)
        elif item_type == 'not':
            ret_list = list()
            for constraint_item in constraint_list:
                ret_list.append(ConstraintNot(constraint_item))
            return ret_list

    def _read_param_reaction_autonomy_level(self):
        """Read and save the reaction_autonomy_level from the parameter server.

        Default value is 100.
        """
        level = rospy.get_param(
            "/arni_countermeasure/reaction_autonomy_level", 100)
        self.__reaction_autonomy_level = level
