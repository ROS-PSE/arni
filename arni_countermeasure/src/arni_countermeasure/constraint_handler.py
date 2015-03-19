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

from outcome import *
import arni_core

import rospy
import helper
import thread


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

        self._read_param_reaction_autonomy_level()

        self._read_param_constraints()

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
            constraint.evaluate_constraint(
                self.__rated_statistic_storage)

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
                        # run reactions parallel since
                        # they could take some time
                        rospy.loginfo('Running countermeasure %s' % str(constraint))
                        thread.start_new_thread(reaction.execute_reaction, ())
                        #reaction.execute_reaction()

                # tell constraint to reset timers
                constraint.notify_of_execution()

    def _read_param_constraints(self):
        """Read all constraints from the parameter server and save them.
        """
        p_constraint = rospy.get_param(
            helper.ARNI_CTM_NS + "constraints", dict())
        #print constraints
        for name in p_constraint:
            const_dict = p_constraint[name]

            # no constraint inside?
            if not 'constraint' in const_dict:
                rospy.logwarn(
                    "Constraint %s is missing a constraint. Skipping."
                    % name)
                break

            root = ConstraintHandler._create_constraint_tree(
                const_dict['constraint'], name)

            min_reaction_interval, reaction_timeout = (
                ConstraintHandler._parse_interval_and_timeout(const_dict))

            reaction_list = ConstraintHandler._parse_reaction_list(
                const_dict, name)

            # is it a valid root?
            if root is not None:
                constraint = Constraint(
                    name, root, reaction_list,
                    min_reaction_interval, reaction_timeout)
                self.__constraint_list.append(constraint)
        rospy.loginfo('Loaded %d countermeasures' % len(self.__constraint_list))

    def _read_param_reaction_autonomy_level(self):
        """Read and save the reaction_autonomy_level from the parameter server.
        """
        self.__reaction_autonomy_level = helper.get_param_num(
            helper.ARNI_CTM_CFG_NS + "reaction_autonomy_level")

    @classmethod
    def _parse_reaction_list(ConstraintHandler, const_dict, name):
        """Parse the dict to a list of reactions.

        :param const_dict:  The dict from the parameter server holding
                            the reactions of a constraint.
        :type:  dict

        :param name:    The name of the constraint. Can be None.
                        Usefull for debugging.
        :type:  string

        :return:    The parsed list of reactions.
        :type:  list of Reactions
        """

        p_reaction_list = const_dict.get('reactions', {})

        reaction_list = list()

        for r_name in p_reaction_list:
            p_reaction = p_reaction_list[r_name]

            action = p_reaction.get('action', 'no action set')

            if 'autonomy_level' in p_reaction:
                autonomy_level = p_reaction['autonomy_level']
            else:
                # set to 0 so it always gets executed
                autonomy_level = 0

            # find out what kind of reaction it is
            if action == 'publish':
                message = p_reaction['message']
                loglevel = p_reaction.get('loglevel', 'loginfo')

                react_publish = ReactionPublishRosOutNode(
                    autonomy_level, message, loglevel)

                reaction_list.append(react_publish)
            else:
                # not publishing a message? we need a node to execute on!
                if 'node' in p_reaction:
                    node = p_reaction['node']
                else:
                    rospy.logwarn(
                        "There is no Node defined in reaction"
                        + " %s of constraint %s. Skipping Reaction."
                        % (r_name, name))
                    break

                if action == 'stop':
                    react_stop = ReactionStopNode(node, autonomy_level)
                    reaction_list.append(react_stop)
                elif action == 'restart':
                    react_restart = ReactionRestartNode(node, autonomy_level)
                    reaction_list.append(react_restart)
                elif action == 'run':
                    if not 'command' in p_reaction:
                        rospy.logwarn(
                            "There is no Command defined in run-reaction"
                            + " %s of constraint %s. Skipping reaction."
                            % (r_name, name))
                        break

                    command = p_reaction['command']
                    react_run = ReactionRun(node, autonomy_level, command)
                    reaction_list.append(react_run)
                elif action == 'no action set':
                    rospy.logwarn(
                        "There is no action for reaction %s in constraint %s."
                        % (r_name, name)
                        + " Ignoring this reaction.")
                else:
                    rospy.logwarn(
                        "The action '%s' in the reaction %s of constraint %s"
                        % (action, r_name, name)
                        + " is not recognised. Ignoring this reaction.")

        return reaction_list

    @classmethod
    def _parse_interval_and_timeout(ConstraintHandler, const_dict):
            """Parse min_reaction_interval
            and reaction_timeout out of the dictionary.

            Both are attributes of a constraint.

            If the interval and timeout is not specified in the dict
            default values are assigned.

            :param const_dict:  The dict from the parameter server
                                holding the interval and timeout.
            :type:  dict

            :return:    the min_reaction_interval and reaction_timeout
            :rtype: tuple (rospy.Duration, rospy.Duration)
            """

            # default values
            min_reaction_interval = helper.get_param_duration(
                helper.ARNI_CTM_CFG_NS + "default/min_reaction_interval")
            reaction_timeout = helper.get_param_duration(
                helper.ARNI_CTM_CFG_NS + "default/reaction_timeout")

            # check if interval and timeout have interger values
            try:
                if 'min_reaction_interval' in const_dict:
                    min_reaction_interval = rospy.Duration(
                        const_dict['min_reaction_interval'])
            except ValueError:
                rospy.logwarn(
                    "min_reaction_interval '%s'"
                    % (const_dict['min_reaction_interval'])
                    + " is of invalid value. (Only numbers allowed.)")

            try:
                if 'reaction_timeout' in const_dict:
                    reaction_timeout = rospy.Duration(
                        const_dict['reaction_timeout'])
            except ValueError:
                rospy.logwarn(
                    "reaction_timeout '%s'"
                    % (const_dict['reaction_timeout'])
                    + " is of invalid value. (Only numbers allowed.)")

            return (min_reaction_interval, reaction_timeout)

    @classmethod
    def _create_constraint_item(ConstraintHandler, item_type, constraint_list):
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

    @classmethod
    def _traverse_dict(ConstraintHandler, c_dict, item_type):
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
                constraint_item = ConstraintHandler._traverse_dict(
                    c_dict[item_type], sub_item)

                # is this item a list? ('not' returns a list)
                if hasattr(constraint_item, "__iter__"):
                    constraint_item_list.extend(constraint_item)
                else:
                    constraint_item_list.append(constraint_item)

            # create and return constraint items
            return ConstraintHandler._create_constraint_item(
                item_type, constraint_item_list)

        else:
            # lets create a leaf (or a list of leafes)
            leaf_list = list()
            # itemtype is not and,or..? must be a leaf
            seuid = item_type

            # better check if its a seuid
            if not arni_core.helper.is_seuid(seuid):
                rospy.logwarn(
                    "There is a wrongly formatted seuid '%s'."
                    % seuid
                    + " Found while parsing a constraint.")
                return None

            for statistic_type in c_dict[item_type]:
                outcome_unformatted = c_dict[item_type][statistic_type]

                if isinstance(outcome_unformatted, basestring):
                    outcome = Outcome.from_str(outcome_unformatted)
                    leaf_list.append(
                        ConstraintLeaf(seuid, statistic_type, outcome))
                else:
                    rospy.logwarn(
                        "The outcome '%s' in an constraint is not a"
                        % outcome_unformatted
                        + " valid type.")

            # only one item? strip!
            if(len(leaf_list) == 1):
                leaf_list = leaf_list[0]

            return leaf_list

    @classmethod
    def _create_constraint_tree(ConstraintHandler, constraint_dict, name):
        """Create a constraint tree from a dictionary.
        (the dict is usually from the parameter server.)

        The dictionary needs to be in a list without any other items.

        Returns None if the tree is not valid.

        :return:   The constraint item containing the complete tree.
        :rtype: ConstraintItem
        """

        root = None

        # remove the list
        if len(constraint_dict) == 1 and type(constraint_dict) is list:
            constraint_dict = constraint_dict[0]
        else:
            rospy.logwarn(
                "Constraint '%s' needs to contain exactly one root" % name
                + " item with the tag '-' to mark it as a listitem")
            return root
        # there can be only one root ;-)
        if len(constraint_dict) == 1:
            possible_root = ConstraintHandler._traverse_dict(
                constraint_dict, constraint_dict.keys()[0])
            # check if it is a list
            if isinstance(possible_root, list):
                if len(possible_root) == 1:
                    root = possible_root[0]
                else:
                    rospy.logwarn(
                        "Parsing the constraint %s was not successful." % name
                        + " The root of the constraint should be only"
                        + " one element, but is a list of  %d elements."
                        % len(possible_root))
            else:
                root = possible_root

        elif len(constraint_dict) == 0:
            rospy.logdebug(
                "Constraint '%s'" % name
                + " has no constraint items. ")
        else:
            rospy.logwarn(
                "Constraint '%s' is starting" % name
                + "with more than one constraint item."
                + " Use 'and'/'or' as first item to add multiple items.")

        return root
