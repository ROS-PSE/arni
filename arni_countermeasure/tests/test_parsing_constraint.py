#!/usr/bin/env python
import unittest
from arni_countermeasure.countermeasure_node import *
from arni_countermeasure.constraint_handler import *
from arni_countermeasure.rated_statistic_storage import *
from arni_countermeasure import helper
from arni_countermeasure.constraint_and import *
from arni_countermeasure.constraint_or import *
from arni_countermeasure.constraint_not import *

import traceback
import rospy

PKG = "arni_countermeasure"


class TestParsingOfConstraints(unittest.TestCase):

    def test_missing_reactions(self):
        """Tests for no error if the field reactions is missing."""
        params = {
            'countermeasure': {
                'constraints': {
                    'john': {
                        'constraint': {}}}}}

        rospy.set_param("/arni", params)
        try:
            ConstraintHandler(None)
        except Exception:
            self.fail(
                "missing the param reactions caused the exception: %s"
                % traceback.format_exc())

    def test_empty_reaction(self):
        """Tests for no error if an reaction is empty."""
        params = {
            'countermeasure': {
                'constraints': {
                    'john': {
                        'reactions': {
                            'name': {
                            }},
                        'constraint': {}}}}}

        rospy.set_param("/arni", params)
        try:
            ConstraintHandler(None)
        except Exception:
            self.fail(
                "an empty reaction caused the exception: %s"
                % traceback.format_exc())

    def _set_param_and(self):
        params = {
            'countermeasure': {
                'config': {
                    'storage_timeout': 10,
                    'reaction_autonomy_level': 50},
                'constraints': {
                    'john': {
                        'reactions': {
                            'two': {
                                'action': 'publish',
                                'node': 'node1',
                                'message': 'node1 has a problem',
                                'loglevel': 'info',
                                'autonomy_level': 13},
                            'one': {
                                'action': 'stop',
                                'node': 'node1',
                                'autonomy_level': 100}},
                        'min_reaction_interval': 5,
                        'constraint': {
                            'or': {
                                'n!node2': {'ram_usage_mean': 'high'},
                                'n!node1': {'cpu_usage_mean': 'high'}}}}}}}

        if rospy.has_param("/arni"):
            rospy.delete_param("/arni")
        rospy.set_param("/arni", params)

# A couple of tests regarding the parsing of reactions.
# tests the method ConstraintHandler._parse_reaction_list

    def test_reaction_autonomy_level_not_set(self):
        params = {
            'reactions': {
                'reac_one': {
                    'action': 'publish',
                    'message': 'bla',
                    'loglevel': 'debug'}}}
        reactions = ConstraintHandler._parse_reaction_list(params, "const")
        self.assertEqual(len(reactions), 1)
        self.assertEqual(
            reactions[0].autonomy_level, 0,
            "Unset autonomy level should lead to level 0.")

    def test_reaction_parse_wrong_action(self):
        """Test parsing an action that is not defined"""
        params = {
            'reactions': {
                'reac_one': {
                    'action': 'multiply',
                    }}}
        reactions = ConstraintHandler._parse_reaction_list(params, "const")
        self.assertEquals(
            len(reactions), 0, "Wrong action should lead to no reaction")

    def test_reaction_multiple_reactions(self):
        """Test parsing two reactions in one constraint."""
        params = {
            'reactions': {
                'reac_one': {
                    'action': 'publish',
                    'node': 'node1',
                    'message': 'node1 has a problem',
                    'loglevel': 'info',
                    'autonomy_level': 13},
                'reac_two': {
                    'action': 'publish',
                    'node': 'node2',
                    'message': 'node2 has a problem',
                    'loglevel': 'info',
                    'autonomy_level': 13}}}
        reactions = ConstraintHandler._parse_reaction_list(params, "const")
        self.assertEqual(len(reactions), 2, "There should be two reactions.")

    def test_reaction_parse_publish_reaction(self):
        """Test if parsing a single publish reaction works."""
        params = {
            'reactions': {
                'reac_one': {
                    'action': 'publish',
                    'message': 'node1 has a problem',
                    'loglevel': 'info',
                    'autonomy_level': 13}}}
        reactions = ConstraintHandler._parse_reaction_list(params, "const")
        self.assertEqual(
            len(reactions), 1,
            "Parsing one reaction should end up with one reaction.")
        reaction = reactions[0]

        self.assertEqual(reaction._message, "node1 has a problem")
        self.assertEqual(reaction._node, None, "node should be None")
        self.assertEqual(reaction.autonomy_level, 13)

    def test_reaction_parse_empty_reaction(self):
        """Set an empty reaction and check if its empty after parsing"""

        reactions = ConstraintHandler._parse_reaction_list(
            dict(), "const")
        self.assertEqual(
            len(reactions), 0,
            "Parsing an empty reaction is not resulting in an empty reaction")

# test the parsing of intervals and timeouts.
# tests the method ConstraintHandler._parse_interval_and_timeout

    def test_interval_valid_values(self):
        """Test if setting interval and timeout works."""
        c_dict = {
            'min_reaction_interval': 30,
            'reaction_timeout': 50
        }
        interval, timeout = ConstraintHandler._parse_interval_and_timeout(
            c_dict)
        self.assertEqual(interval, rospy.Duration(30))
        self.assertEqual(timeout, rospy.Duration(50))

    def test_interval_timeout_not_existing(self):
        """Test if the default values are used if there is no interval and
        timeout set."""
        rospy.set_param(
            helper.ARNI_CTM_CFG_NS + "default/min_reaction_interval", 100)
        rospy.set_param(
            helper.ARNI_CTM_CFG_NS + "default/reaction_timeout", 300)
        interval, timeout = ConstraintHandler._parse_interval_and_timeout({})
        self.assertEqual(
            interval, rospy.Duration(100),
            "Interval should have been set to default value.")
        self.assertEqual(
            timeout, rospy.Duration(300),
            "Timeout should have been set to default value")

    def test_interval_wrong_value(self):
        rospy.set_param(
            helper.ARNI_CTM_CFG_NS + "default/min_reaction_interval", 5)
        rospy.set_param(
            helper.ARNI_CTM_CFG_NS + "default/reaction_timeout", 20)
        interval, timeout = ConstraintHandler._parse_interval_and_timeout(
            {
                'min_reaction_interval': "abc"
            })
        self.assertEqual(
            interval, rospy.Duration(5),
            "Interval should have been set to the default value.")

# test ConstraintHandler._traverse_dict

    def test_traverse_simple_and(self):
        """Tests the traversing through a simple and."""
        c_dict = {
            'and': {
                'n!node2': {'ram_usage_mean': 'high'},
                'n!node1': {'ram_usage_mean': 'high'},
            }
        }
        const_item = ConstraintHandler._traverse_dict(c_dict, 'and')
        self.assertEqual(type(const_item), ConstraintAnd)
        # not so nice way of getting leafes, but will do for testing.
        leafs = const_item._ConstraintAnd__constraint_list
        self.assertEqual(len(leafs), 2)
        self.assertEqual(leafs[0]._ConstraintLeaf__seuid, 'n!node2')
        self.assertEqual(leafs[1]._ConstraintLeaf__seuid, 'n!node1')

    def test_traverse_simple_and_or(self):
        """Tests if a simple or exists in an and."""
        c_dict = {
            'and': {
                'n!node2': {'ram_usage_mean': 'high'},
                'n!node1': {'ram_usage_mean': 'high'},
                'or': {
                    'n!node3': {'ram_usage_mean': 'high'},
                    'n!node4': {'ram_usage_mean': 'high'},
                }
            }
        }
        const_item = ConstraintHandler._traverse_dict(c_dict, 'and')
        leafs = const_item._ConstraintAnd__constraint_list
        self.assertEqual(len(leafs), 3)
        self.assertIn(ConstraintOr, (map(type, leafs)))

    def test_traverse_simple_not(self):
        """Test if a simple not can be traversed."""
        c_dict = {
            'not': {
                'n!node2': {'ram_usage_mean': 'high'},
                'n!node1': {'ram_usage_mean': 'high'},
            }
        }
        const_item = ConstraintHandler._traverse_dict(c_dict, 'not')
        self.assertEqual(len(const_item), 2)
        for not_item in const_item:
            self.assertEqual(type(not_item), ConstraintNot)

    def test_traverse_empty_and(self):
        """Tests if an empty and can be traversed."""
        c_dict = {
            'and': {}
        }
        const_item = ConstraintHandler._traverse_dict(c_dict, 'and')
        leafs = const_item._ConstraintAnd__constraint_list
        self.assertEqual(len(leafs), 0)

    def test_traverse_wrong_format(self):
        """Test for an leaf to be ignored if its not a seuid."""
        c_dict = {
            'ant': {
                'n!node2': {'ram_usage_mean': 'high'},
                'n!node1': {'ram_usage_mean': 'high'},
            }
        }
        const_item = ConstraintHandler._traverse_dict(c_dict, 'ant')
        self.assertEqual(const_item, None)

    def test_traverse_wrong_outcome(self):
        """Test for an outcome thats not a string."""
        c_dict = {
            'n!node2': {
                'ram_usage_mean': {
                    'n!node1': {'ram_usage_mean': 'high'}}},
        }
        const_item = ConstraintHandler._traverse_dict(c_dict, 'n!node2')
        self.assertEqual(const_item, [])


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_parsing_constraint', TestParsingOfConstraints)
