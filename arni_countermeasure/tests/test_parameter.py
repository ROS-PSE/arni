#!/usr/bin/env python
import unittest
from arni_countermeasure.countermeasure_node import *
from arni_countermeasure.constraint_handler import *
from arni_countermeasure.rated_statistic_storage import *

import traceback
import rospy
import sys

PKG = "arni_countermeasure"


## A sample python unit test
class TestParsingOfConstraints(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # def test_working_constraint(self):

    #     params = {
    #         'countermeasure': {
    #             'config': {
    #                 'storage_timeout': 10,
    #                 'reaction_autonomy_level': 50},
    #             'constraints': {
    #                 'john': {
    #                     'reactions': {
    #                         'two': {
    #                             'action': 'publish',
    #                             'node': 'node1',
    #                             'message': 'node1 has a problem',
    #                             'loglevel': 'info',
    #                             'autonomy_level': 13},
    #                         'one': {
    #                             'action': 'stop',
    #                             'node': 'node1',
    #                             'autonomy_level': 100}},
    #                     'min_reaction_interval': 5,
    #                     'constraint': {
    #                         'or': {
    #                             'n!node2': {'ram_usage_mean': 'high'},
    #                             'n!node1': {'cpu_usage_mean': 'high'}}}}}}}

    #     if rospy.has_param("/arni"):
    #         rospy.delete_param("/arni")
    #     rospy.set_param("/arni", params)
    #     try:
    #         cn = CountermeasureNode()
    #         cn._CountermeasureNode__constraint_handler.evaluate_constraints()
    #     except Exception:
    #         self.fail()

    def test_missing_reactions(self):
        """Tests for no error if the field reactions is missing."""
        params = {
            'countermeasure': {
                'constraints': {
                    'john': {
#                        'reactions': { },
                        'constraint': {}}}}}

        rospy.set_param("/arni", params)
        try:
            con_handler = ConstraintHandler(None)
        except Exception:
            self.fail("missing the param reactions caused the exception: %s" % traceback.format_exc())

    def test_empty_reaction(self):
        """Tests for no error if an reaction is empty."""
        params = {
            'countermeasure': {
                'constraints': {
                    'john': {
                        'reactions': {
                            'name': {
                                'node': ''
                            }},
                        'constraint': {}}}}}

        rospy.set_param("/arni", params)
        try:
            con_handler = ConstraintHandler(None)
            output = sys.stdout.getvalue().strip()
            print output
        except Exception:
            self.fail("missing the param reactions caused the exception: %s" % traceback.format_exc())

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

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_parameter', TestParsingOfConstraints)
