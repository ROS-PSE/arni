#!/usr/bin/env python

import unittest
from arni_processing.specification_handler import *
from arni_processing.specification import *

import traceback
import rospy

PKG = "arni_processing"

test_spec = [
    {
        'n!test_node': {
            'node_cpu_usage_mean': [0.03, 0.09],
            'node_ramusage_mean': [3000, 500, 'relative']
        }
    },
    {
        'h!127.0.0.1': {
            'cpu_temp_mean': [30, 60]
        }
    },
    {
        'invalid': {
            'cpu_temp_mean': [30, 60]
        }
    }
]
test_spec2 = {}


class TestLoadingSpecifications(unittest.TestCase):

    __namespace = '/arni/specifications'

    def setUp(self):
        try:
            rospy.delete_param(self.__namespace)
        except KeyError:
            pass

    def test_no_specifications(self):
        """
        Checks if the list of specifications is empty if no specifications are
        on the parameter server.
        """
        sh = SpecificationHandler()
        self.assertEqual(sh.loaded_specifications(), [])

    def test_load_spec(self):
        """
        Checks if a specification is properly loaded.
        """
        seuid = 'n!test_node'
        rospy.set_param(self.__namespace, test_spec[0:1])
        sh = SpecificationHandler()
        self.assertEqual(sh.loaded_specifications(), [seuid])

    def test_load_new_specs(self):
        """
        Checks if the new specification format loads
        """
        rospy.set_param(self.__namespace, test_spec2)
        sh = SpecificationHandler()
        self.assertEqual(sh.loaded_specifications(), ['h!192.168.0.17', 'h!127.0.0.1'])

    def test_reload_spec(self):
        """
        Checks if the list of specifications has been expanded according to new
        parameters after reloading them.
        """
        seuid1 = 'n!test_node'
        seuid2 = 'h!127.0.0.1'
        rospy.set_param(self.__namespace, test_spec[0:1])
        sh = SpecificationHandler()
        rospy.set_param(self.__namespace, test_spec[0:2])
        sh.reload_specifications()
        self.assertItemsEqual(sh.loaded_specifications(), [seuid1, seuid2])

    def test_invalid_seuid(self):
        """
        Checks if invalid identifiers aren't added to the list.
        """
        seuid = 'invalid'
        rospy.set_param(self.__namespace, test_spec[2:1])
        sh = SpecificationHandler()
        self.assertEqual(sh.loaded_specifications(), [])

    def test_existing_fields(self):
        seuid = 'n!test_node'
        rospy.set_param(self.__namespace, test_spec[0:1])
        sh = SpecificationHandler()
        sp = sh.get(seuid)
        self.assertItemsEqual(sp.keys(), test_spec[0][test_spec[0].keys()[0]].keys())
        for k in test_spec[0][test_spec[0].keys()[0]].keys():
            self.assertEqual(test_spec[0][test_spec[0].keys()[0]][k], sp.get(k)[1])


if __name__ == '__main__':
    import rosunit

    test_spec2 = rospy.get_param('/arni/specifications', {})
    rosunit.unitrun(PKG, 'test_loading_specifications', TestLoadingSpecifications)
