#!/usr/bin/env python

import unittest
from arni_processing.specification_handler import *
from arni_processing.rated_statistics import *
from arni_msgs.msg import HostStatistics

import traceback
import rospy
import rosparam

PKG = "arni_processing"

exampleData = {
    'h!127.0.0.1': None
}
exampleMessages = {}
mockSpecs = {}

spec_namespace = '/arni/specifications'


class TestRatingData(unittest.TestCase):

    def test_no_data(self):
        '''
        Rating None will return None.
        '''
        sh = SpecificationHandler()
        res = sh.compare(None, "h!127.0.0.1")
        self.assertEqual(res, None)

    def test_invalid_ident(self):
        '''
        Trying to rate on an invalid identifier will return None.
        '''
        sh = SpecificationHandler()
        res = sh.compare(None, "identifier")
        self.assertEqual(res, None)

    def test_no_ident(self):
        '''
        Trying to rate data without specifying an identifier will return None.
        '''
        sh = SpecificationHandler()
        res = sh.compare(exampleMessages["h!127.0.0.1"], None)
        self.assertEqual(res, None)

    def test_no_spec(self):
        '''
        If no Specification is available, fields will be rated with 2 (unknown).
        '''
        sh = SpecificationHandler()
        res = sh.compare(exampleMessages["h!127.0.0.1"], "h!127.0.0.1")
        for k in res.keys():
            s = res.get_value(k)["state"]
            if isinstance(s, int):
                self.assertEqual(s, 2)

    def test_spec3(self):
        '''
        Values within their limits will be rated with 3 (OK).
        '''
        rospy.set_param(spec_namespace, mockSpecs)
        sh = SpecificationHandler()
        res = sh.compare(exampleMessages["h!127.0.0.1"], "h!127.0.0.1")
        self.assertEqual(res.get_value('cpu_temp_stddev')['state'], 3)
        self.assertEqual(res.get_value('cpu_temp_core')['state'], [3, 3])
        try:
            rospy.delete_param(spec_namespace)
        except KeyError:
            pass

    def test_spec2(self):
        '''
        Values without specifications will be rated with 2 (OK).
        '''
        rospy.set_param(spec_namespace, mockSpecs)
        sh = SpecificationHandler()
        res = sh.compare(exampleMessages["h!127.0.0.1"], "h!127.0.0.1")
        self.assertEqual(res.get_value('ram_usage_max')['state'], 2)
        self.assertEqual(res.get_value('cpu_usage_core_max')['state'], [2, 2])
        try:
            rospy.delete_param(spec_namespace)
        except KeyError:
            pass

    def test_spec10(self):
        '''
        Values below their limits will be rated with 1 (Low).
        Values above their limits will be rated with 0 (High).
        '''
        rospy.set_param(spec_namespace, mockSpecs)
        msg = exampleMessages["h!127.0.0.1"]
        msg.cpu_temp_core_mean[0] -= 30
        msg.cpu_temp_core_mean[1] += 10
        sh = SpecificationHandler()
        res = sh.compare(msg, "h!127.0.0.1")
        self.assertEqual(res.get_value('cpu_temp_core_mean')['state'], [1, 0])
        try:
            rospy.delete_param(spec_namespace)
        except KeyError:
            pass


def setup_messages():
    for k in exampleData.keys():
        if k[0] == "h":
            hs = HostStatistics()
            for field in exampleData[k]:
                try:
                    setattr(hs, field, exampleData[k][field])
                except Exception:
                    pass
            exampleMessages[k] = hs

if __name__ == '__main__':
    import rosunit
    exampleData["h!127.0.0.1"] = rospy.get_param('/arni/test/mockdata/h!127.0.0.1')
    mockSpecs = rospy.get_param(spec_namespace)
    rospy.delete_param(spec_namespace)
    setup_messages()

    rosunit.unitrun(PKG, 'test_rating_data', TestRatingData)

