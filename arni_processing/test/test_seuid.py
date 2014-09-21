#!/usr/bin/env python

import unittest
from arni_core.helper import *
from arni_msgs.msg import HostStatistics

import rospy

PKG = "arni_processing"

spec_namespace = '/arni/specifications'

hs = None


class TestSeuid(unittest.TestCase):
    def test_valid_noarg(self):
        s = SEUID()
        self.assertFalse(s.is_valid())

    def test_valid_override(self):
        s = SEUID("t!valid")
        self.assertFalse(s.is_valid("invalid"))

    def test_valid_from_class(self):
        self.assertRaises(NameError, SEUID, "invalid")
        t = SEUID("n!node_name")
        self.assertTrue(t.is_valid())

    def test_valid_from_fn(self):
        s = SEUID()
        self.assertTrue(s.is_valid("n!node_name"))

    def test_valid_types(self):
        s = SEUID()
        self.assertTrue(s.is_valid("n!/arni_processing"))
        self.assertTrue(s.is_valid("n!/arni/Processing"))
        self.assertFalse(s.is_valid("n!0Node"))
        self.assertFalse(s.is_valid("n!#1Node"))
        self.assertTrue(s.is_valid("h!127.0.0.1"))
        self.assertTrue(s.is_valid("h!myComputer"))
        self.assertTrue(s.is_valid("t!/myTopic"))
        self.assertTrue(s.is_valid("c!/mySubscriber!/myTopic!/myPublisher"))

    def test_valid_from_serialization(self):
        s = SEUID("n!test_node")
        t = SEUID(s.serialize())
        self.assertEqual(str(s), str(t))

    def test_fields_node(self):
        s = SEUID("n!/arni_processing")
        self.assertEqual(s.node, "/arni_processing")
        s = SEUID("n!test_node")
        self.assertEqual(s.node, "test_node")

    def test_fields_override(self):
        s = SEUID("n!test_node")
        self.assertEqual(s.node, "test_node")
        s = SEUID("h!localhost")
        self.assertEqual(s.host, "localhost")
        self.assertEqual(s.node, None)

    def test_fields_connection(self):
        s = SEUID("c!/mySubscriber!/myTopic!/myPublisher")
        self.assertEqual(s.publisher, "/myPublisher")
        self.assertEqual(s.subscriber, "/mySubscriber")
        self.assertEqual(s.topic, "/myTopic")
        self.assertEqual(s.node, None)

    def test_msg_invalid(self):
        self.assertRaises(TypeError, SEUID, ("hello", "world"))

    def test_msg_hostmsg(self):
        s = SEUID(hs)
        self.assertEqual(s.host, "127.0.0.1")

    def test_get_seuid(self):
        s = SEUID(hs)
        self.assertEqual("h!127.0.0.1", s.get_seuid("host"))

    def test_get_seuid_invalid(self):
        s = SEUID()
        self.assertRaises(AttributeError, s.get_seuid, "host")
        s = SEUID(hs)
        self.assertRaises(KeyError, s.get_seuid, "node")


if __name__ == '__main__':
    import rosunit

    data = rospy.get_param('/arni/test/mockdata')
    hs = HostStatistics()
    for k in data.keys():
        try:
            setattr(hs, k, data[k])
        except Exception:
            pass

    rosunit.unitrun(PKG, 'test_seuid', TestSeuid)

