PKG = 'arni_core'
NAME = 'host_lookup_test'

import unittest

from host_lookup import *
from helper import SEUID_DELIMITER

from arni_msgs.msg import RatedStatistics, RatedStatisticsEntity


class TestHostLookup(unittest.TestCase):

    def test_singleton(self):
        """Test if the singleton is working properly."""
        lookup_a = HostLookup()
        lookup_a.add_node("nodename", "hostname")
        lookup_b = HostLookup()
        self.assertEquals(
            lookup_a.get_host("nodename"), lookup_b.get_host("nodename"))

    def test_callback_easy(self):
        """Test if a called callback adds the node - host to the dict."""
        lookup = HostLookup()
        for x in range(1, 3):
            rstat = RatedStatistics()
            rstat.host = "host1"
            rstat.seuid = "n%s%s%d" % (SEUID_DELIMITER, "node", x)
            lookup.callback_rated(rstat)
        test = lookup.get_host("node1") == "host1"
        self.assertTrue(test)

    def test_get_node_list(self):
        """Test if a simple reverse lookup with 9 nodes at one host works."""
        lookup = HostLookup()
        node_orig_list = list()
        for x in range(1, 10):
            rstat = RatedStatistics()
            rstat.host = "host1"
            rstat.seuid = "n%s%s%d" % (SEUID_DELIMITER, "node", x)
            lookup.callback_rated(rstat)
            node_orig_list.append("node%d" % x)
        node_list = lookup.get_node_list("host1")
        self.assertItemsEqual(node_list, node_orig_list)


if __name__ == '__main__':
    unittest.main()
