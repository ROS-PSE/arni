#!/usr/bin/env python

import unittest
from arni_nodeinterface.host_statistics_handler import *
from arni_nodeinterface.host_status import *

import traceback
import rospy

PKG = 'arni_nodeinterface'


class TestStatisticsCalc(unittest.TestCase):

    def test_cpu(self):
        hs = HostStatus('host')
        cpu_usage = [1 , 2 , 3 , 4 , 5 ]
        t = hs.calc_stat_tuple(cpu_usage)
        self.assertEqual(t.max , 5)
        self.assertEqual(t.mean , 3)
        self.assertAlmostEqual(t.stddev, 1.58114, delta = 0.01)

    def test_cpu_core(self):
        hs = HostStatus('host')
        cpu_usage_core = [[1,2,3,4,5], [1,2,3,4,5]]
        ts = [hs.calc_stat_tuple(cpu_usage_core[0]) ,
                hs.calc_stat_tuple(cpu_usage_core[1])]
        for i in range(2):
            self.assertEqual(ts[i].max , 5)
            self.assertEqual(ts[i].mean , 3)
            self.assertAlmostEqual(ts[i].stddev, 1.58114, delta = 0.01)

    def test_empty_list(self):
        hs = HostStatus('host')
        tl = hs.calc_stat_tuple(hs._cpu_usage)
        self.assertEqual(tl, (0,0,0))




if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_statistical_calc', TestStatisticsCalc)
    


