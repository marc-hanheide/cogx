#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest, common

class DomainTest(common.PddlTest):
    
    def testLogistics(self):
        """Testing logistics domain"""
        dom = self.load("testdata/logistics.domain.mapl")

        self.assertEqual(len(dom.actions), 6)

    def testLogisticsAC(self):
        """Testing logistics domain with action costs"""
        dom = self.load("testdata/logistics.acosts.mapl")

        self.assertEqual(len(dom.actions), 6)
        
    def testBlocksworld(self):
        """Testing blocksworld domain"""
        dom = self.load("testdata/blocksworld.domain.pddl")

        self.assertEqual(len(dom.actions), 4)
        
    def testRovers(self):
        """Testing rovers domain"""
        dom = self.load("testdata/rovers.domain.mapl")

        self.assertEqual(len(dom.actions), 10)
        
if __name__ == '__main__':
    unittest.main()    
        
