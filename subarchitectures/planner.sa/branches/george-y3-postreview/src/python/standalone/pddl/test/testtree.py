#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser
from parser import Parser, ParseError


test1 = \
"""
(element
  (a1 a2 a3 a4)
  (b1 () b2 b3)
  (a (b (c d)))
)
"""

class ParseTreeTest(unittest.TestCase):
    def testIterator(self):
        """Test parse tree iterator"""
        
        p = Parser(test1.split("\n"))
        count = 0
        for elem in p.root:
            count += 1

        self.assertEqual(count, len(p.root))
        self.assertEqual(count, 4)

    def testIteratorRecursive(self):
        """Test parse tree iterator recursion"""
        
        p = Parser(test1.split("\n"))
        count = [1]
        lists = [1]
        terminals = [0]
        
        def visit(it):
            for elem in it:
                count[0] += 1
                if elem.is_terminal():
                    terminals[0] += 1
                else:
                    visit(elem)
                    lists[0] += 1

        visit(p.root)

        self.assertEqual(terminals[0], 12)
        self.assertEqual(lists[0], 7)
        self.assertEqual(count[0], 19)
        
    def testIteratorChecks(self):
        """Test parse tree iterator content checking"""
        
        try:
            p = Parser(test1.split("\n"))
            it = iter(p.root)

            it.get("element")
            j1 = iter(it.get(list))
            j2 = iter(it.get(list))
            j3 = iter(it.get(list))
        except ParseError, e:
            self.fail("Exception when checking tokens.")

    def testIteratorEnd(self):
        """Test parse tree iterator bounds checking"""
        
        try:
            p = Parser(test1.split("\n"))
            it = iter(p.root)

            it.get("element")
            j1 = iter(it.get(list))
            j2 = iter(it.get(list))
            j3 = iter(it.get(list))
            it.get()
            
        except ParseError, e:
            self.assertEqual(e.token.string, ")")
            self.assertEqual(e.token.line, 6)
            return
            
        self.fail("No exception when list lenght is exceeded")

    def testIteratorCheckFail1(self):
        """Test iterator terminal checking"""
        
        try:
            p = Parser(test1.split("\n"))
            it = iter(p.root)

            it.get("something")

        except ParseError, e:
            self.assertEqual(e.token.string, "element")
            self.assertEqual(e.token.line, 2)
            return
            
        self.fail("No exception when element doesn't match the expected one.")

    def testIteratorCheckFail2(self):
        """Test iterator sublist checking"""
        
        try:
            p = Parser(test1.split("\n"))
            it = iter(p.root)

            it.get(list)

        except ParseError, e:
            self.assertEqual(e.token.string, "element")
            self.assertEqual(e.token.line, 2)
            return
            
        self.fail("No exception when list lenght is exceeded")

    def testIteratorCheckFail3(self):
        """Test iterator terminal checking 2"""
        
        try:
            p = Parser(test1.split("\n"))
            it = iter(p.root)

            it.get()
            it.get("something")

        except ParseError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line, 3)
            return
            
        self.fail("No exception when list lenght is exceeded")

        
if __name__ == '__main__':
    unittest.main()    
        
