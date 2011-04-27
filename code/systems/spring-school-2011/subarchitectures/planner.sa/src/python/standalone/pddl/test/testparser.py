#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser
from parser import *


testfiles = {}

testfiles['emptytest'] = "()"

testfiles['parse'] = \
"""(element
  (p1 p2
    (key-value)
  )
;; comment

  ()

)
"""

testfiles['parsesep'] = \
"""(element
  (p1:p2
    (key-value)
  )
;; comment

  (:)

)
"""

testfiles['failempty'] = ""

testfiles['failinvalid'] = \
"""this file contains
no valid structure"""


testfiles['endoffile1'] = \
"""(element
  (p1 p2
    (p3)
  )
;; comment
"""

testfiles['endoffile2'] = \
"""(element
  (p1 (p2
    (p3)
  ) 
;; comment
  (
"""

testfiles['endoffile3'] = \
"""(element
  (p1 (p2
    (p3)
  ) 
;; comment
"""

testfiles['endoffile4'] = \
"""(element
  (p1 (p2
    (p3)
  )))
  (p4))
;; comment
"""

class ParserTest(unittest.TestCase):
    def setUp(self):
        self.filenames= {}
        for name,case in testfiles.iteritems():
            (fd, filename) = tempfile.mkstemp()
            file = os.fdopen(fd, "w")
            file.write(case)
            file.close()
            self.filenames[name] = filename

    def tearDown(self):
        for filename in self.filenames.itervalues():
            os.unlink(filename)

    def testFileLoading(self):
        """Testing reading from file"""
        
        try:
            p = Parser.parse_file(self.filenames['emptytest'])
        except:
            self.fail("Exception when opening file")

        self.assert_(p.root is not None, "Parsing failed")

        
#     def testComplexFile(self):
#         """Testing reading a complex file"""
        
#         try:
#             p = Parser.parse_file("/home/goebelbe/src/cogx/planning/trunk/subarchitectures/planner.sa/test_data/cp_test.domain.mapl")
#         except:
#             self.fail("Exception when opening file")
            
#         self.assert_(p.root is not None, "No root node")
#         self.assert_(len(p.root) > 0, "Root node is empty")
        

    def testFileLoadingError(self):
        """Testing file error handling"""
        
        self.assertRaises(IOError, Parser.parse_file, "/a/filename/that/doesnt/exists")

    def testParsingResult(self):
        """Testing basic parsing"""
        
        p = Parser.parse_file(self.filenames['parse'])
        root = p.root
        
        self.assertFalse(root.is_terminal())
        self.assertEqual(len(root), 3)
        self.assertEqual(root[0].token, "element")
        self.assert_(root[0].is_terminal())
        self.assertFalse(root[1].is_terminal())
        self.assertEqual(len(root[1]), 3)
        self.assertEqual(len(root[1][-1]), 1)
        self.assertFalse(root[2].is_terminal())

    def testParsingAnnotations(self):
        """Testing line annotations in parse tree"""

        p = Parser(testfiles['parse'].split("\n"), "testsource")
        root = p.root

        self.assertEqual(root.line(), 1)
        self.assertEqual(root.file(), "testsource")
        self.assertEqual(root[1][2].line(), 3)
        self.assertEqual(root[1][2].file(), "testsource")
        self.assertEqual(root[2].line(), 7)
        self.assertEqual(root[2].file(), "testsource")

    def testParsingAnnotationsFromFile(self):
        """Testing line/file annotations when loading from file"""

        p = Parser.parse_file(self.filenames['parse'])
        root = p.root

        self.assertEqual(root.line(), 1)
        self.assertEqual(root.file(), self.filenames['parse'])
        self.assertEqual(root[1][2].line(), 3)
        self.assertEqual(root[1][2].file(), self.filenames['parse'])
        self.assertEqual(root[2].line(), 7)
        self.assertEqual(root[2].file(), self.filenames['parse'])

        
    def testParsingWithSeparators(self):
        """Testing parsing with additional separators"""
        
        p = Parser(testfiles['parsesep'].split("\n"), "testsource", [":", "-"])
        root = p.root
        
        self.assertFalse(root.is_terminal())
        self.assertEqual(len(root), 3)
        self.assertEqual(root[0].token, "element")
        self.assert_(root[0].is_terminal())
        self.assertFalse(root[1].is_terminal())
        self.assertEqual(len(root[1]), 4)
        self.assertEqual(len(root[1][-1]), 3)
        self.assertFalse(root[2].is_terminal())
        self.assertEqual(len(root[2]), 1)
        
    def testEmptyFile(self):
        """Testing handling of empty files"""
        
        try:
            p = Parser.parse_file(self.filenames['failempty'])
        except ParseError, e:
            self.assertEqual(e.token.line, 0)
            self.assertEqual(e.message, "Empty File")
            return

        self.fail("Empty file did not result in a ParseError")

    def testInvalidContent(self):
        """Testing handling of invalid content"""
        
        try:
            p = Parser.parse_file(self.filenames['failinvalid'])
        except UnexpectedTokenError, e:
            self.assertEqual(e.token, "this")
            self.assertEqual(e.token.line, 1)
            self.assertEqual(e.message, "Expected '(', found 'this'")
            return

        self.fail("Invalid file did not result in a ParseError")

    def testEndOfFile1(self):
        """Testing handling of premature end of file 1"""
        
        try:
            p = Parser.parse_file(self.filenames['endoffile1'])
        except ParseError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line, 1)
            self.assertEqual(e.message, "No closing ')' before end of file")
            return

        self.fail("Too early end of file did not result in a ParseError")

    def testEndOfFile2(self):
        """Testing handling of premature end of file 2"""
        
        try:
            p = Parser.parse_file(self.filenames['endoffile2'])
        except ParseError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line, 6)
            self.assertEqual(e.message, "No closing ')' before end of file")
            return

        self.fail("Too early end of file did not result in a ParseError")

    def testEndOfFile3(self):
        """Testing handling of premature end of file 3"""

        try:
            p = Parser.parse_file(self.filenames['endoffile3'])
        except ParseError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line, 2)
            self.assertEqual(e.message, "No closing ')' before end of file")
            return

        self.fail("Too early end of file did not result in a ParseError")

    def testEndOfFile4(self):
        """Testing handling of tokens after end of root group"""

        try:
            p = Parser.parse_file(self.filenames['endoffile4'])
        except UnexpectedTokenError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line, 5)
            self.assertEqual(e.message, "Expected end of file, found '('")
            return

        self.fail("Content after end of root group did not result in a ParseError")
        
        
if __name__ == '__main__':
    unittest.main()    
        
