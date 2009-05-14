#! /usr/bin/env python2.5

"""Unit tests for all MAPL related stuff"""

# import stdlib modules
import unittest
import os
from itertools import chain

# import some standard mapsim modules
import support
import config

# import modules specific for these tests
import planning.mapl.parser as parser
import planning.mapl.conditions as conditions


os.chdir(support.TEST_DIR)

"""
What should be in here:

- create MAPL fact, turn to string, parse, should be orig
- parse MAPL fact, turn to string, should be orig
- what shoud be result for boolean?
- parse MAPL fact, turn to PDDL, parse PDDL, turn to MAPL, should be orig



specific cases:
- negation
- domain/in_domain
- initially
"""

predicates = [
    ("(pos ?o - localized_obj : ?p - object_pos)", "(pos ?o - localized_obj ?p - object_pos)"),
    ("(doorstate ?d - door : ?s - doorstate)", "(doorstate ?d - door ?s - doorstate)"),
    ("(connects ?d - door ?r1 ?r2 - room)", "(connects ?d - door ?r1 ?r2 - room ?_val - boolean)"),
    ]

facts = [
    ("(pos speaker : r)", "(pos speaker r)"),
    ("(pos hearer : r)", "(pos hearer r)"),
    ("(can_talk_to speaker hearer : true)", "(can_talk_to speaker hearer true)"),
    ("(can_talk_to speaker hearer)", "(can_talk_to speaker hearer true)"),
    ("(K speaker (svar args) true)", "(K_svar speaker args true)"),
    ("(not (pos speaker : r))", "?"),
    ("(not (can_talk_to speaker hearer))", "(can_talk_to speaker hearer false)"),
    ("(not (= speaker hearer))", "(not (= speaker hearer))"),
    ]


class BasicTests(unittest.TestCase):
    def setUp(self):
        pass

    def testNestedListParsing(self):
        """ test basic parsing of MAPL/PDDL structures into nested lists """
        all_defs = chain(predicates, facts)
        for orig_str, _ in all_defs:
            orig_str = orig_str.lower()
            nested_list = str2nl(orig_str)
            new_str = parser.nested_list2paren_expr(nested_list)
            self.assertEqual(orig_str, new_str)

    def testMAPLFactParsing(self):
        """ parse MAPL fact, turn to string, should be orig"""
        for orig_str, _ in facts:
            nl = str2nl(orig_str)
            fact = conditions.Atom(nl[0], nl[1:])
            new_str = fact.mapl_str(add_parens=True)
            orig_str = orig_str.replace(" : true", "")  # the
            self.assertEqual(orig_str, new_str)
            

# helper functions
            
def str2nl(astring):
    return parser.parse_nested_list([astring.lower()])


if __name__ == "__main__":
	unittest.main()
