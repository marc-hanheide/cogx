#! /usr/bin/env python

"""usage: parser "put the red cube to the right of the green cube" 

(slightly) adapted from P.Norvig's AIMA code)"""

from utils import *
from collections import defaultdict

#______________________________________________________________________________
# Grammars and Lexicons

def Rules(**rules): 
    """Create a dictionary mapping symbols to alternative sequences.
    >>> Rules(A = "B C | D E")
    {'A': [['B', 'C'], ['D', 'E']]}
    """
    for (lhs, rhs) in rules.items():
        rules[lhs] = [alt.strip().split() for alt in rhs.split('|')]
    return rules

def Lexicon(**rules):
    """Create a dictionary mapping symbols to alternative words.
    >>> Lexicon(Art = "the | a | an")
    {'Art': ['the', 'a', 'an']}
    """    
    for (lhs, rhs) in rules.items():
        rules[lhs] = [word.strip() for word in rhs.split('|')]
    return rules

class Grammar:
    def __init__(self, name, rules, lexicon):
        "A grammar has a set of rules and a lexicon."
        update(self, name=name, rules=rules, lexicon=lexicon)
        self.categories = defaultdict(list)
        for lhs in lexicon:
            for word in lexicon[lhs]:
                self.categories[word].append(lhs)

    def rewrites_for(self, cat):
        "Return a sequence of possible rhs's that cat can be rewritten as."
        return self.rules.get(cat, ())

    def isa(self, word, cat):
        "Return True iff word is of category cat"
        return word == cat or cat in self.categories[word]

    def __repr__(self):
        return '<Grammar %s>' % self.name
    
def generate_random(grammar, s='S'):
    """Replace each token in s by a random entry in grammar (recursively).
    This is useful for testing a grammar, e.g. generate_random(E_)"""
    import random
    
    def rewrite(tokens, into):
        for token in tokens:
            if token in grammar.rules:
                rewrite(random.choice(grammar.rules[token]), into)
            elif token in grammar.lexicon:
                into.append(random.choice(grammar.lexicon[token]))
            else:
                into.append(token)
        return into

    return ' '.join(rewrite(s.split(), []))

#______________________________________________________________________________
# Chart Parsing


class Chart:
    """Class for parsing sentences using a chart data structure. [Fig 22.7]
    >>> chart = Chart(E0); 
    >>> len(chart.parses('the stench is in 2 2'))
    1
    """

    def __init__(self, grammar, trace=False):
        """A datastructure for parsing a string; and methods to do the parse.
        self.chart[i] holds the edges that end just before the i'th word.
        Edges are 5-element lists of [start, end, lhs, [found], [expects]]."""
        update(self, grammar=grammar, trace=trace)

    def parses(self, words, S='S'):
        """Return a list of parses; words can be a list or string."""
        import string
        if isinstance(words, str):
            words = words.lower()
            if words[-1] in string.punctuation:
                words = words[:-1]
            words = words.split()
        self.parse(words, S)
        # Return all the parses that span the whole input
        return [[i, j, S, found, []]
                for (i, j, lhs, found, expects) in self.chart[len(words)]
                if lhs == S and expects == []]

    def parse(self, words, S='S'):
        """Parse a list of words; according to the grammar.
        Leave results in the chart."""
        self.chart = [[] for i in range(len(words)+1)]
        self.add_edge([0, 0, 'S_', [], [S]])
        for i in range(len(words)):
            self.scanner(i, words[i])
        return self.chart

    def add_edge(self, edge):
        "Add edge to chart, and see if it extends or predicts another edge."
        start, end, lhs, found, expects = edge
        if edge not in self.chart[end]:
            self.chart[end].append(edge)
            if self.trace:
                print '%10s: added %s' % (caller(2), edge)
            if not expects:
                self.extender(edge)
            else:
                self.predictor(edge)

    def scanner(self, j, word):
        "For each edge expecting a word of this category here, extend the edge."
        for (i, j, A, alpha, Bb) in self.chart[j]:
            if Bb and self.grammar.isa(word, Bb[0]):
                self.add_edge([i, j+1, A, alpha + [(Bb[0], word)], Bb[1:]])

    def predictor(self, (i, j, A, alpha, Bb)):
        "Add to chart any rules for B that could help extend this edge."
        B = Bb[0]
        if B in self.grammar.rules:
            for rhs in self.grammar.rewrites_for(B):
                self.add_edge([j, j, B, [], rhs])

    def extender(self, edge):
        "See what edges can be extended by this edge."
        (j, k, B, _, _) = edge
        for (i, j, A, alpha, B1b) in self.chart[j]:
            if B1b and B == B1b[0]:
                self.add_edge([i, k, A, alpha + [edge], B1b[1:]])

def is_terminal(c):
    return isinstance(c,tuple)

class ParseTreeNode(object):
    def __init__(self, name, children):
        update(self, name=name, children=children)
    def pretty_print(self, indent=0):
        print "%s%s" % (indent*' ', str(self.name))
        for child in self.children:
            child.pretty_print(indent+2)
    def is_ambiguous(self):
        return self.name == "" and len(self.children) != 1
    def matches(self, matchlist):
        if isinstance(matchlist, str):
            matchlist = matchlist.split()
        if len(self.children) < len(matchlist):
            return False
        for ptsym, matchsym in zip(self.children, matchlist):
            if ptsym.name == matchsym:
                continue
            elif isinstance(ptsym.name, tuple):
                if matchsym in ptsym.name:
                    continue
            else:
                return False
        return True
    @classmethod
    def from_chart(cls, chart, toplevel=True):
        if toplevel:
#            if len(chart) == 1:
#                return ParseTreeNode.from_chart(chart[0], False)
            tok = ""
            children = [ParseTreeNode.from_chart(parse, False) for parse in chart]
        elif is_terminal(chart):
            tok = chart
            children = []
        else:
            tok = chart[2]
            children = [ParseTreeNode.from_chart(child, False) for child in chart[3]]
        return ParseTreeNode(tok, children)
        

E0 = Grammar('E0',

    Rules( 
    S = 'VP| S Conjunction S',
    VP = 'Verb VP1',
    VP1 = 'VP1end | VP1 Conjunction VP1end',
    VP1end = 'NP PP',
    NP = 'Pronoun | Noun | Article Noun | Article Adjectives Noun | NP RelClause | NP PP| NP Conjunction NP',
#    VP = 'VP NP | VP PP',
    Adjectives = 'Adjective | Adjectives Adjective',
    PP = 'PPstart NP',
    PPstart = 'Preposition | SpatialRel | to the SpatialRel of | in SpatialRel of | Preposition to',
    RelClause = 'RelPronoun Verb PP',
    Noun = 'NounSingular | NounPlural'),


    Lexicon(
    NounSingular = "cube | block | pyramid | ball | box | thing | object",
    NounPlural = "cubes | blocks | pyramids | balls | things | objects",
    SpatialRel = "right | left | front | back",
    Verb = "put | move | is | are",
    Adjective = "big | small | "
    "red | blue | yellow | green | black | white",
    Pronoun = "me | you | I | it",
    Article = "the | a | an",
    Preposition = "to | in | on | near | next",
    Conjunction = "and | or",
    RelPronoun = "that | which | who"
    ))

test_phrases = [
    "put the red things to the right of the blue pyramid",
    "put the red things to the right of the blue pyramid to the right of the red pyramid",
    "put the red things near the blue pyramid to the right of the red pyramid",
    "put the red things that are near the blue pyramid to the right of the red pyramid",
    "put the thing near the thing near the thing",
]

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        phrases = test_phrases
    else:
        phrases = sys.argv[1:]
    chart = Chart(E0)
    parses = [(phrase, ParseTreeNode.from_chart(chart.parses(phrase))) for phrase in phrases]
    ambiguous = 0
    for (phrase, tree) in parses:
        #if tree.is_ambiguous():
            print "Found %d parses for phrase '%s'" % (len(tree.children), phrase)
            ambiguous += 1
    if not ambiguous:
        print "All phrases parsed uniquely!"
    # print out the parse tree for the last one
    phrase, tree = parses[-1]
    for parse in tree.children:
        print "next parse for", phrase
        parse.pretty_print()
