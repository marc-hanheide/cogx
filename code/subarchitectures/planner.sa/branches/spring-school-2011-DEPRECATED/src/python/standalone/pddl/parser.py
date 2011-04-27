#! /usr/bin/env python
# -*- coding: latin-1 -*-

class Token(object):
    def __init__(self, token, line, file):
        self.string = token
        self.line = line
        self.file = file

    def error(self, message):
        raise ParseError(self, message)

    def check_keyword(self, keyword):
        if self.string != keyword:
            raise UnexpectedTokenError(self, "'%s'" % keyword)

    def __eq__(self, other):
        if isinstance(other, Token):
            return self.string == other.string and self.line == other.line and self.file == other.file
        return self.string == other

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "%s (line:%d, file:%s)" % (self.string, self.line, self.file)

        
class Element(list):
    def __init__(self, token, children=None):
        self.parent = None
        self.token = token
        self.endtoken = None
        self.terminal = (children is None)

        if children is not None:
            for child in children:
                self.append(child)
                child.parent = self

    def append(self, child):
        if self.terminal:
            return
        list.append(self, child)

    def end(self, token):
        self.endtoken = token

    def is_terminal(self):
        return self.terminal

    def line(self):
        return self.token.line
    def file(self):
        return self.token.file

    def __iter__(self):
        return ElementIterator(self)

class ElementIterator(object):
    def __init__(self, element):
        if element.is_terminal():
            raise UnexpectedTokenError(element.token, "list")
        self.element = element
        self.it = list.__iter__(element)

    def __iter__(self):
        return self

    def next(self):
        return self.it.next()

    def end(self):
        return self.element.endtoken

    def reset(self):
        return ElementIterator(self.element)

    def no_more_tokens(self, message=None):
        try:
            token =self.next().token
        except StopIteration:
            return

        if not message:
            raise UnexpectedTokenError(token, "')'")
        else:
            raise ParseError(token, message)
        
    def get(self, expected=None, message=None):
        try:
            elem = self.it.next()
            if expected is not None:
                if expected == list and elem.is_terminal():
                    if message is None:
                        message = "'('"
                    raise UnexpectedTokenError(elem.token, message)
                elif expected == "terminal" and not elem.is_terminal():
                    if message is None:
                        message = "identifier"
                    raise UnexpectedTokenError(elem.token, message)
                elif expected != list and expected != "terminal" and elem.token.string != expected:
                    if message is None:
                        message = "'%s'" % expected
                    raise UnexpectedTokenError(elem.token, message)
            return elem
                
        except StopIteration:
            raise EndOfListError(self.end())
            
    
class ParseError(Exception):
    def __init__(self, token, message):
        self.token = token
        self._message = message

    def _get_message(self): return self._message
    def _set_message(self, message): self._message = message
    message = property(_get_message, _set_message)
    
    def __str__(self):
        return "Error in line %d of %s: %s" % (self.token.line, self.token.file, self._message)

class UnexpectedTokenError(ParseError):
    def __init__(self, token, expected=None):
        self.token = token
        if expected:
            self._message = "Expected %s, found '%s'" % (expected, token.string)
        else:
            self._message = "Unexpected token: %s" % token.string

class EndOfListError(ParseError):
    def __init__(self, token):
        self.token = token
        self._message = "Unexpected end of list."
        
    
class Parser(object):
    """Generic parser for LISP-like languages. Creates a parse tree
    with file/linenumber annotations"""
    
    def __init__(self, lines, source=None, separators=[]):
        self.separators = ["(",")"] + separators
        self.source = source

        self.root = None

        tokens = self.tokenize(lines, source)
        try:
            token = tokens.next()
        except StopIteration:
            raise ParseError(Token("", 0, source), "Empty File")
            
        if token != "(":
            raise UnexpectedTokenError(token, "'('")

        self.root = self.parse(token, tokens)
        try:
            spurious = tokens.next()
            raise UnexpectedTokenError(spurious, "end of file")
        except StopIteration:
            pass

    @staticmethod
    def parse_file(filename, separators=[]):
        f = open(filename)
        try:
            p = Parser(f, filename, separators)
        except:
            f.close()
            raise
        
        f.close()
        return p

    @staticmethod
    def parse_as(lines, _class, *args):
        p = Parser(lines)
        return _class.parse(iter(p.root), *args)

    def parse(self, head, tokens):
        try:
            element = Element(head, [])
            token = tokens.next()
            
            while token != ")":
                if token == "(":
                    element.append(self.parse(token, tokens))
                else:
                    element.append(Element(token))
                token = tokens.next()

            element.end(token)
            return element
                
        except StopIteration:
            raise ParseError(head, "No closing ')' before end of file")

        
    def tokenize(self, input, source=""):
        for i, line in enumerate(input):
            line = line.split(";",1)[0]
            for sep in self.separators:
                line = line.replace(sep, " "+sep+" ")
            for token in line.split():
                token = token.strip(" \t\n")
                if token != "":
                    yield Token(token.lower(), i+1, source)
            


def parse_typed_list(it, leftFunc, rightFunc, expectedLeft="identifiers", expectedRight="identifier", rightSideRequired = False):
    left = []
    foundSep = False
    for elem in it:
        if elem.token == "-":
            if not left or foundSep:
                raise ParseError(elem.token, "expected %s before '-'" % expectedLeft)
            foundSep = True
            continue

        if foundSep:
            right = rightFunc(elem)
            yield (left, right)
            left = []
            foundSep = False
            continue

        left.append(leftFunc(elem))

    if foundSep:
        raise UnexpectedTokenError(it.end(), expectedRight)

    if left and rightSideRequired:
        raise UnexpectedTokenError(it.end(), "-")
    elif left:
        yield (left, None)
        
