#! /usr/bin/env python
# -*- coding: latin-1 -*-

__all__ = ["ParseError", "parse_nested_list"]

class ParseError(Exception):
  pass

# Basic functions for parsing MAPL, PDDL, Lisp files.
def parse_nested_list(input_file, error_on_additionals=True):
  tokens = tokenize(input_file)
  next_token = tokens.next()
  if next_token != "(":
    raise ParseError("Expected '(', got %s." % next_token)
  result = list(parse_list_aux(tokens))
  if error_on_additionals:
    for tok in tokens:  # Check that generator is exhausted.
      raise ParseError("Unexpected token: %s." % tok)
  return result
  
def tokenize(input):
  for line in input:
    line = line.split(";", 1)[0]  # Strip comments.
    line = line.replace("(", " ( ").replace(")", " ) ")
    for token in line.split():
      yield token.lower()

def parse_list_aux(tokenstream):
  # Leading "(" has already been swallowed.
  while True:
    try:
      token = tokenstream.next()
    except StopIteration:
      raise ParseError()
    if token == ")":
      return
    elif token == "(":
      yield list(parse_list_aux(tokenstream))
    else:
      yield token

def pretty_print_nested_list(alist, indent=0):
  if not isinstance(alist, list):
    if indent:
      print indent*" ",
    print alist
  else:
    for elmt in alist:
       pretty_print_nested_list(elmt, indent+2)
