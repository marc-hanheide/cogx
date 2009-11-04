#! /usr/bin/env python
# -*- coding: latin-1 -*-

__all__ = ["ParseError", "parse_nested_list"]

class ParseError(Exception):
  pass

non_lower_tokens = {}

# Basic functions for parsing PDDL (Lisp) files.
def parse_nested_list(input_file):
  tokens = tokenize(input_file)
  tokens = remove_initially_statements(list(tokens))
  next_token = tokens.next()
  if next_token != "(":
    raise ParseError("Expected '(', got %s." % next_token)
  result = list(parse_list_aux(tokens))
  for tok in tokens:  # Check that generator is exhausted.
    raise ParseError("Unexpected token: %s." % tok)
  return result
  
def tokenize(input):
  if isinstance(input,basestring):
    input = input.splitlines()
  for line in input:
    line = line.split(";", 1)[0]  # Strip comments.
    line = line.replace("(", " ( ").replace(")", " ) ")
    for token in line.split():
      ltoken = token.lower()
      if ltoken != token and len(token) > 1:  #look for names
        non_lower_tokens[ltoken] = token
      token = ltoken
      #print "tok", token
      yield token

def parse_list_aux(tokenstream):
  # Leading "(" has already been swallowed.
  while True:
    try:
      token = tokenstream.next()
    except StopIteration:
      raise ParseError("Unexpected end of token list")
    if token == ")":
      return
    elif token == "(":
      yield list(parse_list_aux(tokenstream))
    else:
      yield token

def remove_initially_statements(tokens):
  i = 0
  rm_paren = False
  while i < len(tokens):
    token = tokens[i]
    i += 1
    if token == "initially":
      i += 1
      token = "i__"+tokens[i]
      rm_paren = True
      i += 1
    elif rm_paren and token == ")":
        rm_paren = False
        continue
    yield token

def nested_list2paren_expr(nested_list):
  def gen(nl):
    yield "("
    for elmt in nl:
      if isinstance(elmt, list):
        for sub_elmt in gen(elmt):
          yield sub_elmt
      else:
        yield elmt
    yield ")"
  s = " ".join(gen(nested_list))
  s = s.replace("( ", "(").replace(" )", ")")
  return s
  
