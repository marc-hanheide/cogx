#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
import mapltypes as types
import predicates, conditions, effects
from parser import ParseError
from scope import Scope

class Axiom(Scope):
    def __init__(self, pred, args, condition, scope):
        Scope.__init__(self, args, scope)
        self.args = args
        self.predicate = pred
        self.condition = condition
        
    def instantiate(self, mapping):
        if not isinstance(mapping, dict):
            mapping = dict([(param.name, c) for (param, c) in zip(self.args, mapping)])
        Scope.instantiate(self, mapping)

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        args = [types.Parameter(p.name, p.type) for p in self.args]
        
        a = Axiom(self.predicate, args, None, newdomain)
        a.condition = self.condition.copy(a)
        return a
        
        
    @staticmethod
    def parse(it, scope):
        it.get(":derived")
        j = iter(it.get(list, "axiom head"))
        name = j.get("terminal", "predicate identifier").token
        args = predicates.parseArgList(j, scope.types)

        if name.string not in scope.predicates:
            raise ParseError(name, "Axiom head must be declared as a predicate beforehand.")
        pred_candidates = scope.predicates[name.string]

        pred = None
        for p in pred_candidates:
            pred = p
            for a,pa in zip(args, p.args):
                if not a.isInstanceOf(pa.type):
                    pred = None
                    break
            if pred:
                break

        if not pred:
            type_str = " ".join(map(lambda a: str(a.type), args))
            c_str = "\n  ".join(str(p) for p in pred_candidates)
            raise ParseError(name, "no matching predicate found for (%s %s). Candidates are:\n  %s" % (name.string, type_str, c_str))

        axiom = Axiom(pred, args, None, scope)
        axiom.condition = conditions.Condition.parse(iter(it.get(list, "condition")), axiom)
        return axiom
