#! /usr/bin/env python
# -*- coding: latin-1 -*-
from collections import defaultdict

import predicates, conditions
from parser import ParseError
from scope import Scope

class Axiom(Scope):
    def __init__(self, pred, args, condition, scope):
        Scope.__init__(self, args, scope)
        assert isinstance(pred, predicates.Predicate), "not a predicate: %s" % str(pred)
        self.args = args
        self.predicate = pred

        self.predicate_args = self.args[:len(self.predicate.args)]
        self.free_args = self.args[len(self.predicate.args):]
        
        self.condition = condition
        
    def instantiate(self, mapping, parent=None):
        if not isinstance(mapping, dict):
            mapping = dict([(param.name, c) for (param, c) in zip(self.args, mapping)])
        Scope.instantiate(self, mapping, parent)

    def copy(self, newdomain=None):
        if not newdomain:
            newdomain = self.parent
            
        a = Axiom(self.predicate, [], None, newdomain)
        a.args = a.copy_args(self.args)
        a.condition = self.condition.copy(a)
        
        return a
        
        
    @staticmethod
    def parse(it, scope):
        it.get(":derived")
        j = iter(it.get(list, "axiom head"))
        name = j.get("terminal", "predicate identifier").token
        args = predicates.parse_arg_list(j, scope.types)

        if name.string not in scope.predicates:
            raise ParseError(name, "Axiom head must be declared as a predicate beforehand.")
        pred_candidates = scope.predicates[name.string]

        pred = None
        for p in pred_candidates:
            pred = p
            for a,pa in zip(args, p.args):
                if not a.is_instance_of(pa.type):
                    pred = None
                    break
            if pred:
                break

        if not pred:
            type_str = " ".join(str(a.type) for a in args)
            c_str = "\n  ".join(str(p) for p in pred_candidates)
            raise ParseError(name, "no matching predicate found for (%s %s). Candidates are:\n  %s" % (name.string, type_str, c_str))

        axiom = Axiom(pred, args, None, scope)
        axiom.condition = conditions.Condition.parse(iter(it.get(list, "condition")), axiom)
        return axiom


def stratify(axioms):
    def posVisitor(cond, results):
        if isinstance(cond, conditions.LiteralCondition):
            if not cond.negated:
                return [cond.predicate]
            return []
        return sum(results, [])

    def negVisitor(cond, results):
        if isinstance(cond, conditions.LiteralCondition):
            if cond.negated:
                return [cond.predicate]
            return []
        return sum(results, [])

    pred_to_axioms = defaultdict(set)
    for a in axioms:
        pred_to_axioms[a.predicate].add(a)
    derived = pred_to_axioms.keys()

    #order the derived predicates
    R = defaultdict(lambda: 0)
    for a in axioms:
        j = a.predicate
        pos = a.condition.visit(posVisitor)
        neg = a.condition.visit(negVisitor)
        for i in derived:
            if i in neg:
                R[i,j] = 2
            elif i in pos:
                R[i,j] = max(1, R[i,j])
    for j in derived:
        for i in derived:
            for k in derived:
                if min(R[i,j], R[j,k]) > 0:
                    R[i,k] = max(R[i,j], R[j,k], R[i,k])

    assert all(R[d,d] != 2 for d in derived), "Couldn't stratify axioms, negative cycle exists."

    #extract strata
    level = 1
    remaining = set(derived)
    stratification = {}
    nonrecursive = set()
    
    while remaining:
        stratum = set()
        for j in remaining:
            if all(R[i,j] != 2 for i in remaining):
                stratum.add(j)
            if all(R[i,j] != 1 for i in remaining):
                nonrecursive.add(j)

        stratification[level] = stratum
        remaining -= stratum
        level += 1

    return stratification, nonrecursive
    
