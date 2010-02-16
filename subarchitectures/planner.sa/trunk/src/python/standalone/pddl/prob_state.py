#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools, time, random
from itertools import imap, ifilter
from collections import defaultdict

import mapltypes as types
import conditions, problem, effects, durative, mapl
from state import StateVariable, Fact, State
from builtin import *
from predicates import *

class ValueDistribution(defaultdict):
    def __init__(self, d=None, prob=1.0):
        defaultdict.__init__(self, lambda: 0.0)
        if isinstance(d, dict):
            self.update(d)
        elif isinstance(d, types.TypedObject):
            self[d] = prob

    def copy(self):
        return ValueDistribution(d=self)
    
    def _get_det_value(self):
        for val, p in self.iteritems():
            if p > 0.999:
                return val
        return None
    value = property(_get_det_value)

    def normalize(self):
        total = sum(v for k,v in self.iteritems() if k != UNKNOWN)
        if total > 1.0:
            norm = 1.0/total
            self *= norm
        elif total < 1.0:
            self[UNKNOWN] = 1.0-total

    def assert_instanceof(self, svar):
        for val, p in self.iteritems():
            if p <= 0.0:
                continue
            assert val.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(val))

    def __iadd__(self, other):
        for k,v in other.iteritems():
            self[k] = self[k] + v
        return self

    def __add__(self, other):
        new = ValueDistribution(self)
        new += other
        return new

    def __imul__(self, other):
        for k,v in self.iteritems():
            self[k] *= other
        return self
    
    def __mul__(self, other):
        new = ValueDistribution(self)
        new *= other
        return new
        
class ProbFact(Fact):
    def __new__(_class, svar, value):
        if isinstance(value, dict):
            vdist = ValueDistribution(value)
            vdist.assert_instanceof(svar)
            return tuple.__new__(_class, (svar, vdist))
        assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        return tuple.__new__(_class, (svar, ValueDistribution(value)))
   
    def __str__(self):
        values = ["%s (%.2f)" % (str(k), v) for k,v in self.value.iteritems() if v > 0]
        return "%s = {%s}" % (str(self.svar), ", ".join(values))

    @staticmethod
    def from_effect(effect, state=None):
        if isinstance(effect, effects.SimpleEffect):
            det_fact = Fact.from_literal(effect, state)
            return ProbFact(det_fact.svar, det_fact.value)
        elif isinstance(effect, effects.ProbabilisticEffect):
            remaining_effects = []
            p_total = 0

            dist = ValueDistribution()
            
            for p, eff in effect.effects:
                assert p_total <= 1.0
                if p is None:
                    remaining_effects.append(eff)
                else:
                    if not isinstance(p, ConstantTerm):
                        assert s is not None
                        p = s.evaluate_term(p)
                    
                    p_total += p.object.value
            
            return ProbFact()#TODO
        raise  Exception("can't get facts for quantified or conditional effects")
    
    @staticmethod
    def from_tuple(tup):
        return ProbFact(tup[0], tup[1])

        
class ProbabilisticState(State):
    def __init__(self, facts=[], prob=None):
        assert prob is None or isinstance(prob, problem.Problem)
        self.problem = prob
        for f in facts:
            self.set(f)

        self.read_svars = set()
        self.written_svars = set()
        
        self.random = random.Random()

    def copy(self):
        s = ProbabilisticState([], self.problem)
        for svar, values in self.iteritems():
            s[svar] = values.copy()
        return s

    def set_random_seed(self, seed):
        self.random.seed(seed)

    def iterdists(self):
        return (ProbFact.from_tuple(tup) for tup in self.iteritems())

    def iterfacts(self):
        def fact_iter(svar, dist):
            for k,v in dist.iteritems():
                if v > 0:
                    yield Fact(svar, k), v
        return itertools.chain(*(fact_iter(svar, dist) for svar,dist in self.iteritems()))
    
    def set(self, fact):
        if isinstance(fact, ProbFact):
            self[fact.svar] = fact.value
        else:
            self[fact.svar] = ValueDistribution(fact.value)

    def update(self, fact):
        if not isinstance(fact, ProbFact):
            self.set(fact)
            return
        current = self.dist(fact.svar)
        updist = ValueDistribution(fact.value)
        current *= updist[UNKNOWN]
        del updist[UNKNOWN]
        current += updist
        self[fact.svar] = current

    def dist(self, key):
        try:
            return dict.__getitem__(self, key)
        except:
            if isinstance(key.function, Predicate):
                return ValueDistribution(FALSE)
            return ValueDistribution(UNKNOWN)
        
    def __getitem__(self, key):
        try:
            dist = dict.__getitem__(self, key)
            dval = dist.value
            if dval is not None:
                return dval
            return dist
        except:
            if isinstance(key.function, Predicate):
                return FALSE
            return UNKNOWN
        
    def __setitem__(self, svar, value):
        assert isinstance(svar, StateVariable)
        if isinstance(value, dict):
            vdist = ValueDistribution(value)
            vdist.assert_instanceof(svar)
            dict.__setitem__(self, svar, vdist)
        else:
            assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
            dict.__setitem__(self, ValueDistribution(value))

    def __contains__(self, key):
        if isinstance(key, Fact):
            return dict.__contains__(self, key.svar) and self.dist(key.svar)[key.value] > 0.0
        return dict.__contains__(self, key)

    @staticmethod
    def from_problem(problem, seed=None):
        s = ProbabilisticState([], problem)
        if seed is not None:
            s.set_random_seed(seed)
            
        for i in problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                s.apply_effect(i)
            else:
                s.set(Fact.from_literal(i))
        return s


    def get_effect_facts(self, effect, trace_vars=False):
        facts = defaultdict(ValueDistribution)
        
        if isinstance(effect, effects.ProbabilisticEffect):
            remaining_effects = []
            all_effects = []
            p_total = 0.0
            
            for p, eff in effect.effects:
                assert p_total <= 1.0
                if p is None:
                    remaining_effects.append(eff)
                else:
                    p = self.evaluate_term(p, trace_vars=trace_vars)
                    all_effects.append((eff, p.value))
                    p_total += p.value

            if remaining_effects:
                p = (1.0-p_total) / len(remaining_effects)
                for eff in remaining_effects:
                    all_effects.append((eff, p))

            for eff, p in all_effects:
                for svar, val in self.get_effect_facts(eff, trace_vars).iteritems():
                    if isinstance(val, ValueDistribution):
                        facts[svar] += val*p
                    else:
                        facts[svar][val] += p
                
            for dist in facts.itervalues():
                dist.normalize()
            return facts
        
        return State.get_effect_facts(self, effect, trace_vars)
        
    def apply_effect(self, effect, read_vars=None):
        if read_vars is not None:
            facts = self.get_effect_facts(effect, trace_vars=True)
            for svar in self.read_svars:
                read_vars.add(svar)
        else:
            facts = self.get_effect_facts(effect)

        for svar, val in facts.iteritems():
            if isinstance(val, ValueDistribution):
                self.update(ProbFact(svar, val))
            else:
                self[svar] = val


    def determinized_state(self, lower_threshold, upper_threshold):
        s = State(prob=self.problem)
        svar2idvars = defaultdict(list)
        for fact, prob in self.iterfacts():
            if prob >= upper_threshold:
                s.set(fact)
            elif prob > lower_threshold:
                idvar = fact.svar.as_modality(mapl.i_indomain, [fact.value])
                svar2idvars[fact.svar].append(idvar)
        for svar, idvars in svar2idvars.iteritems():
            if svar not in s:
                for idvar in idvars:
                    s[idvar] = TRUE
        return s
            
        
